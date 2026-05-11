import logging
import threading
import time
from dataclasses import dataclass, field

import depthai as dai
import numpy as np

from src.config import CameraConfig, IMUConfig
from src.hardware.imu import IMU, IMUSample

logger = logging.getLogger(__name__)


@dataclass
class Intrinsics:
    """Camera intrinsics from factory calibration"""
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: list[float] = field(default_factory=list)


@dataclass
class DepthFrame:
    """ToF depth frame"""
    timestamp_ns: int       # time.monotonic_ns() when frame was received
    frame_num: int          # frame index
    data: np.ndarray        # 640x480 uint16 depth in mm


@dataclass
class Calibration:
    """Mounting angle auto-calibration result"""
    mounting_angle_rad: float
    accel_std: float        # stationarity quality flag


class Camera:
    """OAK-D-SR-POE depth + IMU capture

    Lifecycle:
        1. camera = Camera(camera_config, imu_config)
        2. camera.start()
        3. calibration = camera.calibrate()
        4. for frame in camera.frames(): ...
        5. camera.stop()
    """

    def __init__(self, camera_config: CameraConfig = CameraConfig(),
                 imu_config: IMUConfig = IMUConfig()):
        self._camera_config = camera_config
        self._imu_config = imu_config
        self._device: dai.Device | None = None
        self._pipeline: dai.Pipeline | None = None
        self._depth_queue: dai.DataOutputQueue | None = None
        self._imu_queue: dai.DataOutputQueue | None = None
        self._imu = IMU(imu_config)
        self._intrinsics: Intrinsics | None = None
        self._frame_num = 0
        self._running = False
        self._imu_thread: threading.Thread | None = None

    @property
    def get_intrinsics(self) -> Intrinsics | None:
        """Factory calibration intrinsics, available after start()"""
        return self._intrinsics

    @property
    def get_imu(self) -> IMU:
        """IMU module for orientation queries"""
        return self._imu

    def start(self):
        """Build pipeline, connect to device, read calibration, start IMU"""
        self._pipeline = self._pipeline()
        self._device = dai.Device(self._pipeline)

        # Read factory calibration from device EEPROM
        calib = self._device.readCalibration()
        matrix = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)
        distortion = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        self._intrinsics = Intrinsics(
            fx=matrix[0][0], fy=matrix[1][1],
            cx=matrix[0][2], cy=matrix[1][2],
            distortion=distortion,
        )

        # Get output queues
        self._depth_queue = self._device.getOutputQueue("depth", maxSize=4, blocking=False)
        self._imu_queue = self._device.getOutputQueue("imu", maxSize=50, blocking=False)

        # Start IMU processing in background thread
        self._imu.start()
        self._running = True
        self._frame_num = 0
        self._imu_thread = threading.Thread(target=self._loop, daemon=True)
        self._imu_thread.start()

        logger.info("Camera started — %dx%d @ %d FPS, intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                     self._camera_config.depth_width, self._camera_config.depth_height,
                     self._camera_config.depth_fps,
                     self._intrinsics.fx, self._intrinsics.fy,
                     self._intrinsics.cx, self._intrinsics.cy)

    def stop(self):
        """Stop capture, close device"""
        self._running = False
        self._imu.stop()
        if self._imu_thread:
            self._imu_thread.join(timeout=2.0)
            self._imu_thread = None
        if self._device:
            self._device.close()
            self._device = None
        logger.info("Camera stopped")

    def calibrate(self) -> Calibration:
        """Auto-calibrate mounting angle from gravity vector while stationary"""
        duration_s = self._camera_config.calibration_duration_s
        samples = []
        deadline = time.monotonic() + duration_s

        # Sample accelerometer for callibration duration
        while time.monotonic() < deadline:
            imu_data = self._imu_queue.tryGet()
            if imu_data is None:
                time.sleep(0.001)
                continue
            for packet in imu_data.packets:
                accel = packet.acceleroMeter
                samples.append([accel.x, accel.y, accel.z])

        if not samples:
            logger.warning("No IMU samples collected during calibration")
            return Calibration(mounting_angle_rad=0.0, accel_std=999.0)

        accel_samples = np.array(samples)

        # Mounting angle: angle between average gravity vector and camera Z-axis
        gravity = np.mean(accel_samples, axis=0)
        gravity /= np.linalg.norm(gravity)
        mounting_angle_rad = float(np.arccos(np.clip(np.dot(gravity, [0, 0, 1]), -1, 1)))

        # Stationarity check: std dev of accel magnitudes
        accel_std = float(np.std(np.linalg.norm(accel_samples, axis=1)))

        if accel_std > self._camera_config.calibration_accel_std_threshold:
            logger.warning("Vehicle may have been moving during calibration (accel_std=%.3f)", accel_std)
        else:
            logger.info("Calibration complete — mounting angle=%.1f°, accel_std=%.4f",
                         np.degrees(mounting_angle_rad), accel_std)

        return Calibration(mounting_angle_rad=mounting_angle_rad, accel_std=accel_std)

    def frames(self) -> DepthFrame | None:
        """Get next depth frame, or None if not available (Non-blocking)"""
        if not self._running or self._depth_queue is None:
            return None

        depth_data = self._depth_queue.tryGet()
        if depth_data is None:
            return None

        frame = DepthFrame(
            timestamp_ns=time.monotonic_ns(),
            frame_num=self._frame_num,
            data=depth_data.getFrame(),
        )
        self._frame_num += 1
        return frame

    def _pipeline(self) -> dai.Pipeline:
        """Construct DepthAI pipeline with ToF depth + IMU nodes"""
        pipeline = dai.Pipeline()

        # ToF depth node
        tof = pipeline.create(dai.node.ToF)
        cam_tof = pipeline.create(dai.node.Camera)
        cam_tof.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_tof.setFps(self._camera_config.depth_fps)
        cam_tof.raw.link(tof.input)

        # Depth output
        depth_out = pipeline.create(dai.node.XLinkOut)
        depth_out.setStreamName("depth")
        tof.depth.link(depth_out.input)

        # IMU node (shares pipeline with ToF)
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, self._imu_config.imu_rate)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, self._imu_config.imu_rate)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        # IMU output
        imu_out = pipeline.create(dai.node.XLinkOut)
        imu_out.setStreamName("imu")
        imu.out.link(imu_out.input)

        return pipeline

    def _loop(self):
        """Background thread to read IMU packets and feed to Madgwick filter"""
        while self._running:
            try:
                imu_data = self._imu_queue.tryGet()
                if imu_data is None:
                    time.sleep(0.001)
                    continue
                for packet in imu_data.packets:
                    accel = packet.acceleroMeter
                    gyro = packet.gyroscope
                    sample = IMUSample(
                        timestamp_ns=time.monotonic_ns(),
                        accel=np.array([accel.x, accel.y, accel.z]),
                        gyro=np.array([gyro.x, gyro.y, gyro.z]),
                    )
                    self._imu.update(sample)
            except Exception:
                if self._running:
                    logger.exception("IMU read error")
                    time.sleep(0.01)
