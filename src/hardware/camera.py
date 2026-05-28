import logging
import threading
import time
from dataclasses import dataclass, field

import cv2
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
                 imu_config: IMUConfig = IMUConfig(), on_imu_sample=None):
        self._camera_config = camera_config
        self._imu_config = imu_config
        self._on_imu_sample = on_imu_sample  # callback per raw IMU sample: fn(IMUSample)
        self._pipeline: dai.Pipeline | None = None
        self._depth_queue = None
        self._rgb_queue = None
        self._imu_queue = None
        self._imu = IMU(imu_config)
        self._intrinsics: Intrinsics | None = None
        self._frame_num = 0
        self._running = False
        self._imu_thread: threading.Thread | None = None
        self._preview_thread: threading.Thread | None = None
        # Latest frames for preview streaming (updated by background thread)
        self._latest_depth: np.ndarray | None = None
        self._latest_rgb: np.ndarray | None = None
        self._latest_depth_frame: DepthFrame | None = None
        self._frame_lock = threading.Lock()

    @property
    def get_intrinsics(self) -> Intrinsics | None:
        """Factory calibration intrinsics, available after start()"""
        return self._intrinsics

    @property
    def get_imu(self) -> IMU:
        """IMU module for orientation queries"""
        return self._imu

    def start(self):
        """Build pipeline, start device, read calibration, start IMU"""
        self._pipeline = dai.Pipeline()

        # ToF depth node (v3: build() auto-connects to CAM_A internally)
        tof = self._pipeline.create(dai.node.ToF)

        # Tune ToF for motion blur reduction at driving speed.
        # Run sensor at max internal FPS (shorter integration = less blur).
        # Burst mode OFF so sensor can hit ~80 FPS output (burst caps at 40).
        # Phase shuffle temporal filter OFF prevents frame reuse across time.
        tof_config = tof.getInitialConfig()
        tof_config.enablePhaseShuffleTemporalFilter = False
        tof_config.phaseUnwrappingLevel = 4                  # keep full range (~7.5m)
        tof_config.enableOpticalCorrection = True
        tof_config.enableBurstMode = False
        tof.setInitialConfig(tof_config)

        tof.build()

        # RGB camera node (CAM_B = left stereo camera, global shutter OV9782)
        cam_rgb = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)

        # IMU node (shares pipeline with ToF)
        imu_node = self._pipeline.create(dai.node.IMU)
        imu_node.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, self._imu_config.imu_rate)
        imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, self._imu_config.imu_rate)
        imu_node.setBatchReportThreshold(1)
        imu_node.setMaxBatchReports(10)

        # Create output queues (v3: directly on node outputs, no XLinkOut)
        self._depth_queue = tof.depth.createOutputQueue()
        self._rgb_queue = cam_rgb.requestOutput((640, 480)).createOutputQueue()
        self._imu_queue = imu_node.out.createOutputQueue()

        # Start pipeline (v3: pipeline.start(), no dai.Device(pipeline))
        self._pipeline.start()

        # Read factory calibration from device EEPROM
        device = self._pipeline.getDefaultDevice()
        calib = device.readCalibration()
        matrix = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)
        distortion = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        self._intrinsics = Intrinsics(
            fx=matrix[0][0], fy=matrix[1][1],
            cx=matrix[0][2], cy=matrix[1][2],
            distortion=distortion,
        )

        # Start IMU and preview processing in background threads
        self._imu.start()
        self._running = True
        self._frame_num = 0
        self._imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self._imu_thread.start()
        self._preview_thread = threading.Thread(target=self._preview_loop, daemon=True)
        self._preview_thread.start()

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
        if self._preview_thread:
            self._preview_thread.join(timeout=2.0)
            self._preview_thread = None
        if self._pipeline:
            self._pipeline.stop()
            self._pipeline = None
        self._latest_depth = None
        self._latest_rgb = None
        self._latest_depth_frame = None
        logger.info("Camera stopped")

    def calibrate(self) -> Calibration:
        """Auto-calibrate mounting angle from gravity vector while stationary"""
        duration_s = self._camera_config.calibration_duration_s
        samples = []
        deadline = time.monotonic() + duration_s

        # Sample accelerometer for calibration duration
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
        """Get latest depth frame from buffer, or None (non-blocking)"""
        with self._frame_lock:
            frame = self._latest_depth_frame
            self._latest_depth_frame = None  # consume it
            return frame

    def get_depth_jpeg(self) -> bytes | None:
        """Latest depth frame as colorized JPEG for preview streaming"""
        depth = self._latest_depth
        if depth is None:
            return None
        # Normalize to 0-255, apply colormap
        max_depth = 7500  # mm, matches phaseUnwrappingLevel=4
        normalized = np.clip(depth.astype(np.float32) / max_depth * 255, 0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        # Zero-depth pixels -> black
        colored[depth == 0] = 0
        _, jpeg = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes()

    def get_rgb_jpeg(self) -> bytes | None:
        """Latest RGB frame as JPEG for preview streaming"""
        rgb = self._latest_rgb
        if rgb is None:
            return None
        _, jpeg = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes()

    def _preview_loop(self):
        """Background thread to consume depth + RGB frames into preview buffers"""
        while self._running:
            try:
                # Depth
                depth_data = self._depth_queue.tryGet()
                if depth_data is not None:
                    raw = depth_data.getFrame()
                    self._latest_depth = raw
                    frame = DepthFrame(
                        timestamp_ns=time.monotonic_ns(),
                        frame_num=self._frame_num,
                        data=raw,
                    )
                    self._frame_num += 1
                    with self._frame_lock:
                        self._latest_depth_frame = frame

                # RGB
                rgb_data = self._rgb_queue.tryGet()
                if rgb_data is not None:
                    self._latest_rgb = rgb_data.getCvFrame()

                if depth_data is None and rgb_data is None:
                    time.sleep(0.001)
            except Exception:
                if self._running:
                    logger.exception("Preview loop error")
                    time.sleep(0.01)

    def _imu_loop(self):
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
                    if self._on_imu_sample:
                        self._on_imu_sample(sample)
            except Exception:
                if self._running:
                    logger.exception("IMU read error")
                    time.sleep(0.01)
