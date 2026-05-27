import logging
import shutil
import signal
import threading
import time

from src.config import Config
from src.hardware.camera import Camera
from src.hardware.gps import GPS
from src.storage.recorder import Recorder

SSD_CHECK_INTERVAL = 300  # check SSD space every 300 frames (~20s at 15 FPS)

logger = logging.getLogger(__name__)


class Profiler:
    """Handles synchronized capture of depth, IMU, and GPS

    Lifecycle:
        1. profiler = Profiler(config)
        2. profiler.start()           # init sensors, calibrate, begin recording
        3. profiler.stop()            # stop recording, shutdown sensors
    """

    def __init__(self, config: Config = Config()):
        self._config = config
        self._recorder = Recorder(config)
        self._camera = Camera(config.camera, config.imu, on_imu_sample=self._recorder.write_imu)
        self._gps = GPS(config.gps, on_fix=self._recorder.write_gps)
        self._running = False
        self._thread: threading.Thread | None = None
        self._start_time: float = 0.0
        self._frame_count: int = 0

    @property
    def is_recording(self) -> bool:
        return self._running

    @property
    def get_camera(self) -> Camera:
        return self._camera

    @property
    def get_gps(self) -> GPS:
        return self._gps

    @property
    def get_recorder(self) -> Recorder:
        return self._recorder

    def init_sensors(self):
        """Initialize camera and GPS (call once on boot)"""
        self._camera.start()
        self._gps.start()
        logger.info("Sensors initialized")

    def start(self):
        """Run calibration, begin recording in background thread"""
        if self._running:
            return

        # Auto-calibrate mounting angle (vehicle must be stationary)
        logger.info("Running mounting angle calibration...")
        calibration = self._camera.calibrate()

        # Start recording to SSD
        self._recorder.start(self._camera.get_intrinsics, calibration)
        self._running = True
        self._start_time = time.monotonic()
        self._frame_count = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("Recording started")

    def stop(self):
        """Stop recording"""
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
            self._thread = None
        self._frame_count = 0
        self._start_time = 0.0

    def shutdown(self):
        """Stop recording and shutdown sensors"""
        self.stop()
        self._camera.stop()
        self._gps.stop()
        logger.info("Profiler shutdown")

    @property
    def get_frame_count(self) -> int:
        return self._frame_count

    @property
    def get_duration_s(self) -> float:
        if not self._running:
            return 0.0
        return time.monotonic() - self._start_time

    def _loop(self):
        """Capture loop: grab frames, sync data, write to disk

        ToF runs at max internal FPS for motion blur reduction, but we only
        save at the configured depth_fps to keep storage reasonable.
        """
        save_interval_ns = int(1e9 / self._config.camera.depth_fps)
        last_save_ns = 0

        try:
            while self._running:
                # Get next depth frame (non-blocking)
                frame = self._camera.frames()
                if frame is None:
                    time.sleep(0.001)
                    continue

                # Decimate: only save at configured depth_fps
                if frame.timestamp_ns - last_save_ns < save_interval_ns:
                    continue
                last_save_ns = frame.timestamp_ns

                # Get interpolated GPS position for this frame
                fix = self._gps.interpolate(frame.timestamp_ns)

                # Get latest IMU orientation
                orientation = self._camera.get_imu.get_orientation

                # Write synchronized data to SSD
                try:
                    self._recorder.write_depth(frame, fix, orientation)
                    self._frame_count += 1
                except OSError:
                    logger.exception("Write error — stopping recording")
                    break

                # Periodic SSD space check
                if self._frame_count % SSD_CHECK_INTERVAL == 0:
                    free_gb = shutil.disk_usage(str(self._config.storage.ssd_mount)).free / (1024 ** 3)
                    if free_gb < self._config.storage.min_free_gb:
                        logger.warning("SSD low: %.1f GB free — stopping recording", free_gb)
                        break
        except Exception:
            logger.exception("Capture loop error")
        finally:
            self._running = False
            self._recorder.stop()
            logger.info("Recording stopped")


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    import uvicorn
    from src.api.server import create_app

    config = Config()
    profiler = Profiler(config)

    # Initialize sensors on boot (before API starts)
    profiler.init_sensors()

    # Graceful shutdown on SIGINT/SIGTERM
    def shutdown(sig, frame):
        logger.info("Shutdown signal received")
        profiler.shutdown()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Start API server (blocks — session start/stop via HTTP)
    app = create_app(profiler, config)
    uvicorn.run(app, host=config.api.host, port=config.api.port)


if __name__ == "__main__":
    main()
