import logging
import signal
import threading
import time

from src.config import Config
from src.hardware.camera import Camera
from src.hardware.gps import GPS
from src.storage.recorder import Recorder

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
        self._camera = Camera(config.camera, config.imu)
        self._gps = GPS(config.gps)
        self._recorder = Recorder(config)
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
        """Capture loop: grab frames, sync data, write to disk"""
        while self._running:
            # Get next depth frame (non-blocking)
            frame = self._camera.frames()
            if frame is None:
                time.sleep(0.001)
                continue

            # Get interpolated GPS position for this frame
            fix = self._gps.interpolate(frame.timestamp_ns)

            # Get latest IMU orientation
            orientation = self._camera.get_imu.get_orientation

            # Write synchronized data to SSD
            self._recorder.write_depth(frame, fix, orientation)
            self._frame_count += 1

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
