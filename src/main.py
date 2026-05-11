import logging
import signal
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

    def start(self):
        """Initialize sensors, run calibration, begin recording"""
        # Start sensors
        self._camera.start()
        self._gps.start()

        # Auto-calibrate mounting angle (vehicle must be stationary)
        logger.info("Running mounting angle calibration...")
        calibration = self._camera.calibrate()

        # Start recording to SSD
        self._recorder.start(self._camera.get_intrinsics, calibration)
        self._running = True
        logger.info("Recording started")

        # Capture loop
        self._loop()

    def stop(self):
        """Stop recording and shutdown sensors"""
        self._running = False

    def _loop(self):
        """Main capture loop: grab frames, sync data, write to disk"""
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

        # Shutdown
        self._recorder.stop()
        self._camera.stop()
        self._gps.stop()
        logger.info("Recording stopped")


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    config = Config()
    profiler = Profiler(config)

    # Graceful shutdown on SIGINT/SIGTERM
    def shutdown(sig, frame):
        logger.info("Shutdown signal received")
        profiler.stop()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    profiler.start()


if __name__ == "__main__":
    main()
