import logging
import threading
from dataclasses import dataclass

import numpy as np
from ahrs.filters import Madgwick

from src.config import IMUConfig

logger = logging.getLogger(__name__)


@dataclass
class IMUSample:
    """Raw IMU reading from BMI270"""
    timestamp_ns: int       # time.monotonic_ns() when sample was received
    accel: np.ndarray       # [ax, ay, az] in m/s²
    gyro: np.ndarray        # [gx, gy, gz] in rad/s


@dataclass
class Orientation:
    """Fused orientation quaternion"""
    timestamp_ns: int
    qw: float
    qx: float
    qy: float
    qz: float


class IMU:
    """Madgwick filter producing orientation quaternions from raw IMU data

    Lifecycle:
        1. imu = IMU(config)
        2. imu.start()
        3. imu.update(sample)
        4. orientation = imu.get_orientation
        5. imu.stop()
    """

    def __init__(self, config: IMUConfig = IMUConfig()):
        self._config = config
        self._lock = threading.Lock()
        self._running = False

        # Madgwick filter state
        self._filter = Madgwick(frequency=config.imu_rate)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z] identity
        self._last_timestamp_ns: int | None = None

    @property
    def get_orientation(self) -> Orientation | None:
        """Most recent fused orientation, or None if no samples processed"""
        with self._lock:
            if self._last_timestamp_ns is None:
                return None
            q = self._quaternion
            return Orientation(
                timestamp_ns=self._last_timestamp_ns,
                qw=q[0], qx=q[1], qy=q[2], qz=q[3],
            )

    def start(self):
        """Initialize filter state"""
        self._running = True
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self._last_timestamp_ns = None
        logger.info("IMU started at %d Hz", self._config.imu_rate)

    def stop(self):
        """Stop accepting samples"""
        self._running = False
        logger.info("IMU stopped")

    def update(self, sample: IMUSample):
        """Feed one raw IMU sample through Madgwick filter"""
        if not self._running:
            return

        with self._lock:
            self._quaternion = self._filter.updateIMU(
                q=self._quaternion,
                gyr=sample.gyro,
                acc=sample.accel,
            )
            self._last_timestamp_ns = sample.timestamp_ns
