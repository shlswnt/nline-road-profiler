from dataclasses import dataclass
from pathlib import Path

@dataclass(frozen=True)
class APIConfig:
    host: str = "0.0.0.0"
    port: int = 8080

@dataclass(frozen=True)
class CameraConfig:
    calibration_duration_s: float = 1.0
    calibration_accel_std_threshold: float = 0.1
    depth_fps: int = 30
    depth_width: int = 640
    depth_height: int = 480


@dataclass(frozen=True)
class GPSConfig:
    serial_port: str = "/dev/gps"
    baudrate: int = 38400
    fix_rate_hz: int = 10
    fix_rate_ms: int = 100
    interpolation_timeout_s: float = 3.0


@dataclass(frozen=True)
class IMUConfig:
    imu_rate: int = 200


@dataclass(frozen=True)
class StorageConfig:
    ssd_mount: Path = Path("/mnt/ssd")
    sessions_dir: Path = Path("/mnt/ssd/sessions")
    compression: str = "lz4"
    min_free_gb: float = 1.0


@dataclass(frozen=True)
class Config:
    api: APIConfig = APIConfig()
    camera: CameraConfig = CameraConfig()
    gps: GPSConfig = GPSConfig()
    imu: IMUConfig = IMUConfig()
    storage: StorageConfig = StorageConfig()
