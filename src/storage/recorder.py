import csv
import io
import json
import logging
import struct
import time
from dataclasses import asdict
from pathlib import Path

import lz4.frame
import numpy as np

from src.config import Config
from src.hardware.camera import Calibration, DepthFrame, Intrinsics
from src.hardware.gps import GPSFix, InterpolatedFix
from src.hardware.imu import IMUSample, Orientation

logger = logging.getLogger(__name__)

# IMU binary format: timestamp_ns(q) + accel xyz(3f) + gyro xyz(3f) = 32 bytes
IMU_STRUCT = struct.Struct("<q3f3f")


class Recorder:
    """Writes one drive session to SSD

    Lifecycle:
        1. recorder = Recorder(config)
        2. recorder.start(intrinsics, calibration)
        3. recorder.write_depth(frame, fix, orientation)
        4. recorder.write_imu(sample)
        5. recorder.write_gps(fix)
        6. recorder.stop()
    """

    def __init__(self, config: Config = Config()):
        self._config = config
        self._session_dir: Path | None = None
        self._session_id: str | None = None
        self._frame_index_writer: csv.writer | None = None
        self._frame_index_file: io.TextIOWrapper | None = None
        self._gps_writer: csv.writer | None = None
        self._gps_file: io.TextIOWrapper | None = None
        self._imu_file: io.BufferedWriter | None = None
        self._frames_dir: Path | None = None
        self._running = False

    @property
    def get_session_id(self) -> str | None:
        return self._session_id

    @property
    def get_session_dir(self) -> Path | None:
        return self._session_dir

    def start(self, intrinsics: Intrinsics, calibration: Calibration):
        """Create session directory and open all output files"""
        self._session_id = time.strftime("%Y%m%d_%H%M%S")
        self._session_dir = self._config.storage.sessions_dir / self._session_id
        self._frames_dir = self._session_dir / "frames"
        self._frames_dir.mkdir(parents=True)

        # Write metadata
        metadata = {
            "session_id": self._session_id,
            "start_time": time.time(),
            "intrinsics": {
                "fx": intrinsics.fx,
                "fy": intrinsics.fy,
                "cx": intrinsics.cx,
                "cy": intrinsics.cy,
                "distortion": intrinsics.distortion,
            },
            "mounting_angle_rad": calibration.mounting_angle_rad,
            "calibration_accel_std": calibration.accel_std,
            "config": {
                "camera": asdict(self._config.camera),
                "imu": asdict(self._config.imu),
                "gps": asdict(self._config.gps),
            },
        }
        with open(self._session_dir / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2, default=str)

        # Open frame index CSV
        self._frame_index_file = open(self._session_dir / "frame_index.csv", "w", newline="")
        self._frame_index_writer = csv.writer(self._frame_index_file)
        self._frame_index_writer.writerow([
            "frame_num", "timestamp_ns", "lat", "lon", "alt",
            "heading", "speed", "qw", "qx", "qy", "qz",
            "depth_file", "geo_confidence",
        ])

        # Open GPS trace CSV
        self._gps_file = open(self._session_dir / "gps_trace.csv", "w", newline="")
        self._gps_writer = csv.writer(self._gps_file)
        self._gps_writer.writerow([
            "timestamp_ns", "lat", "lon", "alt",
            "heading", "speed", "fix_quality", "num_satellites",
        ])

        # Open raw IMU binary file
        self._imu_file = open(self._session_dir / "imu.bin", "wb")

        self._running = True
        logger.info("Session started: %s", self._session_dir)

    def stop(self):
        """Flush and close all output files"""
        self._running = False

        if self._frame_index_file:
            self._frame_index_file.close()
            self._frame_index_file = None
        if self._gps_file:
            self._gps_file.close()
            self._gps_file = None
        if self._imu_file:
            self._imu_file.close()
            self._imu_file = None

        logger.info("Session stopped: %s", self._session_id)

    def write_depth(self, frame: DepthFrame, fix: InterpolatedFix, orientation: Orientation | None):
        """Write one depth frame + its associated GPS and orientation to disk"""
        if not self._running:
            return

        # Compress and write depth frame
        depth_file = f"{frame.frame_num:06d}.depth.lz4"
        compressed = lz4.frame.compress(frame.data.tobytes())
        with open(self._frames_dir / depth_file, "wb") as f:
            f.write(compressed)

        # Write frame index row
        qw, qx, qy, qz = (0, 0, 0, 0)
        if orientation:
            qw, qx, qy, qz = orientation.qw, orientation.qx, orientation.qy, orientation.qz

        self._frame_index_writer.writerow([
            frame.frame_num, frame.timestamp_ns,
            fix.lat, fix.lon, fix.alt, fix.heading, fix.speed,
            qw, qx, qy, qz,
            depth_file, fix.confidence,
        ])

    def write_imu(self, sample: IMUSample):
        """Write one raw IMU sample to binary file (for offline reprocessing)"""
        if not self._running:
            return

        self._imu_file.write(IMU_STRUCT.pack(
            sample.timestamp_ns,
            sample.accel[0], sample.accel[1], sample.accel[2],
            sample.gyro[0], sample.gyro[1], sample.gyro[2],
        ))

    def write_gps(self, fix: GPSFix):
        """Write one raw GPS fix to trace CSV"""
        if not self._running:
            return

        self._gps_writer.writerow([
            fix.timestamp_ns, fix.lat, fix.lon, fix.alt,
            fix.heading, fix.speed, fix.fix_quality, fix.num_satellites,
        ])
