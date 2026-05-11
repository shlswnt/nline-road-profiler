import csv
import json
import struct
import sys
from pathlib import Path

import lz4.frame
import numpy as np

IMU_STRUCT = struct.Struct("<q3f3f")
EXPECTED_FRAME_BYTES = 640 * 480 * 2  # uint16


def validate(session_dir: Path):
    errors = []
    warnings = []

    # Metadata
    metadata_path = session_dir / "metadata.json"
    if not metadata_path.exists():
        errors.append("metadata.json missing")
        return print_results(session_dir, errors, warnings)

    with open(metadata_path) as f:
        metadata = json.load(f)

    accel_std = metadata.get("calibration_accel_std", 0)
    if accel_std > 0.1:
        warnings.append(f"Calibration suspect - accel_std={accel_std:.4f} (> 0.1)")
    elif accel_std > 0.05:
        warnings.append(f"Calibration acceptable - accel_std={accel_std:.4f} (0.05-0.1)")

    # Frame index
    index_path = session_dir / "frame_index.csv"
    if not index_path.exists():
        errors.append("frame_index.csv missing")
        return print_results(session_dir, errors, warnings)

    with open(index_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    index_count = len(rows)

    # Depth frames
    frames_dir = session_dir / "frames"
    if not frames_dir.exists():
        errors.append("frames/ directory missing")
        return print_results(session_dir, errors, warnings)

    depth_files = sorted(frames_dir.glob("*.depth.lz4"))
    file_count = len(depth_files)

    if file_count != index_count:
        errors.append(f"Frame count mismatch — {file_count} files vs {index_count} index rows")

    # Verify depth files decompress to correct size (sample up to 20)
    sample_indices = np.linspace(0, max(file_count - 1, 0), min(20, file_count), dtype=int)
    for i in sample_indices:
        path = depth_files[i]
        try:
            data = lz4.frame.decompress(path.read_bytes())
            if len(data) != EXPECTED_FRAME_BYTES:
                errors.append(f"{path.name}: decompressed to {len(data)} bytes, expected {EXPECTED_FRAME_BYTES}")
        except Exception as e:
            errors.append(f"{path.name}: failed to decompress — {e}")

    # Check for frame number gaps in index
    if rows:
        frame_nums = [int(r["frame_num"]) for r in rows]
        expected = list(range(frame_nums[0], frame_nums[-1] + 1))
        missing = set(expected) - set(frame_nums)
        if missing:
            errors.append(f"{len(missing)} missing frame numbers in index (e.g. {sorted(missing)[:5]})")

    # Check geo confidence
    if rows:
        confidences = [r["geo_confidence"] for r in rows]
        no_fix = confidences.count("no_fix")
        stale = confidences.count("stale")
        good = confidences.count("good")
        if no_fix > 0:
            warnings.append(f"GPS: {no_fix} frames with no_fix ({no_fix/len(rows)*100:.1f}%)")
        if stale > 0:
            warnings.append(f"GPS: {stale} frames with stale fix ({stale/len(rows)*100:.1f}%)")

    # GPS trace
    gps_path = session_dir / "gps_trace.csv"
    if not gps_path.exists():
        warnings.append("gps_trace.csv missing")
        gps_rows = []
    else:
        with open(gps_path) as f:
            gps_rows = list(csv.DictReader(f))

    # Check for GPS gaps (> 500ms between fixes at 10 Hz)
    if len(gps_rows) > 1:
        gps_timestamps = [int(r["timestamp_ns"]) for r in gps_rows]
        gaps = []
        for i in range(1, len(gps_timestamps)):
            gap_ms = (gps_timestamps[i] - gps_timestamps[i - 1]) / 1e6
            if gap_ms > 500:
                gaps.append(gap_ms)
        if gaps:
            warnings.append(f"GPS: {len(gaps)} gaps > 500ms (max {max(gaps):.0f}ms)")

    # IMU binary
    imu_path = session_dir / "imu.bin"
    if not imu_path.exists():
        warnings.append("imu.bin missing")
        imu_count = 0
    else:
        imu_size = imu_path.stat().st_size
        if imu_size % IMU_STRUCT.size != 0:
            errors.append(f"imu.bin size ({imu_size}) not divisible by {IMU_STRUCT.size} bytes")
        imu_count = imu_size // IMU_STRUCT.size

        # Check for timestamp jumps (sample first and last 100)
        if imu_count > 1:
            with open(imu_path, "rb") as f:
                first_ts = IMU_STRUCT.unpack(f.read(IMU_STRUCT.size))[0]
                f.seek(-IMU_STRUCT.size, 2)
                last_ts = IMU_STRUCT.unpack(f.read(IMU_STRUCT.size))[0]
            imu_duration_s = (last_ts - first_ts) / 1e9
            expected_samples = imu_duration_s * 200
            if abs(imu_count - expected_samples) / max(expected_samples, 1) > 0.1:
                warnings.append(f"IMU: expected ~{expected_samples:.0f} samples for {imu_duration_s:.1f}s, got {imu_count}")

    # Summary
    print(f"\n{'=' * 50}")
    print(f"Session: {session_dir.name}")
    print(f"{'=' * 50}")

    # Duration from frame index timestamps
    if rows:
        first_ts = int(rows[0]["timestamp_ns"])
        last_ts = int(rows[-1]["timestamp_ns"])
        duration_s = (last_ts - first_ts) / 1e9
        minutes = int(duration_s // 60)
        seconds = int(duration_s % 60)
        print(f"Duration:       {minutes}m {seconds}s")
    else:
        print(f"Duration:       --")

    print(f"Depth frames:   {file_count}")
    print(f"GPS fixes:      {len(gps_rows)}")
    print(f"IMU samples:    {imu_count}")

    if rows:
        fps = file_count / max(duration_s, 1)
        print(f"Avg FPS:        {fps:.1f}")
        print(f"GPS coverage:   {good}/{len(rows)} frames ({good/len(rows)*100:.1f}%)")

    # Session size
    total_bytes = sum(f.stat().st_size for f in session_dir.rglob("*") if f.is_file())
    print(f"Total size:     {total_bytes / (1024**2):.1f} MB")

    mounting_angle = metadata.get("mounting_angle_rad", 0)
    print(f"Mounting angle: {np.degrees(mounting_angle):.1f}°")
    print(f"Accel std:      {accel_std:.4f}")

    print_results(session_dir, errors, warnings)


def print_results(session_dir: Path, errors: list, warnings: list):
    if warnings:
        print(f"\nWarnings ({len(warnings)}):")
        for w in warnings:
            print(f"  ! {w}")

    if errors:
        print(f"\nErrors ({len(errors)}):")
        for e in errors:
            print(f"  X {e}")
        print(f"\nRESULT: FAIL")
    elif warnings:
        print(f"\nRESULT: PASS (with warnings)")
    else:
        print(f"\nRESULT: PASS")


def main():
    if len(sys.argv) != 2:
        print("Usage: python -m tools.validate_session <session_dir>")
        sys.exit(1)

    session_dir = Path(sys.argv[1])
    if not session_dir.is_dir():
        print(f"Error: {session_dir} is not a directory")
        sys.exit(1)

    validate(session_dir)


if __name__ == "__main__":
    main()
