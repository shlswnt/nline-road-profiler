import logging
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial
from pynmeagps import NMEAReader, NMEAMessage
from pyubx2 import UBXMessage

from src.config import GPSConfig

logger = logging.getLogger(__name__)


@dataclass
class GPSFix:
    """GPS fix from parsed NMEA sentences"""
    timestamp_ns: int       # time.monotonic_ns() when fix was received
    lat: float              # degrees
    lon: float              # degrees
    alt: float              # meters above MSL
    heading: float          # degrees true north (from RMC/VTG)
    speed: float            # m/s (from RMC)
    fix_quality: int        # 0=invalid, 1=GPS, 2=DGPS
    num_satellites: int


@dataclass
class InterpolatedFix:
    """GPS position interpolated to a specific timestamp"""
    lat: float
    lon: float
    alt: float
    heading: float
    speed: float            
    confidence: str         # "good", "stale", or "no_fix"


class GPS:
    """Reads NMEA from NEO-M9N, buffers fixes, interpolates per frame

    Lifecycle:
        1. gps = GPS(config)
        2. gps.start()
        3. fix = gps.interpolate(frame_timestamp_ns)
        4. gps.stop()
    """

    # Buffer holds ~5 seconds of 10 Hz fixes
    BUFFER_SIZE = 50

    def __init__(self, config: GPSConfig = GPSConfig()):
        self._config = config
        self._buffer: deque[GPSFix] = deque(maxlen=self.BUFFER_SIZE)
        self._lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._running = False

        # Partial state built across GGA + RMC sentence pairs
        self._pending_lat: float | None = None
        self._pending_lon: float | None = None
        self._pending_alt: float = 0.0
        self._pending_fix_quality: int = 0
        self._pending_num_satellites: int = 0

    @property
    def get_fix(self) -> GPSFix | None:
        """Most recent fix in buffer, or None if empty"""
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def start(self):
        """Open serial port, configure 10 Hz, start read thread"""
        self._serial = serial.Serial(
            self._config.serial_port,
            self._config.baudrate,
            timeout=1.0,
        )

        # Configure NEO-M9N to 10 Hz fix rate (100ms)
        msg = UBXMessage.config_set(
            layers=1, transaction=0,
            cfgData=[("CFG_RATE_MEAS", self._config.fix_rate_ms)],
        )
        self._serial.write(msg.serialize())
        logger.info("GPS configured to %d Hz", self._config.fix_rate_hz)

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("GPS started on %s at %d baud", self._config.serial_port, self._config.baudrate)

    def stop(self):
        """Stop read thread and close serial port"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
        logger.info("GPS stopped")

    def interpolate(self, timestamp_ns: int) -> InterpolatedFix:
        """Linearly interpolate GPS position at given monotonic timestamp"""
        with self._lock:
            # Check existance (whether buffer is empty)
            if not self._buffer:
                return InterpolatedFix(0, 0, 0, 0, 0, confidence="no_fix")

            fixes = list(self._buffer)

        # Check staleness (nearest fix is older than timeout) against most recent fix
        age_s = (timestamp_ns - fixes[-1].timestamp_ns) / 1e9
        if age_s > self._config.interpolation_timeout_s:
            f = fixes[-1]
            return InterpolatedFix(f.lat, f.lon, f.alt, f.heading, f.speed, confidence="stale")

        # Find bracketing fixes
        before, after = None, None
        for i in range(len(fixes) - 1):
            if fixes[i].timestamp_ns <= timestamp_ns <= fixes[i + 1].timestamp_ns:
                before, after = fixes[i], fixes[i + 1]
                break

        # Timestamp is before all fixes or after all fixes, uses nearest
        if before is None:
            f = fixes[0] if timestamp_ns < fixes[0].timestamp_ns else fixes[-1]
            return InterpolatedFix(f.lat, f.lon, f.alt, f.heading, f.speed, confidence="good")

        # Linear interpolation
        span = after.timestamp_ns - before.timestamp_ns
        t = (timestamp_ns - before.timestamp_ns) / span if span > 0 else 0.0

        return InterpolatedFix(
            lat=before.lat + t * (after.lat - before.lat),
            lon=before.lon + t * (after.lon - before.lon),
            alt=before.alt + t * (after.alt - before.alt),
            heading=before.heading + t * (after.heading - before.heading),
            speed=before.speed + t * (after.speed - before.speed),
            confidence="good",
        )

    def _loop(self):
        """Background thread to read and parse NMEA sentences"""
        gps = NMEAReader(self._serial)
        while self._running:
            try:
                raw, msg = gps.read()
                if isinstance(msg, NMEAMessage):
                    self._parse(msg)
            except Exception:
                if self._running:
                    logger.exception("GPS read error")
                    time.sleep(0.1)

    def _parse(self, msg: NMEAMessage):
        """Parse GGA and RMC sentences into GPS fixes"""
        # GGA provides: lat, lon, alt, fix quality, satellite count
        if msg.msgID == "GGA":
            self._pending_lat = getattr(msg, "lat", None)
            self._pending_lon = getattr(msg, "lon", None)
            self._pending_alt = float(getattr(msg, "alt", 0) or 0)
            self._pending_fix_quality = int(getattr(msg, "quality", 0) or 0)
            self._pending_num_satellites = int(getattr(msg, "numSV", 0) or 0)

        # RMC provides: heading, speed, and triggers fix creation
        elif msg.msgID == "RMC":
            # RMC completes the fix by combining with pending GGA data
            lat = getattr(msg, "lat", None)
            lon = getattr(msg, "lon", None)
            if lat is None or lon is None:
                return

            # Prefer GGA lat/lon if available (has altitude), fall back to RMC
            fix = GPSFix(
                timestamp_ns=time.monotonic_ns(),
                lat=self._pending_lat if self._pending_lat is not None else lat,
                lon=self._pending_lon if self._pending_lon is not None else lon,
                alt=self._pending_alt,
                heading=float(getattr(msg, "cog", 0) or 0),
                speed=float(getattr(msg, "spd", 0) or 0) * 0.514444,  # knots → m/s
                fix_quality=self._pending_fix_quality,
                num_satellites=self._pending_num_satellites,
            )

            with self._lock:
                self._buffer.append(fix)

            # Reset pending state
            self._pending_lat = None
            self._pending_lon = None
