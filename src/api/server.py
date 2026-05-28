import asyncio
import json
import logging
import shutil
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from src.config import Config
from src.main import Profiler

logger = logging.getLogger(__name__)


def create_app(profiler: Profiler, config: Config) -> FastAPI:
    """Create FastAPI app wired to profiler instance"""

    app = FastAPI()
    clients: list[WebSocket] = []

    @app.post("/session/start")
    async def start_session():
        if profiler.is_recording:
            return {"status": "error", "message": "Already recording"}
        # Run in thread so calibration doesn't block event loop
        await asyncio.to_thread(profiler.start)
        return {"status": "ok", "session_id": profiler.get_recorder.get_session_id}

    @app.post("/session/stop")
    def stop_session():
        if not profiler.is_recording:
            return {"status": "error", "message": "Not recording"}
        profiler.stop()
        return {"status": "ok"}

    @app.get("/stream/{mode}")
    async def stream(mode: str):
        """MJPEG stream of depth (colorized) or RGB camera"""
        if mode not in ("depth", "rgb"):
            return {"status": "error", "message": "Mode must be 'depth' or 'rgb'"}

        camera = profiler.get_camera

        async def generate():
            while True:
                if mode == "depth":
                    jpeg = camera.get_depth_jpeg()
                else:
                    jpeg = camera.get_rgb_jpeg()

                if jpeg is not None:
                    yield (b"--frame\r\n"
                           b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                await asyncio.sleep(0.066)  # ~15 FPS preview

        return StreamingResponse(
            generate(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/session/status")
    def get_status():
        gps_fix = profiler.get_gps.get_fix
        disk = shutil.disk_usage(str(config.storage.ssd_mount))

        return {
            "recording": profiler.is_recording,
            "session_id": profiler.get_recorder.get_session_id,
            "duration_s": round(profiler.get_duration_s, 1),
            "frame_count": profiler.get_frame_count,
            "gps": {
                "has_fix": gps_fix is not None,
                "lat": gps_fix.lat if gps_fix else None,
                "lon": gps_fix.lon if gps_fix else None,
                "satellites": gps_fix.num_satellites if gps_fix else 0,
            },
            "ssd": {
                "free_gb": round(disk.free / (1024 ** 3), 1),
                "total_gb": round(disk.total / (1024 ** 3), 1),
            },
        }

    @app.get("/sessions")
    def list_sessions():
        sessions_dir = config.storage.sessions_dir
        if not sessions_dir.exists():
            return {"sessions": []}

        sessions = []
        for d in sorted(sessions_dir.iterdir(), reverse=True):
            if not d.is_dir():
                continue
            # Calculate session size
            size_bytes = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
            sessions.append({
                "session_id": d.name,
                "size_mb": round(size_bytes / (1024 ** 2), 1),
            })
        return {"sessions": sessions}

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()
        clients.append(ws)
        try:
            while True:
                # Push status every second
                gps_fix = profiler.get_gps.get_fix
                disk = shutil.disk_usage(str(config.storage.ssd_mount))

                euler = profiler.get_camera.get_imu.get_euler
                status = {
                    "recording": profiler.is_recording,
                    "duration_s": round(profiler.get_duration_s, 1),
                    "frame_count": profiler.get_frame_count,
                    "gps": {
                        "has_fix": gps_fix is not None,
                        "lat": gps_fix.lat if gps_fix else None,
                        "lon": gps_fix.lon if gps_fix else None,
                        "satellites": gps_fix.num_satellites if gps_fix else 0,
                    },
                    "imu": {
                        "active": euler is not None,
                        "roll": round(euler[0], 1) if euler else None,
                        "pitch": round(euler[1], 1) if euler else None,
                        "yaw": round(euler[2], 1) if euler else None,
                    },
                    "ssd": {
                        "free_gb": round(disk.free / (1024 ** 3), 1),
                    },
                }
                await ws.send_text(json.dumps(status))
                await asyncio.sleep(1)
        except WebSocketDisconnect:
            clients.remove(ws)

    # Serve static UI files
    web_dir = Path(__file__).parent.parent.parent / "web"
    if web_dir.exists():
        app.mount("/", StaticFiles(directory=str(web_dir), html=True), name="static")

    return app
