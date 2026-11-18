"""FastAPI-Server: liefert die Seite und streamt Bild + Zustand ueber WebSocket.

Protokoll:
  Server -> Browser   Binaerframe = JPEG, Textframe = JSON (Zustand/Metadaten)
  Browser -> Server   Textframe   = JSON-Kommando (siehe SimSession._apply)
"""
import asyncio
import contextlib
import json
import pathlib

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .sim import SimSession

STATIC = pathlib.Path(__file__).resolve().parent / "static"
STREAM_HZ = 30.0

sim = SimSession()


@contextlib.asynccontextmanager
async def lifespan(app):
    sim.start()
    yield
    sim.stop()


app = FastAPI(title="G1 Steuerstand", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


@app.get("/")
async def index():
    return FileResponse(str(STATIC / "index.html"))


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    await sock.send_text(json.dumps({"type": "meta", "data": sim.meta()}))

    async def pump():
        """Bild und Zustand mit fester Rate zum Browser."""
        last = None
        while True:
            frame = sim.frame()
            if frame is not None and frame is not last:
                await sock.send_bytes(frame)
                await sock.send_text(json.dumps({"type": "state", "data": sim.state()}))
                last = frame
            await asyncio.sleep(1.0 / STREAM_HZ)

    task = asyncio.create_task(pump())
    try:
        while True:
            msg = await sock.receive_text()
            try:
                cmd = json.loads(msg)
            except json.JSONDecodeError:
                continue
            sim.send(cmd)
            if cmd.get("type") == "mount":
                # Neuaufbau dauert ~180 ms; danach frische Metadaten schicken
                await asyncio.sleep(0.35)
                await sock.send_text(json.dumps({"type": "meta", "data": sim.meta()}))
    except WebSocketDisconnect:
        pass
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
