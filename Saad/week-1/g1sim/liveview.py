"""Live-Ansicht einer laufenden Sim im Browser (ab Lernpfad L1): Kamerabild + Status, nur zum Zuschauen.

Ersatz fuer das MuJoCo-Fenster, das auf diesem Mac einfriert. Gleiches Prinzip wie scripts/57_teleop.py: Der Hauptthread
rendert (das OpenGL des Macs verlangt einen Thread) und legt JPEG-Bilder ab, ein kleiner Webserver liefert sie als
MJPEG-Strom an den Browser. update() einmal je Schleifendurchlauf aufrufen - gerendert wird hoechstens FPS-mal je
Wanduhr-Sekunde, damit die Ansicht die Sim kaum bremst. Ziele lassen sich als Bodenmarken einzeichnen (markers), ein geplanter Weg als Punktspur (path).
Mit video=PFAD schreibt derselbe Renderer zusaetzlich ein mp4 (30 Bilder je Sim-Sekunde, Marken inklusive): nur EIN
OpenGL-Renderer im Prozess - zwei gleichzeitig liessen den Prozess am 2026-10-02 im Grafiktreiber haengen.
Ist der Port belegt (z. B. von einem haengenden alten Lauf), nimmt die Ansicht den naechsten freien bis port + 9.
Optional top view (map=...): a status key "_map" with bounds, table/obstacle outlines, planned path, walked trail, robot and
goal poses, drawn as SVG next to the camera image (scripts/68_walk_to_table.py). Status keys starting with "_" are not listed.
"""
import shutil
import subprocess
import http.server
import json
import math
import pathlib
import threading
import time

import cv2
import mujoco
import numpy as np

PAGE = """<!doctype html><html lang="de"><head><meta charset="utf-8"><title>G1 live</title>
<style>body{font:14px system-ui,sans-serif;margin:16px;background:#f5f6f7;color:#1d2d3e}
main{display:flex;gap:16px;flex-wrap:wrap}img{max-width:100%;border-radius:8px;background:#000}
table{border-collapse:collapse}td{padding:3px 10px 3px 0;vertical-align:top}td:first-child{color:#556b82}
h1{font-size:18px;margin:0 0 12px}#map{display:none;width:480px;height:480px;border-radius:8px;background:#fff}
.legend{font-size:12px;color:#556b82;margin-top:4px}</style></head><body><h1 id="t">G1 live</h1><main>
<img src="/stream.mjpg" width="640" height="480" alt="Live-Bild der Simulation">
<div id="mapbox" style="display:none"><svg id="map" role="img" aria-label="Top view"></svg><div class="legend">top view, 1 m grid ·
<b style="color:#8c6b4d">table</b> · <b style="color:#7a8794">obstacles</b> · <b style="color:#e76500">planned path, dock</b> ·
<b style="color:#0070f2">walked, robot</b></div></div><table id="s"></table></main>
<script>
const P=pts=>pts.map(([x,y])=>`${x},${-y}`).join(' ');
function arrow(p,c,r){const [x,y,a]=p,tx=x+r*Math.cos(a),ty=y+r*Math.sin(a);
return `<circle cx="${x}" cy="${-y}" r="${r*0.35}" fill="none" stroke="${c}" stroke-width="2" vector-effect="non-scaling-stroke"/>`+
`<line x1="${x}" y1="${-y}" x2="${tx}" y2="${-ty}" stroke="${c}" stroke-width="3" vector-effect="non-scaling-stroke"/>`}
function drawMap(m){const box=document.getElementById('mapbox'),svg=document.getElementById('map');
if(!m){box.style.display='none';return}box.style.display='block';svg.style.display='block';
const [x0,x1,y0,y1]=m.bounds;svg.setAttribute('viewBox',`${x0} ${-y1} ${x1-x0} ${y1-y0}`);
let h='';for(let x=Math.ceil(x0);x<=x1;x++)h+=`<line x1="${x}" y1="${-y1}" x2="${x}" y2="${-y0}" stroke="#e4e8ec" stroke-width="1" vector-effect="non-scaling-stroke"/>`;
for(let y=Math.ceil(y0);y<=y1;y++)h+=`<line x1="${x0}" y1="${-y}" x2="${x1}" y2="${-y}" stroke="#e4e8ec" stroke-width="1" vector-effect="non-scaling-stroke"/>`;
if(m.table)h+=`<polygon points="${P(m.table)}" fill="#8c6b4d"/>`;
(m.obstacles||[]).forEach(o=>h+=`<polygon points="${P(o)}" fill="#7a8794"/>`);
if(m.path&&m.path.length)h+=`<polyline points="${P(m.path)}" fill="none" stroke="#e76500" stroke-width="2" stroke-dasharray="4 3" vector-effect="non-scaling-stroke"/>`;
if(m.trail&&m.trail.length)h+=`<polyline points="${P(m.trail)}" fill="none" stroke="#0070f2" stroke-width="2" vector-effect="non-scaling-stroke"/>`;
if(m.pre)h+=arrow(m.pre,'#f0a060',0.25);if(m.dock)h+=arrow(m.dock,'#e76500',0.3);if(m.robot)h+=arrow(m.robot,'#0070f2',0.35);
svg.innerHTML=h}
async function poll(){try{const s=await (await fetch('/status')).json();
document.getElementById('t').textContent=s.titel||'G1 live';drawMap(s._map);
document.getElementById('s').innerHTML=Object.entries(s).filter(([k])=>k!=='titel'&&!k.startsWith('_'))
.map(([k,v])=>`<tr><td>${k}</td><td>${Array.isArray(v)?v.join('<br>'):v}</td></tr>`).join('');}catch(e){}
setTimeout(poll,500)}poll();</script></body></html>""".encode()


class LiveView:
    FPS = 10
    WIDTH, HEIGHT = 640, 480  # Laufmodell: offscreen-Framebuffer hoechstens 640x480

    VIDEO_FPS = 30

    def __init__(self, model, data, port=8020, video=None, serve=True, track_body="pelvis", distance=4.5, azimuth=135.0,
                 elevation=-35.0):
        self.data = data
        self.renderer = mujoco.Renderer(model, height=self.HEIGHT, width=self.WIDTH)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, track_body)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = distance, azimuth, elevation
        self.lock = threading.Condition()
        self.frame, self.frame_id, self.status = b"", 0, {}
        self.markers = []  # (x, y, yaw, rgba) je Bodenmarke
        self.path = []     # (x, y) Punkte des geplanten Wegs
        self.next_render = 0.0
        self.next_video = 0.0
        self.video, self.ffmpeg, self.video_frames = video, None, 0
        if video:
            ffmpeg = shutil.which("ffmpeg")
            if ffmpeg is None:
                raise RuntimeError("ffmpeg nicht gefunden - 'brew install ffmpeg'")
            pathlib.Path(video).parent.mkdir(parents=True, exist_ok=True)
            self.ffmpeg = subprocess.Popen(
                [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{self.WIDTH}x{self.HEIGHT}", "-r", str(self.VIDEO_FPS), "-i", "-",
                 "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(video)], stdin=subprocess.PIPE)
        self.server, self.url = None, None
        if serve:
            for p in range(port, port + 10):
                try:
                    self.server = http.server.ThreadingHTTPServer(("127.0.0.1", p), self._handler())
                    break
                except OSError:
                    print(f"Port {p} belegt - versuche {p + 1}")
            if self.server is None:
                raise OSError(f"Ports {port}-{port + 9} alle belegt")
            threading.Thread(target=self.server.serve_forever, daemon=True).start()
            self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
            print(f"Live-Ansicht: {self.url}")

    def update(self, status=None, markers=None, path=None):
        """Status, Marken und Weg uebernehmen; hoechstens FPS-mal je Sekunde ein neues Bild rendern."""
        with self.lock:
            if status is not None:
                self.status = status
            if markers is not None:
                self.markers = list(markers)
            if path is not None:
                self.path = list(path)[::max(1, len(path) // 80)]  # hoechstens ~80 Punkte zeichnen
        now, sim_t = time.monotonic(), float(self.data.time)
        want_live = self.server is not None and now >= self.next_render
        want_video = self.ffmpeg is not None and sim_t >= self.next_video
        if not (want_live or want_video):
            return
        self.renderer.update_scene(self.data, camera=self.cam)
        for x, y, yaw, rgba in self.markers:
            self._add_marker(x, y, yaw, rgba)
        for x, y in self.path:
            self._add_dot(x, y)
        rgb = self.renderer.render()
        if want_video:
            self.ffmpeg.stdin.write(rgb.tobytes())
            self.video_frames += 1
            self.next_video = sim_t + 1.0 / self.VIDEO_FPS
        if not want_live:
            return
        self.next_render = now + 1.0 / self.FPS
        ok, jpeg = cv2.imencode(".jpg", rgb[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.frame, self.frame_id = jpeg.tobytes(), self.frame_id + 1
                self.lock.notify_all()

    def _add_marker(self, x, y, yaw, rgba):
        """Flache Scheibe (Radius 10 cm = Zieltoleranz) und ein Pfeil in Soll-Blickrichtung, nur im Bild, ohne Physik."""
        scene = self.renderer.scene
        if scene.ngeom + 2 > scene.maxgeom:
            return
        rgba = np.asarray(rgba, dtype=np.float32)
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([0.10, 0.10, 0.003]),
                            np.array([x, y, 0.004]), np.eye(3).flatten(), rgba)
        scene.ngeom += 1
        tip = np.array([x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw), 0.01])
        # mjv_connector sets only type, size, pos and mat; everything else (material, texture, category ...) must come from
        # mjv_initGeom first. Without it the slot kept whatever memory was there: mjr_render crashed with a bus error in
        # scripts/68_walk_to_table.py (2026-10-08) - L1/L2 had just been lucky.
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                            np.eye(3).flatten(), rgba)
        mujoco.mjv_connector(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, 0.02, np.array([x, y, 0.01]), tip)
        scene.ngeom += 1

    def _add_dot(self, x, y):
        """Kleiner blauer Punkt auf dem Boden, nur im Bild."""
        scene = self.renderer.scene
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]),
                            np.array([x, y, 0.02]), np.eye(3).flatten(), np.array([0.0, 0.44, 0.95, 0.9], dtype=np.float32))
        scene.ngeom += 1

    def close(self):
        if self.server is not None:
            self.server.shutdown()
        if self.ffmpeg is not None:
            self.ffmpeg.stdin.close()
            self.ffmpeg.wait()
            print(f"Video: {self.video} ({self.video_frames} Bilder, {self.video_frames / self.VIDEO_FPS:.1f} s)")
        self.renderer.close()

    def _handler(self):
        view = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    self._send(PAGE, "text/html; charset=utf-8")
                elif self.path == "/status":
                    with view.lock:
                        body = json.dumps(view.status, ensure_ascii=False).encode()
                    self._send(body, "application/json; charset=utf-8")
                elif self.path.startswith("/stream.mjpg"):
                    self._stream()
                else:
                    self.send_error(404)

            def _send(self, body, content_type):
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # the browser left (reload, tab closed) - nothing to answer

            def _stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = -1
                try:
                    while True:
                        with view.lock:
                            view.lock.wait_for(lambda: view.frame_id != last, 2.0)
                            frame, last = view.frame, view.frame_id
                        if frame:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                             % len(frame) + frame + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        return Handler
