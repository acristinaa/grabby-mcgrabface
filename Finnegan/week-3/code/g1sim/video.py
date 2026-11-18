"""Offscreen-Video einer laufenden Sim - Ersatz fuer das MuJoCo-Fenster, das in ~/dev/g1-ext einfriert."""
import pathlib
import shutil
import subprocess

import mujoco


class VideoRecorder:
    """Kamera folgt einem Body, Bilder nach Sim-Zeit (30 pro Sim-Sekunde), geschrieben ueber ffmpeg.

    Die Sim laeuft auf dem Mac langsamer als Echtzeit - das Video spielt trotzdem in Echtzeit ab.
    """

    FPS = 30
    WIDTH, HEIGHT = 640, 480

    def __init__(self, model, data, path, track_body="pelvis", distance=3.0, azimuth=135.0, elevation=-15.0,
                 renderer=None):
        """renderer: reuse a mujoco.Renderer the script already owns (its size sets the video size; it is not closed here)."""
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg nicht gefunden - 'brew install ffmpeg'")
        self.data = data
        self.path = pathlib.Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.own_renderer = renderer is None
        self.renderer = renderer if renderer is not None else mujoco.Renderer(model, height=self.HEIGHT, width=self.WIDTH)
        width, height = self.renderer.width, self.renderer.height
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, track_body)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = distance, azimuth, elevation
        self.proc = subprocess.Popen(
            [ffmpeg, "-y", "-loglevel", "error",
             "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
             "-r", str(self.FPS), "-i", "-",
             "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(self.path)],
            stdin=subprocess.PIPE,
        )
        self.next_t = 0.0
        self.frames = 0

    def grab(self):
        """Ein Bild, sobald seit dem letzten Bild 1/30 s Sim-Zeit vergangen ist."""
        t = float(self.data.time)
        if t < self.next_t:
            return
        self.renderer.update_scene(self.data, camera=self.cam)
        self.proc.stdin.write(self.renderer.render().tobytes())
        self.frames += 1
        self.next_t = t + 1.0 / self.FPS

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()
        if self.own_renderer:
            self.renderer.close()
        print(f"Video: {self.path} ({self.frames} Bilder, {self.frames / self.FPS:.1f} s)")
