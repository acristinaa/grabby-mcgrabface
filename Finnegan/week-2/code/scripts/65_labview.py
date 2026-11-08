"""Lab view: watch sim runs live and compare results in the browser (Live | History | Training).

Works in either environment, imports no MuJoCo (it only reads files, so it cannot hang in the graphics driver):
    .venv/bin/python scripts/65_labview.py            -> open http://127.0.0.1:8030 (next free port up to 8039)
Then start a run in a second terminal, e.g. slowed down to real time so the phases can be followed:
    ~/dev/g1-ext/.venv/bin/python scripts/54_grasp_scripted.py --randomize --trials 5 --watch 1
The runs write their progress to runs/labview/ (g1sim/labview.py); this server only shows those files:
  Live     every running run: live image (MJPEG from runs/labview/live/<run_id>.jpg), trial, phase, success so far
  History  runs/labview/records/*.json plus the older 54 reports runs/*.json, with a success-rate chart; runs with a replay
           video (record field "video", e.g. 67 --watch) get a play button: replay, loop and speed in the browser
  Training every VLA training log in runs/ (lerobot-train output, e.g. runs/l1w_s7_run1.log from scripts/train_smolvla.sh):
           progress, time left, loss curve, learning rate and the saved checkpoints; refreshed every 5 s
Binds 127.0.0.1 only. Stop with Ctrl+C.
"""
import argparse
import datetime
import http.server
import json
import os
import pathlib
import re
import sys
import time
import urllib.parse

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim.labview import REACH_REASONS, REASON_SHORT, ROOT, reason_label  # noqa: E402

PORT = 8030
RUNS = REPO / "runs"
STALE_S = 30.0  # no status update for this long and the process is gone -> "stopped"
STREAM_FPS = 10
RUN_ID = re.compile(r"/live/([A-Za-z0-9_-]+)\.mjpg")
VIDEO_ID = re.compile(r"/video/([A-Za-z0-9_-]+)\.mp4")
VIDEO_DIRS = (REPO / "runs" / "labview" / "videos", REPO / "docs" / "videos")  # the only places a video is served from
CHUNK = 1 << 20
START_HINT = "~/dev/g1-ext/.venv/bin/python scripts/54_grasp_scripted.py --randomize --trials 5 --watch 1"
# lerobot-train log lines: "step:1K smpl:8K ep:11 epch:0.04 loss:1.149 grdn:4.3 lr:2.0e-05 updt_s:0.880 data_s:0.005" - step and
# samples are rounded ("1K"), so the exact step comes from the progress bar ("1234/45000 [") and each loss line sits log_freq after
# the previous one.
TRAIN_LINE = re.compile(r"step:\S+ smpl:\S+ ep:\S+ epch:([\d.]+) loss:([\d.]+) grdn:([\d.]+) lr:([\d.e+-]+) "
                        r"updt_s:([\d.]+) data_s:([\d.]+)")
TRAIN_BAR = re.compile(r"(\d+)/(\d+) \[")
TRAIN_RUNNING_S = 120.0  # log written within this many seconds -> "running"
TRAIN_POINTS = 400  # loss curve points sent to the page


def pid_alive(pid):
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except (TypeError, ValueError, OSError):
        return False
    return True


def live_runs(live_dir):
    """All status files, newest first. A run whose status is old and whose process is gone is marked "stopped"."""
    now, runs = time.time(), []
    for path in pathlib.Path(live_dir).glob("*.status.json"):
        try:
            status = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        status["silent_s"] = round(now - status.get("updated_at", 0), 1)
        if status.get("state") == "running" and status["silent_s"] > STALE_S and not pid_alive(status.get("pid")):
            status["state"] = "stopped"
        status["has_frame"] = (path.parent / f"{status.get('run_id')}.jpg").exists()
        status["has_video"] = video_file(path.parent.parent / "records", status.get("run_id")) is not None
        runs.append(status)
    return sorted(runs, key=lambda s: s.get("updated_at", 0), reverse=True)


def video_file(records_dir, run_id):
    """Replay video of a run (from its record), or None. Only files inside VIDEO_DIRS are served."""
    try:
        video = json.loads((pathlib.Path(records_dir) / f"{run_id}.json").read_text()).get("video")
    except (OSError, ValueError, TypeError):
        return None
    if not video:
        return None
    path = (REPO / video).resolve()
    return path if path.is_file() and any(path.is_relative_to(d.resolve()) for d in VIDEO_DIRS) else None


def reasons(failures):
    return [{"reason": k, "label": reason_label(k), "short": REASON_SHORT.get(k, k), "count": n}
            for k, n in sorted(failures.items(), key=lambda kv: -kv[1])]


def rate(n, d):
    return n / d if d else None


def record_row(record, path):
    counts = record.get("counts", {})
    trials, passed, reachable = counts.get("attempted", 0), counts.get("passed", 0), counts.get("reachable", 0)
    return {"id": record.get("run_id", path.stem), "date": record.get("started_at"), "script": record.get("script"),
            "task": record.get("task"), "object": record.get("object"), "trials": trials, "passed": passed,
            "reachable": reachable, "success_rate": rate(passed, trials), "reachable_success_rate": rate(passed, reachable),
            "reasons": reasons(record.get("failures", {})), "settings": " ".join(record.get("argv", [])) or "defaults",
            "source": str(path.name), "video": bool(record.get("video"))}


def old_report_row(path):
    """Row for an older scripts/54 report (runs/*.json), or None if the file has another shape."""
    try:
        report = json.loads(pathlib.Path(path).read_text())
    except (OSError, ValueError):
        return None
    if not (isinstance(report, dict) and isinstance(report.get("trials"), int) and "passed_trials" in report
            and isinstance(report.get("failures_by_first_failed_check"), dict)
            and isinstance(report.get("results"), list)
            and all(isinstance(r, dict) and "checks" in r and "passed" in r for r in report["results"])):
        return None
    failures = report["failures_by_first_failed_check"]
    trials, passed = report["trials"], report["passed_trials"]
    reachable = trials - sum(failures.get(k, 0) for k in REACH_REASONS)
    mtime = datetime.datetime.fromtimestamp(pathlib.Path(path).stat().st_mtime).astimezone()
    if report.get("randomize"):
        settings = "randomized: " + (", ".join(report.get("varied_groups") or []) or "nothing")
    else:
        settings = "nominal start"
    return {"id": pathlib.Path(path).stem, "date": mtime.isoformat(timespec="seconds"), "script": "54",
            "task": "grasp_lift", "object": "bottle", "trials": trials, "passed": passed, "reachable": reachable,
            "success_rate": rate(passed, trials), "reachable_success_rate": rate(passed, reachable),
            "reasons": reasons(failures), "settings": f"{settings}, seed {report.get('seed', '?')}",
            "source": pathlib.Path(path).name, "video": False}


def history(records_dir, runs_dir):
    """Run records plus older 54 reports in one row shape, newest first. Reports a record points to are not listed twice."""
    rows, covered = [], set()
    for path in pathlib.Path(records_dir).glob("*.json"):
        try:
            record = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if record.get("report"):
            covered.add((REPO / record["report"]).resolve())
        rows.append(record_row(record, path))
    for path in pathlib.Path(runs_dir).glob("*.json"):
        if path.resolve() not in covered and (row := old_report_row(path)) is not None:
            rows.append(row)
    return sorted(rows, key=lambda r: _when(r["date"]), reverse=True)


def _when(date):
    """Sort key for ISO dates with time zone (string order breaks when the offset changes, e.g. summer -> winter time)."""
    try:
        return datetime.datetime.fromisoformat(date).timestamp()
    except (TypeError, ValueError):
        return 0.0


def training_runs(runs_dir):
    """Every lerobot-train log in runs_dir (newest first): config, exact progress, loss curve, speed, time left, checkpoints."""
    out = []
    for path in sorted(pathlib.Path(runs_dir).glob("*.log"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            with open(path, "rb") as f:
                if b"ot_train.py" not in f.read(65536):
                    continue
            text = path.read_text(errors="replace")
        except OSError:
            continue

        def conf(key, default=None):
            m = re.search(rf"'{key}': ([^,\n]+)", text)
            return m[1].strip("'\" ") if m else default

        total, log_freq = int(conf("steps", 0) or 0), int(conf("log_freq", 1) or 1)
        bars = TRAIN_BAR.findall(text)
        step = int(bars[-1][0]) if bars else 0
        rows = [tuple(float(v) for v in m) for m in TRAIN_LINE.findall(text)]
        last = step // log_freq * log_freq
        curve = [[last - (len(rows) - 1 - i) * log_freq, r[1]] for i, r in enumerate(rows)]
        if len(curve) > TRAIN_POINTS:
            k = -(-len(curve) // TRAIN_POINTS)
            curve = [[curve[i + k - 1][0] if i + k - 1 < len(curve) else curve[-1][0],
                      sum(c[1] for c in curve[i:i + k]) / len(curve[i:i + k])] for i in range(0, len(curve), k)]
        recent = rows[-10:]
        s_per_step = sum(r[4] + r[5] for r in recent) / len(recent) if recent else None
        age = time.time() - path.stat().st_mtime
        state = ("finished" if "End of training" in text else "running" if age < TRAIN_RUNNING_S else "stopped")
        if "Traceback" in text and state != "finished":
            state = "crashed" if age >= TRAIN_RUNNING_S else state
        out_dir = conf("output_dir")
        checkpoints = []
        if out_dir and pathlib.Path(out_dir, "checkpoints").is_dir():
            checkpoints = sorted(int(c.name) for c in pathlib.Path(out_dir, "checkpoints").iterdir() if c.name.isdigit())
        out.append({
            "log": path.name, "job": conf("job_name") or path.stem, "state": state, "silent_s": round(age),
            "step": step, "steps": total, "log_freq": log_freq, "batch_size": conf("batch_size"),
            "dataset": conf("root"), "output_dir": out_dir, "device": conf("device"),
            "loss": rows[-1][1] if rows else None, "loss_min": min((r[1] for r in rows), default=None),
            "lr": rows[-1][3] if rows else None, "epochs": rows[-1][0] if rows else None,
            "s_per_step": s_per_step, "eta_s": (total - step) * s_per_step if s_per_step and state == "running" else None,
            "checkpoints": checkpoints, "curve": curve,
        })
    return out


def make_server(port=PORT, root=ROOT, runs_dir=RUNS, tries=10):
    """ThreadingHTTPServer on 127.0.0.1, first free port from port to port + tries - 1 (port 0 = any free port)."""
    live_dir, records_dir = pathlib.Path(root) / "live", pathlib.Path(root) / "records"

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            path = urllib.parse.urlsplit(self.path).path
            if path == "/":
                self._send(PAGE, "text/html; charset=utf-8")
            elif path == "/api/live":
                self._json({"now": time.time(), "runs": live_runs(live_dir)})
            elif path == "/api/runs":
                self._json(history(records_dir, runs_dir))
            elif path == "/api/training":
                self._json({"now": time.time(), "runs": training_runs(runs_dir)})
            elif path.startswith("/live/"):
                match = RUN_ID.fullmatch(path)
                if match is None:
                    self.send_error(400, "invalid run id")
                elif not (live_dir / f"{match[1]}.jpg").exists():
                    self.send_error(404, "no live image for this run")
                else:
                    self._stream(live_dir / f"{match[1]}.jpg")
            elif path.startswith("/video/"):
                match = VIDEO_ID.fullmatch(path)
                video = video_file(records_dir, match[1]) if match else None
                if video is None:
                    self.send_error(404, "no replay video for this run")
                else:
                    self._file(video, "video/mp4")
            else:
                self.send_error(404)

        def _json(self, data):
            self._send(json.dumps(data, ensure_ascii=False).encode(), "application/json; charset=utf-8")

        def _send(self, body, content_type):
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _file(self, file_path, content_type):
            """A file with HTTP range support: browsers (Safari always) ask for byte ranges to play and seek a video."""
            size = file_path.stat().st_size
            start, end = 0, size - 1
            match = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range", ""))
            if match and (match[1] or match[2]):
                if match[1]:
                    start, end = int(match[1]), min(int(match[2]) if match[2] else size - 1, size - 1)
                else:  # suffix range: the last n bytes
                    start = max(0, size - int(match[2]))
                if start > end:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
                self.send_response(206)
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            else:
                self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(end - start + 1))
            self.end_headers()
            try:
                with open(file_path, "rb") as f:
                    f.seek(start)
                    left = end - start + 1
                    while left > 0 and (chunk := f.read(min(CHUNK, left))):
                        self.wfile.write(chunk)
                        left -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _stream(self, frame_path):
            """MJPEG: send the image whenever the file changes (at most STREAM_FPS per second); every 5 s anyway, so a
            closed browser tab is noticed and this thread ends."""
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last, sent_at = None, 0.0
            try:
                while True:
                    try:
                        mtime = frame_path.stat().st_mtime_ns
                        if mtime != last or time.monotonic() - sent_at > 5.0:
                            frame = frame_path.read_bytes()
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                             % len(frame) + frame + b"\r\n")
                            last, sent_at = mtime, time.monotonic()
                            time.sleep(1.0 / STREAM_FPS)
                            continue
                    except FileNotFoundError:
                        pass
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

    for p in range(port, port + tries):
        try:
            server = http.server.ThreadingHTTPServer(("127.0.0.1", p), Handler)
            server.daemon_threads = True
            return server
        except OSError:
            print(f"Port {p} in use - trying {p + 1}")
    raise OSError(f"Ports {port}-{port + tries - 1} all in use")


PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8"><title>G1 lab view</title>
<meta name="viewport" content="width=device-width, initial-scale=1"><meta name="color-scheme" content="light dark">
<style>
:root{--bg:#f5f6f7;--card:#fff;--fg:#1d2d3e;--muted:#5b738b;--line:#d9dee3;--accent:#0a6ed1;--ok:#188918;--bad:#bb0000;
--warn:#c35500;--track:#e5e9ed}
@media (prefers-color-scheme:dark){:root{--bg:#12171c;--card:#1b232b;--fg:#e8edf2;--muted:#9aabbc;--line:#2e3a46;
--accent:#4db1ff;--ok:#5dc122;--bad:#ff6b6b;--warn:#ff9a3c;--track:#2a3540}}
*{box-sizing:border-box}body{font:14px/1.45 system-ui,-apple-system,sans-serif;margin:0;background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:24px;padding:12px 24px;border-bottom:1px solid var(--line);background:var(--card)}
h1{font-size:17px;margin:0;font-weight:600}nav{display:flex;gap:4px}
nav button{font:inherit;border:0;background:none;color:var(--muted);padding:6px 14px;border-radius:6px;cursor:pointer}
nav button.on{background:var(--accent);color:#fff}main{padding:20px 24px;max-width:1280px}
section{display:none}section.on{display:block}
.card{display:flex;gap:20px;flex-wrap:wrap;background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px;
margin-bottom:16px}.media{width:640px;max-width:100%;aspect-ratio:16/10;background:#000;border-radius:8px;overflow:hidden;
display:flex;align-items:center;justify-content:center;color:#aaa}.media img{width:100%;height:100%;object-fit:contain}
.info{flex:1;min-width:300px}.info h2{font-size:16px;margin:0 0 2px}.meta{color:var(--muted);font-size:12px;margin-bottom:12px}
.badge{font-size:11px;font-weight:600;padding:2px 8px;border-radius:10px;margin-left:8px;vertical-align:middle;color:#fff}
.running{background:var(--ok)}.stopped{background:var(--bad)}.finished{background:var(--muted)}
.row{display:flex;justify-content:space-between;gap:12px;padding:5px 0;border-bottom:1px solid var(--line)}
.row span{color:var(--muted)}.row b{font-weight:600;text-align:right}
.bar{height:8px;background:var(--track);border-radius:4px;margin:4px 0 8px;overflow:hidden}
.bar i{display:block;height:100%;background:var(--accent);width:0;transition:width .3s}
.strip{list-style:none;padding:0;margin:6px 0 10px;display:flex;flex-wrap:wrap;gap:4px}
.strip li{font-size:12px;padding:2px 8px;border-radius:10px;background:var(--track);color:var(--muted)}
.strip li.done{color:var(--ok)}.strip li.cur{background:var(--accent);color:#fff;font-weight:600}
.warn{color:var(--warn);font-size:12px}.empty{background:var(--card);border:1px dashed var(--line);border-radius:10px;
padding:28px;color:var(--muted)}code{background:var(--track);padding:2px 6px;border-radius:4px;font-size:12px;color:var(--fg)}
table{border-collapse:collapse;width:100%;background:var(--card);border:1px solid var(--line);border-radius:10px}
th,td{text-align:left;padding:7px 10px;border-bottom:1px solid var(--line);vertical-align:top}
th{color:var(--muted);font-weight:600;font-size:12px}td.n{text-align:right;white-space:nowrap}
.chart{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 16px;margin:20px 0 8px}
.chart h3{font-size:14px;margin:0 0 6px}.note{color:var(--muted);font-size:12px;margin:4px 0 16px}
svg text{fill:var(--muted);font-size:11px}.legend{font-size:12px;color:var(--muted)}
.play{font:inherit;font-size:12px;border:1px solid var(--accent);color:var(--accent);background:none;border-radius:6px;
padding:3px 10px;cursor:pointer;white-space:nowrap}.play:hover{background:var(--accent);color:#fff}
#player{display:none;position:fixed;inset:0;background:rgba(0,0,0,.6);align-items:center;justify-content:center;z-index:9;
padding:16px}#player.on{display:flex}#player .box{background:var(--card);border-radius:10px;padding:14px;width:min(900px,100%)}
#player video{width:100%;border-radius:8px;background:#000;display:block}
#player .ctl{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-top:10px;color:var(--muted);font-size:13px}
#player .ctl button,#player select{font:inherit;border:1px solid var(--line);background:var(--card);color:var(--fg);
border-radius:6px;padding:4px 10px;cursor:pointer}#player .ttl{font-weight:600;color:var(--fg);flex:1;min-width:200px}
</style></head><body>
<header><h1>G1 lab view</h1><nav><button data-tab="live">Live</button><button data-tab="history">History</button>
<button data-tab="training">Training</button></nav></header>
<main>
<section id="live"><div id="cards"></div><div id="nolive" class="empty">No run is active. Start one with:<br><br>
<code>START_HINT</code></div></section>
<section id="history"><table><thead><tr><th>Date</th><th>Script</th><th>Object</th><th>Trials</th><th>Success</th>
<th>Reachable success</th><th>Main failure reasons</th><th>Video</th></tr></thead><tbody id="rows"></tbody></table><div id="charts"></div></section>
<section id="training"><div id="tcards"></div><div id="notrain" class="empty">No training log in runs/ yet. Start one with:<br><br>
<code>caffeinate -i scripts/train_smolvla.sh run1 &gt; runs/l1w_s7_run1.log 2&gt;&amp;1</code><br><br>No robot moves during training;
evaluation runs show up in Live.</div></section>
</main>
<div id="player" role="dialog" aria-label="Replay video"><div class="box"><video id="vid" controls playsinline></video>
<div class="ctl"><span class="ttl" id="vtitle"></span><button id="vreplay">↺ Replay</button>
<label><input type="checkbox" id="vloop" checked> Loop</label>
<label>Speed <select id="vspeed"><option>0.25</option><option>0.5</option><option selected>1</option><option>2</option></select></label>
<button id="vclose">Close</button></div></div></div>
<script>
const $=(s,el=document)=>el.querySelector(s);
const pct=(n,d)=>d?`${n} / ${d} (${Math.round(100*n/d)}%)`:'–';
function el(tag,attrs={},text){const e=document.createElement(tag);Object.assign(e,attrs);if(text!==undefined)e.textContent=text;return e}
function show(tab){document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('on',b.dataset.tab===tab));
document.querySelectorAll('section').forEach(s=>s.classList.toggle('on',s.id===tab));location.hash=tab;
if(tab==='history')loadHistory();if(tab==='training')pollTraining()}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>show(b.dataset.tab));

// ---- Replay video player (History and finished live cards)
const vid=$('#vid');
function playVideo(id,title){vid.src=`/video/${id}.mp4`;vid.loop=$('#vloop').checked;$('#vtitle').textContent=title;
$('#player').classList.add('on');vid.onloadedmetadata=()=>{vid.playbackRate=+$('#vspeed').value;vid.play()}}
function closeVideo(){vid.pause();vid.removeAttribute('src');vid.load();$('#player').classList.remove('on')}
$('#vreplay').onclick=()=>{vid.currentTime=0;vid.play()};$('#vloop').onchange=e=>vid.loop=e.target.checked;
$('#vspeed').onchange=e=>vid.playbackRate=+e.target.value;$('#vclose').onclick=closeVideo;
$('#player').onclick=e=>{if(e.target.id==='player')closeVideo()};
document.addEventListener('keydown',e=>{if(e.key==='Escape')closeVideo()});
const playButton=(id,title)=>{const b=el('button',{className:'play',type:'button'},'▶ Video');b.onclick=()=>playVideo(id,title);return b};

// ---- Live: one card per run, built once (so the MJPEG image is not restarted), text updated every poll
function card(s){const c=el('article',{className:'card',id:'run-'+s.run_id});
c.innerHTML=`<div class="media"><span>No live image for this run</span></div><div class="info"><h2></h2><div class="meta"></div>
<div class="row"><span>Trial</span><b class="trial"></b></div><div class="bar"><i></i></div>
<div class="row"><span>Phase</span><b class="phase"></b></div><ol class="strip"></ol>
<div class="row"><span>Success so far</span><b class="succ"></b></div>
<div class="row"><span>… of reachable starts</span><b class="reach"></b></div>
<div class="row"><span>Last failure</span><b class="lastfail"></b></div><div class="warn"></div><div class="vid"></div></div>`;return c}
function update(c,s){const n=s.trials_planned,k=s.counts||{};
const h=$('h2',c);h.textContent=`${s.script} · ${s.task} · ${s.object}`;h.append(el('span',{className:'badge '+s.state},s.state));
const started=s.started_at?s.started_at.slice(11,16):'';
$('.meta',c).textContent=`started ${started} · `+(s.watch?`watch ${s.watch}x real time`:'full speed (use --watch 1 to follow)')+
` · sim ${s.sim_t??0} s`+(s.git?` · git ${s.git.commit}${s.git.dirty?' (changed)':''}`:'');
const cur=s.state==='running'&&s.trial!==null?Math.min(s.trial+1,n):k.done;
$('.trial',c).textContent=`${cur} / ${n}`;$('.bar i',c).style.width=(n?100*k.done/n:0)+'%';
const order=s.phase_order||[];
$('.phase',c).textContent=s.phase?(s.phase_index!==null?`${s.phase_index+1} of ${order.length}: `:'')+s.phase_label:'–';
const strip=$('.strip',c);strip.replaceChildren(...order.map((p,i)=>{const done=s.state!=='running'||i<s.phase_index;
const now=s.state==='running'&&i===s.phase_index;
return el('li',{className:now?'cur':done?'done':'',title:p.key},(now?'► ':done?'✓ ':'· ')+p.label)}));
$('.succ',c).textContent=pct(k.passed,k.done);$('.reach',c).textContent=pct(k.passed,k.reachable);
const lf=s.last_failure;$('.lastfail',c).textContent=lf?`trial ${lf.trial+1}: ${lf.label}`:'none';
$('.warn',c).textContent=s.state==='stopped'?'This run stopped without finishing (process gone).':
s.state==='running'&&s.silent_s>10?`No update for ${Math.round(s.silent_s)} s.`:'';
const vbox=$('.vid',c);if(s.state==='finished'&&s.has_video&&!vbox.firstChild){vbox.style.marginTop='10px';
vbox.append(playButton(s.run_id,`${s.script} · ${s.object} · trial 1`))}
const media=$('.media',c);if(s.has_frame&&!$('img',media)){
media.replaceChildren(el('img',{src:`/live/${s.run_id}.mjpg`,alt:'Live image of the simulation'}))}}
async function pollLive(){try{const d=await (await fetch('/api/live')).json();
// show running runs, stopped runs of the last 10 min and finished runs of the last 5 min
const runs=d.runs.filter(s=>s.state==='running'||(s.state==='stopped'&&s.silent_s<600)||(s.state==='finished'&&s.silent_s<300));
const box=$('#cards'),keep=new Set(runs.map(s=>'run-'+s.run_id));
[...box.children].forEach(c=>{if(!keep.has(c.id))c.remove()});
runs.forEach(s=>{let c=document.getElementById('run-'+s.run_id);if(!c){c=card(s);box.append(c)}update(c,s)});
$('#nolive').style.display=runs.length?'none':'block'}catch(e){}setTimeout(pollLive,500)}

// ---- History: table + success rate over time, one chart per task
async function loadHistory(){const rows=await (await fetch('/api/runs')).json();
$('#rows').replaceChildren(...rows.map(r=>{const tr=el('tr',{title:`${r.source} · ${r.settings}`});
const reasons=r.reasons.map(x=>`${x.short} ${x.count}`).join(' · ')||'–';
const td=(t,cls='')=>el('td',{className:cls},t);
const rs=td(reasons);rs.title=r.reasons.map(x=>`${x.count}× ${x.label}`).join('\\n');
tr.append(td(r.date?r.date.slice(0,16).replace('T',' '):'?'),td(r.script),td(r.object),td(String(r.trials),'n'),
td(pct(r.passed,r.trials),'n'),td(pct(r.passed,r.reachable),'n'),rs);
const vt=td('');if(r.video)vt.append(playButton(r.id,`${(r.date||'').slice(0,16).replace('T',' ')} · ${r.script} · ${r.object} · trial 1`));
tr.append(vt);return tr}));
const byTask={};rows.forEach(r=>(byTask[r.task]=byTask[r.task]||[]).push(r));
$('#charts').replaceChildren(...Object.entries(byTask).map(([task,rs])=>chart(task,rs.slice().reverse())))}
function chart(task,rs){const W=760,H=230,L=44,R=16,T=12,B=34,ns='http://www.w3.org/2000/svg';
const box=el('div',{className:'chart'});box.append(el('h3',{},`Success rate over time · task ${task} · ${rs.length} runs`));
const svg=document.createElementNS(ns,'svg');svg.setAttribute('viewBox',`0 0 ${W} ${H}`);svg.setAttribute('width','100%');
const add=(tag,a,text)=>{const e=document.createElementNS(ns,tag);for(const k in a)e.setAttribute(k,a[k]);
if(text!==undefined)e.textContent=text;svg.append(e);return e};
const x=i=>rs.length>1?L+i*(W-L-R)/(rs.length-1):(L+W-R)/2,y=v=>T+(1-v)*(H-T-B);
[0,.5,1].forEach(v=>{add('line',{x1:L,x2:W-R,y1:y(v),y2:y(v),stroke:'currentColor','stroke-opacity':.15});
add('text',{x:L-8,y:y(v)+4,'text-anchor':'end'},Math.round(v*100)+'%')});
const step=Math.max(1,Math.ceil(rs.length/8));
rs.forEach((r,i)=>{if(i%step===0||i===rs.length-1)add('text',{x:x(i),y:H-12,'text-anchor':'middle'},(r.date||'').slice(5,10))});
[['reachable_success_rate','var(--accent)',true],['success_rate','var(--muted)',false]].forEach(([key,col,fill])=>{
const pts=rs.map((r,i)=>[x(i),r[key]]).filter(p=>p[1]!==null);
add('polyline',{points:pts.map(([px,v])=>`${px},${y(v)}`).join(' '),fill:'none',stroke:col,'stroke-width':2});
rs.forEach((r,i)=>{if(r[key]===null)return;const c=add('circle',{cx:x(i),cy:y(r[key]),r:4,stroke:col,'stroke-width':2,
fill:fill?col:'var(--card)'});const t=document.createElementNS(ns,'title');
t.textContent=`${(r.date||'').slice(0,16).replace('T',' ')} · ${r.source}\\n${key==='success_rate'?'all starts':'reachable starts'}: `+
`${Math.round(100*r[key])}% · ${r.settings}`;c.append(t)})});
box.append(svg);box.append(el('div',{className:'legend'},'● reachable starts (filled)   ○ all starts (hollow)'));
box.append(el('div',{className:'note'},'Each point is one run, oldest on the left. Older runs may use different settings '+
'(nominal start or randomized starts, other randomization groups, other code), so compare like with like: hover a point '+
'for its settings. "Reachable starts" leaves out starts the arm cannot reach, which are a start problem, not a grasp error.'));
return box}
// ---- Training: one card per lerobot-train log, rebuilt every 5 s while the tab is open
const hms=s=>{s=Math.round(s);const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?`${h} h ${m} min`:`${m} min`};
function lossChart(curve,total){const W=760,H=240,L=48,R=16,T=12,B=30,ns='http://www.w3.org/2000/svg';
const svg=document.createElementNS(ns,'svg');svg.setAttribute('viewBox',`0 0 ${W} ${H}`);svg.setAttribute('width','100%');
const add=(tag,a,text)=>{const e=document.createElementNS(ns,tag);for(const k in a)e.setAttribute(k,a[k]);
if(text!==undefined)e.textContent=text;svg.append(e);return e};
if(!curve.length){add('text',{x:L,y:H/2},'no loss values yet');return svg}
const maxL=Math.max(...curve.map(c=>c[1])),xs=s=>L+s/(total||curve[curve.length-1][0]||1)*(W-L-R),ys=v=>T+(1-v/maxL)*(H-T-B);
[0,.5,1].forEach(f=>{add('line',{x1:L,x2:W-R,y1:ys(f*maxL),y2:ys(f*maxL),stroke:'currentColor','stroke-opacity':.15});
add('text',{x:L-8,y:ys(f*maxL)+4,'text-anchor':'end'},(f*maxL).toFixed(2))});
[0,.25,.5,.75,1].forEach(f=>add('text',{x:xs(f*total),y:H-8,'text-anchor':'middle'},Math.round(f*total/1000)+'k'));
add('polyline',{points:curve.map(([s,v])=>`${xs(s)},${ys(v)}`).join(' '),fill:'none',stroke:'var(--accent)','stroke-width':2});
return svg}
function trainCard(r){const c=el('article',{className:'card'});const info=el('div',{className:'info'});
const h=el('h2',{},r.job);h.append(el('span',{className:'badge '+(r.state==='running'?'running':r.state==='finished'?'finished':'stopped')},r.state));
info.append(h,el('div',{className:'meta'},`${r.log} · batch ${r.batch_size} · ${r.device} · ${r.dataset||''}`));
const row=(k,v)=>{const d=el('div',{className:'row'});d.append(el('span',{},k),el('b',{},v));info.append(d)};
row('Step',`${r.step.toLocaleString()} / ${r.steps.toLocaleString()} (${r.steps?Math.round(100*r.step/r.steps):0}%)`);
const bar=el('div',{className:'bar'});const fill=el('i');fill.style.width=(r.steps?100*r.step/r.steps:0)+'%';bar.append(fill);info.append(bar);
row('Loss now / lowest',r.loss===null?'–':`${r.loss.toFixed(3)} / ${r.loss_min.toFixed(3)}`);
row('Learning rate',r.lr===null?'–':r.lr.toExponential(1));
row('Passes through the data',r.epochs===null?'–':r.epochs.toFixed(2));
row('Speed',r.s_per_step?`${r.s_per_step.toFixed(2)} s per step`:'–');
row('Time left',r.eta_s?`${hms(r.eta_s)} (done ~${new Date(Date.now()+r.eta_s*1000).toTimeString().slice(0,5)})`:
r.state==='running'?'–':r.state);
row('Checkpoints saved',r.checkpoints.length?r.checkpoints.map(s=>(s/1000)+'k').join(', '):'none yet');
if(r.state!=='running'&&r.state!=='finished')info.append(el('div',{className:'warn'},
`Log not written for ${hms(r.silent_s)}. Continue with: scripts/train_smolvla.sh resume ${r.job.replace('g1_volleyball_','')}`));
const ch=el('div',{className:'chart'});ch.style.flexBasis='100%';ch.style.margin='0';
ch.append(el('h3',{},'Loss over training steps (lower = closer to the teacher)'),lossChart(r.curve,r.steps));
c.append(info,ch);return c}
async function pollTraining(){if(!$('#training').classList.contains('on'))return;
try{const d=await (await fetch('/api/training')).json();$('#tcards').replaceChildren(...d.runs.map(trainCard));
$('#notrain').style.display=d.runs.length?'none':'block'}catch(e){}setTimeout(pollTraining,5000)}
show(['live','history','training'].includes(location.hash.slice(1))?location.hash.slice(1):'live');pollLive();
</script></body></html>""".replace("START_HINT", START_HINT).encode()


def main():
    parser = argparse.ArgumentParser(description="Lab view: watch sim runs and compare results in the browser")
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()
    server = make_server(args.port)
    print(f"Lab view: http://127.0.0.1:{server.server_address[1]}  (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
