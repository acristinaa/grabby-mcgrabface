"""Stufe 2.0 - Schleife Sim <-> Policy-Server ueber openpis Websocket-Protokoll: eine Episode.

Startet den Ersatz-Server (openpi_g1/stand_in.py) in der openpi-Umgebung und verbindet die Greifszene als openpi-Client,
genau wie openpi examples/aloha_sim: Runtime + ActionChunkBroker + WebsocketClientPolicy. Eine Episode, Standard 10 s
Sim-Zeit = 300 Regeltakte; alle action_horizon Takte holt der Client einen neuen Aktions-Chunk vom Server.
Pruefungen:
  1. Server-Metadaten passen: 26 Aktionsnamen wie action_space.NAMES, Startpose, action_horizon
  2. Episode vollstaendig, Rohaufnahme besteht episode.validate()
  3. genau ein Server-Aufruf je action_horizon Takte
  4. Umrechnung verlustfrei: jede Aktion, die in der Sim ankommt, ist die Startpose (so antwortet der Ersatz-Server)
  5. Roboter haelt die Startpose (< 0,1 rad), kein Kontakt mit Tisch oder Objekten, Objekte < 5 mm
Ohne Fenster und ohne DDS. Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/60_openpi_closed_loop.py [--seconds 10] [--port 8000]
Gegen einen schon laufenden Server (spaeter: trainiert, auf dem GPU-Rechner): --no-server --host <adresse>;
dann entfallen 4 und 5, weil die Antwort nicht mehr bekannt ist.
Ausgabe: runs/stufe2_0_openpi_closed_loop.json, runs/stufe2_0_openpi/episode_000.npz, runs/stufe2_0_stand_in_server.log
"""
import argparse
import json
import math
import pathlib
import socket
import subprocess
import sys
import time
import urllib.request
import warnings

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from openpi_client import action_chunk_broker, base_policy, websocket_client_policy  # noqa: E402
from openpi_client.runtime import runtime as _runtime  # noqa: E402
from openpi_client.runtime import subscriber as _subscriber  # noqa: E402
from openpi_client.runtime.agents import policy_agent  # noqa: E402

from g1sim import action_space, episode, openpi_env, table_env, table_scene  # noqa: E402

OPENPI_PY = pathlib.Path.home() / "dev" / "g1-ext" / "openpi" / ".venv" / "bin" / "python"
OUT = REPO / "runs" / "stufe2_0_openpi_closed_loop.json"
EPISODE_DIR = REPO / "runs" / "stufe2_0_openpi"
SERVER_LOG = REPO / "runs" / "stufe2_0_stand_in_server.log"
ACTION_TOLERANCE = 1e-4
MAX_ARM_SAG = 0.1
MAX_OBJECT_DRIFT = 0.005

warnings.filterwarnings("ignore", message=".*parent has")


class TimedPolicy(base_policy.BasePolicy):
    """Misst jeden Server-Aufruf (Wanduhr, inklusive Netz)."""

    def __init__(self, policy):
        self.policy, self.calls_ms = policy, []

    def infer(self, obs):
        start = time.monotonic()
        result = self.policy.infer(obs)
        self.calls_ms.append((time.monotonic() - start) * 1000)
        return result

    def reset(self):
        self.policy.reset()


class Monitor(_subscriber.Subscriber):
    """Beobachtet die Sim waehrend der Episode: Kontakte, ankommende Aktionen, Absacken, Objektdrift."""

    def __init__(self, env):
        self.env = env

    def on_episode_start(self):
        self.start = {name: self.env.object_position(name) for name in table_scene.OBJECTS}
        self.touched, self.action_error, self.started = set(), 0.0, time.monotonic()

    def on_step(self, observation, action):
        self.touched.update(self.env.robot_contacts())
        error = float(np.abs(np.asarray(action["actions"]) - action_space.READY_POSE).max())
        self.action_error = max(self.action_error, error)

    def on_episode_end(self):
        self.wall_s = time.monotonic() - self.started
        self.sim_s = float(self.env.data.time)
        self.drift = {name: float(np.linalg.norm(self.env.object_position(name) - self.start[name]))
                      for name in table_scene.OBJECTS}
        self.arm_sag = float(np.abs(self.env.state()[:14] - action_space.READY_POSE[:14]).max())


def port_in_use(host, port):
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def start_stand_in(host, port, timeout_s):
    """Ersatz-Server in der openpi-Umgebung starten und warten, bis /healthz antwortet."""
    if port_in_use(host, port):
        raise SystemExit(f"Port {port} ist belegt - laeuft schon ein Server? lsof -nP -iTCP:{port} -sTCP:LISTEN")
    SERVER_LOG.parent.mkdir(exist_ok=True)
    proc = subprocess.Popen([str(OPENPI_PY), "-m", "openpi_g1.run", "stand-in", "--host", host, "--port", str(port)],
                            cwd=REPO, stdout=SERVER_LOG.open("w"), stderr=subprocess.STDOUT)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise SystemExit(f"Ersatz-Server beendet (Exit {proc.returncode}) - siehe {SERVER_LOG.relative_to(REPO)}")
        try:
            with urllib.request.urlopen(f"http://{host}:{port}/healthz", timeout=1) as response:
                if response.status == 200:
                    return proc
        except OSError:
            time.sleep(1.0)
    proc.kill()
    raise SystemExit(f"Ersatz-Server nach {timeout_s:.0f} s nicht bereit - siehe {SERVER_LOG.relative_to(REPO)}")


def stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seconds", type=float, default=10.0, help="Episodenlaenge in Sim-Sekunden")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--task", default=openpi_env.DEFAULT_TASK)
    parser.add_argument("--no-server", action="store_true", help="keinen Ersatz-Server starten, laufenden Server nutzen")
    parser.add_argument("--server-timeout", type=float, default=300.0, help="Sekunden, bis der Ersatz-Server bereit ist")
    args = parser.parse_args()
    stand_in = not args.no_server

    started = time.monotonic()
    server = start_stand_in(args.host, args.port, args.server_timeout) if stand_in else None
    server_ready_s = time.monotonic() - started
    try:
        env = table_env.TableEnv()
        client = websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
        meta = client.get_server_metadata()
        horizon = int(meta.get("action_horizon", 0))
        if horizon <= 0:
            raise SystemExit(f"Server-Metadaten ohne action_horizon: {meta}")
        timed = TimedPolicy(client)
        environment = openpi_env.TableEnvironment(env, task=args.task, max_steps=round(args.seconds * table_env.CONTROL_HZ))
        saver = openpi_env.EpisodeSaver(EPISODE_DIR, task=args.task)
        monitor = Monitor(env)
        agent = policy_agent.PolicyAgent(policy=action_chunk_broker.ActionChunkBroker(timed, action_horizon=horizon))
        _runtime.Runtime(environment=environment, agent=agent, subscribers=[monitor, saver]).run()
    finally:
        if server is not None:
            stop(server)

    steps = environment.max_steps
    ep = episode.load(saver.saved[0])
    problems = episode.validate(ep)
    reset_pose = meta.get("reset_pose", [])
    checks = {
        "server_metadaten": list(meta.get("action_names", [])) == list(action_space.NAMES)
        and len(reset_pose) == action_space.DIM and bool(np.allclose(reset_pose, action_space.READY_POSE, atol=1e-6)),
        "episode_vollstaendig": len(ep["timestamp"]) == steps,
        "episode_gueltig": not problems,
        "ein_aufruf_je_chunk": len(timed.calls_ms) == math.ceil(steps / horizon),
    }
    if stand_in:
        checks.update({
            "aktionen_verlustfrei": monitor.action_error < ACTION_TOLERANCE,
            "startpose_gehalten": monitor.arm_sag < MAX_ARM_SAG,
            "kein_kontakt_roboter_tisch_objekte": not monitor.touched,
            "objekte_bleiben_liegen": max(monitor.drift.values()) < MAX_OBJECT_DRIFT,
        })
    calls = timed.calls_ms
    result = {
        "server": "stand_in" if stand_in else f"{args.host}:{args.port}",
        "server_ready_s": round(server_ready_s, 1),
        "server_metadata": {key: meta[key] for key in ("robot", "action_horizon", "stand_in") if key in meta},
        "steps": steps,
        "sim_s": round(monitor.sim_s, 2),
        "wall_s": round(monitor.wall_s, 1),
        "realtime_factor": round(monitor.sim_s / monitor.wall_s, 2),
        "server_calls": len(calls),
        "latency_ms": {"first": round(calls[0], 1),
                       "median_after_first": round(float(np.median(calls[1:])), 1) if len(calls) > 1 else None,
                       "max_after_first": round(max(calls[1:]), 1) if len(calls) > 1 else None},
        "max_action_error": float(f"{monitor.action_error:.2e}"),
        "arm_sag_rad": round(monitor.arm_sag, 4),
        "object_drift_mm": {key: round(value * 1000, 2) for key, value in monitor.drift.items()},
        "robot_contacts": sorted(monitor.touched),
        "episode": str(saver.saved[0].relative_to(REPO)),
        "episode_problems": problems,
        "checks": {key: bool(value) for key, value in checks.items()},
        "passed": bool(all(checks.values())),
    }
    env.close()
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps({key: value for key, value in result.items() if key not in ("checks", "passed")}, ensure_ascii=False))
    for key, value in checks.items():
        print(f"  {'ok  ' if value else 'FAIL'} {key}")
    print(f"gespeichert: {OUT.relative_to(REPO)}, {result['episode']}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
