"""Level 1 Step 4 (plan Stage 5) - closed-loop eval of a policy served by scripts/81_smolvla_server.py, in the walk world.

The policy gets the head image, the 27-D state and the prompt (policy_wire.TASK; with --from-handover the demo's own sentence)
at a request tick, answers a 50-action chunk;
the first --exec-steps (25) actions run with one env.apply each (1/30 s), then the next request. Synchronous: the latency of
the model is not modelled (R2, "latency_modelled": false in the report; server and round-trip times are recorded in ms and in
ticks at 30 Hz). Hand channels are clipped to [0, 1] (R5); NaN/Inf actions fail the trial (policy_output_invalid).
Success and failure come from the event checks in g1sim/vla_eval.py (3 s placement latch, forbidden contacts, timeout).
Modes:
  --no-walkup --seed S     held-out starts: exactly 69's handover (one reset per trial, snapshot, first observation); seeds
                           >= 100 are refused (they recorded the training demos); seed 0 = main held-out set, 1 = reserve
  --from-handover NPZ|DIR  a recorded demo's handover restored in a fresh TableEnv; with a replay server (81 --replay) every
                           trial is compared with vla_eval.reference_replay: the replay gate (REPLAY GATE PASS / FAIL)
  --teacher                the sim teacher (ball_grasp.TwoHandBallGrasp) from the same handovers instead of a server: the
                           paired upper bound and the check that its verdict agrees with the event checks
  --train-variations N --demos DIR   first N evenly spaced training demos from their handovers, then the main trials; the
                           report gets the plan E verdict (undertrained / generalization gap / consistent)
  --expect-chunk NPY       (from-handover) the first chunk of the first trial against 81 --self-test --save-chunk:
                           WIRE CHECK max_abs_diff=<x> tol=0.01 PASS|FAIL
Report runs/vla_eval_<name>.json (rewritten after every trial), lab view task "vla_eval" (--no-labview turns it off).
Final line: RESULT {json}. Exit 0 = run complete and no check failed (any success rate), 1 = run complete but the replay gate,
the wire check or the teacher agreement failed, 2 = aborted (server error, contract mismatch, unreadable npz, task mismatch;
the partial report has "aborted": reason).
Writes the scene XML (table_scene.write) at start: never run it while 67/69 record (they read that file).
Environment ~/dev/g1-ext (sim); the server runs in the SmolVLA env:
    ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/81_smolvla_server.py --replay "$T7/smoke_v1" --port 8765 &
    ~/dev/g1-ext/.venv/bin/python scripts/82_vla_eval.py --from-handover "$T7/smoke_v1" --trials 5 --name s4_replay_smoke
    ~/dev/g1-ext/.venv/bin/python scripts/82_vla_eval.py --no-walkup --seed 0 --trials 5 --teacher --name s4_teacher_seed0
"""
import argparse
import json
import pathlib
import sys
import time
import warnings
import zipfile

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import (action_space, ball_grasp, episode, labview, policy_wire, randomization, table_env, table_scene,  # noqa: E402
                   vla_eval)

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message=".*encountered in matmul")  # false alarm numpy 2 + Apple Accelerate

RUNS = REPO / "runs"
WIRE_TOL = 0.01
GIT = labview.git_state()
LIVE = {"env": None, "image": None, "sent": None, "sim_s": None}  # lab view: env, last head image sent, sim s of the ended trial


class Abort(Exception):
    """Stop the run with exit 2; the message goes into the report as "aborted"."""


class LiveClient:
    """PolicyClient wrapper: keeps the last head image sent (lab view frame, no extra render: that would move the camera noise)
    and the first raw chunk since the last clear (wire check)."""

    def __init__(self, client):
        self.client, self.metadata, self.first_chunk = client, client.metadata, None

    def reset(self, episode=None, seed=0):
        self.client.reset(episode=episode, seed=seed)

    def infer(self, obs, tick):
        LIVE["image"] = obs[episode.POLICY_KEYS["head_image"]]
        reply = self.client.infer(obs, tick)
        if self.first_chunk is None:
            self.first_chunk = np.array(reply["actions"])
        return reply

    def close(self):
        self.client.close()


def sim_time():
    if LIVE["sim_s"] is not None:
        return LIVE["sim_s"]
    return LIVE["env"].data.time if LIVE["env"] is not None else 0.0


def live_frame():
    image = LIVE["image"]
    if image is None or image is LIVE["sent"]:
        return None
    LIVE["sent"] = image
    return labview.encode_jpeg(image)


def parse():
    parser = argparse.ArgumentParser(description="Level 1 Step 4 - closed-loop policy eval (82)")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--no-walkup", action="store_true", help="held-out starts drawn like 69 (no walk-up)")
    mode.add_argument("--from-handover", type=pathlib.Path, metavar="NPZ_OR_DIR", help="recorded demos' handovers")
    parser.add_argument("--seed", type=int, default=0, help="--no-walkup: < 100 (0 = held-out set, 1 = reserve)")
    parser.add_argument("--trials", type=int, default=5, help="no-walk: trials; from-handover: evenly spaced files")
    parser.add_argument("--teacher", action="store_true", help="run the sim teacher instead of a policy server")
    parser.add_argument("--server", default=f"127.0.0.1:{policy_wire.DEFAULT_PORT}", help="host:port of 81")
    parser.add_argument("--server-timeout-s", type=float, default=120.0)
    parser.add_argument("--connect-wait-s", type=float, default=60.0)
    parser.add_argument("--exec-steps", type=int, default=25, help="actions executed per chunk, 1..50")
    parser.add_argument("--timeout-s", type=float, default=45.0, help="sim time after the handover")
    parser.add_argument("--train-variations", type=int, metavar="N", help="N training demos first (with --demos)")
    parser.add_argument("--demos", type=pathlib.Path, help="training demo folder for --train-variations")
    parser.add_argument("--expect-chunk", type=pathlib.Path, metavar="NPY", help="wire check against 81 --save-chunk")
    parser.add_argument("--name", help="report name (runs/vla_eval_<name>.json)")
    parser.add_argument("--no-labview", action="store_true", help="write no lab view files (runs/labview/)")
    parser.add_argument("--randomization", type=pathlib.Path, default=randomization.CONFIG,
                        help="randomization config (default configs/randomization.json; configs/randomization_v1.json = run1's)")
    args = parser.parse_args()
    if args.no_walkup and args.seed >= 100:
        parser.error("--seed must be < 100 with --no-walkup: seeds >= 100 recorded the training demos")
    if not 1 <= args.exec_steps <= vla_eval.MAX_EXEC_STEPS:
        parser.error(f"--exec-steps must be 1..{vla_eval.MAX_EXEC_STEPS}")
    if args.trials < 1 or args.timeout_s <= 0:
        parser.error("--trials and --timeout-s must be positive")
    if (args.train_variations is None) != (args.demos is None):
        parser.error("--train-variations and --demos go together")
    if args.expect_chunk and (args.teacher or not args.from_handover):
        parser.error("--expect-chunk needs --from-handover and a server (no --teacher)")
    host, _, port = args.server.rpartition(":")
    if not host or not port.isdigit():
        parser.error("--server must be host:port")
    args.host, args.port = host, int(port)
    args.mode = "no-walk" if args.no_walkup else "from-handover"
    if args.name is None:
        args.name = ("teacher" if args.teacher else "policy") + (f"_nowalk_s{args.seed}" if args.no_walkup else "_handover")
    return args


def connect(args):
    try:
        client = policy_wire.PolicyClient(args.host, args.port, timeout_s=args.server_timeout_s,
                                          connect_wait_s=args.connect_wait_s)
    except (OSError, ValueError, policy_wire.PolicyServerError) as error:
        raise Abort(f"server {args.server}: {type(error).__name__}: {error}")
    hello = client.metadata
    problems = []
    if hello.get("action_dim") != action_space.DIM:
        problems.append(f"action_dim {hello.get('action_dim')} != {action_space.DIM}")
    if not isinstance(hello.get("chunk_size"), int) or hello["chunk_size"] < args.exec_steps:
        problems.append(f"chunk_size {hello.get('chunk_size')} < exec_steps {args.exec_steps}")
    if list(hello.get("image_shape") or []) != list(policy_wire.IMAGE_SHAPE):
        problems.append(f"image_shape {hello.get('image_shape')} != {list(policy_wire.IMAGE_SHAPE)}")
    if problems:
        client.close()
        raise Abort(f"server hello breaks the contract: {'; '.join(problems)}")
    return LiveClient(client)


class Run:
    """One 82 run: sections of trials, the report and the lab view."""

    def __init__(self, args, client, reporter, trials_planned):
        self.args, self.client, self.reporter = args, client, reporter
        self.timeout_ticks = int(round(args.timeout_s * table_env.CONTROL_HZ))
        self.replay = client is not None and client.metadata.get("mode") == "replay"
        self.index = 0
        self.path = RUNS / f"vla_eval_{args.name}.json"
        self.report = {
            "header": {"script": "82", "git": GIT, "argv": sys.argv[1:], "mode": args.mode, "seed": args.seed,
                       "source": str(args.from_handover) if args.from_handover else None, "teacher": args.teacher,
                       "server": None if client is None else client.metadata, "prompt": "per demo (npz task)" if args.from_handover else policy_wire.TASK,
                       "randomization": str(args.randomization),
                       "exec_steps": args.exec_steps, "timeout_s": args.timeout_s, "timeout_ticks": self.timeout_ticks,
                       "latency_modelled": False, "trials_planned": trials_planned, "started_at": labview.now_iso()},
            "sections": {}, "verdict": None, "replay_gate": None, "wire_check": None, "aborted": None}

    def write(self):
        RUNS.mkdir(exist_ok=True)
        labview.write_atomic(self.path, json.dumps(self.report, indent=2, ensure_ascii=False, default=float))

    def section(self, name):
        self.report["sections"][name] = {"rows": [], "summary": None}

    def add(self, section, row):
        rows = self.report["sections"][section]["rows"]
        rows.append(row)
        self.report["sections"][section]["summary"] = vla_eval.summarize(rows)
        self.write()
        if self.reporter:
            self.reporter.trial_end(row["passed"], row["reason"], {"reachable": row["reached_handover"],
                                                                   "error": row.get("error")})
        latch = f" latch {row['time_to_latch_s']} s" if row["passed"] else ""
        extra = f", teacher {'PASS' if row['teacher']['passed'] else row['teacher']['first_failed']}, agree {row['agree']}" \
            if "teacher" in row else ""
        print(f"{section} {row['id']}: {'PASS' if row['passed'] else row['reason']}{latch}, furthest "
              f"{row.get('furthest_event')}, requests {row.get('requests')}, wall {row.get('wall_s')} s{extra}", flush=True)

    def trial(self, env, first_obs, stem, seed, recorded_states=None, prompt=policy_wire.TASK):
        """One trial from the handover env is at. prompt: the sentence sent to the policy. Returns the row (without id)."""
        LIVE.update(env=env, image=first_obs["head_image"], sim_s=None)
        if self.reporter:
            self.reporter.trial_start(self.index, {"episode": stem})
        self.index += 1
        tracker = vla_eval.EventTracker(env.object_position(ball_grasp.BALL), timeout_ticks=self.timeout_ticks)
        on_tick = self.reporter.on_tick if self.reporter else None
        if self.args.teacher:
            row = vla_eval.run_teacher_trial(env, tracker, on_tick)
        else:
            try:
                row = vla_eval.run_policy_trial(env, self.client, prompt, tracker, self.args.exec_steps, stem, seed,
                                                recorded_states, on_tick, first_obs=first_obs)
            except vla_eval.ContractError as error:
                raise Abort(f"policy reply breaks the contract: {error}")
            except (OSError, ValueError, policy_wire.PolicyServerError) as error:
                raise Abort(f"server error: {type(error).__name__}: {error}")
        row["stance_offsets"] = env.variation.get("arrival")
        LIVE["sim_s"] = row["sim_s"]  # the trial's end, not the env's clock (the teacher runs on after a latch)
        return row

    def from_handover(self, section, paths):
        self.section(section)
        for i, path in enumerate(paths):
            try:
                ep = episode.load(path)
            except (OSError, ValueError, KeyError, zipfile.BadZipFile) as error:
                raise Abort(f"{path}: unreadable npz ({type(error).__name__}: {error})")
            if "handover_json" not in ep:
                raise Abort(f"{path}: no handover snapshot")
            if ep["task"] not in policy_wire.TASKS:
                raise Abort(f"{path}: task {ep['task']!r} is none of {policy_wire.TASKS}")
            stem, seed = path.stem, policy_wire.noise_seed(path.stem)
            if section == "main" and i == 0 and self.client is not None:
                self.client.first_chunk = None
            env = table_env.TableEnv(scene=vla_eval.SCENE, variation_config=randomization.load(self.args.randomization))
            try:  # the demo's own sentence: the replay server checks the prompt against it, a policy saw it in training
                row = self.trial(env, vla_eval.restore(env, ep), stem, seed, recorded_states=ep["state"], prompt=ep["task"])
            finally:
                LIVE["env"] = None
                env.close()
            row = {"id": stem, "path": str(path), "noise_seed": seed, "prompt": ep["task"], **row}
            if self.replay:
                row["reference"] = vla_eval.reference_replay(ep, self.timeout_ticks)
                row["gate_problems"] = vla_eval.replay_gate_problems(row, row["reference"])
                for problem in row["gate_problems"]:
                    print(f"REPLAY GATE FAIL {stem}: {problem}", flush=True)
            if section == "main" and i == 0 and self.args.expect_chunk:
                self.report["wire_check"] = wire_check(self.args.expect_chunk, self.client.first_chunk)
            self.add(section, row)

    def no_walk(self, section, seed, trials):
        self.section(section)
        env = table_env.TableEnv(scene=vla_eval.SCENE, seed=seed, variation_config=randomization.load(self.args.randomization))
        try:
            for h in vla_eval.no_walk_handovers(env, trials):
                trial_id = f"s{seed}_t{h['trial']:04d}"
                if h["error"]:
                    if self.reporter:
                        self.reporter.trial_start(self.index)
                    self.index += 1
                    LIVE["sim_s"] = 0.0
                    self.add(section, {"id": trial_id, "passed": False, "reason": "handover_failed", "reached_handover": False,
                                       "furthest_event": None, "error": h["error"]})
                    continue
                noise = seed * 100000 + h["trial"]
                row = self.trial(env, h["obs"], None, noise)
                handover = {k: v.tolist() if hasattr(v, "tolist") else str(v)
                            for k, v in episode.handover_arrays(h["snapshot"]).items()}
                self.add(section, {"id": trial_id, "noise_seed": noise, **row, "handover": handover})
        finally:
            LIVE["env"] = None
            env.close()


def wire_check(expected_path, chunk):
    expected = np.load(expected_path, allow_pickle=False)
    if chunk is None or chunk.shape != expected.shape:
        diff = float("inf")
    else:
        diff = float(np.abs(chunk.astype(np.float64) - expected.astype(np.float64)).max())
    passed = diff <= WIRE_TOL
    print(f"WIRE CHECK max_abs_diff={diff} tol={WIRE_TOL} {'PASS' if passed else 'FAIL'}", flush=True)
    return {"max_abs_diff": diff, "tol": WIRE_TOL, "passed": passed, "expected": str(expected_path),
            "shape": None if chunk is None else list(chunk.shape)}


def short(summary):
    if summary is None:
        return None
    keys = ("trials", "reached_handover", "passed", "vla_success", "end_to_end", "failures", "replay_gate", "teacher_agreement")
    return {k: summary[k] for k in keys if k in summary}


def main():
    args = parse()
    train_paths = []
    if args.train_variations:
        try:
            train_paths = vla_eval.pick_evenly(vla_eval.list_demos(args.demos), args.train_variations)
        except FileNotFoundError as error:
            print(f"aborted: {error}")
            return 2
    main_paths = []
    if args.from_handover:
        try:
            main_paths = vla_eval.pick_evenly(vla_eval.list_demos(args.from_handover), args.trials)
        except FileNotFoundError as error:
            print(f"aborted: {error}")
            return 2
    planned = len(train_paths) + (len(main_paths) if args.from_handover else args.trials)

    table_scene.write(scene=vla_eval.SCENE)
    reporter = None if args.no_labview else labview.Reporter(
        script="82", task="vla_eval", object=f"{args.name}_{args.mode}", trials=planned, argv=sys.argv[1:],
        sim_time_getter=sim_time, frame_getter=live_frame)
    client, run = None, None
    try:
        client = None if args.teacher else connect(args)
        run = Run(args, client, reporter, planned)
        run.write()
        if client is not None:
            print(f"server: {json.dumps(client.metadata)}", flush=True)
        if train_paths:
            run.from_handover("train", train_paths)
        if args.from_handover:
            run.from_handover("main", main_paths)
        else:
            run.no_walk("main", args.seed, args.trials)
    except Abort as error:
        if run is None:
            run = Run(args, client, None, planned)
        run.report["aborted"] = str(error)
        print(f"aborted: {error}", flush=True)
    finally:
        if client is not None:
            client.close()

    report, sections = run.report, run.report["sections"]
    if "train" in sections and sections.get("main", {}).get("summary") and sections["train"]["summary"]:
        report["verdict"] = vla_eval.verdict(sections["train"]["summary"], sections["main"]["summary"])
        v = report["verdict"]
        print(f"VERDICT {v['verdict']} (train {v['train']} CI {v['train_ci95']}, held-out {v['heldout']} CI "
              f"{v['heldout_ci95']})")
    gated = [r for s in sections.values() for r in s["rows"] if "gate_problems" in r]
    if gated:
        report["replay_gate"] = {"passed": sum(not r["gate_problems"] for r in gated), "trials": len(gated),
                                 "problems": [f"{r['id']}: {p}" for r in gated for p in r["gate_problems"]]}
        if not report["replay_gate"]["problems"] and not report["aborted"]:
            print("REPLAY GATE PASS", flush=True)
    taught = [r for s in sections.values() for r in s["rows"] if "teacher" in r]
    failed = [bool(report["replay_gate"] and report["replay_gate"]["problems"]),
              bool(report["wire_check"] and not report["wire_check"]["passed"]),
              any(not r["agree"] for r in taught)]
    code = 2 if report["aborted"] else 1 if any(failed) else 0
    report["exit"] = code
    run.write()
    if reporter:
        reporter.finish(summary=short(sections.get("main", {}).get("summary")), report_path=run.path)
    result = {"name": args.name, "mode": args.mode, "teacher": args.teacher, "report": str(run.path.relative_to(REPO)),
              "main": short(sections.get("main", {}).get("summary")), "train": short(sections.get("train", {}).get("summary")),
              "verdict": report["verdict"] and report["verdict"]["verdict"], "replay_gate": report["replay_gate"],
              "wire_check": report["wire_check"], "aborted": report["aborted"], "exit": code}
    print("RESULT " + json.dumps(result, default=float), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
