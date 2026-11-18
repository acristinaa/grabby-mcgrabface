---
phase: level1-walkup
plan: step4-policy-server-eval
revision: 2 (2026-10-15, planner; answers .planning/reviews/level1-step4-plan.md, see "Revision 2" at the end)
depends_on:
  - Step 6 data (300 demos level1_walkup_v1 + 5 smoke_v1 on the T7, handover snapshot in every npz)  # done 2026-10-14
  - Step 7 smoke checkpoint ~/dev/g1-ext/smolvla-runs/smoke/checkpoints/000200/pretrained_model     # done 2026-10-15
files_modified:
  # Task 1 owns
  - g1sim/policy_wire.py                 # new: wire protocol, TASK, noise_seed, run1 guards (stdlib + numpy), torch-free ReplayPredictor
  - scripts/81_smolvla_server.py         # new
  - tests/test_unit_policy_wire.py       # new
  # Task 2 owns
  - g1sim/vla_eval.py                    # new: event tracker, reference replay, replay gate, Wilson, stall, chunk loop, verdict
  - scripts/82_vla_eval.py               # new
  - g1sim/labview.py                     # additive: vla_eval phase table, PHASE_ORDERS["vla_eval"], 3 reason keys
  - tests/test_unit_vla_eval.py          # new
  - tests/test_integration_vla_eval.py   # new
  - tests/test_unit_labview.py           # one line: the 4th phase table in test_phase_keys_are_unique_across_tasks
  # Task 3 owns
  - scripts/83_open_loop_check.py        # new
  # integration step owns (after the gate)
  - COMMANDS.md, PROGRESS.md (LOG + NEXT STEP row 4), docs/LEARNING_PATH.md §10, .planning/level1-walkup/PLAN.md (new section M)
---

# Level 1 Step 4: SmolVLA policy server, eval script, open-loop check (plan Stage 5, held-out no-walk mode from L/M1)

Repo root (all paths relative to it):
`/Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot`.
`PY=~/dev/g1-ext/.venv/bin/python` (sim env: mujoco 3.13, lerobot 0.6.2, numpy 2.2.6, openpi-client, pytest).
`SV=~/dev/g1-ext/smolvla-test/.venv/bin/python` (SmolVLA env: lerobot 0.6.1, torch 2.11, transformers 5.5.4, numpy 2.2.6; **no**
pytest, websockets, msgpack, grpc, mujoco). `T7="/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos"`.
`SMOKE=~/dev/g1-ext/smolvla-runs/smoke/checkpoints/000200/pretrained_model`.

Scope: Step 4 only. No walk-up (Step 2), no training changes, no change to `table_env.py`, `ball_grasp.py`, `episode.py`, 67, 69, 79, 80.

## 0. Hard constraints while `run1` trains (lerobot-train on MPS, PID 32059, `--output_dir=~/dev/g1-ext/smolvla-runs/run1`)
- Nothing in this plan uses MPS/GPU, starts lerobot-train, or reads `~/dev/g1-ext/smolvla-runs/run1`. Every model-loading check
  uses `$SMOKE` with `--device cpu --threads 4` (measured: 4.6 s load, ~5 s per chunk, probe `probe_cpu_chunk.log`). The MPS
  latency check (< 1 s per chunk) is deferred to the Stage 9 preflight and reported as "not run" until then.
- The code enforces this too (m2, R4): `SmolVLAPredictor` refuses `--device mps` while a lerobot-train process exists (unless
  `--force-mps`), and refuses a checkpoint inside a running training's `--output_dir` (unless `--allow-during-training`; even then
  only a complete checkpoint dir whose newest file is older than 60 s). R4 is **not approved**: no Step 4 command passes
  `--allow-during-training`, and no Step 4 command points at run1 except the guard check on a non-existent path (Task 3 verify).
- Replay mode imports neither torch nor lerobot, so the replay gate runs during training (sim-only, ~1-2 s CPU per trial).
- Nothing writes to the T7: 81/82/83 open npz files read-only; all output goes to `runs/` (gitignored); scratch probes live in the
  scratchpad below.
- Gate output always goes to a log file in full (`> runs/<name>.log 2>&1`), never through `tail`/`head`. Every background server a
  command block starts is killed at the end of the same block (`kill $SERVER; wait $SERVER`); if a block is split across shells,
  `pkill -f 81_smolvla_server.py` (matches only 81, never lerobot-train).

Probe evidence in `/private/tmp/claude-501/-Users-finneganadam-Documents-Projekt-Tim-and-Lukas-Cursor-Software-Engineering-SAP-Robot-Project-First-Ideas/f0ff7f6a-b54c-4e4d-a5d8-61eb8f19683e/scratchpad/step4/`
(`probe_proc.*`, `teacher_repro.*`, `probe_cpu_chunk.*`, `probe_replay_events.*`, `probe_static_runs.*`, `probe_window_range.*`,
reviewer `review_gate_preview.*`, `review_openloop_baselines.*`, revision 2 `plan2_obs_probe.*`).
Not repo code; every number taken from them is re-measured by a test or gate below.

## Assumptions
- Assuming: the scene file under `~/dev/g1-ext/g1_brainco_sim` is today's `volleyball_walk` (the teacher repro and the replay probes
  reproduced recorded demos bit-exactly without rewriting it). 82 still calls `table_scene.write(scene="volleyball_walk")` at start,
  exactly like `69:203`.
- Assuming: the run1 checkpoints will have the same config/processor layout as the smoke checkpoint (same `train_smolvla.sh`, same
  `--policy.path=lerobot/smolvla_base`). 81 asserts the layout at load time instead of trusting this.
- Assuming: the 5 `smoke_v1` demos (seed 100) are the replay gate set; they are not in the training dataset (seeds 101-104).
- Assuming: the user's "replay 5/5" means: 5 recorded demos, handover restored from the npz, actions served by 81 over the wire,
  executed by 82 in 25-step chunks, success latched by 82's event checks, **and** the observation 82 sent at every request tick is
  the recorded one (F2).
- Assuming: a running training is visible as a process whose command line contains `lerobot-train` and `--output_dir=<dir>`
  (true for run1, `pgrep -fl lerobot-train` today). If `--output_dir` cannot be parsed, the run1 guard refuses every checkpoint
  (conservative).

## 1. Decisions with evidence

### (1) Transport: stdlib TCP + JSON header + raw numpy bytes (new `g1sim/policy_wire.py`), server in the SmolVLA env (R1: approved)
| Option | Evidence | Verdict |
|---|---|---|
| openpi websocket (`openpi-client`) | Sim env has it (editable, `~/dev/g1-ext/openpi/packages/openpi-client`). SmolVLA env has neither `websockets` nor `msgpack` (import check). `openpi-client` itself pins `numpy>=1.22.4,<2.0.0` (`openpi-client/pyproject.toml:8`) while the SmolVLA env runs numpy 2.2.6: a normal install would try to downgrade numpy in the env the training is running from. The server half (`WebsocketPolicyServer`) is in the full `openpi` package, not in `openpi-client`, so we would write the server anyway. | Rejected for Step 4. Revisit after run1. |
| lerobot async inference (`lerobot/async_inference/policy_server.py`) | Needs `grpc` (`policy_server.py:38`, not installed) and moves observations/actions with `pickle.loads` (`:28`, `:183`, `:236`). | Rejected (install + pickle). |
| SmolVLA inside the sim env | Sim env has no `transformers`, `num2words`, `accelerate`; its lerobot is 0.6.2 (editable) while the checkpoint was written by 0.6.1. Installing transformers 5.x there risks moving `huggingface-hub` (1.31 there vs 1.33 in the SmolVLA env) under the walking sim. The real robot will also run model and robot I/O as separate processes. | Rejected. |
| **stdlib TCP** | Both envs have numpy 2.2.6; `g1sim/__init__.py` imports only `pathlib`, and `$SV -c "from g1sim import episode, action_space"` works (checked). No install, no pickle, testable with pytest in the sim and project envs. | **Chosen.** |

**Message format (the shared contract; fixed here so Tasks 1, 2 and 3 build against it in parallel):**
- One message = 4-byte big-endian header length + UTF-8 JSON header + the raw C-order bytes of each array listed in
  `header["arrays"] = [{"name": str, "dtype": str, "shape": [int, ...]}, ...]`, in that order.
- Bounds checked by `recv` **before reading the body** (m4): header length ≤ `MAX_HEADER` (64 KiB); `arrays` is a list of at most
  8 entries; each `dtype` in `{"uint8", "float32"}` (anything else is refused on send and on receive, so no object arrays can ever
  appear); each `shape` is a list of at most 4 Python ints, each ≥ 0 (no bools, floats, strings, negatives); the sum of
  `prod(shape) * itemsize` ≤ `MAX_BYTES` (16 MiB; one 200x320x3 image is 192 kB). Any violation → `ValueError`, nothing allocated.
  `send` applies the same dtype/rank checks to what it sends.
- Types:
  - server → client on connect: `{"type": "hello", "mode": "replay"|"smolvla", "chunk_size": 50, "action_dim": 27,
    "image_shape": [200, 320, 3], "checkpoint": str|null, "step": int|str|null, "device": str, "source": str|list}`.
  - client → server `{"type": "reset", "episode": <npz stem>|null, "seed": int}` → `{"type": "ok"}`.
  - client → server `{"type": "infer", "tick": int, "prompt": str}` + arrays `observation/head_image` (uint8 200x320x3) and
    `observation/state` (float32 (27,)) → `{"type": "actions", "server_ms": float, "checks": dict}` + array `actions`
    (float32 (50, 27)).
  - any problem → `{"type": "error", "message": str}`; the client raises `PolicyServerError`; the connection stays usable.
- `checks` (F2): `{}` from the SmolVLA predictor. From the replay predictor, at a tick with a recorded frame (`tick < T`):
  `{"recorded_tick": tick, "obs_state_err": max |state - recorded state[tick]| (float, must be 0.0),
  "obs_image_maxdiff": max |image - recorded image[tick]| in grey levels (int), "obs_image_mad": mean |image - recorded image[tick]|
  (float), "obs_prompt_ok": prompt == npz task (bool)}`; at `tick >= T`: `{"recorded_tick": null, "obs_prompt_ok": bool}`.
  The predictor only measures; the thresholds live in `vla_eval` (decision (4)).
- The observation keys are `episode.POLICY_KEYS` (`episode.py:31`) plus `prompt` (`episode.py:34-39`): the same payload
  `openpi_g1/policy.py` reads, so a later websocket transport is a client swap, not a format change.
- Server binds `127.0.0.1` by default (`--host` to expose it later on the backpack), one client at a time, `--port 0` = ephemeral
  port. It binds and prints `LISTENING <port>` (flushed) only **after** the predictor is loaded; the client retries the connect
  for up to `connect_wait_s`, so no shell sleep/poll is needed between starting a server and starting 82.
- Tick convention (review): `tick` = number of actions applied since the handover (0-based request tick, frame `tick` = the
  observation before action `tick`); event ticks and the latch tick are 1-based = number of actions applied when the event is seen.
  Requests at ticks 0, 25, 50, ... ⇒ requests until a latch at L = `ceil(L / 25)`.

`g1sim/policy_wire.py` API (exact names; Task 2 and Task 3 code against these, nothing else):
```
TASK = "put the volleyball in the basket"          # m5: the one prompt constant for 81/82/83 (equal to 69's TASK, test-pinned)
CHUNK = 50; ACTION_DIM = 27; IMAGE_SHAPE = (200, 320, 3); MAX_BYTES = 16 * 2**20; MAX_HEADER = 64 * 2**10; DEFAULT_PORT = 8765
class PolicyServerError(RuntimeError)
send(sock, header: dict, arrays: dict[str, np.ndarray] | None = None) -> None
recv(sock) -> tuple[dict, dict[str, np.ndarray]]                  # loops until all bytes; ConnectionError on EOF; ValueError on bounds
check_observation(obs: dict) -> list[str]                         # problems; [] = valid (image shape/dtype, state (27,) float32 finite, prompt non-empty str)
noise_seed(stem: str) -> int                                      # zlib.crc32(stem.encode()); the reset seed for a recorded episode
replay_chunk(actions: np.ndarray, tick: int, chunk=CHUNK) -> np.ndarray   # actions[tick:tick+chunk], padded with the last row; tick < 0 -> ValueError
class ReplayPredictor(sources)       # list of npz files and/or folders; skips ._* and *.part; indexes episodes by file stem
                                     # (the same stem in two sources -> ValueError at start); .metadata; reset(episode, seed) loads that
                                     # stem's actions, state, head_image, task (unknown/None stem -> ValueError);
                                     # infer(obs, tick) -> (replay_chunk(...), checks)
serve(predictor, host, port, on_listen=print, verbose=False) -> None   # hello, then reset/infer loop; check_observation before predictor.infer
class PolicyClient(host, port, timeout_s=120, connect_wait_s=60)       # .metadata = hello; reset(episode=None, seed=0);
                                     # infer(obs, tick) -> {"actions", "server_ms", "round_trip_ms", "checks"}; close()
# run1 guards (m2, R4); pure functions, unit-tested with injected command lines and tmp dirs
training_cmdlines() -> list[str]                                  # `pgrep -fl lerobot-train` lines (empty if none)
training_output_dirs(cmdlines) -> list[pathlib.Path] | None       # resolved --output_dir of each; None if any line has none
checkpoint_ready(ckpt: Path, now: float, min_age_s=60) -> str | None   # None = complete (config.json, model.safetensors,
                                     # policy_preprocessor.json, policy_postprocessor.json) and newest file older than min_age_s
device_problem(device, cmdlines, force_mps) -> str | None         # "mps" while a training runs and not force_mps -> message
checkpoint_problem(ckpt, cmdlines, allow_during_training, now) -> str | None   # inside a running output_dir (or output_dir
                                     # unknown) and not allowed -> refusal; allowed -> checkpoint_ready(...); decides from the
                                     # resolved path string, before any existence check or file read
```
Predictor interface (duck-typed, used by `serve`): `.metadata: dict`, `reset(episode: str | None, seed: int)`,
`infer(obs: dict, tick: int) -> tuple[np.ndarray (50, 27) float32, dict]`.

`SmolVLAPredictor` (in `scripts/81_smolvla_server.py`; 83 imports it, so its signature is part of the contract):
`SmolVLAPredictor(checkpoint, device="cpu", threads=None, force_mps=False, allow_during_training=False)`, same interface as above
(`checks` = `{}`), plus `.load_s`. Order inside `__init__`: (1) `device_problem` → `SystemExit(2)` with the message; (2) resolve the
checkpoint (`.../pretrained_model` or its parent; `Path.resolve()` so `checkpoints/last` gives its real step) and
`checkpoint_problem` → `SystemExit(2)`; (3) existence/layout of the files; (4) `HF_HUB_CACHE`/`HF_HUB_OFFLINE`; (5) lazy torch and
lerobot imports; (6) load. So a refusal never imports torch or reads the checkpoint.

### (2) Inference API for a SmolVLA checkpoint in lerobot 0.6.1 (installed sources, SmolVLA env site-packages; confirmed by review)
- Config: `PreTrainedConfig.from_pretrained(ckpt)`, then `cfg.device = <device>`; policy: `SmolVLAPolicy.from_pretrained(ckpt,
  config=cfg)` (`policies/pretrained.py:169-227`: loads `model.safetensors` from a local dir, `.to(config.device)`, `.eval()`).
  It also loads the SmolVLM2 backbone from the HF cache first (log: "Loading HuggingFaceTB/SmolVLM2-500M-Video-Instruct weights"),
  so the predictor sets `HF_HUB_CACHE=~/dev/g1-ext/smolvla-test/hf_cache/hub` and `HF_HUB_OFFLINE=1` (`os.environ.setdefault`)
  **before** importing lerobot, like `train_smolvla.sh:17-18` (the CPU probe ran with exactly these and loaded offline in 4.6 s).
- Processors: `make_pre_post_processors(cfg, pretrained_path=ckpt, preprocessor_overrides={"device_processor": {"device": dev}})`
  (`policies/factory.py:150`, loads both pipelines at `:200-219`; the saved preprocessor has `"device": "mps"`, so the override is
  required on CPU). Pre steps (probe): rename (empty map) → add batch dim → newline on task → tokenizer (max 48) → device →
  normalizer (MEAN_STD for state and action, IDENTITY for images). Post steps: unnormalizer → device cpu. Normalization comes from
  the checkpoint, never from our code.
- Input: `prepare_observation_for_inference({"observation.images.camera1": uint8 HWC copy, "observation.state": float32 (27,)},
  device, task=prompt)` (`policies/utils.py:97-138`: uint8 → float/255, HWC → CHW, batch dim, device; `task` string). Pass copies:
  the helper converts the dict in place via `torch.from_numpy`.
- Call: `policy.predict_action_chunk(batch, noise=noise)` (`modeling_smolvla.py:231-241`), not `select_action` (`:243-269`, keeps
  its own queue). Output `(1, 50, 32)` is unpadded to 27 dims inside `_get_action_chunk` (`:216-218`); `post(actions)` →
  `(1, 50, 27)` float32 on CPU. Returns `actions[0].numpy()`.
- Noise: `(1, chunk_size, max_action_dim) = (1, 50, 32)` (`:726-742`), drawn from a CPU `torch.Generator` re-seeded at each
  `reset(seed)` and moved to the device. CPU probe: same noise → identical chunk (max diff 0.0); two random noises → chunks up to
  0.79 apart. Paired trials are reproducible on CPU; on MPS it is reported, not assumed.
- **Checkpoint quirk:** `input_features` still lists smolvla_base's `observation.state` `[6]` and `camera1..3` at 256x256. Harmless
  (`prepare_state` pads 27 → 32, `:409-413`; missing cameras skipped with `empty_cameras` 0, `:340`, `:373-375`). 81 asserts from
  the normalizer stats file (`policy_preprocessor_step_*_normalizer_processor.safetensors`): `action` output shape (27,),
  `observation.state.mean` 27 entries, `observation.images.camera1` among `cfg.image_features`, `cfg.type == "smolvla"`,
  `chunk_size == 50`.
- Measured on CPU (4 threads, training running): ~5.0-5.2 s per chunk, parameters bf16. MPS (~200 ms per chunk with smolvla_base,
  2026-10-07) is the Stage 9 expectation, not re-measured.

### (3) Action execution and timing
- 82 asks for a chunk at tick `t` (observation = `env.observation()` taken fresh before the action of tick `t`, the same "pending
  observation" as the recorder, `69:130-143`), executes `chunk[:exec_steps]` (default 25) with one `env.apply` per action = 1/30 s
  sim time (`table_env.py:270-283`), then asks again at `t + exec_steps`. `--exec-steps` must be 1..50.
- **Latency is not modelled in Step 4 (R2: approved), and the report says so** (`latency_modelled: false`). The loop is synchronous.
  The report records server time and round-trip time per request (mean, p95, max, and "= N ticks at 30 Hz"). `--delay-ticks K` is a
  Stage 9 option. The per-tick actuator latency of the domain randomization (`physics.latency_s`, `table_env.py:167`) still applies.
- The client clips hand channels 14..25 to [0, 1] (R5: approved) and counts clipped values per trial; arms and waist pass unchanged.
  Non-finite actions → the trial fails with reason `policy_output_invalid`. A wrong shape aborts the run (exit 2). Clipping is a
  no-op on the recorded actions, so replay stays bit-exact.
- Speed: 82 renders only at request ticks. Expected per trial on MPS: ≤ 54 requests × ~0.25 s + ~3 s sim ≈ 15-20 s; 60 trials ≈ 20 min.

### (4) Event checks, the replay gate and metrics (pure code in `g1sim/vla_eval.py`)
Per tick after `env.apply` (`rest` = ball position at the handover, as the teacher's `rest`, `ball_grasp.py:194`):
- `lifted`: first tick with `ball_z - rest_z >= ball_grasp.MIN_LIFT` (0.06, `ball_grasp.py:94`).
- `carried`: first tick with `ball_z - rest_z >= 0.06` **and** ball xy inside the basket's outer footprint
  (`|ball_xy - basket_xy| <= BASKET_INNER_HALF + BASKET_WALL` = 0.145 m per axis, `table_scene.py:61-63`, basket from `data.xpos`).
- `placing`: all 4 `ball_grasp.placement_checks(env)` (`ball_grasp.py:136-150`) true on this tick; the streak resets on any false tick.
- `forbidden`: first tick with a robot contact whose other body is in `ball_grasp.FORBIDDEN` (`ball_grasp.py:100`). The trial stops.
- **Success latches** at the first tick where the streak reaches `3 * 30 = 90` ticks with no forbidden contact before; the trial stops.
- Timeout: `--timeout-s` (default 45) of sim time after the handover = 1350 ticks.
- Replay evidence (F1, `review_gate_preview.log`, the exact 10 demos the gate selects, recorded actions then the last held):
  latch at ticks 703-754 = 23.4-25.1 s, i.e. 23 ticks before to 28 ticks after the end of the recording (T = 726);
  `state_err 0.0` on all 10; requests = `ceil(latch/25)` on all 10. Natural variation is ±1 s, so **no absolute time band is
  gated**; the gate compares against a reference replay instead (below). 45 s leaves ~20 s of slack for a slower policy.
- **Reference replay (F1):** `vla_eval.reference_replay(ep, timeout_ticks)` restores `ep`'s handover in a **fresh** TableEnv and
  applies `actions[min(t, T-1)]` per tick through the same `EventTracker` — no chunks, no wire, no `policy_wire` import, so it is
  independent of `replay_chunk` and the server. It returns the event ticks, `passed`, `reason` and `latch_tick` (deterministic:
  10/10 bit-exact in the probe).
- **Observation thresholds (F2), measured in `plan2_obs_probe.log`** (the same 10 demos, fresh TableEnv, render only at request
  ticks like 82, compared with the recorded frame at that tick): tick 0 image max grey diff 0 on all 10 (earlier probe: 0-2);
  later request ticks image MAD 3.03-3.39 (two independent noise draws, `rgb_noise_std` up to 3); state bit-equal at every request
  tick. A broken client would send: stale tick-0 frame MAD 13-72 (state off by 0.007-1.97); the previous request's frame MAD
  3.3-43 (state 0.0-1.38); the frame one tick late MAD 3.1-10 (state 0.0-0.07). Per request the image alone cannot separate every
  case, but over a trial the bit-equal state does: any stale/late observation differs in state at some request tick. Constants:
  `OBS_IMAGE_MAXDIFF_TICK0 = 2`, `OBS_IMAGE_MAD_MAX = 4.0` (18 % above the measured 3.39), `obs_state_err` must be exactly 0.0.
- **`vla_eval.replay_gate_problems(row, reference) -> list[str]`** (empty = pass), the Step 4 done criterion per trial:
  passed (latched); `latch_tick` and every event tick equal to the reference replay's; `replay_state_error == 0.0`;
  `obs_state_err_max == 0.0`; `obs_image_maxdiff_tick0 <= 2`; `obs_image_mad_max <= 4.0`; `obs_prompt_ok_all`; `obs_checked` equal to
  the number of requests at ticks < T (every request with a recorded frame was checked); `requests == ceil(latch_tick / exec_steps)`; not stalled. Each problem names its field.
- Failure reason = the first failed check in the teacher's order (`ball_grasp.py:259-264`): `no_forbidden_contacts` →
  `ball_lifted` → `ball_carried_to_basket` → the first false placement check at the last tick (`ball_in_basket`,
  `ball_on_basket_floor`, `ball_at_rest`, `hands_off_ball`) → `placement_not_held_3s`. Plus `policy_output_invalid` and
  `handover_failed`. The eval's `ball_carried_to_basket` adds the footprint condition (lens 3, documented; teacher mode measures both).
- `furthest_event` ∈ handover < lifted < carried < placing < latched.
- **Stall.** A trial is **stalled** if it failed and some 60-tick window (2 s) of executed actions has a range (max over the 27 dims
  of max - min) below `STALL_RANGE = 0.01`. Teacher margin: smallest 60-tick range in 43 demos is 0.038 (`probe_window_range.log`).
  In a replay the held last action starts at tick 726 and the latch comes at most 28 ticks later (< 60), and successes are never
  "stalled". m3: the policy side is unmeasured (fresh noise per chunk can make a hovering policy jump between chunks), so a second,
  progress-based diagnostic `no_progress` is reported next to it: failed by timeout, no new event and ball displacement < 0.02 m
  over the last 150 ticks (5 s). Both are diagnostics, not gates; `STALL_RANGE` is re-calibrated on the first run1 eval.
- Per trial row: id (seed + trial, or demo stem), passed, reason, furthest_event, tick/second of every event, time to latch,
  forbidden pair, stalled + start, no_progress, requests, server_ms and round_trip_ms (mean/max), hand clips, sim_s, wall_s, stance
  offsets, and in from-handover mode `replay_state_error` (max |state - recorded state| over the recorded ticks reached) and the
  observation aggregates `obs_state_err_max`, `obs_image_maxdiff_tick0`, `obs_image_mad_max`, `obs_prompt_ok_all`, `obs_checked`
  (from the replay server's `checks`; `null` against a SmolVLA server), and `reference` (the reference replay's event ticks) when
  the server hello says `mode == "replay"`.
- **Denominators:** `vla_success = passes / trials that reached the handover`; `end_to_end = passes / all trials`, both with Wilson
  95 % intervals (z = 1.959964). Summary: failure table by reason, furthest-event counts, stalled and no_progress counts, latency
  mean/p95/max, time to latch mean, hand clips total, and in replay mode `replay_gate: {"passed": k, "trials": n, "problems": [...]}`.
- `--train-variations N --demos <dir>` (Stage 9 form): N evenly spaced training demos from-handover first, then the main no-walk
  trials, one report with both sections and the plan E verdict (R3: approved): `round(train, 6) < 0.6` → "undertrained: resume
  training, do not add demos"; else `round(train - heldout, 6) > 0.2` → "generalization gap: more demos or narrower ranges"; else
  "consistent" (rounding so that 0.8 vs 0.6 is "consistent", not a float-error "gap"). Both intervals printed next to the verdict.
- Lab View: `labview.Reporter(script="82", task="vla_eval", object=f"{name}_{mode}", trials=N, argv, sim_time_getter, frame_getter=
  last head image as JPEG (no extra render: an extra render would move `noise_rng` and change the next observation))`; `on_tick(phase)`
  with phase keys `vla_start`, `vla_lifted`, `vla_carried`, `vla_placing`; `trial_end(passed, reason, {"reachable":
  reached_handover, "error": ...})`. `--no-labview` turns it off (as in 69).

### (5) Held-out no-walk mode = 69's handover, step for step
`82 --no-walkup --seed S --trials N` does exactly what `69` does before the first recorded frame:
`table_scene.write(scene="volleyball_walk")` (`69:203`) → `TableEnv(scene="volleyball_walk", seed=S, variation_config=
randomization.load())` (`69:204`) → per trial **one** `env.reset(randomize=True)` (`69:214`) → `snapshot = env.snapshot_handover()`
(`69:127`) → `obs = env.observation()` (`69:130`) → policy loop. `env.rng` is consumed only in `reset` (`table_env.py:124`) and
`noise_rng` is re-seeded per reset index (`:116`), so trial k of seed S is the k-th draw of that seed (confirmed by review).
- Seeds: `--no-walkup` refuses `--seed >= 100` (training seeds; the mirror of `69:193-194`). Seed 0 = main held-out set, seed 1 = reserve.
- Reset (noise) seeds sent to the server: no-walk `seed * 100000 + trial`; from-handover `policy_wire.noise_seed(stem)` (crc32 of
  the stem, so the same episode always gets the same noise, independent of which subset was selected, and 81 `--self-test` can
  reproduce it). Collisions between the two families are possible in principle and harmless (different observations).
- `--teacher`: runs `ball_grasp.TwoHandBallGrasp(env, on_tick=...)` from the same handover instead of the server, with the same event
  tracker. Paired upper bound and teacher-verdict vs event-verdict agreement. No-walk rows carry the handover snapshot
  (`episode.handover_arrays` JSON form) so a failed VLA trial can be replayed with `--teacher` later.
- Evidence the path is exact today: `teacher_repro.log` (seed 101, one reset, snapshot, first observation, teacher) reproduced
  `episode_s101_t0000.npz` handover state bit-exactly, first image grey diff 0, first 726 actions max diff 0.0.
- `--from-handover <npz|dir>`: `TableEnv(scene="volleyball_walk")`, `env.restore_handover(episode.handover_snapshot(ep))`
  (`table_env.py:255-268`, `episode.py:100-104`) → `obs = env.observation()`. 82 asserts `ep["task"] == policy_wire.TASK` (m5; abort
  exit 2 otherwise). A folder is listed sorted, skipping `._*` and `*.part`; `--trials N` takes N evenly spaced files.
- Walk mode: not built (needs Step 2); the CLI has no walk flag yet.

### (6) Open-loop check (`83_open_loop_check.py`, SmolVLA env, no sim, no server) — gate recalibrated (m1)
- Imports 81 with `importlib` (pattern `69:48-50`) and uses its `SmolVLAPredictor`, so preprocessing and the run1 guards are 81's.
- Frames: from `--demos` (default `$T7/level1_walkup_v1`), F = `--frames` (default 20, `>= 2` else argparse error) demos evenly
  spaced over the sorted list, frame i at tick `round(1 + i * (T - 52) / (F - 1))`. Fixed noise per frame (CPU generator seeded
  with i), identical for every checkpoint. Only the members `state`, `actions`, `head_image` are read (each is decompressed in full
  by `np.load`; harmless at F ≤ 20).
- Per checkpoint (`--checkpoints <dir ...>` or `--run <run_dir>` = all `checkpoints/[0-9]*/pretrained_model`, `last` skipped, sorted
  by step) and per group (arms 0-13 rad, hands 14-25, waist 26 rad), predicted chunk vs recorded `actions[t:t+50]`:
  - `hold_mae` per frame = MAE(1-25) of repeating `actions[t-1]`; a frame is a **motion frame** for that group if
    `hold_mae >= HOLD_FLOOR = 0.005`, else a **still frame**.
  - `mae_all` (1-50) and `mae_exec` (1-25) over all frames; `mae_motion` and `hold_motion` (means over motion frames);
    `skill = 1 - mae_motion / hold_motion`; `mae_still` (model drift where the teacher holds still; reported only).
  - `spread` = std of the recorded target rows over all F x 50 rows, mean over the group's dims — one definition, used in the
    report only, not gated (it is the std across phases and barely constrains anything).
- Gate (diagnostic; does not block Stage 9), per group with at least `MIN_MOTION_FRAMES = 3` motion frames: `skill > 0` at the last
  checkpoint (beats hold-still where the teacher moves), and `mae_exec` ≤ 1.1x the previous checkpoint. A group with fewer motion
  frames prints `n/a (hold-still ~0 on k/F frames)` and is not gated. Reference on recorded data (`review_openloop_baselines.log`,
  default set): still frames arms 7/20, hands 16/20, waist 12/20 → motion frames 13 / 4 / 8, so all three groups are gated on the
  default set. On the Task 3 verify set (smoke, 4 frames): motion frames arms 1, hands 0, waist 3 → arms and hands `n/a`.
- Output `runs/l1w_open_loop_<label>.json` (device, threads, frames, ticks, per checkpoint per group: all fields above + gate) and a
  printed table.
- Device: default `cpu`; `--device mps` only after run1 (guard in `SmolVLAPredictor`, `--force-mps` to override). Run1 checkpoints:
  refused while lerobot-train runs (R4 not approved) unless `--allow-during-training`; with it, a checkpoint that is not ready
  (`policy_wire.checkpoint_ready`) is skipped with a printed reason, never read. Compare checkpoints only on the same device.

### (7) Tests
- `tests/test_unit_policy_wire.py` (Task 1; project `.venv` and sim env; numpy only): round trip over `socket.socketpair()` of uint8
  and float32 arrays bit-exact and shape-exact; header-only message; float64/object dtype refused on send and on receive; header
  length > `MAX_HEADER` refused without reading the body; **m4:** shapes `[-1]`, `["3"]`, `[2.0]`, `[True]`, rank 5, 9 arrays, and
  `[100000, 100000, 3]` uint8 each refused with `ValueError` before any body byte is read (the test sends only the header and checks
  that `recv` raises instead of blocking); EOF mid-message → ConnectionError; `check_observation` rejects 400x640x3 uint8,
  200x320x3 float32, state (26,), state with NaN, empty prompt, and accepts the valid one; `noise_seed("episode_s100_t0000")` is a
  fixed int and differs for t0001; `replay_chunk` at tick 0, at T-10, beyond T, negative tick → ValueError; `ReplayPredictor` on a
  tmp folder with a `._episode_x.npz` decoy and a `.part` file: lists only real episodes, unknown stem → ValueError; **F2:** with a
  synthetic episode (T = 60, random state/images, task = `TASK`) `infer` with the recorded frame 0 → `obs_state_err 0.0`,
  `obs_image_maxdiff 0`, `obs_prompt_ok True`; with frame 0 sent at tick 25 → `obs_state_err > 0`; with a wrong prompt →
  `obs_prompt_ok False`; at tick ≥ T → `recorded_tick None`; **guards (m2, R4):** `device_problem("mps", [run1 line], False)` → message,
  with `force_mps` → None, `("cpu", ...)` → None, no training → None; `training_output_dirs` parses the real run1 line from §0 and
  returns None for a line without `--output_dir`; `checkpoint_problem` for a non-existent path under a running output dir → refusal
  without touching the filesystem (monkeypatched `os.stat` raising if called), outside → None, unknown output dir → refusal; with
  `allow_during_training`: a tmp checkpoint missing `model.safetensors` → not ready, files with mtime now - 10 s → not ready, mtime
  now - 120 s → None; `serve` + `PolicyClient` on an ephemeral port in a thread with a ReplayPredictor: hello metadata, reset,
  infer returns the expected rows and `checks`, a bad image gets an `error` reply that raises `PolicyServerError` and the
  connection stays usable; the client started 0.5 s before the server thread binds still connects (`connect_wait_s`).
- `tests/test_unit_vla_eval.py` (Task 2; numpy only, no MuJoCo, no `policy_wire`): Wilson (8, 10) → (0.4902, 0.9433), (0, 5) →
  (0.0, 0.4345), (5, 5) → (0.5655, 1.0), (40, 50) → (0.6696, 0.8876) at 1e-4; tracker: lifted/carried ticks, latch exactly at the
  90th consecutive placing tick, a single false tick at 89 resets the streak, forbidden contact before the latch → done with reason
  `no_forbidden_contacts`, timeout with all 4 true but streak 60 → `placement_not_held_3s`, timeout after lift only →
  `ball_carried_to_basket`, furthest_event order; stall: range 0.009 in a failed trial → stalled, 0.011 → not, a success with a
  static hold → not stalled; no_progress: ball still for 150 ticks before timeout → True, moving 3 cm → False; chunk schedule:
  request ticks for exec 25 over 100 ticks = [0, 25, 50, 75], exec 0 or 51 → error; hand clip counts and leaves in-range float32
  values bit-identical; evenly spaced demo selection; verdict (0.5/0.9 → undertrained, 0.9/0.5 → generalization gap, 0.8/0.7 →
  consistent, **0.8/0.6 → consistent**, 0.8/0.59 → gap); `replay_gate_problems`: a passing row → `[]`; each of latch ≠ reference,
  `obs_state_err_max 1e-7`, `obs_image_maxdiff_tick0 3`, `obs_image_mad_max 4.1`, prompt false, requests off by one → exactly
  that field named.
- `tests/test_integration_vla_eval.py` (Task 2; sim env, `needs_g1ext`, integration; module fixture like
  `test_integration_volleyball_walk.py:17-26`):
  (a) no-walk contract: two TableEnvs seed 100; one runs 69's sequence by hand for 2 trials (reset → snapshot → observation), the
  other `vla_eval.no_walk_handovers(env, 2)`; snapshot state, variation and first image identical per trial.
  (b) skip unless the T7 is mounted: `vla_eval` no-walk at seed 101 trial 0 gives `handover_state` equal to
  `$T7/level1_walkup_v1/episode_s101_t0000.npz` (bit-exact; read-only).
  (c) end to end without the T7 and without torch (m6): load 69 with `importlib.util.spec_from_file_location` (the `69:48-50`
  pattern; `69` has a `__main__` guard at `:254`), `table_scene.write`, `TableEnv(seed=100)`, then `env.reset(randomize=True)` +
  `sixty_nine.record(env, 100, trial, tmp_path)` until one entry is `"saved"` (at most 3 trials) — the npz and its sidecar go to
  `tmp_path`, never the T7. Assert `sixty_nine.TASK == policy_wire.TASK` and the npz `task` equals it (m5). Start
  `81 --replay <tmp npz> --port 0 --verbose` as a subprocess with the **sim env** python, read `LISTENING <port>` from its stdout,
  run `vla_eval.run_policy_trial` against it plus `vla_eval.reference_replay` on the same npz: `replay_gate_problems == []`. After
  the client closes, the server log has `torch loaded: False`. The fixture terminates the subprocess in a `finally`
  (`terminate()`, `wait(10)`, `kill()` if still alive). Then `--teacher` on the same snapshot: teacher passed, event verdict passed.
  (d) stale-observation mutation (F2c), permanent: the same server and npz, with a test-only client wrapper that sends the
  tick-0 observation at every request; `replay_gate_problems` is non-empty and names `obs_state_err_max` (and `obs_image_mad_max`).
  Expected (a)-(d) ~30-40 s.
- `tests/test_unit_labview.py:89` (Task 2): add `labview.VLA_EVAL_PHASE_LABELS` to the tuple.
- 81's checkpoint mode cannot be pytest-tested (no pytest in the SmolVLA env): `81 --self-test` is the check (Task 1 verify), and
  the wire-equivalence check (§3) ties it to 82.

## 2. Tasks

Parallelism: the contract above (message format, `policy_wire` API, predictor interface, `SmolVLAPredictor` signature, `checks`
keys, `replay_gate_problems` thresholds) is fixed in this plan, so **Tasks 1, 2 and 3 are built in parallel with disjoint files**:
| Task | Owns (creates/edits only these) | Uses from other tasks (read-only, by contract) |
|---|---|---|
| 1 | `g1sim/policy_wire.py`, `scripts/81_smolvla_server.py`, `tests/test_unit_policy_wire.py` | nothing |
| 2 | `g1sim/vla_eval.py`, `scripts/82_vla_eval.py`, `g1sim/labview.py`, `tests/test_unit_vla_eval.py`, `tests/test_integration_vla_eval.py`, `tests/test_unit_labview.py` | `policy_wire.PolicyClient`, `TASK`, `noise_seed` (only in 82 and the integration test; `vla_eval.py` never imports `policy_wire` — client and prompt are passed in — so Task 2's unit tests run before Task 1 lands); 81 as a subprocess |
| 3 | `scripts/83_open_loop_check.py` | `SmolVLAPredictor` from 81; `policy_wire.checkpoint_ready`, `training_cmdlines` |
Real data dependencies only at integration: Task 2's integration test and the Step 4 gate need Task 1; Task 3's verify needs
Task 1. Integration order: Task 1 verify → Task 2 integration test + mutations → §3 gate → Task 3 verify → docs.

<task type="auto">
<name>Task 1 - Wire protocol, replay predictor with observation checks, run1 guards and SmolVLA policy server (81)</name>
<files>g1sim/policy_wire.py (new), scripts/81_smolvla_server.py (new), tests/test_unit_policy_wire.py (new)</files>
<action>
- `g1sim/policy_wire.py`: exactly the API, bounds and message format in decision (1). Imports: `json`, `os`, `pathlib`, `socket`,
  `struct`, `subprocess` (pgrep), `time`, `zlib`, numpy, `g1sim.episode` (for `load`) only. Module docstring states that it must stay
  importable in the SmolVLA env (no mujoco/torch). `recv` reads with a loop until the length is complete and validates header and
  array specs before reading the body (m4). `serve` answers `error` for: unknown message type, `check_observation` problems,
  predictor `ValueError`; it keeps the connection. With `verbose` it logs one line per request (tick, server_ms) and on client
  disconnect `torch loaded: <"torch" in sys.modules>`; it never prints per tick by default.
- `ReplayPredictor.infer` computes the `checks` dict of decision (1) against the recorded frame (`int16` differences for the image)
  and returns it with the chunk. It loads an episode's arrays at `reset` (not at start: 300 episodes in the training folder).
- `scripts/81_smolvla_server.py`, modes (mutually exclusive, one required):
  `--replay <npz|dir> [...]` → `ReplayPredictor` (no torch import anywhere on this path);
  `--checkpoint <dir>` → `SmolVLAPredictor(checkpoint, device, threads, force_mps, allow_during_training)` as in decision (1)/(2),
  with the init order there (guards first). `reset(episode, seed)` re-seeds a CPU `torch.Generator` and calls `policy.reset()`;
  `infer` maps `observation/head_image` → `observation.images.camera1`, `observation/state` → `observation.state`, `prompt` → task,
  draws noise (1, 50, 32), `predict_action_chunk`, `post`, returns `((50, 27) float32, {})`. `metadata`: checkpoint path, step
  (from the **resolved** folder name, int if numeric), device, chunk_size, action_dim, image_shape, lerobot version.
  Flags: `--host 127.0.0.1`, `--port 8765` (0 = ephemeral), `--device mps|cpu` (default mps, refused while training runs),
  `--force-mps`, `--allow-during-training`, `--threads`, `--verbose`,
  `--self-test <npz> [--self-test-frame 0] [--save-chunk <path.npy>]`: load; `reset(stem, policy_wire.noise_seed(stem))`, infer the
  recorded frame (built with `episode.policy_observation(frame, TASK)`, passed through `check_observation` first); reset with the
  same seed and infer again (assert identical on cpu, report on mps); assert shape (50, 27) and finite; send a 400x640 image through
  `check_observation` and print the rejection; print load s, chunk ms, the seed, and save the first chunk to `--save-chunk`; exit 0/1
  without opening a port.
  Docstring with both commands and the run1 rules (no `--device mps` and no run1 checkpoint while training runs).
- Tests: decision (7) list for `test_unit_policy_wire.py`.
</action>
<verify>
- `.venv/bin/python -m pytest tests/test_unit_policy_wire.py > runs/l1w_s4_t1_unit_project.log 2>&1` and
  `$PY -m pytest tests/test_unit_policy_wire.py > runs/l1w_s4_t1_unit_sim.log 2>&1` → all pass (read the summary line in the file).
- Import guard: `$SV -I -c "import sys; sys.path.insert(0, '<repo>'); import g1sim.policy_wire, g1sim.episode; print('ok')" > runs/l1w_s4_t1_import.log 2>&1` → ok.
- Replay server starts and is killed (sim env, no torch):
  ```
  $PY scripts/81_smolvla_server.py --replay "$T7/smoke_v1" --port 0 > runs/l1w_s4_t1_replay_server.log 2>&1 &
  SERVER=$!
  $PY -c "import socket,sys,time,re; sys.path.insert(0,'.'); from g1sim import policy_wire as w
  for _ in range(50):
      m = re.search(r'LISTENING (\d+)', open('runs/l1w_s4_t1_replay_server.log').read())
      if m: break
      time.sleep(0.2)
  c = w.PolicyClient('127.0.0.1', int(m.group(1))); print(c.metadata); c.close()" > runs/l1w_s4_t1_replay_client.log 2>&1
  kill $SERVER; wait $SERVER
  ```
  → hello with `mode replay` in the client log.
- MPS guard without any risk (training is running): `$SV scripts/81_smolvla_server.py --checkpoint /nonexistent/pretrained_model --device mps --self-test "$T7/smoke_v1/episode_s100_t0000.npz" > runs/l1w_s4_t1_mps_guard.log 2>&1`
  → exit 2 with the "lerobot-train is running" message (a broken guard would only reach "checkpoint not found"; nothing is loaded either way).
- Checkpoint mode on CPU only:
  `$SV scripts/81_smolvla_server.py --checkpoint "$SMOKE" --device cpu --threads 4 --self-test "$T7/smoke_v1/episode_s100_t0000.npz" --save-chunk runs/l1w_s4_selftest_chunk.npy > runs/l1w_s4_t1_selftest.log 2>&1`
  → exit 0; log shows chunk (50, 27), finite, same-seed identical True, 400x640 rejected, load ≈ 5 s, chunk ≈ 5 s, the seed;
  `runs/l1w_s4_selftest_chunk.npy` exists (used by the §3 wire check).
</verify>
<done>
- Unit tests green in both envs; the SmolVLA env imports `g1sim.policy_wire`; replay server starts, answers hello and is killed.
- MPS guard refuses while run1 trains; `--self-test` passes on the smoke checkpoint on CPU with the layout assertions active.
- Not run until run1 has finished (reported as open): `--device mps` latency < 1 s per chunk.
</done>
</task>

<task type="auto">
<name>Task 2 - Eval script (82) with event checks, reference replay, replay gate, held-out no-walk mode, teacher mode and Lab View task vla_eval</name>
<files>g1sim/vla_eval.py (new), scripts/82_vla_eval.py (new), g1sim/labview.py, tests/test_unit_vla_eval.py (new),
tests/test_integration_vla_eval.py (new), tests/test_unit_labview.py</files>
<action>
- `g1sim/vla_eval.py` (numpy + g1sim; MuJoCo only inside the functions that take an env; **no `policy_wire` import**):
  constants `STALL_WINDOW = 60`, `STALL_RANGE = 0.01`, `PROGRESS_WINDOW = 150`, `PROGRESS_MIN_M = 0.02`, `HOLD_TICKS = 90`,
  `HANDS = slice(14, 26)`, `EVENTS = ("handover", "lifted", "carried", "placing", "latched")`, `TRAIN_LOW = 0.6`, `GAP = 0.2`,
  `OBS_IMAGE_MAXDIFF_TICK0 = 2`, `OBS_IMAGE_MAD_MAX = 4.0`;
  `wilson(k, n, z=1.959964)`; `EventTracker(rest_xyz, hz=30, timeout_ticks)` with `update(tick, ball_xyz, basket_xyz, placement:
  dict, forbidden_pair | None)` → `done`, `passed`, `reason`, `furthest_event`, event ticks, `phase`, `no_progress`;
  `measure(env)`; `stalled(actions, passed)`; `request_ticks(total, exec_steps)`; `clip_hands(chunk)`; `pick_evenly(paths, n)`;
  `list_demos(path)`; `no_walk_handovers(env, n)` generator; `restore(env, ep)`;
  `run_policy_trial(env, client, prompt, tracker, exec_steps, episode_stem, seed, recorded_states=None, on_tick=None)` (takes a
  fresh `env.observation()` at every request tick and aggregates the server's `checks` into the row fields of decision (4));
  `reference_replay(ep, timeout_ticks)` (fresh TableEnv, per-tick recorded actions, closes its env); `replay_gate_problems(row,
  reference)`; `run_teacher_trial(env, tracker, on_tick=None)`; `summarize(rows)`; `verdict(train_summary, heldout_summary)` with
  the rounding of decision (4).
- `scripts/82_vla_eval.py`: modes `--no-walkup` (with `--seed`, refused ≥ 100) or `--from-handover <npz|dir>`; `--trials N`;
  `--teacher` (no server); `--server 127.0.0.1:8765`, `--server-timeout-s 120`, `--connect-wait-s 60`; `--exec-steps 25`;
  `--timeout-s 45`; `--train-variations N` + `--demos <dir>`; `--expect-chunk <npy>` (from-handover only: compare the first chunk
  of trial 0 with the file, print `WIRE CHECK max_abs_diff=<x> tol=0.01 PASS|FAIL`); `--name`; `--no-labview`.
  Checks the hello (`action_dim == 27`, `chunk_size >= exec_steps`, `image_shape == [200, 320, 3]`) before trial 0; prompt =
  `policy_wire.TASK` (from-handover: assert equal to the npz `task`); reset seeds as decision (5). In from-handover mode with a
  `mode == "replay"` server it runs `reference_replay` per trial and `replay_gate_problems`, prints `REPLAY GATE PASS` or
  `REPLAY GATE FAIL <trial>: <problems>`. Report `runs/vla_eval_<name>.json` rewritten after every trial (`labview.write_atomic`)
  with header (git, argv, mode, seed, server hello, exec_steps, timeout, `latency_modelled: false`), rows, summary per section,
  verdict, `replay_gate`, `wire_check`. Final line `RESULT {json summary}`.
  Exit codes: 0 = run complete and no gate/check failed (any success rate when no replay server and no `--expect-chunk`);
  1 = run complete but the replay gate or the wire check failed; 2 = aborted (server error, contract mismatch, unreadable npz,
  task mismatch) with the partial report written and `"aborted": reason`.
- `g1sim/labview.py` (additive only): `VLA_EVAL_PHASE_LABELS = {"vla_start": "VLA in control, before the lift", "vla_lifted":
  "Ball lifted 6 cm", "vla_carried": "Ball carried over the basket", "vla_placing": "Ball placed, 3 s check running"}`;
  `PHASE_ORDERS["vla_eval"]`; `phase_label` also looks in the new table (before the WALK fallback); `_REASONS` gets
  `policy_output_invalid` ("Policy sent NaN or Inf actions", "invalid output"), `placement_not_held_3s` ("All placement checks held,
  but not for 3 s before the timeout", "not held 3 s"), `handover_failed` ("No valid start could be drawn", "no start"). No key renamed.
- Tests: decision (7) lists for `test_unit_vla_eval.py` and `test_integration_vla_eval.py`; one tuple entry in `test_unit_labview.py:89`.
</action>
<verify>
- Before Task 1 lands: `.venv/bin/python -m pytest tests/test_unit_vla_eval.py tests/test_unit_labview.py > runs/l1w_s4_t2_unit_project.log 2>&1` → all pass.
- After Task 1 lands: `$PY -m pytest tests/test_unit_vla_eval.py tests/test_unit_labview.py tests/test_integration_vla_eval.py > runs/l1w_s4_t2_sim.log 2>&1`
  → all pass; (b) runs only with the T7 mounted (it is). The integration fixture leaves no 81 process behind:
  `pgrep -fl 81_smolvla_server.py > runs/l1w_s4_t2_leftover.log 2>&1` → empty (exit 1).
- Mutations (integration step, after Tasks 1 + 2 are merged; revert each, re-run green):
  (i) `HOLD_TICKS = 89` → the latch unit test fails by name; (ii) `replay_chunk` returns `actions[tick+1:...]` → integration (c)
  fails on `replay_state_error` and the reference latch; (iii) in `run_policy_trial` take the observation once before the loop and
  reuse it → integration (c) fails on `obs_state_err_max`. Logs `runs/l1w_s4_mut_<i>.log`.
- Teacher mode, paired upper bound and event-check agreement:
  `$PY scripts/82_vla_eval.py --from-handover "$T7/smoke_v1" --trials 5 --teacher --name s4_teacher_smoke --no-labview > runs/l1w_s4_t2_teacher_smoke.log 2>&1`
  → 5/5, teacher verdict and event verdict agree on every trial.
  `$PY scripts/82_vla_eval.py --no-walkup --seed 0 --trials 5 --teacher --name s4_teacher_seed0 > runs/l1w_s4_t2_teacher_seed0.log 2>&1`
  → completes, both verdicts agree per trial, Lab View record `vla_eval` exists under `runs/labview/records/`.
- `$PY scripts/82_vla_eval.py --no-walkup --seed 100 > runs/l1w_s4_t2_seed_error.log 2>&1` → argparse seed error, exit 2.
- Full gate output goes to the log; read the `RESULT` line from the file.
</verify>
<done>
- Unit and integration tests green (project env + sim env), labview uniqueness test covers the new table, no server left running.
- The stale-observation test (d) and mutation (iii) fail the replay gate; mutations (i)-(ii) fail their tests.
- Teacher mode: 5/5 on the smoke handovers with event checks agreeing; seed-0 no-walk teacher run completes with both verdicts agreeing.
- The no-walk path reproduces 69's handover (integration (a), and (b) bit-exact against the recorded s101_t0000).
</done>
</task>

<task type="auto">
<name>Task 3 - Open-loop check (83) with calibrated gate and run1 refusal</name>
<files>scripts/83_open_loop_check.py (new)</files>
<action>
- As decision (6). SmolVLA env. Imports 81 via `importlib.util.spec_from_file_location` and builds one `SmolVLAPredictor` per
  checkpoint (released before the next: `del` + `gc.collect()`; one model in memory at a time).
- Flags: `--checkpoints <dir ...>` | `--run <run_dir>`; `--demos <dir>` (default `$T7/level1_walkup_v1`); `--frames 20` (≥ 2);
  `--device cpu` (default) | `mps`; `--force-mps`; `--allow-during-training`; `--threads`; `--label` (output name).
- Before anything is listed or loaded: for every checkpoint path (and for `--run`, the run dir itself) call
  `policy_wire.checkpoint_problem(path, policy_wire.training_cmdlines(), allow_during_training, time.time())`. A refusal → exit 2
  with the message, nothing read. With `--allow-during-training`, a not-ready checkpoint is skipped with its reason. The MPS guard
  comes from `SmolVLAPredictor`.
- Per checkpoint and group: the fields and gate of decision (6) (`HOLD_FLOOR = 0.005`, `MIN_MOTION_FRAMES = 3`, skill on motion
  frames, `mae_still` reported, `spread` reported only, `n/a` groups). Prints the table and the gate line; writes the JSON.
</action>
<verify>
- (after Task 1) `$SV scripts/83_open_loop_check.py --checkpoints "$SMOKE" --demos "$T7/smoke_v1" --frames 4 --device cpu --threads 4 --label s4_smoke > runs/l1w_s4_t3_open_loop.log 2>&1`
  → exit 0, table with arms/hands/waist rows: arms and hands `n/a (hold-still ~0 on 3/4 and 4/4 frames)`, waist with a skill value
  (3 motion frames); `runs/l1w_open_loop_s4_smoke.json` written; no gate value asserted (a 200-step model).
- `$SV scripts/83_open_loop_check.py --frames 1 --checkpoints "$SMOKE" > runs/l1w_s4_t3_frames1.log 2>&1` → argparse error, exit 2.
- Run1 refusal without touching run1 (path does not exist; the guard decides from the path):
  `$SV scripts/83_open_loop_check.py --checkpoints ~/dev/g1-ext/smolvla-runs/run1/checkpoints/999999/pretrained_model --device cpu > runs/l1w_s4_t3_run1_guard.log 2>&1`
  → exit 2 with the "inside a running training's output dir" message (a broken guard would only reach "not found").
- MPS refusal: `$SV scripts/83_open_loop_check.py --checkpoints "$SMOKE" --device mps --frames 2 > runs/l1w_s4_t3_mps_guard.log 2>&1`
  → exit 2, refusal message from `SmolVLAPredictor`, no "Loading" line in the log. Run only after the 81 MPS guard check in
  Task 1 passed (same guard code), so a broken guard cannot reach MPS here.
</verify>
<done>
- 83 runs on the smoke checkpoint on CPU and writes the JSON with motion/still split and n/a groups; refuses run1 while training
  and MPS while training. R4 stays open: run1 checkpoints are checked after run1 unless the user approves CPU checks during it.
</done>
</task>

## 3. Step 4 gate (after Tasks 1 + 2; the plan's done criterion "replay 5/5")
Replay gate, server in the **SmolVLA env** so the env boundary is crossed; one shell block:
```
$SV scripts/81_smolvla_server.py --replay "$T7/smoke_v1" "$T7/level1_walkup_v1" --port 8765 > runs/l1w_s4_replay_server.log 2>&1 &
SERVER=$!
$PY scripts/82_vla_eval.py --from-handover "$T7/smoke_v1" --trials 5 --name s4_replay_smoke > runs/l1w_s4_replay_smoke.log 2>&1; echo "exit $?" >> runs/l1w_s4_replay_smoke.log
$PY scripts/82_vla_eval.py --from-handover "$T7/level1_walkup_v1" --trials 5 --name s4_replay_train > runs/l1w_s4_replay_train.log 2>&1; echo "exit $?" >> runs/l1w_s4_replay_train.log
kill $SERVER; wait $SERVER
pgrep -fl 81_smolvla_server.py > runs/l1w_s4_leftover.log 2>&1   # must be empty
```
Pass (both logs): `REPLAY GATE PASS`, `exit 0`, hello mode "replay", 5/5 latched, and per trial (read from the report JSON):
latch tick and every event tick equal to the reference replay of the same npz; `replay_state_error == 0.0`;
`obs_state_err_max == 0.0`; `obs_image_maxdiff_tick0 <= 2`; `obs_image_mad_max <= 4.0`; prompt ok; requests = `ceil(latch/25)`;
not stalled. (Stems are unique across the two folders because they carry seed and trial.)

Wire equivalence on a real checkpoint, CPU (F2b), plus the no-walk smoke run; one shell block, after Task 1's self-test wrote
`runs/l1w_s4_selftest_chunk.npy`:
```
$SV scripts/81_smolvla_server.py --checkpoint "$SMOKE" --device cpu --threads 4 --port 8766 > runs/l1w_s4_ckpt_server.log 2>&1 &
SERVER=$!
$PY scripts/82_vla_eval.py --from-handover "$T7/smoke_v1/episode_s100_t0000.npz" --trials 1 --timeout-s 2 --server 127.0.0.1:8766 --expect-chunk runs/l1w_s4_selftest_chunk.npy --name s4_wire_check --no-labview > runs/l1w_s4_wire_check.log 2>&1; echo "exit $?" >> runs/l1w_s4_wire_check.log
$PY scripts/82_vla_eval.py --no-walkup --seed 0 --trials 1 --timeout-s 4 --server 127.0.0.1:8766 --name s4_ckpt_smoke > runs/l1w_s4_ckpt_smoke.log 2>&1; echo "exit $?" >> runs/l1w_s4_ckpt_smoke.log
kill $SERVER; wait $SERVER
pgrep -fl 81_smolvla_server.py > runs/l1w_s4_leftover2.log 2>&1   # must be empty
```
Pass: wire check `WIRE CHECK ... PASS`, exit 0, `max_abs_diff` expected 0.0 (same CPU, same 4 threads, same noise seed
`noise_seed("episode_s100_t0000")`, tick-0 image diff 0 on this demo) and in any case ≤ 0.01, far below the 0.79 spread between
noise draws. It proves that what 82 sends over the wire at tick 0 (image, state, prompt) yields the same chunk as the recorded frame
fed directly. The no-walk smoke run completes in < 1 min (5 requests x ~5 s), trial fails (a 200-step model; expected), latency
stats present, exit 0. Not run in Step 4 (open until run1 ends): the MPS latency check and the Stage 9 eval.

## 4. Risks
- **MPS contention with run1.** Mitigation: CPU-only checks; the MPS guard lives in `SmolVLAPredictor` (covers 81 and 83); the guard
  checks run on a non-existent checkpoint path so a broken guard cannot load anything. Stage 9 preflight runs the MPS self-test first.
- **Reading run1 while it writes (R4 open).** 83/81 refuse run1 checkpoints while lerobot-train runs; with the user's OK and
  `--allow-during-training`, only complete dirs older than 60 s are read.
- **Latency not modelled.** Sim success overstates the real robot (200 ms ≈ 6 ticks). Mitigation: latency in ticks; `--delay-ticks` in Stage 9.
- **Observation drift between 82 and training data** (F2). Mitigation: the replay gate checks state bit-equal and the image at noise
  level per request tick; the wire check ties the chunk through the wire to the direct path; test (d) proves a stale observation fails.
  Residual: the image check cannot tell a one-tick-late image from noise by itself; the bit-equal state check catches that case.
- **Checkpoint config inherits smolvla_base input_features.** 81 asserts the stats layout.
- **HF cache/offline:** set in code before imports.
- **Event semantics differ from the teacher's checks.** Teacher mode measures both on the same handovers; disagreement blocks the gate.
- **Stall metric** uncalibrated on the policy side (m3): `no_progress` reported next to it; re-calibrate on the first run1 eval.
- **Non-determinism on MPS:** paired comparisons are only exact on CPU; the report states the device.
- **R11:** 82 and integration (c) call `table_scene.write`, which rewrites the scene XML any running 67/69 uses. Never run them while
  the factory records. Training is unaffected.
- **T7 unplugged mid-run:** abort with exit 2, partial report kept; nothing is written to the T7.
- **Hands clipped, arms not:** the real adapter must clip arms/waist (safety layer), out of scope here.
- **Training-variation eval uses seen data** by design; held-out = seed 0 no-walk only (R6: held-out frame set for 83 later).
- **Wire robustness:** header and array bounds, dtype whitelist, partial reads, EOF are unit-tested; localhost by default.

## 5. Review lenses walked (`~/Documents/ClaudesOrdner/_harness/templates/project/.planning/REVIEW-LENSES.md`; not in this repo)
1. Guard coverage: observation checks run in `serve` before any predictor; the replay predictor's `checks` cover the content; the
   MPS/run1 guards sit in `SmolVLAPredictor`, which both 81 and 83 use; hand clipping in the client covers replay and SmolVLA.
2. Newly reachable: `phase_label` finds a 4th table → uniqueness test extended; Lab View gets task "vla_eval"; 82 and the
   integration test write the scene XML (R11); a localhost TCP port opens; integration (c) imports 69 (and through it 67, mink) —
   69 has a `__main__` guard, its module-level `OUT` on the T7 is never used because `record` gets `tmp_path`.
3. Changed meaning: `ball_carried_to_basket` = height + footprint; `stalled` replaces plan E's definition; 82's exit 1 is new
   (gate/wire failure), documented in COMMANDS.md.
4. Primitive can express it: `restore_handover` in a fresh TableEnv reproduces state bit-exactly (10/10); the image check needs a
   noise-level bound, not equality (measured); NaN actions travel as raw float32 and are caught by the finite check.
5. Shared consumers: `labview.py` changed additively; `episode`, `ball_grasp`, `TableEnv`, 69 are read, not changed; `TASK` is
   duplicated between 69 and `policy_wire` on purpose (69 is frozen) and pinned equal by integration (c); `g1sim/__init__.py` must
   stay import-light (import guard in Task 1 verify).

## 6. Effort and order
Task 1 ~2.5 h, Task 2 ~4 h, Task 3 ~1.5 h, built in parallel; integration + mutations + gate ~1 h. Docs after the gate (integration
step owns them): PROGRESS LOG + NEXT STEP row 4, `docs/LEARNING_PATH.md` §10 (server/client, chunking, Wilson interval, open-loop vs
closed-loop, why the replay gate checks the observation), COMMANDS.md (81/82/83 commands, exit codes, run1 guards),
`.planning/level1-walkup/PLAN.md` section M (transport, stall definition, latency note, no-walk mode, replay gate). Files plus
commit message go to the user, who commits and pushes himself.

## 7. Rulings
Accepted by the user (plan recommendations): **R1** stdlib TCP now; **R2** latency unmodelled in Step 4, reported in ticks;
**R3** verdict thresholds 0.6 / 0.2; **R5** hand clipping to [0, 1] in the eval; **R6** held-out open-loop frame set later
(`--save-episodes` in 82's teacher mode on seed 1).
Open:
- **R4 open-loop on run1 checkpoints during training** (CPU, `--threads 2`, ~2 min per checkpoint, reads only complete dirs older
  than 60 s). Recommendation: yes, it catches a bad training trend before the 45k steps finish; but it reads run1's folder, so only
  with your OK. Until then 83 refuses and nothing in Step 4 runs it.

## Revision 2 (review response, `.planning/reviews/level1-step4-plan.md`)
| Finding | Change |
|---|---|
| F1 latch band fails 3/10 gate demos | Absolute 24-26 s band dropped. `vla_eval.reference_replay` (fresh TableEnv, per-tick recorded actions, no wire, no `replay_chunk`) gives the reference; the gate requires latch tick and every event tick equal (decision (4), §3). Probe text corrected to 703-754 ticks, -23..+28 vs T. Tick convention stated (decision (1)). |
| F2 observation half unchecked | (a) `ReplayPredictor.infer` returns `checks` (state err, image maxdiff/MAD, prompt vs npz task); 82 aggregates them; `replay_gate_problems` requires state 0.0, tick-0 image ≤ 2, later MAD ≤ 4.0 (measured 3.03-3.39, `plan2_obs_probe.log`), prompt ok. (b) Wire check: 81 `--self-test --save-chunk` on frame 0 with `noise_seed(stem)` vs 82 `--from-handover ... --expect-chunk` through the CPU smoke server, tol 0.01, expected 0.0 (§3). (c) Integration test (d) sends the tick-0 observation every request and must fail the gate; mutation (iii) does the same in the real code. |
| m1 83 gate | Motion/still frames per group (`HOLD_FLOOR` 0.005); skill on motion frames > 0; groups with < 3 motion frames `n/a`; spread one definition, report only; the 0.104 window-std comparison removed. |
| m2 MPS guard | `device_problem` in `policy_wire`, called first in `SmolVLAPredictor` (81 + 83), `--force-mps`; checked on a non-existent checkpoint path. |
| m3 stall | `no_progress` diagnostic added; `STALL_RANGE` re-calibrated on the first run1 eval; both diagnostics only. |
| m4 array bounds | `recv` validates dtype, rank ≤ 4, non-negative int shapes, ≤ 8 arrays, total bytes ≤ `MAX_BYTES`, header ≤ `MAX_HEADER`, before the body; unit tests. |
| m5 prompt | `policy_wire.TASK` used by 81/82/83; 82 asserts npz task equality; integration (c) pins it equal to 69's `TASK` (69 not edited: frozen recorder). |
| m6 recorder copy | Integration (c) loads 69 with importlib and calls `record(env, 100, trial, tmp_path)`. |
| Nit `rest` line | `ball_grasp.py:194`. |
| Nit step "last" | Checkpoint path resolved before reading the step. |
| Nit `--frames 1`, np.load | `--frames >= 2`; the text says each member is decompressed in full. |
| Nit verdict float boundary | Rounded comparisons; unit test 0.8/0.6 → consistent. |
| Nit reset seeds | From-handover uses `noise_seed(stem)` (crc32); no-walk `seed*100000+trial`; overlap documented as harmless. |
| Nit servers left running | Every block ends with `kill $SERVER; wait $SERVER` and a `pgrep` leftover check; the client's `connect_wait_s` replaces shell polling; integration fixture terminates its subprocess. |
| Nit young run1 checkpoints (R4) | `checkpoint_ready` (complete files, newest ≥ 60 s old), used only with `--allow-during-training`; refusal by default (R4 not approved). |
| Also | Disjoint file ownership table (§2); `vla_eval.py` does not import `policy_wire`, so Task 2's unit tests are independent of Task 1; the `replay_chunk` mutation moved to the integration step (it edits a Task 1 file). |
