# Review: Level 1 Step 4 build (Tasks 1-3), finished review

Reviewer: independent (did not write the code, no author reasoning seen). Date 2026-10-15. Model: opus.
Scope: `g1sim/policy_wire.py`, `scripts/81_smolvla_server.py`, `tests/test_unit_policy_wire.py`, `g1sim/vla_eval.py`,
`scripts/82_vla_eval.py`, `g1sim/labview.py` (diff), `tests/test_unit_vla_eval.py`, `tests/test_integration_vla_eval.py`,
`tests/test_unit_labview.py` (diff), `scripts/83_open_loop_check.py`, against `.planning/level1-walkup/STEP4-PLAN.md` rev 2.
Probes and logs: `/private/tmp/claude-501/-Users-finneganadam-Documents-Projekt-Tim-and-Lukas-Cursor-Software-Engineering-SAP-Robot-Project-First-Ideas/f0ff7f6a-b54c-4e4d-a5d8-61eb8f19683e/scratchpad/step4/review_code/`.
Hard rules kept: CPU only, no MPS, run1 not touched, no lerobot-train, nothing written to the T7, my one server (port 8799)
killed (`wait` 143, `pgrep` exit 1 afterwards).

## Verdict 1: did the code do what the plan said? YES, with one contract deviation (F1)
- Wire format, API names, hello/reset/infer/error types, `checks` keys, tick convention, `TASK`, `noise_seed`,
  `replay_chunk`, ReplayPredictor (stem index, `._*`/`.part` skipped, load at reset), `serve`/`PolicyClient`
  (`connect_wait_s`), and the run1 guards match decision (1). No pickle anywhere: `np.load(..., allow_pickle=False)`,
  JSON headers, raw bytes with a uint8/float32 whitelist.
- `SmolVLAPredictor` init order matches (guards, then path/layout, then HF env, then torch/lerobot imports, then load);
  `predict_action_chunk` is `@torch.no_grad()` in lerobot 0.6.1 (checked in site-packages, `modeling_smolvla.py:231`).
- Observation contract against `episode.py` and 69: keys `episode.POLICY_KEYS` + `prompt`; 82 builds the payload with
  `episode.policy_observation` like 81's self-test; image 200x320x3 uint8 equals the dataset feature
  `observation.images.camera1` [200, 320, 3] (`meta/info.json`); the prompt equals the dataset task
  (`tasks.parquet`: `['put the volleyball in the basket']`) and the npz task of all 305 T7 demos (`hand_range.log`).
  The no-walk sequence is 69's (reset, snapshot, observation). I also checked that the handover of trial k does not depend
  on what ran in trial k-1 (150 wild arm actions vs none: state, handover JSON, first image and state all bit-equal,
  `reset_probe.log`), so teacher/VLA trials of the same seed are paired as the plan claims.
- Event checks against `ball_grasp.py`: `MIN_LIFT`, the footprint 0.145, `placement_checks` (same 4 keys, same order),
  `FORBIDDEN`, latch at 90 consecutive all-true ticks, the failure order of `ball_grasp.py:259-264`. Constants are copies,
  pinned by integration (a).
- Clipping is a no-op on recorded data as claimed: hand actions over all 305 demos are in [0.0, 0.3] (`hand_range.log`).
- Wilson numbers re-derived by hand for (8, 10) = (0.4902, 0.9433); denominators as planned (`vla_success` over trials that
  reached the handover, `end_to_end` over all; handover failures only in no-walk mode).
- Deviations, all acceptable: `run_policy_trial(first_obs=...)` (needed so the handover render is not repeated, which would
  move `noise_rng`); exit 1 also on teacher disagreement (documented in the 82 docstring); 83 also re-checks each checkpoint
  in the loop.

## Verdict 2: was the plan right? YES
The gate design (a reference replay without the wire, per-request observation checks in the replay server, a CPU wire check
against the self-test chunk, a permanent stale-observation test and three mutations) catches the failure modes that matter
for the handover: an observation that is stale or late, an off-by-one in the chunk, and a broken latch. The image noise
bound (MAD <= 4.0) cannot catch every one-tick-late image on its own, but the bit-equal state check covers that (plan
risk section). The plan notes below are minor.

## Gate results, re-derived
- Replay gate, smoke_v1: I **re-ran** it myself (81 replay in the SmolVLA env on port 8799, 82 in the sim env,
  `--name s4_review_replay_smoke --no-labview`): `REPLAY GATE PASS`, exit 0, 5/5, the same latch ticks 754/753/736/711/721,
  requests 31/31/30/29/29, the server held one connection for all 5 trials and logged `torch loaded: False`
  (`review_replay_smoke.log`, `review_replay_server.log`; report `runs/vla_eval_s4_review_replay_smoke.json`, gitignored).
- Both gate JSONs (smoke + train) re-derived with my own script (`rederive.log`): on all 10 rows, latch and lifted/carried/
  placing ticks equal the reference; `requests == ceil(latch/25)`; `obs_checked == len(range(0, min(latch, T), 25))`;
  `replay_state_error`, `obs_state_err_max` 0.0; tick-0 image diff 0; MAD 3.17-3.39; hand clips 0; not stalled. Agrees
  with `runs/l1w_s4_gate_json_check.log`.
- Wire check: `max_abs_diff 0.0`, hello mode smolvla, step 200, CPU, 4 threads. Smoke no-walk run: 1 trial, exit 0, ended at
  tick 61 on `no_forbidden_contacts` (`rh_right_pinky_proximal_link` touched `basket`), latency stats present.
- Stale test (d) and mutations (i)-(iii): green/red as planned in `runs/l1w_s4_gate_tests_sim.log` and
  `runs/l1w_s4_gate_mut_*.log` (mutation iii names `obs_state_err_max 1.25`, mutation ii names `latch_tick 753 vs 754`).
- Logs postdate the last code edits (newest code 11:34:50; gate tests 11:37, gate runs 11:38-11:39).

## Findings

### F1 (Minor): one message kills the server (bounds check raises OverflowError instead of ValueError)
- Where: `g1sim/policy_wire.py:66` `int(np.prod(shape, dtype=np.int64))`. A shape entry above int64 (e.g. `[2**70]`) passes the
  `type(n) is int and n >= 0` check at `:64`, then `np.prod` raises `OverflowError`. `_handle` (`:206-212`) only catches
  `ConnectionError`/`ValueError` and `serve` (`:254`) only catches `OSError`, so the exception ends `serve` and the 81 process.
- Reproduced: `wire_probe.log`: `huge_int -> OverflowError`, live server `server thread alive: False`. Products that overflow
  int64 silently wrap (`[2**32, 2**32]` gives 0 bytes, `[2**62, 3]` a negative count); those happen to end in a
  `ValueError` later (reshape or negative `bytearray`), so they are refused only by luck.
- Impact: low in practice (localhost, our own client), but it breaks the m4 contract ("any violation -> ValueError, nothing
  allocated") and the plan's "the connection stays usable". It does not affect any gate result.
- Fix: bound each entry (e.g. `n <= MAX_BYTES`) before the product, or compute with Python ints (`math.prod`). Add `[2**70]`
  and `[2**32, 2**32]` to `test_recv_refuses_bad_specs_before_the_body`.

### F2 (Nit): any non-ValueError from the predictor also ends the server
- `policy_wire.py:229-239`: a `RuntimeError` from `predictor.infer` (e.g. a torch error in SmolVLA mode) is not caught, so 81
  exits; 82 then aborts with exit 2 and a partial report. Reproduced with a stub predictor (`wire_probe2.log`,
  `predictor_runtimeerror ... server alive: False`). Within the plan (it only asks for ValueError), but for Stage 9 runs of
  60 trials an `error` reply plus a server-side log line would be kinder. All other malformed inputs I tried (non-string
  name, bad UTF-8, a JSON list header, float seed, list prompt) got an `error` reply and the server stayed up.

### F3 (Medium, outside Step 4 code, blocks a "tests green" claim): `test_unit_labview.py::test_server_endpoints` fails
- Every Step 4 pytest log shows it (`runs/l1w_s4_t2_unit_project.log`, `l1w_s4_t2_sim.log`, `l1w_s4_gate_tests_project.log`
  `1 failed, 85 passed`, `l1w_s4_gate_tests_sim.log` `1 failed, 90 passed`), and my re-run gives the same
  (`unit_rerun.log`).
- Cause: the uncommitted `scripts/65_labview.py` change (Step 7 Training tab, modified 10:20, not in the Step 4 file list)
  removed the placeholder "Training metrics appear here" that `tests/test_unit_labview.py:170` asserts. Not a Step 4 defect,
  but Task 2's done line "Unit and integration tests green" is not literally true, and a commit of the current tree has a
  red suite. Update the assertion (or the page) in the Step 7 change, or report the failure in the Step 4 report.

### F4 (Minor, plan semantics): a placement without a lift counts as success
- `g1sim/vla_eval.py:101-103` latches on 90 placing ticks whatever `ticks["lifted"]`/`["carried"]` are; the teacher's own
  verdict needs `ball_lifted` and `ball_carried_to_basket`, and the eval's own failure order puts `ball_lifted` first.
  Reproduced at unit level: a ball never lifted but placed 90 ticks gives `passed True`, events lifted/carried `None`
  (`latch_nolift.log`). Reachable in principle: the stand is 0.08 m and the basket wall 0.10 m high
  (`table_scene.py:57,62`), so a ball carried or swept at under 6 cm above rest can still drop into the basket. Not hit by any
  replay or teacher run. Either require `lifted` before the latch, or flag such rows (`latched_without_lift`) in the summary,
  so the VLA success rate is not higher than what the teacher would accept.

### F5 (Nit, plan): the run1 guard refuses everything during a resumed run
- `train_smolvla.sh resume` runs `lerobot-train --config_path=... --resume=true` with no `--output_dir`, so
  `training_output_dirs` returns None and `checkpoint_problem` refuses every checkpoint, even the smoke one, during the
  planned second sitting of run1. That is the safe side and stated in the plan's assumptions; worth one line in COMMANDS.md.
  Optionally derive the run dir from `--config_path` (`<run>/checkpoints/last/pretrained_model/train_config.json`).

### F6 (Nit, plan): teacher agreement is only tested on passes
- 10/10 teacher trials passed (`l1w_s4_t2_teacher_smoke.log`, `l1w_s4_t2_teacher_seed0.log`), so the event checks and the
  teacher verdict have only been compared on successes; the failure ordering is covered only by unit tests. Fine for Step 4.
  Keep it in mind when reading the Stage 9 failure table.

### F7 (Nit): an untracked `.DS_Store` at the repo root
- It is not in `.gitignore`; keep it out of the commit.

## Checks not run (named, they do not block Step 4's done criterion, which the plan scopes to replay 5/5 + wire check)
- MPS latency < 1 s per chunk and anything on run1 checkpoints: forbidden while run1 trains (plan §0). Open until Stage 9.
- I did not re-run the integration tests, the mutations, the train-set replay gate, the wire check or 83; I read their logs
  and re-derived the report JSONs. I re-ran the smoke replay gate and the three unit test files.

## Overall: GO for Step 4
Implementation fidelity: yes (F1 is a small contract gap that does not touch any gate result). Plan correctness: yes.
Before the commit: fix F1 (a few lines + 2 test cases), and resolve or report F3 (red labview test from the out-of-scope
65 change). F4 should be decided before the first run1 eval (Stage 9), because it changes what counts as a success.
