# Review: Level 1 Step 5 recorder, data contract lens (2026-10-14)

Stage: finished review. Reviewer did not author the work and saw no author reasoning.
Under review: `scripts/69_volleyball_demo_factory.py` (new, mtime 17:02:50) and `g1sim/episode.py` (diff: `handover_arrays`,
`handover_snapshot`, `EpisodeRecorder.save(extra=...)`; **changed again during this review at 17:09:49**: `save()` now writes
through a new `write_verified()` (BytesIO, `.part` temp, fsync, size check, `os.replace`). Both versions were reviewed.)
Spec: PLAN.md D, I, K, Stage 6 task; PROGRESS.md "24-hour plan" (no-walk v1). Lens: recorded data contract.

## Reviewer disclosure (read first)

While testing the unmounted-SSD guard I ran `69 --seed 100 --demos 1` with the default `--out`. My first `ls /Volumes` showed no T7;
by the time that command ran (17:06:44) the T7 had been mounted, so the guard correctly let it through and it **recorded one demo into
`/Volumes/T7/.../g1-demos/smoke_v1/` (episode_s100_t0000.npz/.json, factory_s100.json)**. The author's own smoke run had started at
17:06:20 with the same seed and `--name smoke_v1`, so two processes were writing the same file names at the same time.
Consequence: the "0-byte episode right after the T7 was plugged in" that the `write_verified` docstring names as its reason was most
likely caused by my concurrent writer (`np.savez_compressed(path)` truncates the same path), **not** by the exFAT/fskit driver.
`write_verified` is fine to keep (atomic temp + rename is good practice), but the claim that the T7 driver produces 0-byte files is
unverified and should not go into docs or PROGRESS as a fact. The folder was cleared and the author's 5-demo smoke ran
afterwards (17:10-17:14) with no interference; that is the set I verified below. I wrote nothing else to the T7.

## Evidence I re-derived (scripts and outputs in the session scratchpad `review/`)

1. **Frame alignment, no off-by-one (all frames).** `align_check.py`: fresh `TableEnv(seed=12345)`, an unrelated randomized
   reset, then `restore_handover(handover_snapshot(ep))`, then per t: `observation()` -> compare with frame t -> `apply(actions[t])`.
   Dry demo: 726/726 images max diff 0 grey levels, states max diff 0.0. Shifted by one (obs vs frame t+1): state median diff
   0.0058 rad, image mean diff 4.6 grey levels, so the exact match is not an artefact. Observation-before-action and
   action-actually-commanded both hold (69:77-87 with ball_grasp.py:352-355, `on_tick` after `env.apply`).
2. **All 5 T7 smoke demos** (`smoke_check.py`, one fresh env, restored in reverse order t4..t0, so trials > 0 and restore after a
   previous episode are covered): each 726 frames, `episode.validate == []`, success True, sidecar has all checked keys, sidecar
   `validate == []`, timestep 1/300, seed 100, closed-loop replay image max diff 0 and state max diff 0.0. 5 distinct variations.
   Waist tracking 0.004-0.017 rad (< 0.05).
3. **Trim rule exact.** Tick count from the teacher constants (ball_grasp.py:87-90, phase calls 196-238): 696 ticks through
   `face_front` + 30 = 726, equal to every recorded episode. In the npz the last waist-action change is at index 695 (last
   face_front tick) and `actions[-31:]` are identical (final face_front tick + 30 settle ticks). Implementation 69:80-87:
   settle ticks 1..30 recorded, tick 31+ returns early; the 3 s placement check is not recorded. At the end of the recording the
   restored env already passes all 4 `placement_checks`. (D's older "30 ticks after back_off" is superseded by I.)
4. **First frame = snapshot.** `snapshot_handover()` is taken after `reset()` (whose returned observation already drew noise) and
   before `pending = [env.observation()]` (69:74-77); restore reproduces frame 0 image and state exactly (items 1, 2).
5. **npz round trip.** Loaded with `allow_pickle=False` under numpy 2.2.6 (g1-ext) and 1.26.4 (openpi): all keys load,
   `handover_json` is a 0-d `<U3197`, parses, PCG64 `state`/`inc` 128-bit ints intact; `handover_state` (798,) f64,
   `handover_head_eq_data` (2, 11) f64. Restore is bit-exact (items 1, 2).
6. **Determinism.** My accidental T7 copy of seed 100 trial 0 was array-for-array equal to the scratchpad dry demo (all 10 keys),
   and `zipfile.testzip()` passed on the T7 copy: same seed + trial reproduces the same demo, and a 115 MB npz writes correctly to the
   T7 via the plain `np.savez_compressed(path)` path.
7. **Guards.** `--seed 5` -> exit 2, "--seed must be >= 100". `--out /Volumes/NotMountedReviewProbe/...` -> exit 2, "is not
   mounted". (Unplugged-T7 case itself not re-run: the T7 was mounted by then; the code path is the same `is_dir` check.)
8. **Only successes saved.** 69:96-97 returns before any write when `passed` is False or the run raised RuntimeError /
   NoSolutionFound; the failure reason (`first_failed` or the exception text) is logged.
9. **Sidecar vs plan D.** Present: seed, trial, mode, stance_source, full variation, stop (goal) / stance / arrival error, handover
   pelvis z + roll + pitch, timestep, geometry (r and bearings of stand and basket, stance clearance), teacher checks, frames, wall
   time; walkup/detection explicitly null in no-walk mode. The geometry is computed after the run, but I recomputed it at the
   restored handover: identical values.

## Findings

No blockers, no majors.

**m1 (minor) - saved-but-invalid demos count as saved and are not marked invalid in the npz.** 69:99-119. A trial that passes the
teacher but fails the factory checks (frames outside 600-800, waist tracking > 0.05 rad) is still written with `success=True`,
counted toward `--demos` (69:147), and `episode.validate()` on the npz is clean, because those two checks live only in the sidecar
`validate` list. Trigger: one variation with waist lag > 0.05 rad (measured max so far 0.017, so unlikely; frames are a constant 726).
Impact: Stage 7's converter/validator skips "invalid" episodes only if it reads the sidecar. Fix options: do not save/count such
trials, or make Stage 7 read sidecar `validate`.

**m2 (minor) - some exceptions end the loop; a kill between npz and sidecar leaves an npz without sidecar.** 69:145 (`env.reset`
RuntimeError after 200 draws: negligible), 69:100 (OSError from the write, now raised by `write_verified` after 3 tries),
69:117 (sidecar write). `factory_s<seed>.json` is written only at the end (69:165), so a crash or Ctrl-C loses the per-process
report. Impact is small: a re-run with the same seed/name regenerates the same trials bit-exactly (evidence 6) and overwrites them.
Stage 7 must require the sidecar (plan already says "sidecar present").

**m3 (minor) - nothing stops two processes with the same `--seed` and `--name`.** 69:135-136, `write_verified` uses a fixed
`<name>.part` temp name. Two such processes write the same files at once; this happened today (disclosure above). Low risk in the
planned 4-process run (seeds 100-103), but a lock file or a refusal when `factory_s<seed>.json` / `episode_s<seed>_t0000.npz`
already exists would prevent it.

**m4 (minor) - Stage 6 scope not built and not recorded as deferred.** Missing: `--walkup/--no-walkup`, `--allow-provisional`,
Lab View labels and the factory's Lab View reporter. Walk mode is deferred on purpose. The provisional-distribution opt-in was
replaced by K's `stance_source` label. The Lab View part is not mentioned anywhere as deferred; note it in PLAN/PROGRESS.

**m5 (minor) - plan numbering is stale.** PLAN.md:20-21, 182, 411, 421, 432, 443-444 still say `68_volleyball_demo_factory.py` and
`69_convert_smolvla.py`. 69 is now the factory, so the Stage 7 converter needs a new number before it is created.

**m6 (minor) - sidecar lacks the teacher's grasp/place waist yaws and a code version.** Plan C asks for "waist yaws" in the
logged geometry. They can be recovered from `actions[:, 26]`. A git commit hash would tie the dataset to the locked teacher.

**i1 (info, for Stage 7 and 82) - exFAT AppleDouble files are real.** Every file written to the T7 gets a 4096-byte `._<name>`
twin (seen for every npz/json in smoke_v1). `pathlib.Path.glob("*.npz")` matches `._episode_...npz` (verified, Python 3.12.14),
while `glob("episode_*.npz")` and shell `*` do not. The converter, `80_validate_dataset.py` and `82 --from-handover <dir>` must
glob `episode_*.npz` or skip `._*`.

**i2 (info) - no unit test pins the handover round trip.** `handover_arrays` / `handover_snapshot` are only proven by scratch
scripts. A small test (snapshot -> npz -> load -> restore -> same state) would guard the eval contract. The `default=str` fallback
in `handover_arrays` would silently stringify an unknown type; there are none today (the round trip is bit-exact).

**i3 (info, plan) - known distribution caveats stay.** In no-walk mode the handover is a settled reset (zero velocity, 0.3 s
settle), not a post-walk arrival, so D's "recorded distribution equals the eval handover distribution" holds only for the
`--from-handover` eval, not for the walk-mode eval. Every demo is the same 726-frame timeline with long static holds (stall risk,
already in plan E). Both are already labelled (`stance_source`, E's stall metric). No action needed for v1.

## Verdicts

1. **Implementation fidelity (did the code do what the spec said): YES, with minor gaps.** Episodes start at the handover. Each
   frame is the observation before the action plus the action actually commanded, with no off-by-one. The trim is exactly
   face_front + 30 ticks. The handover snapshot is in the npz, restores bit-exactly in a fresh env, and loads under numpy 1 and 2
   with allow_pickle=False. Only successes are saved. Seeds >= 100 and unmounted volumes are refused. The sidecar covers plan D for
   no-walk mode. Gaps: m1-m6 (deferred Stage 6 parts, stale numbering, small hardening).
2. **Spec correctness (is the spec right for the purpose): YES.** The data contract fits SmolVLA training (27-D state/action,
   200x320 head image, 30 fps, prompt) and the plan E replay/eval (the snapshot restores the exact first frame and the full
   trajectory). The no-walk caveat (i3) is known and labelled.
3. **Done criterion "5 test demos saved and valid on the T7": MET**, verified by me against
   `/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/smoke_v1/` (author run 17:10-17:14): 5/5 validate clean, sidecars
   complete, bit-exact closed-loop restore.

## Overall: GO

Fix m1 or cover it in Stage 7 before the 300-demo run. Fix m5 before Stage 7 creates its scripts. Correct the `write_verified`
rationale (disclosure) in docs. Not run by me: the unplugged-T7 refusal with the real `T7` name (the drive was mounted; same code
path verified with another volume name), and the 3-process throughput check from the Stage 6 verify list.
