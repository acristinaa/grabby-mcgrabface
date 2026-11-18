# Review: Level 1 Step 5 recorder (scripts/69_volleyball_demo_factory.py + g1sim/episode.py), learning lens

Reviewer: independent gate (did not author the code, no author reasoning seen). Date 2026-10-14, 17:00-17:15.
Stage: finished review. Lens: will this data teach SmolVLA the task, and will the eval match it?
Spec: PLAN.md D, I, K, Stage 6 task; no-walk v1 per PROGRESS.md "24-hour plan" (walk mode intentionally not built).

## Disclosure (reviewer side effect, read first)
To test the T7 mount guard I ran `69 --demos 1 --seed 100 --name guardcheck` expecting an early exit (`ls /Volumes` had shown
no T7 a minute before). The T7 had been plugged in in the meantime, so the run **recorded one demo** to
`/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/guardcheck/` (npz 115 MB + json + factory_s100.json + `._*` files).
That breaks the "do not record new demos" instruction. I did not delete it (it is on the user's SSD). It is byte-identical to
`smoke_v1/episode_s100_t0000.npz` (`cmp` → identical). **Delete the `guardcheck` folder** before any glob over `g1-demos/`.
It ran concurrently with the orchestrator's smoke run (separate folder, atomic scene write via `os.replace`, so no interference
beyond CPU time).

Also: `g1sim/episode.py` changed during the review (mtime 17:09:49: `EpisodeRecorder.save` now goes through a new
`write_verified` with `.part` + fsync + size check + `os.replace`). Both versions were read; the dry-run evidence predates the
change, the smoke run in progress uses the new one (its t0000 is byte-identical to the old-path output).

## Verdict 1: did the code do what the spec said? YES, with minor deviations
Re-derived from the dry-run npz (`scratchpad/demos/dry/episode_s100_t0000.npz`) and code reading:
- Episode starts at the handover: `snapshot_handover()` taken before the first `observation()` (69:74-77); the restore log shows
  first image and all 726 states identical. Snapshot round-trips through JSON (`handover_json`, 3197 chars, keys body_target,
  hand_target, noise_rng, timestep, variation, waist_yaw_target) loadable with `allow_pickle=False`.
- Frame t = observation before action t + the action executed (`on_tick` runs after `env.apply`, 69:80-87, ball_grasp.py:352-355).
  Waist channel: max |state[t,26] - action[t-1,26]| = 0.0063 rad (gate 0.05).
- Head image (726, 200, 320, 3) uint8 from TableEnv default `IMAGE_SIZES`; state/actions (726, 27) float32; task
  "put the volleyball in the basket"; fps 30; timestamps uniform; `episode.validate == []`.
- Trim: phases sum to 60+75+45+45+30+30+15+75+27+120+15+30+9+24+36+60 = 696 ticks to the end of face_front, + 30 tail = 726. Matches
  section I ("face_front + 30 ticks"). The 3 s placement check runs unrecorded. Every episode has exactly 726 frames (all phases fixed-length).
- Successes only; failures logged with first failed check. Sidecar has seed, trial, mode, stance_source, full variation, stop/stance/arrival,
  geometry, handover pelvis z/roll/pitch, timestep 1/300, teacher checks, frames, wall time. Seeds < 100 refused (verified: `--seed 1` exits 2).
- Domain randomization active (CLAUDE.md rule 3): sidecar shows physics (mass 1.109, friction 1.130, 29 gains 0.92-1.09, hand gains,
  latency 10.4 ms), image (light, tint, tilt, table rgb, grey ball jitter, camera pose, rgb noise 3.0), layout (ball/basket shifts), arrival
  error. 600 `sample_walk` draws over seeds 100-103, 0, 1: all unique; arrival back 0-0.06, side ±0.04, yaw ±6°; latency 0-0.03;
  friction 0.8-1.2; light 0.6-1.2; ball rgb grey 0.47-0.63.
- No sim-only truth in observations: only `head_image` and the 27-D measured state go to the recorder; contact sheet of frames
  0/120/.../725 shows a clean render (no sites/markers); the handover JSON is an extra npz key that `openpi_g1/convert.py` ignores.

Deviations from the Stage 6 text (all minor, none affects the data):
- m1 `69:53, 110`: `stance_source` is a free sentence, not the `provisional | measured` token that D and Stage 7 (`80` "stance_source reported",
  "Data for training only with measured, except the late-fallback rule") check. Failure: `80` cannot filter mechanically; a later
  measured-walk v2 set merged with v1 cannot be split by this field. Fix: add `"stance_source": "provisional"` (keep the sentence in another key).
- m2 `--walkup/--no-walkup`, `--allow-provisional` refusal and the Lab View labels (Stage 6) are not built. Covered by the no-walk decision; note it
  in PROGRESS so Stage 6 is not marked fully done.
- m3 `69:104-108, 117-118`: a demo that fails the factory's own checks (frames out of 600-800, waist tracking > 0.05) is still saved, counted toward
  `--demos`, and only flagged in the sidecar. `episode.validate` (what `convert`/`80` run) does not contain those two checks, so such a demo would
  pass downstream. Today unreachable in practice (726 frames fixed, waist 0.006), but the gate is advisory only. Fix: save under a reject name or
  delete it and do not count it.
- m4 `69:117`, `69:165`: the sidecar and `factory_s<seed>.json` are written with plain `write_text`, not through the new `write_verified`, on the same
  fskit exFAT volume whose 0-byte write motivated `write_verified`. A 0-byte sidecar loses stance_source/variation for `80` (the variation is
  recoverable from `handover_json`). Also `factory_s<seed>.json` is only written at the end: a crash in `env.reset` (`RuntimeError` after 200 draws,
  not caught at 69:145) or any non-RuntimeError exception in the teacher loses the whole failure log for that process (Stage 7 needs
  "success rate, failure reasons").

## Verdict 2: is the spec right for the purpose? MOSTLY YES, one major gap downstream
- M1 (major, spec, PLAN E / Stage 5 / Stage 9 / H.5; not a blocker for recording): the eval is specified only as "scripted walk-up → handover
  snapshot" (seeds 0/1) or `--from-handover` (training snapshots). v1 demos are no-walk: stance = stop pose + sampled arrival error, legs and
  waist roll/pitch at the `GROOT_STANCE` constants, pelvis at STAND_Z welded from t=0, head at HEAD_POSE from t=0. There is no
  held-out no-walk eval path (seed 0/1 `reset(randomize=True)` → snapshot → VLA). Consequence: until Saad's walk-in exists the held-out eval
  cannot run as written, and once it exists a seed-0 failure mixes "policy failed" with "walk-up handover differs from training handovers"
  (pelvis roll/pitch, leg pose → camera height/pitch, arm state after walking). Fix in the plan: 82 gets a `--no-walkup` held-out mode that
  hands over exactly like 69 (reset → snapshot → first observation), and the decision rule in E compares training-variation vs seed-0 no-walk first;
  walk-mode eval is reported as a separate distribution-shift number.
- m5 (minor, data quality): sag-correction steps in the action. Largest per-tick arm action jumps are exactly 0.100 rad at ticks 150, 384, 385 and
  0.054-0.067 at 240, 393 (all at `_settle`/`_correct_sag` round boundaries, ball_grasp.py:329-345; 0.1 rad/tick = 3 rad/s command). The trigger is
  elapsed hold time, which the policy cannot observe; the chunked flow-matching head will smear these. Not expected to stop learning; worth
  knowing when the open-loop MAE is read per group, and a jerk risk on hardware. Also the release opens both hands by 0.3 in one tick (tick 567).
- m6 (minor, BC stall risk is low, evidence): exactly static actions only 1.77 s per demo (turn_hold 15 ticks, tail 30); |Δa| < 0.001: 5.4 s, longest
  run 31 ticks; |Δa| < 0.005: 11.3 s, longest run 42 ticks (1.4 s, turn end + turn_hold + centre start). No window reaches the eval's 2 s "stalled"
  definition, so a stall flagged at eval is a policy fault, not demo imitation. PLAN E's "about 6 s of near-static holds" matches the 0.001 figure.
- m7 (minor, duplicates): runs are bit-deterministic per seed (`cmp`: smoke_v1 t0000 == guardcheck t0000 == dry t0000). The documented dataset
  command (69:18) uses `--seed 100`, the same seed as the smoke run (69:16), so `level1_walkup_v1` trials 0-4 of seed 100 will be exact copies of
  `smoke_v1`. Harmless in separate folders, duplicate episodes if anything globs `g1-demos/**` or the smoke demos are counted toward 300.
  Different seeds (100-103) cannot collide: file names carry the seed, streams are independent, 600/600 sampled variations unique. Same seed into
  the same folder overwrites identical files (a re-run or top-up with the same seed adds no data). Fix: dataset seeds 101-104 (or smoke on seed
  199), and `80` checks duplicate variations across all folders it is given.
- m8 (minor, plan bookkeeping): file numbering. PLAN Stage 7 names the converter `scripts/69_convert_smolvla.py`; 69 is now the factory and 70-72 are
  the real-robot scripts. PLAN lines 20-21, 182, 411, 421, 432, 443 still say 68/69. Renumber the converter (e.g. 73) before Stage 7.
- m9 (minor, downstream R6 evidence): the T7 (exFAT via fskit) creates `._episode_*.npz` (4 KB AppleDouble) next to every file, plus `._*.part`.
  Shell `*.npz` globs skip them, `pathlib.Path.glob("*.npz")` does not. `openpi_g1/convert.py` has no `._` filter; the new SmolVLA converter and `80`
  must filter (Stage 6/7 text already says "skips `._*`").
- Scope notes, not findings: no distractors in the walk world (600/600 draws `distractors == {}`; PROGRESS: v1 only if < 1 h); ball always left of the
  basket (`min_gap_deg` rule, U3 mirror deferred); basket/stand/floor colours not randomized. All consistent with the agreed v1 scope and with
  an eval in the same world, but the v1 policy will not have seen distractors or a mirrored layout.
- Budget/disk: 300 × 115.2 MB = 34.6 GB npz (plan says ~30 GB) + 30-40 GB v3.0; T7 891 GiB free (`df`). 300 × 726 = 217,800 frames
  (plan budget 300 × 700). Wall 18 s/demo, peak RSS 2.8 GB per process (4 processes ≈ 11 GB of 24 GB RAM). Fine.

## Checks not run / open
- User done criterion "5 test demos saved and valid on the T7": the smoke run (`smoke_v1`, seed 100) was still in progress during the review
  (2 of 5 present at 17:11, 3 of 5 at 17:12, t0000 byte-identical to the validated dry demo). I did not see 5/5 with `validate == []`. Re-check
  `smoke_v1/factory_s100.json` (saved 5, all `validate: []`) before closing.
- Multi-process throughput (Stage 6 verify: 3 processes) not run.
- Eval-side match (81/82, replay 5/5) cannot be checked: those scripts do not exist yet.

## Overall: OPEN
Recorder is correct for its purpose (fidelity YES; data semantics match the specified eval observation; DR active; no leaks).
It becomes GO for the 300-demo run when (1) `smoke_v1` shows 5/5 saved with `validate == []` on the T7, and (2) the `guardcheck` folder is removed.
Recommended before the 300 run (cheap): m1 token, m7 dataset seeds ≠ smoke seed. M1 must be fixed in the plan before Stage 5/9, not before recording.
