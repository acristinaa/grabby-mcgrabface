# Finnegan – week 2 (status 8 Nov 2026)

Everything since the week-1 snapshot: Level 1 trained twice, the real-robot software, safe arm paths, and the hand-over for the
Level 2 brain track. The full code is in [`code/`](code/) (a snapshot of my repo `finneganadam-maker/UnitreeG1-Robot`, commit
`8bc5c9c`; the PC address is replaced by `<pc-tailscale-ip>`).

**Short version:** in simulation the VLA (SmolVLA) picks up the volleyball with both hands and puts it in the basket on its own
in **72 %** of new situations (run1). A second, bigger training (run2, 1,691 demos, 50 h) finished on the Hive PC on 6 Nov and
**is not evaluated yet**. On the robot we have only run the read-only checks so far (19 Oct). For SAP on **13 Nov** the software
to move the arms safely is built and tested in sim; a few safety decisions are still open (see below).

---

## What to read (in this order)

| # | File | Why |
|---|---|---|
| 1 | this README | status, open points, plan to 13 Nov |
| 2 | [`code/PROGRESS.md`](code/PROGRESS.md), top block "NEXT STEP" + "Robot visit milestones" | the live status and the visit plan |
| 3 | [`code/docs/ROBOT_DAY_RUNBOOK.md`](code/docs/ROBOT_DAY_RUNBOOK.md) | the step-by-step plan for the day at the robot (Parts A, B, C) |
| 4 | [`code/.planning/real-robot/SAFE-PATHS.md`](code/.planning/real-robot/SAFE-PATHS.md) | how the arms get up and back without hitting the table |
| 5 | [`code/docs/L2_BRAIN_TRACK.md`](code/docs/L2_BRAIN_TRACK.md) | Saad's track: QR codes + Qwen as the "brain" (Level 2/3) |
| 6 | [`code/docs/LEARNING_PATH.md`](code/docs/LEARNING_PATH.md) §10 | plain-language explanations of every concept we used |
| 7 | [`code/PLAN.md`](code/PLAN.md) | strategy and the binding safety rules for the real robot |

Code map: `code/g1sim/` = library (sim, teacher, safety layer, real-robot I/O) · `code/scripts/NN_*.py` = one script per step
(67 teacher, 69 record, 79/80 convert + validate, 81 policy server, 82 eval, 84–89 real robot) · `code/tests/` = unit,
integration, end-to-end tests (all pass in 3 Python environments).

---

## What I did since week 1

| Area | Result |
|---|---|
| **Teacher (programmed demonstrator)** | Two-hand grasp hardened: finger clearance, natural arm motion, follows the real stand pose. Under full randomization 200/200, 300/300 |
| **Randomization v2** | Table height −4…+1 cm, robot stance, ±8 cm object offsets, mass/friction ×0.8–1.2, motor gains ×0.9–1.1, latency 0–67 ms, light, 12 textures, table with legs, head camera ±2.5 cm/±5°, 5 different task sentences |
| **run1** (SmolVLA, 300 demos, 45k steps, Mac) | **72 %** on 50 new starts (40k checkpoint), 70 % at 45k. Teacher on the same starts: 50/50. Reading: learned the demos, generalizes only partly → more varied data |
| **v2 data + run2** (Hive PC) | 1,691 demos recorded on the PC (0 problems in validation), run2 = 90,000 steps, 50 h, finished 6 Nov 05:49, final loss 0.012 |
| **Real-robot stack** | 84 read-only checklist, safety layer (joint limits, speed caps, staleness, ramps), 86 robot adapter with a motion lock, 85 arm simulator for rehearsals, 87 table-clearance check, 89 self-test for the laptops, runbook. Robot day 19 Oct: only the read-only part ran |
| **Safe arm paths** (6 Nov, reviewed GO) | The arms move only along fixed routes that 87 has checked (≥ 5 cm from table, stand and basket): up beside the body first, then forward. Faults hold or go home along these routes. New `86 recover` brings the arms home after a crash and refuses if it is not sure where they are |
| **Level 2 prep** | `scripts/90_head_frames.py` (full-resolution head frames + labels), 12 sample frames, the brief for Saad (`docs/L2_BRAIN_TRACK.md`, branch `l2-brain`) |

Where the intelligence is: the **VLA grasp and place** looks at the ball through the head camera and decides every arm, hand and
waist motion itself. The waypoints and the safety layer are the engineered, checked frame around it (like a crane that always lifts
before it swings).

---

## What is on the Hive PC (`ubucode`, GTX 1060 6 GB, 15 GB RAM)

| What | Where on the PC | Size |
|---|---|---|
| v2 dataset (LeRobot format, 1,691 demos) | `~/datasets/g1_volleyball_walkup_v2` | 137 GB |
| run2 checkpoints 10k … 90k (`last` = 90k) | `~/dev/g1-ext/smolvla-runs/run2/checkpoints/<step>/pretrained_model` | 12 GB total, 865 MB per model |
| Logs (recording, convert, validate, training) | `~/UnitreeG1-Robot/runs/l1v2_*.log` | small |
| Code | `~/UnitreeG1-Robot` (commit `968f7c7`, older than this snapshot) | |

run1's checkpoints are on the Mac and the T7 (`DATA Collection/smolvla-runs/run1/`, sha256-verified .tar files).

### How to get to it (Tailscale + SSH)

The PC and the Mac are in the same Tailscale network, so this works from anywhere, not only at CODE. Ask Finnegan to add your key.

```bash
ssh grabby@<pc-tailscale-ip>                                   # log in
ssh grabby@<pc-tailscale-ip> 'tail -3 ~/UnitreeG1-Robot/runs/l1v2_pc_chain.log'   # quick status
# copy the final model (865 MB) to the laptop
rsync -avP grabby@<pc-tailscale-ip>:dev/g1-ext/smolvla-runs/run2/checkpoints/090000/pretrained_model/ \
      ~/dev/g1-ext/smolvla-runs/run2/090000/pretrained_model/
# copy eval reports back after an eval
rsync -avP grabby@<pc-tailscale-ip>:UnitreeG1-Robot/runs/vla_eval_run2_*.json runs/
```

Do not copy the 137 GB dataset over Tailscale (it goes through a relay, slow). The dataset stays on the PC; only models and reports move.

### How we process it

1. **Eval on the PC** (the GPU is free now): the policy server `81` loads a checkpoint, `82` runs the sim trials against it.
   ```bash
   cd ~/UnitreeG1-Robot
   ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/81_smolvla_server.py \
       --checkpoint ~/dev/g1-ext/smolvla-runs/run2/checkpoints/090000 --device cuda &
   ~/dev/g1-ext/.venv/bin/python scripts/82_vla_eval.py --no-walkup --seed 0 --trials 100 \
       --train-variations 10 --demos <v2 demo folder> --name run2_090k_seed0
   ```
   Same 50 held-out starts as run1 first (fair comparison), then 100–200 for a tighter number. Run it for 2–3 checkpoints
   (e.g. 60k, 80k, 90k) and keep the best.
2. **Pick the checkpoint**, copy it to the policy laptop, pack it as one `.tar` with a sha256 sum and attach it to a GitHub release
   (as done for run1), so the robot day uses exactly that file.
3. **Sim2Sim**: run the chosen model through the real adapter (84 → 86 → 85 arm simulator) before it ever moves the real arms.

---

## What is open for Level 1 at SAP (13 Nov)

Level 1 finish line (from `PROGRESS.md`): sim ≥ 85 % on new starts · walk mode ≥ 80 % · real robot ≥ 3/10.
Today: 72 % · not measured · 0 attempts.

| # | Must do before 13 Nov | Who | Status |
|---|---|---|---|
| 1 | **Eval run2** (above) and choose the checkpoint | Finnegan | not started, PC idle |
| 2 | **Safety decisions** for the arm paths (what the arms do per fault type, waist limit, recover tolerance; list in `SAFE-PATHS.md`, rulings F1–F5, S1–S6) | Finnegan | open, defaults are built in |
| 3 | Update runbook Part C + PLAN rules to the new paths (`86 recover` instead of `86 release` after a crash) | Finnegan + Claude | after #2 |
| 4 | **Sim2Sim** of the chosen checkpoint through 84 → 86 → 85 + a final review | Finnegan + Claude | after #1 |
| 5 | Self-test `89` on the operator laptop(s) | everyone who operates | open |
| 6 | Policy laptop ready: SmolVLA env, the checkpoint, Ethernet to the robot | Finnegan | open |
| 7 | On site: record the real factory arm pose (M0), run `87 --factory-pose` on it, table position in cm | at SAP | on the day |

Not needed for visit 1: walking (visit 2), QR/Qwen brain (Saad's track, a bonus at most, read-only).

### Visit plan (decided 4 Nov)
- **Visit 1 (13 Nov, Potsdam):** robot placed at the table → checked waypoints raise the arms to the ready pose → the VLA grasps and
  places on its own → checked paths bring the arms back. Read-only first, then motion only after the runbook gates pass.
- **Visit 2 (date open):** full walk-up to the table, then grasp and place. Needs: walk + grasp in one sim world, walk-mode eval
  ≥ 80 %, real walking (Unitree LocoClient), the lessons from visit 1.

---

## Next steps (in order)

1. Eval run2 on the PC (#1) → compare with 72 %.
2. Finnegan's safety rulings (#2) → runbook/PLAN text (#3).
3. Sim2Sim + final review (#4), release the checkpoint with sha256.
4. Laptops: self-test 89, policy laptop setup (#5, #6).
5. In parallel: Saad's Level 2 brain track (QR + Qwen) on branch `l2-brain`; Level 2 sim world (2 coloured balls + 2 boxes with QR codes).
6. Housekeeping: PC hardening, the recorder memory leak in `69`.

Honest expectation: manipulation sim-to-real is hard; a first real success rate is often low. Visit 1 is about running the whole
chain safely and learning what the real robot does differently (torso lean, camera look, timing), not about a perfect score.
