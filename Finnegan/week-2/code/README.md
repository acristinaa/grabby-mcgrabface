# Unitree G1 + BrainCo Revo2 — learned pick and place (SAP project)

A Unitree G1 humanoid (29 DoF, BrainCo Revo2 hands) picks up a volleyball with both hands and puts it in a basket. The grasp is
learned: a **SmolVLA** vision-language-action model sees the head camera, gets the task sentence and moves arms, hands and waist.
Everything is built and tested in MuJoCo first, then run on the real robot.

## Status (2026-11-08)
| What | State |
|---|---|
| Sim teacher (programmed demonstrator) | two-hand grasp, fully randomized scene, 200/200 |
| run1: SmolVLA, 300 demos, MacBook | **72 %** on unseen starts |
| run2: SmolVLA, 1,691 demos, PC at CODE (GTX 1060), 90k steps / 50 h | trained 2026-11-06, **evaluation next** |
| Real-robot software (84–89): read-only checks, safety layer, robot adapter, checked safe arm routes | built + reviewed; robot day 2026-10-19 ran the read-only part |
| Next robot visit | SAP Potsdam 2026-11-13: safe routes lift the arms, the VLA grasps and places |

## Documents
| File | Purpose |
|---|---|
| `PROGRESS.md` | current status (top block) + log. **Read this first.** |
| `PLAN.md` | strategy, phases, binding safety rules |
| `docs/ROBOT_DAY_RUNBOOK.md` | step-by-step plan for a day at the real robot |
| `.planning/real-robot/SAFE-PATHS.md` | safe arm routes around the table |
| `docs/L2_BRAIN_TRACK.md` | Level 2/3: QR codes + local Qwen orchestrator |
| `docs/LEARNING_PATH.md` | plain-language explanations (section 10) |
| `CLAUDE.md` | working instructions for Claude Code |

## Code map
- `g1sim/` library: sim env, scene, teacher, randomization, policy wire, safety layer, real-robot I/O
- `scripts/NN_*.py` one script per step: 67 teacher · 69 record demos · 79/80 convert + validate · 81 policy server · 82 eval ·
  84 read-only checklist · 85 arm simulator · 86 robot adapter · 87 table clearance · 89 laptop self-test · 90 head frames
- `tests/` unit, integration and end-to-end tests (`scripts/run_tests.sh` runs all three environments)

## Environments
- Sim + tools: `~/dev/g1-ext/.venv` (Python 3.12, `requirements-record.txt`); MuJoCo viewer on macOS only with `mjpython`
- SmolVLA (training + policy server): `~/dev/g1-ext/smolvla-test/.venv` (lerobot 0.6.1); training on an NVIDIA GPU, the Mac only
  for small tests
