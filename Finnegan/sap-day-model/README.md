# SAP day model: SmolVLA run3 (150k steps)

The model file is 907 MB; GitHub allows at most 100 MB per file in a branch, so it lives in the release
**[sap-day-run3-model](https://github.com/acristinaa/grabby-mcgrabface/releases/tag/sap-day-run3-model)**.
This folder installs it with one command.

- Trained on the PC on the 1,691 v2 simulation demos, 150,000 steps (continued from run2's 90k weights).
- Simulation test: 68 / 100 new situations; on the harder v2 scenes with the safety fence on: 30 / 50. Simulation only.
- Code that runs it: `Finnegan/week-3/code` (runbook: `Finnegan/week-3/code/docs/ROBOT_DAY_RUNBOOK.md`).

## Install on the robot laptop (from the repo root)

```
bash Finnegan/sap-day-model/get_model.sh
```

It downloads the model (resumes if the internet drops), checks both checksums and puts it in
`~/g1-robot-day/smolvla-runs/run3/checkpoints/150000/pretrained_model`, where the self-test and the runbook expect it.
It ends with `MODEL: OK`. `model.safetensors` sha256: `ef86fffee140e065e0e19625e125af3dc1c1a9d4d0ae1384983c90d5221db931`.

Then set up the code as in `Finnegan/week-3/README.md` and run the self-test (`SELFTEST: PASS`).
