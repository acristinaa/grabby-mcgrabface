# Review: docs update 2026-10-07 (reviewer agent, fresh context)

**Verdict: NO-GO.** It is a narrow NO-GO. It turns to GO once F1 and F2 are applied, and F3 is strongly recommended. Each one is a one-line change.

**Fixes I made: none.** My reviewer role says I must not edit the work I am reviewing, and that rule outranks the script's instruction to fix things. So below I give the exact text to change, for whoever applies it. No path was given for a gate record file, so none was written under `.planning/reviews/`. This answer is the record.

## Verdict 1 – Implementation fidelity (do the edits match the brief?): yes, apart from F1 and F2

I checked these against the real files, not against the brief alone:
- **D2 numbers**, recomputed from `runs/d2_54_seed0.json` and `runs/d2_54_seed1.json`:
  - Seed 0: 25/50 passed. Median tilt 2.7117°, max 4.365°. No starts were rejected.
  - Seed 1: 35/50 passed. Median tilt 2.396°, max 3.929°.
  - "20/20 and 31/31 on the old reachable set", "16/16 teleop" and "40.7 mm vs 10 mm" all match `.planning/reviews/D2-measurement.md` (F1, F3).
- **D4 probe**, against `runs/d4_probe.log`: 615 frames, 20.5 s, 91 MB and 94 MB, about 16 s each, 27 min and 9.3 GB for 100 demos. All match.
- **Script descriptions** for 64 and 70–72 match their docstrings: seed default 100, `demos.nosync`, 3 m, 0.3 m/s, `--dry-run`, listen-only probe, the four basics in 72.
- **"Port 8020 still held (checked 2026-10-07)"** is true. Process 4289 (`62_rooms_check.py`) is in state UE and listens on 127.0.0.1:8020.
- **"lo0 in 6 places"** is true: walk_sim:14, 50:48, 52:29, 61:30, connect_probe:15, sim_env_probe:23.
- **Brief A–C:** every item is present in PROGRESS (next steps 0–6, log entries 10-02 through 10-07, the BLOCKERS update with all of section B.10) and in PLAN (blocks a–g).
- **No overclaims** about the robot, meeting or Docker: the real-robot scripts are "never run on the real robot", the probe "ran against the sim", the meeting outcome is "pending", and Docker is "waiting for go".
- **START_PROMPT:** every pitfall from HEAD survives in English. It also adds the D2 sweep caveat and the real-robot network notes.
- **The four docs agree:** the next step is the Level 1 build, and 29 DoF is resolved in all of them.

## Verdict 2 – Plan correctness: sound, with the gaps below

**F1 (minor, overclaim, check 2).** In `docs/LEARNING_PATH.md`, the "Status 2026-10-07" line under the D table says: "With Level 1 the dataset becomes pick **and place** (into the box)." But `PLAN.md` block (g), the brief and the unchanged D3 #3 all say the dataset v1 change is only a proposal. Replace the sentence with:
`Proposal, not decided yet: with Level 1, dataset v1 would become pick **and place** (into the box) – see PLAN.md, SCOPE UPDATE 2026-10-07 (g).`

**F2 (minor, still-valid content deleted, check 3).** The old PROGRESS section "Open with the user" ("Offen beim Nutzer") was removed and not carried over. START_PROMPT dropped the same items. They are still open:
- try teleop yourself (`57_teleop.py --randomize`)
- answer the comprehension questions for L1 and L2; logbook §9 is still "(still empty)"
- run `scripts/run_tests.sh` (about 2–3 min). This matters more now, because it would confirm the 2 known test failures.

Add this line after "**Waiting on others:**" in PROGRESS, and the same line in START_PROMPT:
`**Open for the user:** run `scripts/run_tests.sh` (~2–3 min; 2 known failures expected, see D2 log) · try teleop (`57_teleop.py --randomize`) · answer the L1 + L2 comprehension questions, fill logbook §9.`

**F3 (minor, a consumer of the docs is now broken).** PROGRESS renamed its heading to `## NEXT STEP`. But the project `CLAUDE.md`, under "How we work in every session", rule 2, still says to work only on the item under "NÄCHSTER SCHRITT" (the old German name of "NEXT STEP"). A fresh session following CLAUDE.md will look for a heading that no longer exists. The smallest fix inside these four files is to rename the heading to `## NEXT STEP (NÄCHSTER SCHRITT)` (keeping the old German name in brackets).

`CLAUDE.md` is outside the four files and was not updated. Two parts of it are now stale:
- its goal is still "fetch the red bottle from the next room"
- it points only to "SCOPE UPDATE 2026-09-14"

It needs its own update.

**F4 (note, wording).** The PROGRESS Docker log says "paid plan needed for large companies … since the 2026-08-26 terms". That suggests the large-company rule is new, which the brief does not say. Change it to "per the Docker Desktop terms updated 2026-08-26".

**F5 (note).** RF-DETR is called "Our choice" (LEARNING_PATH §10.15) and "Status: DONE (decision)" (PROGRESS log). The brief only says "Recommended instead", and PLAN says "Default". If the user has not explicitly confirmed it, write "Recommended" or "Default" instead.

**F6 (note).** The `docs/LEARNING_PATH.md` header (lines 3–8, "Status as of 2026-10-02", the goal is the fetch task) still states the old goal. Add one line pointing to the §1 "Update 2026-10-07" block.

**F7 (note).** The LEARNING_PATH L3a status says "On the 20 starts that were reachable before, 20/20". That is seed 0 only. Append "(seed 0; seed 1: 31/31)".

## Still open
- F1–F7 have not been applied.
- `CLAUDE.md` is stale (see F3).
- I could not check two items against any evidence file:
  - LEARNING_PATH §10.17 (critical path) is not in the brief. It is a general explanation, so the risk is low.
  - The external-resource facts (BotBrain, supat-roong, Reddit, AWS prices, Docker terms) appear in the docs exactly as the brief states them, but I did not check them against their original sources.
- The independent D2 code review gate is still OPEN. This is unchanged and correctly recorded.

Files:
- /Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot/PROGRESS.md
- /Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot/PLAN.md
- /Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot/docs/LEARNING_PATH.md
- /Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot/docs/START_PROMPT.md
- /Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot/CLAUDE.md (out of scope, stale)