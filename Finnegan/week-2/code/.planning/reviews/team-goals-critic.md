# Review: docs/TEAM_GOALS_PROPOSAL.md — plan critic (robotics project lead lens)

Reviewer: independent gate, not the author. Date 2026-10-06. Document not edited.
Checked against: `docs/LEARNING_PATH.md` (§1, §5 table, L3a, L3, L4), `PROGRESS.md` (NEXT STEP), `PLAN.md` Phase 5, `docs/MASTERPLAN.md` §14,
`g1sim/rooms.py`, `g1sim/navigation.py`, `g1sim/perception.py`, `scripts/70_real_goto.py`, `scripts/71_real_probe.py`,
`scripts/72_real_check.py`, `runs/real_probe.json`, `.planning/reviews/D2-measurement.md`.

Verdicts: **Goal is right (GO on the direction). Plan as written: NO-GO until findings 1–4 are in** — they are the gaps that would
sink the project, not wording issues. Findings 5–10 make the plan measurable and the timeline honest.

Facts from the repo that drive this review:
- Real robot never connected: `runs/real_probe.json` has `"iface": "lo0"` and `imu_rpy [0,0,0]`, i.e. the probe listened to the sim on
  this Mac, not to the G1. Scripts 70/71/72 for the real robot exist but have never run against hardware.
- Real robot and sim walk differently: sim walking = GR00T controller, arms via `rt/lowcmd` (LEARNING_PATH L3). Real walking in our scripts =
  Unitree factory controller (`LocoClient.SetVelocity`, `70_real_goto.py:160`), arms via `rt/arm_sdk` (`72_real_check.py:111`).
- Sim position comes from the simulator itself (`navigation.py:38 pose_from_qpos`), not from a sensor.
- Object finder = "red pixels" only (`perception.py:15 red_mask`). A fire extinguisher is red.
- Sim scene (`rooms.py`) has walls, a door opening, one table. No extinguisher, no door leaf, no exit sign, no ball.
- Teammates' safety-check code is not in this repo (grep for extinguisher/EHS/emergency exit finds only the proposal and notes).
- Walk + grasp in one world (L3) is not started; LEARNING_PATH rates it "large, riskiest step". Placing is not built.
  L4 orchestrator (state machine with time limits) is not built. LEARNING_PATH's own targets: L3 ≥ 8/10 grasps, L4 ≥ 7/10 full runs.
- 23 vs 29 DoF still open; D2 gate F1: with the current grasp a 23-DoF wrist does not reach the grasp pose.

---

## 1. (critical) The sequence has no "middle" and no failure path

**Problem.** Step 1 "SAP sends a task" and step 7 "report to SAP" have nothing in between: what receives the SAP task, what runs the
steps, what writes the report? And no step says what happens when it fails, so "report matches what really happened" cannot be tested
for the failure case. Also: the SAP EHS API covers the *report* side (incidents/observations). Which SAP object *starts* the task is open.

**Text change** — in §2 "The sequence", replace the intro line with:

> Between SAP and the robot sits a **small bridge program** on our laptop (cable to the robot, Wi-Fi to SAP and AWS). It picks up the
> task, starts the robot's step-by-step program, and sends the result back. In week 1 we fix the task and report format (a short JSON),
> so the SAP side and the robot side can be built at the same time.

and add one line after step 7:

> **If a step fails:** every step has a time limit and one retry. If it still fails, the robot stops, puts the object down if it holds
> one, and reports *"failed at step X, because Y"* to SAP. A failed run with a correct report still counts as a correct report.

In §8, replace question 2 with:

> 2. **SAP:** which SAP system **starts** the task (e.g. an inspection/maintenance order or a small BTP app) and which API **receives**
> the report (EHS)? Who asks, by when?

## 2. (critical) The real robot is planned as an end event, but it is the second-biggest unknown

**Problem.** The real G1 has never been connected (probe ran on `lo0` against the sim). On the real robot, walking runs on Unitree's
factory controller and the arm on `rt/arm_sdk`; in sim it is GR00T and `rt/lowcmd`. So balance behaviour tuned in sim does not carry over
automatically. 23/29 DoF decides whether the grasp must be redone. Leaving the first real contact to "weeks 1–2, one bullet" and the
real attempt to weeks 6–7 leaves no time to react.

**Text change** — in §4 "In", replace "One real-robot attempt + lessons learned" with:

> A real-robot track from week 1, step by step: connect + 23/29-DoF check → walk to a point → move arm, hand, read camera →
> grasp while standing at a fixed table → full attempt.

In §7, replace the "23 or 29 joints?" bullet with:

> - **23 or 29 joints?** We check this on day 1 with `scripts/71_real_probe.py` on the real robot. If it is 23, the grasp must be redone
> (the current one fails with a locked wrist).
> - **Sim walks with a different controller than the real robot** (GR00T in sim, Unitree's own controller on the robot). So on the robot
> we always **stop before grasping** and test the standing grasp early.

## 3. (critical) Real-robot safety and access are missing

**Problem.** Nothing in the doc covers how we keep people and a ~35 kg robot safe, or when we get the robot. The damping command on the
remote makes the G1 go limp and fall (our own `70_real_goto.py` warns about this), so a harness/gantry or spotter matters.
`70_real_goto.py` already demands "robot secured, second person at the remote, 3 m clear" — the team doc should say the same.
`MASTERPLAN.md §14.1` lists "e-stop procedure tested" as a precondition; `PLAN.md` Phase 5 says "gantried, human on e-stop".

**Text change** — add one bullet to §7:

> - **Safety on the real robot:** never alone; a second person always holds the remote; robot in a harness or with a spotter for every
> new movement; slow speeds; 3 m cleared. We write a 5-line safety checklist before the first session.

Add to §8:

> 10. **Robot access:** where is the robot, who may run it, how many hours per week can we book, and who is the second person?

## 4. (critical) "Check the room" is not defined, and the sim cannot do it yet

**Problem.** "Checks the fire extinguisher and the emergency exit" has no definition of *where* the robot looks, *what* it decides,
or *what* it outputs. The sim scene has no extinguisher, no door leaf, no sign. And our object finder looks for "red pixels" — a red
extinguisher in the same room will be found as "the object".

**Text change** — replace step 2 of the sequence with:

> 2. **Inspect (report only):** the robot walks to two fixed viewpoints (extinguisher, exit door), takes one photo each, and answers
> fixed yes/no questions: extinguisher in place? sign visible? exit path free? door closed? Output: yes/no per question + the photo.
> It does not touch anything.

Add to §4 "Out":

> Realistic extinguisher and door models in sim. **In sim we fake the inspection result** (to test the flow); the real detection is
> tested on real photos from our room.

In "Which object?", add one sentence:

> The object must **not be red** (or we switch to a better object finder), because the fire extinguisher is red.

## 5. (major) How the team's existing safety check gets connected is open

**Problem.** "This is the safety check you already built" assumes it plugs in. It is not in this repo, and we do not know whether it
runs in sim or on the real robot, on which camera, or what it outputs.

**Text change** — replace "*→ this is the safety check you already built.*" with:

> *→ this is the safety check you already built. It becomes one step with a fixed input (a camera photo) and output (the yes/no answers
> above). Question for you: where does it run today (sim or real robot), which camera does it use, what does it return?*

## 6. (major) Success metrics are not measurable as written

**Problem.** "Correct report" has no ground truth. "Lies still at the target spot" has no tolerance or time. "Finds the object" has
no time limit or accuracy. "Almost works" on the real robot cannot be failed.

**Text change** — in the per-step table, replace three cells and the real-robot line:

| Step | Success means |
|---|---|
| Inspect | 10 prepared test scenes (e.g. 5 OK, 5 with one fault: extinguisher missing, exit blocked, door open). The report is compared to the scene's known answer. |
| Find | Object located within 60 s, position error < 5 cm |
| Carry + place | Object inside the box / within 10 cm of the mark, and still (no movement for 3 s after release) |

> **On the real robot:** at least 3 attempts, filmed. For each attempt a table: which step passed, which failed, why.

## 7. (major) The "9/10 × 9/10 ≈ 8/10" math is too optimistic, and 8/10 is above our own L4 target

**Problem.** 0.9 × 0.9 = 0.81 only if *every other step is perfect*. Walking happens twice, plus find, inspect and report. With
realistic 95 % for each walk: 0.9 × 0.9 × 0.95 × 0.95 ≈ 0.73. A system that really works 81 % of the time passes an "8 out of 10"
test only ~71 % of the time (binomial). LEARNING_PATH sets L4 (full run) at ≥ 7/10 and L3 (walk + grasp) at ≥ 8/10 — the doc asks for
grasp ≥ 9/10 while the repo's own plan accepts 8/10.

**Text change** — replace the main sim goal and the note under the table with:

> 1. **Simulation:** the full sequence works in sim in **at least 7 out of 10 runs** (random start positions), and **every failed run
> names the step and the reason**.
>
> (The per-step targets help us find the weak step. The full run is lower than each step, because the step failure rates add up.)

## 8. (major) Timeline: the critical path is serial — three people do not shorten it

**Problem.** The sim chain (placing → walk + grasp in one world → orchestrator with carry + place → reliable runs) is one dependency
chain; extra people cannot split it. Everything else (real robot, SAP bridge, inspection) can run in parallel. The table mixes both,
has no decision point if L3 fails, and is not tied to a pitch date (Q9 still open). Real attempt in weeks 6–7 leaves zero buffer.

**Text change** — replace the §6 estimate line and table with:

> The **critical path** is the sim chain. It does not get shorter with more people. The other three lanes run next to it.

| Weeks | Sim chain (critical path) | Real robot | SAP + inspection |
|---|---|---|---|
| 1 | Placing at the fixed table | Connect, 23/29-DoF check, safety checklist | JSON format fixed · SAP stand-in page · inspection test scenes |
| 2–3 | Walk + grasp in one world (hardest step) | Walk to a point · arm, hand, camera checks | Real SAP if access is there · inspection on real photos |
| **End of 3** | **Decision:** walk + grasp ≥ 5/10? If not → plan B (slower arm, re-measure before closing) | | |
| 4 | Full flow with carry + place, failure handling | Standing grasp at a fixed table | Bridge connects SAP ↔ robot program |
| 5 | **Sim demo ≥ 7/10** | First full attempt | Report checked against run log |
| 6 | Buffer · pitch | More attempts · lessons learned | |

## 9. (minor) Make the hardest step smaller: walk, stop, grasp — and place into a box

**Problem.** "Walking + grasping together" reads like grasping while walking. The repo's L3 plan is "walk, stop, grasp"; that is also
what the real factory controller allows safely. And a ball "lying still at the target spot" on a flat table will roll; LEARNING_PATH §10.7
already notes that dropping into a box is easier than placing upright.

**Text change** — in §3 last row and §7, replace "walking + grasping together" with "**walking to the object and grasping it in one
world (walk, stop, then grasp)**". In "Which object?", replace the last sentence with:

> **Suggestion:** the target spot is a **box or bowl** on the second table. Then a ball cannot roll away, and "placed correctly" simply
> means "inside the box".

and replace the risk "We need a second table or a marked spot / bowl" with "We need a second table (or one table + a box)."

## 10. (minor) In/Out: LiDAR "out" is fine, but then the real robot needs another way to find the tables

**Problem.** In sim the robot's position comes from the simulator. On the real robot it comes from odometry, which drifts. Placing at
± 10 cm cannot rely on odometry alone. The SAP stand-in is only a fallback in §7, but it is what unblocks parallel work.

**Text change** — in §4, replace the LiDAR row with:

| In | Out |
|---|---|
| Odometry for the rough way, **the camera for the last metre** (object, box, or a printed marker on each table) | A LiDAR map |

and replace "SAP: start the task, receive the report" with:

> SAP: start the task, receive the report — **via a stand-in page from week 1**, the real SAP system as soon as we have access.

---

Not checked here (other lenses): wording/plain English, the cost and compute figures, the SAP News citations.
