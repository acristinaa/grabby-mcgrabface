# Plan gate review: `.planning/real-robot/PLAN.md` (rev 1, 01-potsdam-bringup)

Reviewer: independent reviewer (opus). I did not write the plan and saw none of the planner's reasoning. Stage: plan review (pre-build).
Date: 2026-10-16. `.planning/REVIEW-LENSES.md` is not in this repo. I used the harness template
(`~/Documents/ClaudesOrdner/_harness/templates/project/.planning/REVIEW-LENSES.md`).
Constraints I kept: no sims, no GPU, no edits. I read code and fetched web sources (raw GitHub, `gh api`). Downloads are in
`/private/tmp/claude-501/rr-review/web/`.

## Verdict 2: are the factual claims correct? (re-derived)

| Claim | Result | Evidence |
|---|---|---|
| `rt/arm_sdk` uses motors 12-28 and the weight is in `motor_cmd[29].q` | **Supported** | GitHub master `example/g1/high_level/g1_arm7_sdk_dds_example.py` is byte-identical to the local clone at 65691c8 (`diff`; repo head is now 814556d, 2026-09-21, and this file is unchanged). `WaistYaw=12`, `WaistRoll=13`, `WaistPitch=14` ("INVALID ... waist locked"), arms 15-28, `kNotUsedJoint = 29 # NOTE: Weight` (lines 40-64). `arm_joints` = 14 arm joints + waist yaw/roll/pitch (lines 91-103). Weight is set at line 135, the ramp-out is at lines 164-168, kp 60 / kd 1.5 at lines 74-75. All the plan's line numbers are correct. **Context the plan leaves out:** both official examples set weight = 1 **on the first message**, with q = the measured angles (py line 135, `unitree_sdk2` C++ line 149), and so does xr_teleoperate (`robot_arm.py` `_ctrl_motor_state`: `kNotUsedJoint0.q = 1.0`). The C++ example ramps out only to 0.6 in 2 s and then writes 0 (C++ lines 239-255). The "linear blend" assumption therefore stays unverified, as the plan itself says. |
| BrainCo topics, types and value ranges | **Supported** | `brainco_hand_service` `main.cpp` (fetched from main, pushed 2026-06-01). Baud 460800 (line 24), `/dev/ttyUSB*`/`ttyHAND*`/`ttyUN*` (lines 32-34), topics `rt/brainco/<ns>/cmd` and `/state` with go2 `MotorCmds_`/`MotorStates_` (lines 6-7, 135, 139). q and dq are clamped to [0,1] and scaled by 1000 (lines 106-107). State q = pos/1000 and tau_est = current/1000 (lines 118-120). 100 Hz (line 146). The buffer is initialised to q = 0 and dq = 1 (lines 136-137). There is **no timeout**: `SubscriptionBase::isTimeout` exists (`include/dds/Subscription.h`) but `hand_worker` never calls it, so the hand keeps the last command. Finger order comes from the README. xr_teleoperate `robot_hand_brainco.py` lines 139, 141-142 and 290-294 match the plan. |
| `72_real_check.py` leaves motors 13/14 at kp 0 | **Supported** (and it also leaves q at 0) | `72_real_check.py:34` `ARM_SDK_JOINTS = [12] + list(range(15, 29))`, line 113 builds a fresh `unitree_hg_msg_dds__LowCmd_()`, and lines 117-119 write only those joints. `unitree_sdk2py/idl/default.py:210, 219`: every MotorCmd defaults to (mode 0, q 0, dq 0, tau 0, kp 0, kd 0). Under a linear blend at w = 1, waist roll/pitch go limp. |
| Sim head camera FOV, tilt and 200x320 | **Supported, with one gap** (see M6) | `payload.xml:51` `fovy="65"`, `xyaxes="0 -1 0 0 0 1"` gives a view along body +x. `payload.xml:47` gives tilt axis +y and range -0.349..1.571, and `payload.py:6-8` says positive = down. `table_scene.py:39` gives tilt 0.85 rad = 48.7 deg. `table_env.py:40` gives (200, 320) and `policy_wire.py:36` gives (200, 320, 3). Horizontal FOV = 2·atan(tan 32.5°·1.6) = 91.1°. The randomization is ±1.5 deg and ±5 mm (`configs/randomization.json:47-48`). Gap: §2.4 says "world pitch = torso pitch + tilt" and leaves out the 2.29° mount pitch of `xl330_link` (`payload.xml:39`, quat y 0.01998). |
| Joint-order mapping | **Supported** | `action_space.py:5-9, 34` (0-6 → 15-21, 7-13 → 22-28, 26 → 12) and `table_env.py:44`. Hand order 14-19 / 20-25 matches `brainco.MOTORS` (`brainco.py:16`) and the service order. The plan's §2.3 table is correct. |
| Nothing in the existing DDS sim consumes `rt/arm_sdk` | **Supported** | I grepped `~/dev/g1-ext/{lerobot,g1_brainco_sim,unitree-g1-mujoco,unitree_sdk2_python}`. The only hits are SDK *examples* (writers) and the H2 loco client. In the repo the only hit is `scripts/72_real_check.py`. The bridge subscribes only to `rt/lowcmd` and `rt/dex3/*/cmd` (`g1_brainco_sim/sim/unitree_sdk2py_bridge.py:76, 80, 84`). GR00T writes motors 0-14 (`controllers/gr00t_locomotion.py:199-205`), and the arms come from `send_action` (`unitree_g1.py:552+`). |
| (extra) "Whether Unitree honours waist yaw via arm_sdk is not documented" | **Partly outdated** | unitree_sdk2_python issue #156 (2026-05-25, G1_23, R1+X): "rt/arm_sdk usable, waist yaw controllable, the two arms' 10 joints not controllable". This is public evidence that waist yaw is honoured, and also evidence that arm control through arm_sdk depends on mode and firmware. |

## Verdict 1: is the plan right?

The overall architecture is right: arm_sdk only, no `rt/lowcmd`/Damp, ramps, a hard clip plus a demo envelope, a speed limit, a lead limit, stale watchdogs, signal-flag Ctrl+C, a sim-to-sim gate against a blend stand-in, and a read-only first step. The safety spec still has one internal contradiction that would produce unplanned motion at the very first ramp-in on hardware. It has no recovery path for the "our program died while w > 0" state that its own kill test creates. Its sim gate cannot pass as written. And it does not fit the stated time window. Details below.

### BLOCKER

**B1. The demo envelope is applied to scripted moves, so the gate forces either a ramp-in jump or an unpassable check.**
- Trigger: rule 5 turns the envelope (dataset min/max ± 0.10) "on by default" without saying which phases it covers. Task 2's done check says "every received target inside limits **and envelope** ... in every run (incl. F1-F7)", and F4 says "nothing beyond limits/envelope in 85's report". Meanwhile rule 3 sets the ramp-in targets to the **measured** angles, and rule 12 (soft exit) moves "back to the start pose".
- Evidence: `stats.json` shows shoulder pitch max -0.250 on both sides, so the envelope max is -0.15 rad. A hanging factory arm has shoulder pitch around 0 or positive. 85's "factory-like" sim pose is 0 for all arm joints (`GROOT_DEFAULT_ANGLES` = 0 for 15-28, `gr00t_locomotion.py:35-38`; `config_unitree_g1.py:54`). The start pose is therefore outside the envelope in sim, and almost certainly on the robot too. Other narrow dims: L shoulder roll -0.21..0.36, L shoulder yaw -0.10..0.75, R shoulder yaw -0.78..0.10.
- Impact: an implementer who follows the done check clips the ramp-in targets. Then, as w rises from 0 to 1, the arms are pulled ≥ 0.15-0.4 rad toward the clipped pose during the step the operator expects to be motionless. That likely trips the 0.35 rad lead fault, or it moves the arm towards a person on a "nothing should move" step. If the implementer does not clip, the gate fails. The soft-exit "return to start" can never reach the start, and the envelope streak (30 ticks) fires during `ready`.
- Fix: limit the envelope to the **policy/replay phase** only. Ramp-in, move-to-ready, return-to-start and ramp-out use hard limits plus the speed limit only. 86 tags every message with its phase in the log, 85's report records it, and the gate checks "inside envelope" only for `phase == policy`. Add a unit test: ramp-in from a start pose outside the envelope produces command == measured on every tick. Pin `BALL_READY_POSE` inside the envelope with a test (right shoulder pitch -0.25 sits exactly on the data min/max, so it is inside only because of the 0.10 margin).

### MAJOR

**M1. There is no recovery path after the adapter dies with weight > 0, and the kill test creates exactly that state.**
- Trigger: step 2 `kill -9`s 86 at the ready pose. The same state comes from a Mac crash, a cable pull or a LowState-stale fault, because the ramp-out messages never arrive. Q11 (what Unitree does when arm_sdk stops) is unanswered, and the public doc is JS-only.
- Impact: if Unitree **latches** the last message, the arms stay at the ready pose with w = 1. Every later 84/86 start begins its ramp at w ≈ 0. That first message drops w from 1 to about 0, and the arms snap from the ready pose to the factory pose under factory gains. If Unitree instead **times out and drops to w = 0**, the same snap happens at kill time. The plan says nothing about what to do after the kill.
- Evidence: rule 3 ("in 0 -> 1 over 2 s"). Step 2 has no follow-up action. Unitree's own examples start at w = 1 with q = measured (py line 135, C++ line 149), which is jump-free whatever the controller's current state.
- Fix: (a) add a `release` command (84 and 86): the first message is w = 1 with every SDK motor at its measured q and `SIM_GAINS`, then ramp 1 → 0 over 2 s. Make it the documented first action after any crash or kill. (b) At every start, compare the arm pose to the factory pose recorded in bring-up step 0. If they differ by more than 0.1 rad, refuse the 0 → 1 ramp and point to `release`. (c) Do the kill test first right after the single-joint test (elbow +0.25, close to the factory pose), not at the ready pose, so the possible snap is small. Write both outcomes and the next command into the runbook.

**M2. A fault exit keeps pushing into whatever blocked the arm.**
- Trigger: rule 7, a lead > 0.35 rad for 0.5 s, i.e. a blocked arm or contact (possibly a person). Rule 12 sets the fault-exit targets to the last command while w ramps out over 2 s.
- Impact: the arm keeps pressing with up to kp × lead (80 × 0.35 ≈ 28 Nm at the shoulder), decaying only with w over 2 s, after 0.5 s of pressing before the fault even fires.
- Fix: on lead/contact faults, set the targets to the measured q at fault time (stop pushing), then ramp out. Keep "last command" only for a stale LowState, where no measured angles exist. Consider 0.25 rad / 0.25 s for the lead fault. Add a unit test.

**M3. The preflight "LowState ≥ 200 Hz" cannot pass in the DDS sim.**
- Evidence: `runs/real_probe.json`, recorded against this sim on lo0: `"lowstate_hz": 127.8`.
- Impact: Task 1's sim run (probe must PASS) and every 86 preflight in Task 2's e2e get refused. The implementer then either loses Saturday time or quietly weakens the check.
- Fix: make the threshold a named constant per iface (real: ≥ 400 Hz, since the G1 publishes 500; lo0: ≥ 100 Hz) and test it. Keep the 0.1 s stale limit; it is fine at 128 Hz.

**M4. The "foreign `rt/arm_sdk` writer" check uses DDS discovery, so it will likely refuse on the real robot.**
- Trigger: `71` reads `BuiltinTopicDcpsPublication` (`71_real_probe.py:17, 45`), which lists writers whether or not they publish. BrainCoTech switches off Unitree's built-in arm service (`ServiceSwitch("g1_arm_example", 0)`) because it conflicts with `/arm_sdk` (plan §2.1). A resident but idle writer of that kind is plausible. I could not confirm this, so it is unverified.
- Impact: step 0 FAILs, and "abort: a foreign `rt/arm_sdk` writer" stops all motion for the day, or forces a service switch-off on the owner's robot.
- Fix: refuse only when **samples** arrive on `rt/arm_sdk` (or `rt/lowcmd`) within 2 s, or any sample with w > 0. Log discovered-but-silent writers (participant GUID) as a WARNING for the owner. 86 must run this check before it creates its own writer.

**M5. The waist goes from a single +0.10 rad test straight to ±0.55 rad replays, and R7 has no switch.**
- Evidence: §2.6 waist range ±0.55 rad (dataset -0.714..0.680, so the envelope is about ±0.8). Step 2 tests only +0.10, and step 4 then replays the full demo. R7 ("hold the waist at 0 for the rest of the day") is a bring-up rule, but 86's flag list (§5 Task 2 step 3) has no waist option.
- Impact: the first large torso yaw on a balancing robot carrying hands and arms forward happens in an uncontrolled jump. And the R7 fallback cannot actually be selected on the day.
- Fix: add `--waist-max RAD` (default 0.15 on a non-lo0 iface, then 0.35, then the envelope) and `--waist-hold` (the R7 implementation: motor 12 is held at its start angle and the policy's waist channel is logged but not sent) to 86. Cover both in a unit test and one e2e run. Add a waist-escalation line to step 4.

**M6. The head-camera tilt target is wrong by about 1.3° in world terms.**
- Evidence: in the sim the optical axis's world pitch below horizontal is d455 tilt 48.7° + `xl330_link` mount pitch +2.29° (`payload.xml:39`, `quat="0.99980033 0 0.01998267 0"`, +y = nose down) + pelvis pitch -0.96° (`table_scene.py:72`, euler about +y, negative = nose up; the waist chain has no extra rotation, `g1_29dof_no_hand.xml:225-239`), about **50.0° down**. Plan §2.7 and step 1 set the tilt to 48.7° and check it with a phone inclinometer, which measures *world* pitch.
- Impact: this uses up almost the whole ±1.5° randomization before any real-world error. That matters for Tier C, which is already a long shot.
- Fix: compute the world pitch of `head_camera` by forward kinematics at the handover stance (mj_forward only, no stepping; or by hand as above) and print that number in the runbook as the inclinometer target. Set the D455 by world pitch, not by servo angle.

**M7. The cut line is not credible and there is no fallback for Tier B.**
- Evidence: §6 puts the cut at "Sat 20:00", three hours after the stated window ends (Sat 10-17). The schedule is 12-13 h wall clock against about 13 h available, including the 1 h plan review, so there is zero slack. Code review (2 h) and fixes (1.5 h) are placed after the e2e, which leaves no room for a second review round. The cut line only drops Tier C. Nothing says what happens on Sunday if the Tier B e2e or its code review is not GO.
- Fix: add checkpoints. By Sat 13:00 85 and 86 must run the e2e. By Sat 15:00 the e2e is green and the review has started; otherwise Sunday runs Tier A only (84 including the `fill_arm_sdk` single-joint test) plus `release`. Add the rule "no code that has not passed its review gate touches the robot". Cut these first because they are over-built for 1.5 days: (1) the FrameStream binary protocol plus SSH ControlMaster for Tier A, where one 1280x800 frame per check is enough and streaming is Tier C only; (2) the MuJoCo pose-mirror render (see m1; low value); (3) e2e fault runs F3-F5, which duplicate the SafetyFilter unit tests (keep F1, F2, F6 and F7, the process-level behaviours); (4) 84 driving 72 as a subprocess and parsing its German stdout. Re-implementing the LowState reader and the hand read/move (about 30 lines in `real_io`) is less brittle than parser pins.

### MINOR

- **m1. The pose mirror cannot detect sign errors on joints that sit near 0** (wrist roll/yaw and shoulder yaw in a hanging pose, and also 0 in `BALL_READY_POSE`). The step-0 abort ("mirror does not match → order/sign error") claims more than the check can show. Fix: add an 84 `arm --sweep` that moves each of the 14 arm joints +0.10 rad, about 11 s each, compared against a sheet of sim-rendered "+ direction" images. This replaces the mirror render if time is short.
- **m2. Rule 12 does not say what the hands do on exit.** The service keeps the last command forever (`main.cpp:106`, no timeout). Spec: a soft exit returns the hands to their start values together with the arms, and a fault exit holds them so the ball is not dropped. Put a one-line "open both hands" command (`86 hands-open` or a 72 variant) in the runbook.
- **m3. GRABBER runs under PC2's `python3`.** An Orin NX on JetPack 5 is typically Ubuntu 20.04 / Python 3.8 (unverified). Add a test `ast.parse(GRABBER, feature_version=(3, 8))`, use numpy + cv2 only, and add a version print to the camera step.
- **m4. 1280x800 YUYV at 30 fps is about 61 MB/s, which needs USB 3.** The camera step's "≥ 10 fps" is stricter than the policy needs (one frame per request, every 25 actions ≈ 0.8-3.3 s). Use ≥ 5 fps, default `--fps 15`, and record `lsusb -t` speed. Add "is the D455 already opened by another process (realsense node)?" to Q2.
- **m5. `fake_camera.py` writes already-converted RGB, so the e2e never exercises `to_policy_frame`, and a BGR/RGB swap would pass every gate.** Add a unit test with pure red/blue BGR input.
- **m6. 85 must blend into a copy of the LowCmd.** `publish_lowcmd` reuses the persistent `self.msg` (`unitree_g1.py:250-266`), and arms 15-28 are only rewritten by `send_action`. An in-place blend makes the "own" arm targets drift to the SDK values, so the sim ramp-out never returns to the factory pose and the gate measures the wrong thing.
- **m7. Refuse `--fast` and `--speed > 0.5` on a non-lo0 iface on day 1** (enforces R4 in code, not only in the runbook).
- **m8. Process robustness:** treat SIGTERM and SIGHUP like Ctrl+C. Run under `caffeinate -i` with the charger on. Add a gate criterion from 85's report: the maximum gap between arm_sdk messages is < 60 ms in G2. This matters because 86 runs a 500 Hz LowState callback, a frame reader and a policy thread under one GIL.
- **m9. Robot mode depends on firmware.** 72's "R1+X" note may be wrong: issue #170 (G1 EDU+ 29 DoF, software 1.5.3) says R1+X is rejected and arm_sdk works in R2+A; issue #156 says that in R1+X on G1_23 only the waist responds. The runbook should record the firmware version (Q10) and agree the mode with the owner. BrainCoTech recommends single-waist mode for upper-body work.
- **m10. Ball stand:** the 40k candidate's most common sim failure is the little finger brushing the stand. The real stand should be light and compliant (a cardboard tube, 8 cm high, 10 cm inner diameter) so that a brush neither hurts the Revo2 fingers nor counts as a "table contact" abort in step 5/6. Say this explicitly in the abort rule.
- **m11. Mac side:** add the macOS application firewall (allow incoming connections for the venv python, or DDS samples never arrive) and SSH key setup on PC2 (only with the owner's OK) to the packing/runbook list.
- **m12. Housekeeping:** the eval is finished, so the §0 gating and the "40k or 45k" wording are stale (the candidate is 40k, 36/50). Step 2 cites "Q12", but the table ends at Q11. `.planning/REVIEW-LENSES.md` is missing from the repo.

## Lens walk (harness template)
1. Guard coverage: the `finally`/signal ramp covers in-process exits. It does not cover the restart *after* an uncovered exit (M1). The fault ramp keeps the wrong target for contact (M2).
2. Newly reachable: the first real waist yaw beyond 0.10 rad (M5). Ramp-in with targets outside the envelope (B1). The state "w = 1 latched, no sender" after the deliberate kill (M1).
3. Meaning changes: the envelope is meant for policy targets but would apply to scripted targets (B1). A discovered writer is read as an active controller (M4).
4. Can the primitive express it: `71`'s DCPSPublication discovery cannot tell idle writers from active ones (M4). `fake_camera` cannot express a raw camera frame (m5).
5. Shared consumers: `unitree_g1.py`'s persistent `self.msg` is shared by GR00T and `send_action` (m6). Nothing existing is modified, which is good.

## Checks I could not run (keep in mind on the day)
- What Unitree's controller does when arm_sdk messages stop (latch vs timeout): the official doc is JS-rendered and no public source states it. The plan's kill test plus M1's `release` command covers both cases.
- Whether a resident `rt/arm_sdk` writer exists on the owner's G1 (M4), PC2's Python version (m3), and the D455's USB link speed (m4).
- Whether G2's tracking criterion (median < 0.05 rad) holds with the DDS sim at about 0.5x real time. I did not run sims, as instructed.

## Overall: **NO-GO**
One blocker (B1) and seven majors. All of them are plan-text changes plus small additions (`release`, waist flags, phase tagging, per-iface threshold, sample-based writer check, FK tilt number, checkpoints). None needs new research. A revision that addresses B1 and M1-M7 should be re-reviewed quickly (about 30 min) before the build starts.

---

# Re-review (revision 2)

Same reviewer, same constraints: no sims, no edits to the plan or code. I checked every item against the plan text, not against the
"Review response" table. Rulings R1-R8 are treated at their recommended defaults. Line numbers refer to PLAN.md rev 2 (593 lines).

## Status of the earlier findings (checked in the plan text)

| # | Verdict | Where in rev 2 / remark |
|---|---|---|
| B1 envelope on scripted moves | **Fixed** | Rule 5 (lines 248-251, policy phase only), contract phases (229-230), Task 2 step 1/4/5 (phase arg, ramp-in test, `BALL_READY_POSE` pin, sent_log), done (412-413). The fix wording itself introduces N1. |
| M1 no recovery after death with w > 0 | **Partly fixed** | `release` (201-202, 301-302, 367), factory-pose start check (239-240), kill test next to the elbow test (481-483), e2e K (400-401), runbook outcomes (426-427). New problems: N2 (release's own ramp-out swing, plus the lead check inside the blend) and N4 (the factory-pose record can be overwritten or stale). |
| M2 fault keeps pushing | **Fixed** | Rule 12 (266-269), `fault_targets` (357-358), unit test (384-385). The threshold choice is discussed under n2. |
| M3 200 Hz vs sim 127.8 Hz | **Fixed** | `LOWSTATE_HZ_MIN` (194-195), rule 2, Task 1 done (340). See n1 on the 400 Hz value. |
| M4 discovery-based writer check | **Fixed for `rt/arm_sdk`** | `active_writers` counts samples (207-208, 237-238, 296-297) and has a unit test (324-325). Residual unverified risk on `rt/lowcmd`, see n3. |
| M5 waist escalation / R7 switch | **Fixed** | Rule 6b (254-256), flags (377-378), e2e W (399), step 4 escalation (488-490). Gap at step 5, see n4. |
| M6 tilt target | **Fixed** | §2.4 (138-142), §2.7 (174), runbook (427). The wording slip is in n6. |
| M7 schedule / cut line | **Not fixed** | Checkpoints and fallbacks exist (456-461), but they contradict the plan's own effort numbers and the user's window. See N3. |
| m1-m12 | **Fixed** | m1: sweep (316-319, 129-131). m2: rule 12 + `hands-open` (259-260, 266-268). m3: 217, 323. m4: 146-147, 303-305. m5: 322-323. m6: 155-158, 361-363. m7: 253, 378, 385. m8: 265, 275-276, 382. m9: 66-68, 473. m10: 173, 492 (see n5). m11: 429-430. m12: 42-43, 36, 69. |

## New findings introduced by revision 2

### BLOCKER

**N1. Ramp-in re-reads the measured angles every tick, which removes all position stiffness as w approaches 1.**
- Text: rule 3 (lines 243-244) says "every SDK motor targeted at its **measured** angle **on every tick** (command == measured, no motion)". The
  unit test (383-384) says "command == measured on every tick", and the done check (413) says "ramp-in commands equal the measured angles".
- Mechanism: if the target at tick t is q(t − 20 ms), our PD part produces kp·(q(t−dt) − q(t)) ≈ −kp·dt·q̇. That is extra damping
  (about 1 Nm·s/rad at kp 50) and **no spring**. Under the linear blend the arm is held only by the factory controller's (1 − w) share.
  As w → 1, the arms sag under gravity. A rough estimate for an arm with a Revo2 hand is about 8 Nm at the shoulder, with only kd
  holding it back. If an implementer also applies this to 13/14 (rule 3 says "every SDK motor", rule 4 says "start angle"; the
  two contradict each other), the 3-DoF waist pitch carries the whole torso with no spring.
- Reachable: the first `86 ready` on the robot (step 2, line 484), which is the first 86 motion. 84 is not affected, because it reuses 72's
  `arm_profile`, where `hold` is captured once (`72_real_check.py:106`), and `release_sequence(measured, ...)` also takes the angles
  once. The gate would not catch it: 85's report checks tracking only at w = 1 in G2, and "ramp-in commands equal the measured
  angles" *passes* for exactly this behaviour.
- Fix (one line plus a test): freeze q0 = the measured angles at the **first** ramp-in tick and command q0 for the whole ramp. Change the
  unit test to "command == q0 on every tick". Add gate checks: the arm and waist (12-14) move < 0.03 rad during `ramp_in` in 85's
  report, and the e2e test first fails on a planted per-tick re-read.

### MAJOR

**N2. `release` can itself cause a fast swing, and the 0.25 rad lead check trips inside the blend.**
- `release` is used precisely when the arms are away from the factory pose (rule 2: gap > 0.10; after a kill near the ready pose the
  gap can be about 1 rad). `release_sequence` (201-202) sends w = 1 at the measured angles, which is jump-free, and then goes
  **straight into a 2 s smoothstep 1 → 0**. Under a linear blend the arm then travels the whole gap to the factory pose in 2 s:
  peak ≈ π/2 · gap / 2 s, which is about 0.8 rad/s for a 1 rad gap. That breaks rule 6's 0.3 rad/s for scripted moves, and it happens
  at the moment people are standing close because something just went wrong.
- During that ramp the command stays at the release angles while the measured arm moves away under the factory share. So
  |command − measured| grows past 0.25 rad and rule 7 fires a contact fault in the middle of the release. The same applies to any
  `ramp_out` that starts away from the start pose (soft exit after the 6 s cap). Rule 7 (257) does not say in which phases it applies.
- Fix: `release` = (1) w = 1 at the measured angles, (2) move at w = 1 to the recorded factory pose at 0.3 rad/s with the lead check
  active, (3) ramp out. Without a factory record, do the ramp out only if the gap is < 0.10, otherwise ask the operator. Turn the lead check off
  while w < 1 (`ramp_in`, `ramp_out`, the release ramp). In those phases the measured arm legitimately departs from our command.
  e2e K: check the peak joint speed during the release (< 0.35 rad/s), not only the first message.

**N3. The checkpoints are infeasible by the plan's own effort numbers, and Saturday is longer than the user's window.**
- The user's window is Sat 10-17. Line 55 says "Saturday 10-17 (~09:00-19:00)" and §6 plans 09:00-19:00 (455-460), which is 10 h
  instead of 7.
- Critical path is track B. Friday has about 3.5 h after the re-review, enough for real_safety (2.25 h) plus part of 85. Saturday
  needs the rest of 85 (≥ 0.75 h) + 86 (3.25 h) + e2e (1.25 h). "Checkpoint 13:00: 85 and 86 run the e2e" therefore needs 4 h of work
  done before 13:00 even with a 09:00 start and 85 started on Friday, which the plan does not schedule: zero slack. With a 10:00 start the
  checkpoint misses by about 1 h.
- After e2e green (realistically about 14:30-15:30): review 2 h + fixes 1.5 h + re-check comes to about 19:00 at best, 2 h past 17:00. Track B
  alone is about 12.5 h serial (8.75 + 2 + 1.5 + re-check), more than the stated "11-12 h wall clock". On these numbers the most likely
  outcome is the plan's own fallback, "Sunday = Tier A only". That is safe, but the plan presents it as the exception.
- Fix (pick one, and confirm the Saturday hours with the user): (a) make 85 its own track starting Friday, since it depends only on the
  DDS sim and the arm_sdk message, not on real_safety; (b) review real_safety + 86 file by file as they land, not after the e2e;
  (c) state honestly that Sunday will probably be Tier A + `release` + `86 ready` only, with the replay as a stretch. Move the
  checkpoints to times that fit the effort (for example 14:00 / 16:00 with a 10:00 start), or reduce scope.

**N4. The factory-pose record that M1's protection depends on can be stale or silently overwritten.**
- `joints` writes `runs/real_factory_pose.json` every time it runs (299-300). The file path is the same for sim and robot (196). If anyone
  re-runs step 0 after a kill with the arms latched (a natural "let me re-check the joints"), the latched pose becomes the "factory
  pose". The start check then passes, and the next ramp-in from w ≈ 0 against a latched w = 1 gives exactly the snap that M1 was
  meant to prevent. A pose file left over from the sim e2e would produce false refusals on the day. `factory_pose_gap` returns
  `None` when there is no file (211), and rule 2 does not say what happens then.
- Fix: record the pose only with an explicit `--record-factory-pose`, only on a non-lo0 iface, only right after a `release` or at
  the first step 0 of the day. Store iface, date and robot id, and refuse to overwrite a record that differs by more than 0.10 rad
  without an operator "yes". On a non-lo0 iface, `None` means refusal ("run 84 joints first"). Use a separate file for the sim.

### MINOR
- **n1. 400 Hz robot threshold.** It is a refusal threshold with no safety value: staleness (> 0.1 s) is the real guard, and a 50 Hz
  sender is well served at 200 Hz. A Python receiver behind a USB Ethernet adapter, or a firmware that publishes below 500 Hz (I have
  not verified the 500 Hz figure here), would then cancel the day at step 0 (line 476). Fix: refuse below 200 Hz and only warn below 400 Hz.
- **n2. Lead limit 0.25 rad / 0.25 s during the policy phase.** At 0.25x speed the velocity lag is small, and the walking controller has
  no share at w = 1, so it is not the source. Gravity sag is: about 8-9 Nm at the shoulder with the arm forward and the ball, divided by
  kp 50, is about 0.17 rad (estimate). That is close to 0.25. G2 accepts sim tracking up to 0.20 rad (410), which leaves only 0.05 margin
  to a trip. False trips are safe (they stop the robot) but cost time. Fix: gate on max lead < 0.15 rad in G2. On the day, log the lead
  during `86 ready --hold 5`; if it is > 0.10 at hold, stop and discuss gains (R2) before replaying.
- **n3. Samples on `rt/lowcmd` (robot) are still a hard refusal.** Whether Unitree's own normal-mode processes publish `rt/lowcmd` on the
  external DDS domain is unverified. If they do, step 0 cancels all motion. Fix: `rt/lowcmd` samples give a WARNING plus an operator/owner
  decision, recorded in the bring-up json. Samples on `rt/arm_sdk` stay a refusal.
- **n4. The step 5 waist limit is unspecified** (491-492), so the robot default 0.15 applies. A demo trajectory replayed with its waist
  clipped puts the hands and ball where the demo never went: near the stand, the basket and the table edge. Fix: step 5 uses the limit
  reached in step 4. If that is < 0.8, do only the no-ball clearance pass at that limit.
- **n5. A light stand plus "brush logged, not an abort" does not weaken the automatic rules.** The contact fault (rule 7) still fires
  on any blocking contact, and a light stand yields instead of blocking. It does turn a clear operator rule into a judgement call. Fix: define a
  brush as "no contact fault and the stand moved < 2 cm". If the stand is knocked over, the ball rolls off, or a finger snags, abort
  the attempt. Do not fix the stand to the table.
- **n6. Small text slips.** "D455 top face, which is normal to the optical axis" (141): the optical axis lies in the top face's plane. The
  method (phone flat on the top face reads the axis pitch) is right, but someone reading the sentence literally might put the phone on
  the front face. Step 2 uses `84 arm --delta 0.10` (482), but Task 1 names no delta flag (only `|delta| <= ...`, 315). The sweep moves
  ±0.10 "away from the nearer limit" while the sheet shows "+0.3 rad" (317-318), so the sheet should show each joint's actual sweep
  direction.

## Answers to the coordinator's specific questions
- **Can `release` cause a snap?** Its first message cannot: w = 1 at the measured angles is jump-free whether or not the controller latched,
  and it matches Unitree's own first message. Its ramp-out can: a 2 s blend over the full gap gives about 0.8 rad/s for a 1 rad gap,
  and the lead check fires in the middle of it (N2). The factory-pose start check is sound in principle, but its record can be
  corrupted (N4).
- **0.25 rad / 0.25 s false trips?** Not from the walking controller's arm motion at w = 1. Yes inside every w < 1 phase that starts away
  from our command (release, late ramp-out; N2). Possibly from gravity sag at extended poses (n2).
- **400 Hz:** risky for availability and adds no safety (n1).
- **Light stand + logged brush:** no automatic safety rule is weakened. It needs a crisp definition (n5).
- **Timeline:** does not fit as written (N3).

## Checks I could not run
- Unitree's latch/timeout behaviour, the real LowState rate seen by a Python receiver on the Mac, whether `rt/lowcmd` has samples in
  normal mode, and real gravity sag at kp 50 are all only knowable on the robot. I ran no sims, so N1's sag magnitude in 85 is
  derived from the control law, not measured.

## Overall (revision 2): **NO-GO**
One new blocker (N1, a one-line spec fix plus a test) and three new majors (N2, N3, N4). B1, M2-M6 and all minors from the first round are
fixed. M1 and M7 are only partly fixed (N2/N4 and N3). A rev 3 that freezes q0 at ramp-in, makes `release` move at 0.3 rad/s before
ramping, scopes the lead check to w = 1, protects the factory-pose record, and re-times Saturday against the user's actual hours
should pass quickly.

---

# Re-review (revision 4)

Same reviewer, same constraints: no sims, no edits to the plan or code. This was a time-boxed review of about 20 minutes. Line references
are to PLAN.md rev 4 (509 lines).

## 1. Do the revision 2-3 safety fixes survive, and do they apply to `real_motion` / 88?
Yes. **N1**: q0 is read once by `freeze()` (contract, `real_motion`); rule 3 applies it to 12-28, and a unit test uses a
sagging fake reader that first fails on a per-tick re-read (Task 3 step 4). **N2**: `release_sequence` holds, then moves at 0.3 rad/s,
then ramps out; `LeadMonitor` counts only at w == 1.0; the gate requires a release peak ≤ 0.35 rad/s with no contact fault. **N4**:
rule 15 keeps separate robot and sim files, `--record-factory-pose` only right after a release, no silent overwrite, and a missing
record means refusal except for `release`. **n1**: 200 Hz refuse / 400 Hz warn. **n3**: `rt/lowcmd` samples give a warning. **n6**: the
wording slips are fixed (§2.4 "axis lies in that plane", 88 `--delta`, the sweep sheet shows the actual direction). 88's `arm` reuses
72's `arm_profile` at q0, which is jump-free. Rules that only matter for replay/VLA are rightly moot. **No regression found.**

## 2. Is the Tier M gate safe for operators who are not the author?

**R4-1 (major). The gate is accident-proofing, not tamper-proofing, and it covers too few files.**
- Rule 16 diffs only `real_motion.py`, `real_io.py` and `88`. Not covered: 88 imports `arm_profile` from `scripts/72_real_check.py`,
  plus `g1sim/action_space.py`, `brainco.py` and whatever `real_motion` imports, and the editable `unitree_sdk2py` install.
- `docs/ROBOT_DAY_MOTION.json` itself is not checked. Setting `"motion_allowed": true, "commit": "<HEAD sha>"` locally, with or without a
  commit, passes the gate. No one can stop someone determined, but one "helpful" edit on site is enough today.
- Fix: diff against the pinned sha **every repo file loaded into `sys.modules`** (computed at runtime) plus the JSON. Require the
  JSON to be unmodified and HEAD to equal the pushed commit named in the runbook. Say in the runbook that the gate guards against
  accidents, not misuse.
- Stronger fix, and my recommendation (see below): if Track M is not shipped, **do not push 88/`real_motion` to the operators at all**.
  A file that is not on the laptop cannot be run around a gate.

**R4-2 (major). Failure half-way through a motion step has no on-site recovery beyond `release`.**
- If a fault or crash leaves the arms latched at w = 1 and `88 release` then refuses or crashes, nobody on site can debug it. The
  runbook's only fallback is "the owner uses the remote". Rule 1 allows damping only while the robot **hangs** in the harness, but
  step 1 says "standing, harness on" (slack).
- The plan also has the operators run the **kill test** on Sunday (step M). That deliberately creates the latched state, whose
  recovery depends on never-hardware-tested code working the first time.
- Fix if motion stays: drop the kill test from Sunday. Require the robot to hang in the gantry (not stand on slack) for all of step M,
  so the owner's damping is always a safe last resort. Add a runbook box "release refused or crashed → owner: remote; we do nothing more".

**Recommendation on motion: off on Sunday (R9 = "no motion from our code").** My reasons:
- Not one line of the publishing code (`fill_arm_sdk`, blend assumption, latch handling) will have touched real hardware.
- The controller's latch and blend behaviour is unpublished.
- Recovery depends on that same code.
- The motion review happens at about 23:00-00:00 by one seat.
- The author is unreachable.
- Nobody on site can tell a firmware quirk from a bug.

The learning goals of Tier M can mostly be reached read-only: the owner runs their own known-good arm motion (their tool, their
responsibility) while `84 record` logs LowState. That shows the mode/firmware behaviour, the factory pose and joint directions. The finger
test is low risk, but keeping one simple rule ("our code sends nothing to the robot") is easier for non-authors to follow than a
partial exception. Build Track M later (or tonight only if A and H finish early) for the next session with Finnegan present.

## 3. Is "84 never publishes" enforceable as described?
**R4-3 (major). Not as specified: the plan contradicts itself, and grep is too weak.**
- Rule 1 says `real_io`, `real_camera`, `84` **and `89`** contain no `ChannelPublisher`. Task 2 step 3 says 89 publishes a LowState_
  for the loopback round trip. As written, the never-cut static test either fails or gets satisfied by moving the publisher somewhere
  grep does not look.
- Grepping named files also misses `cyclonedds.pub.DataWriter` (a publisher without `ChannelPublisher`), transitive imports (any
  g1sim module that publishes, e.g. `brainco_env`), and `getattr` tricks.
- Fix:
  - (a) Put the loopback publisher in a helper under `tests/helpers/` that refuses any non-loopback iface, and have 89 call it as a
    subprocess.
  - (b) Static test: ban `ChannelPublisher`, `DataWriter`, `Publisher(` and `.Write(` in 84/89/real_io/real_camera.
  - (c) Import test: run `84 --dry-run` in a subprocess and assert that `sys.modules` contains no `real_motion`, `brainco_env` or
    `cyclonedds.pub`.
  - (d) Runtime guard: 84 replaces `unitree_sdk2py.core.channel.ChannelPublisher` with a function that raises at start-up.
  - (e) Keep the existing behavioural check (85 sees zero arm_sdk messages during 84), and add a discovery check that 84's participant
    owns no writer on any topic.
- **R4-4 (minor):** 89's loopback publisher must also refuse to run while any interface has a 192.168.123.x address. If CycloneDDS
  fell back to a real interface, a fake `rt/lowstate` would land on the robot network.

## 4. Can the runbook, setup script and self-test be validated tonight?
**R4-5 (major). The Linux path will hit the one untested branch: cyclonedds 0.10.2 has no Python 3.12 wheel.**
- Evidence: PyPI `cyclonedds/0.10.2` lists wheels for cp37-cp310 only (macOS x86_64/arm64, manylinux x86_64, win), plus an sdist.
- With the plan's Python 3.12 pin, `pip install cyclonedds==0.10.2` builds from source on **every** OS and needs the C library and
  `CYCLONEDDS_HOME`. That is exactly the fallback that is untested on Linux. On top of that, Ubuntu 22.04 has no `python3.12-venv`
  without a PPA.
- Fix: robot env on **Python 3.10** (manylinux and macOS arm64 wheels bundle the C library, so no C build). The code it imports is
  3.10-clean: I checked `ast.parse(feature_version=(3, 10))` on `episode`, `policy_wire`, `action_space`, `brainco`, 71 and 72 (all ok).
  Change the GRABBER/real_* tests to `feature_version=(3, 10)` for the laptop side. Keep 3.12 only for the policy env.
- What tonight can and cannot validate:
  - Setup `--robot-env` on a fresh macOS prefix: yes (and with 3.10 it is just a wheel install).
  - `--policy-env` on a fresh prefix: the plan's verify reuses the existing SmolVLA env, so the setup block is **not** exercised even
    on macOS. Run it tonight, about 2.7 GB plus torch, in the background.
  - `--release`: cannot run before the release exists. Test it against a **draft** release or a local tar plus `shasum -c` plus unpack
    path.
  - The runbook dry walk on macOS: yes.
  - Linux/CUDA: no. Saturday's self-test by the operators is the only check, so the runbook must say who reads the result if Finnegan
    is travelling: `SELFTEST: PASS` is the self-contained criterion, and on FAIL Sunday is questions plus photos.
- **R4-6 (minor):** `PROGRESS.md` and `docs/START_PROMPT.md` still show 70/72 commands. In the integration step, mark 70, 72 `arm` and
  72 `hand --bewegen` as "never on the robot day" so a teammate browsing the repo does not pick them up.

## 5. Does the 6 h, three-track plan fit, and is the cut order right?
**R4-7 (major). Not by its own numbers.**
- Track A is 4.5 h of build, but C1 at T0+3.5 requires "84 sim run done".
- Track M is 5.5 h (6 with the sheet), but C2 at T0+4.5 requires "e2e green" before the motion review even starts.
- The motion review (0.75 h at 23:00), fixes and re-check end at about 00:30 at best, which is then the GO that lets the user flip
  motion on at midnight.
- 19.5 agent-hours in 6 h with one review seat has no slack, and the user's release upload (865 MB) and pushes come last.
- **The cut order is wrong:** item 1 (sweep sheet) only matters if Track M exists, and Track M itself is item 4. Track M should be cut
  **first**; that matches the recommendation above. Without M the plan's own estimate is about 11 agent-hours, about 4.5 h wall clock,
  which fits 18:30-00:30 with about 1.5 h slack for review fixes and the dry walk.
- Proposed order: (1) Track M (and with it the sweep sheet); (2) the FOV intrinsics sub-check; (3) `record --policy` plus the policy env;
  (4) the `cuda` line; (5) the dry walk. The never-cut list is right.

## Overall (revision 4): **NO-GO as written, with a short path to GO**
- If the user rules **R9 = no motion on Sunday** (my recommendation) and Track M is not shipped, the remaining blockers to GO are:
  R4-3 (fix the 89 contradiction and strengthen the no-publisher enforcement), R4-5 (Python 3.10 robot env), and R4-7 (cut order and
  checkpoints for A and H). R4-1 and R4-2 then fall away.
- If motion stays: R4-1, R4-2 and R4-7 must also be fixed, and the Track M GO should not be given at midnight under time pressure.

**Not checked:** the Linux setup, CycloneDDS on Linux `lo` for 89's loopback test, and how long the GitHub upload takes. The
operators' Saturday self-test is the only check for the first two.

---

# Re-review (revision 5)

Same reviewer, same constraints: no sims, no edits to the plan or code. This was a focused review of about 20 minutes. Working default:
R9 = no motion from our code on Sunday, and Track M is deferred. Line references are to PLAN.md rev 5 (438 lines).

## 1. Are R4-3, R4-5, R4-7 and the minors really fixed (plan text)?
- **R4-3: fixed.** Rule 1 a-d (lines 143-156):
  - (a) static test over real_io, real_camera, 84, 89 **and 71**, with widened names, ast plus a text pass, seen failing first;
  - (b) import check;
  - (c) start-up guard;
  - (d) `own_writers == 0` recorded per run, plus sim_host counting 0 `rt/arm_sdk` samples.
  - The loopback publisher moved to `tests/helpers/loopback_lowstate_pub.py`, called by 89 as a subprocess, so the 89 contradiction is
    gone.
  - **The planner's correction to my R4-3 (c) is right.** I re-derived it from the local SDK at `65691c8`: `core/channel.py:8` does
    `from cyclonedds.pub import DataWriter` at module level, and `ChannelPublisher` builds its writer with that module-global name
    (`channel.py:127`). So `cyclonedds.pub` is always loaded, and patching `channel.DataWriter` catches every `ChannelPublisher`, including
    copies that other SDK modules imported earlier, because the lookup happens at call time. Patching `cyclonedds.pub.DataWriter` covers
    direct use.
  - The guard is in-process only. 71 runs as a subprocess without it, but 71 is in the static list and only creates readers and the
    builtin discovery reader (`71_real_probe.py:17, 45`), so that is acceptable.
- **R4-5: fixed.** Robot env on Python 3.10 with `--only-binary cyclonedds` (lines 105-110, 228-229), so a missing wheel fails loudly.
  Policy env stays 3.12 via uv on Ubuntu. Both setup blocks and the release path (`--release-file`, the same code as `--release` after the
  download) actually run tonight (lines 268-279).
- **R4-7: fixed in structure** (A + H only, clock-time checkpoints, cut order with Track M already cut and the FOV check first). The
  per-agent load is discussed in section 3 below.
- **R4-4: fixed** (`robot_net_present()` used by both test publishers and by 89, lines 126, 138-140, 245). **R4-6: fixed** (runbook "never"
  box, and §13 marks the 70/72 moving commands in `PROGRESS.md` and `docs/START_PROMPT.md`).
- Every safety fix from revisions 2-4 that concerns motion now lives in the deferred Track M / Appendix D. Nothing on the Sunday path
  publishes, so none of them is needed on Sunday.

## 2. Did anything new break?
- **Can `sim_host.py` reach the robot network? No, at least not on Sunday.** It needs the sim env (lerobot, mujoco), which exists only on
  Finnegan's Mac; the operators' robot env cannot run it. It refuses non-loopback ifaces and refuses when a 192.168.123.x address is
  present. Implementation note (**minor r5-1**): LeRobot's `UnitreeG1(is_simulation=True)` initialises DDS on `"lo"`
  (`unitree_g1.py:362`), which does not exist on macOS. sim_host must pre-initialise on `lo0` before the LeRobot import, as
  `scripts/52_brainco_hand_dds.py:28-29` does, and must not rely on LeRobot's default. Otherwise CycloneDDS's interface choice is not
  under our control.
- **Can the loopback publisher reach the robot network? Not by design**, and the one remaining window is small. It refuses a
  non-loopback iface and a present 192.168.123.x address, and 89 refuses too ("unplug the robot cable"). **Minor r5-2:** the address
  check runs only at start. Bound the run with `--seconds`, which exists, and re-check `robot_net_present()` once per second, exiting
  if the cable appears. Also assert after `ChannelFactoryInitialize` that CycloneDDS actually bound the loopback interface. An unknown
  interface name should make Cyclone fail, but that should be checked, not assumed.
- **Is the owner-motion recording read-only on our side? Yes.** It is `84 record` (guard, readers only, `own_writers == 0`). Foreign
  `rt/arm_sdk` samples are only recorded (rule 2), and the motion comes from the owner's own tool with the owner's consent (Q12, rule 3).
  - **Minor r5-3:** use KEEP_LAST 1 (or best-effort) QoS for 84's readers on `rt/arm_sdk`/`rt/lowcmd`. A reliable KEEP_ALL writer in the
    owner's tool could otherwise be slowed by a lagging Python reader. This is unlikely with Unitree defaults, but it is a one-line choice.
  - **Minor r5-4:** add to the runbook: operators stand outside arm reach during step 6, and the label records which tool and mode the
    owner used (Q12). If the owner's tool is Unitree's official `g1_arm7_sdk_dds_example`, its targets are known (shoulder roll ±π/2,
    elbow π/2), and that gives joint directions for free.
- **Minor r5-5:** Ubuntu 24.04 has no `python3.10` apt package. Make `uv python install 3.10` the default path on Ubuntu as well, not only
  for 3.12. The teammates' OS is still unknown.

## 3. Does the timeline fit?
**In total yes: per agent, no. The fix is a reassignment (major r5-6, schedule only, no safety impact).**
- Capacity from 19:20 to 23:15 is about 3.9 h per agent. Build is 9.5 h across three implementers.
- **A-io** has 3.25 h of build (real_io 0.75, 84 1.75, guards and no-publisher tests 0.75) plus the 84 sim run in two envs, all before C2
  at 22:30. That is about 3.2 h available with zero slack, and the table puts "84, guards, no-publisher tests" into 20:05-21:45 (1.7 h for
  2.5 h of work). The table also gives wave-1 fixes to the A-cam column, though real_io belongs to A-io.
- **A-cam** has 2 h of work in 3.9 h (about 1.9 h slack).
- **H's runbook**, the never-cut artifact the non-authors depend on, is estimated at 1.5 h but gets one 45-min slot (21:45-22:30). The
  dry walk and review wave 2 then run in parallel on a first draft.
- Fix:
  - Move the guard and no-publisher tests (rule 1 a-c, 0.75 h) and wave-1 fixes to real_camera from A-io to A-cam.
  - Have H start the runbook skeleton at 20:05, while the setup runs, which is mostly waiting. It can be written against the contract's
    CLI and finished against the real `--help` after C2.
  - With that change the plan fits 19:00-23:30 with about 30 min slack. The cut order is right (FOV, policy extras, cuda, dry walk,
    questions), and the never-cut list is right.

## Checks I could not run
Ubuntu/CUDA (only the operators' Saturday `SELFTEST: PASS` checks those), whether CycloneDDS fails closed on an unknown interface name
(r5-2), and the release upload time.

## Overall (revision 5): **GO**, on one condition
Apply the A-io → A-cam reassignment and start the runbook early (r5-6) when the tracks are dispatched. That is a staffing change, not a
plan-content change, so it needs no further re-review. The minors r5-1 to r5-5 can be folded in during the build. They need no new
review round, but the code review should check r5-1 and r5-2 explicitly.

---

# Plan review rev 6 - round 1

Reviewer: independent plan reviewer (opus), pre-build stage. I did not write the plan and saw none of the planner's reasoning. Started 2026-10-18 15:17.
I ran no sims and edited no plan or code. Read-only checks I did run: imports in the three envs, dataset parquet reads with the SmolVLA env (`python -I`),
`pgrep`, `pmset`, git ignore rules. Line references are to PLAN.md rev 6 (711 lines).

## Facts re-derived (not taken from the plan)
| Claim | Result | Evidence |
|---|---|---|
| arm7 example: 12/13/14 waist, 15-28 arms, weight in slot 29, w = 1 on the first message, kp 60 / kd 1.5, 50 Hz | Supported | `unitree_sdk2_python` @ 65691c8, `g1_arm7_sdk_dds_example.py:40-64, 69-75, 135` |
| SDK publishes through the module-level `DataWriter` | Supported | `core/channel.py:8` import, `:127` writer, `:256` `ChannelPublisher` |
| LeRobot rewrites the persistent `self.msg` and initialises DDS on `"lo"` | Supported | `unitree_g1.py:250-266, 362, 370` |
| Gains 12-14 kp 250 / kd 5; arms 50,50,80,80,40,40,40 / 3,3,3,3,1.5,1.5,1.5 | Supported | `table_env.py:42-43` |
| `BALL_READY_POSE` inside the envelope only via the 0.10 margin | Supported | recomputed from `meta/stats.json`: shoulder pitch and elbow sit exactly on the data bound (margin 0.10) |
| 81 accepts `mps|cpu` only; hello carries `mode`, `chunk_size`, `action_dim`, `image_shape` | Supported | `81_smolvla_server.py:101-103, 159`; `policy_wire.py:172` |
| `PolicyClient(host, port, timeout_s, connect_wait_s)` | Supported (defaults 120/60; the plan passes 2/10) | `policy_wire.py:267` |
| LowState 127.8 Hz in the DDS sim | Supported | `runs/real_probe.json:38` |
| Demo episode 0 starts and ends at `BALL_READY_POSE` | Supported | parquet episode 0: 726 frames (24.2 s); `action[0]` and `action[-1]` equal `BALL_READY_POSE` |
| Demo speeds vs. rate limits | New fact | episode 0 peak arm speed 3.0 rad/s at 1x (1.5 at 0.5x, so the 1.0 rad/s limit clips briefly); waist peak 0.39 rad/s (0.2 at 0.5x) |
| cyclonedds/SDK import works in the sim env without `DYLD_LIBRARY_PATH` | Supported | `$PY -c "import cyclonedds.domain, unitree_sdk2py.core.channel"` -> ok, so B2's verify without `$DDS` is fine |
| Earlier fixes B1, M1-M6, N1, N2, N4, n1-n6, m1-m11, R4-1..R4-6, r5-1..r5-5 present in the text | Supported | Spot-checked each against sections 3.3, 4.1-4.3, 5, 7 (not against the response table): `ENVELOPE_PHASES = ("policy",)`, `ramp_in(q0)` frozen, `release_plan`, lead only at w == 1, `fault_targets`, `LOWSTATE_HZ`, sample-based `active_writers`, `--waist-max/--waist-hold`, 50.0 deg, factory-pose date/tolerance/conflict/sim file, gantry gate (d), no kill mode (f), static + import + guard checks, KEEP_LAST, uv 3.10 |

## Verdict A (plan correctness): safety design
The core motion safety design is sound and every earlier fix survives: arm_sdk only, frozen q0, slow release to a recorded pose, lead check only at
w = 1, measured-angle fault targets, waist 13/14 held, phase-scoped envelope, stale checks, sample-based foreign-writer check, a motion gate that
fails closed, and the gantry plus owner's remote as the last resort. "Our program dies" is covered by: the factory-pose check plus `release`
(arms), `hands-open` (hands; the BrainCo service has no timeout), and the runbook box. Three findings remain, two of them new to revision 6
because the motion now runs **at the table** (M4-M6).

### MAJOR

**RV6-1. Nothing checks that the scripted arm paths clear the table in M4, M5 and M6. On a rough estimate, the fingers sweep through the
table's front edge.**
- Trigger: every `86 ready`/`replay`/`shadow`/`run` starts with ramp_in at the hanging factory pose and then `to_ready`, a straight joint-space
  line to `BALL_READY_POSE` at 0.3 rad/s (section B2 step 1). Every exit runs `to_start`, a straight line back to q0 = the hanging pose (rule 12).
  `release` and the fault ramp-out also end at the hanging pose. Part C runs M4 (replay), M5 (shadow) and M6 (VLA run) with the table in place
  at the sim layout (Part B step 3, tape marks).
- Evidence:
  - `table_scene.py:78-82`: the hips are at table height, and the nominal stop puts the pelvis 0.2455 m from the table's front edge.
  - `g1_29dof_no_hand.xml` left arm chain: the upper arm hangs about 0.20 m down at zero. The forearm points forward from the elbow
    (+0.10, +0.038, +0.046 m, then the Revo2 hand, so about 0.3 m to the fingertips). The elbow axis is +y, so a positive elbow angle points
    the forearm down.
  - If the real factory pose has the forearms hanging down (elbow clearly positive, as in Unitree's normal stance), the fingertip swings on a
    radius of about 0.3 m around an elbow that sits about 0.1 m above table height. Half-way up, the fingertip is about 0.3 m in front of the
    pelvis and below the table top, which puts it **inside the table's front edge**.
  - The reverse path (`to_start` after every attempt, `release`) crosses the same spot. A contact there trips the lead fault, and the fault
    ramp-out then hands the arm to the factory controller, which pulls it towards the hanging pose, into the table, over 2 s.
  - No test can see this. 85 uses the default scene `scene_29dof_brainco.xml`, which has no table (`config.yaml:4`; the table scenes are
    separate files). The demos never made this move near a table: episode 0 starts and ends at `BALL_READY_POSE`, because the sim robot
    walks up with its arms already raised.
- Impact: finger or hand impact on the table edge at 0.3 rad/s on the first M5 or M6 start, and the factory controller pressing into the
  table on the exit. This risks damage to the Revo2 fingers, and M4-M6 may fault on every start. People are not at risk (1.5 m clear, gantry).
- This is an estimate from the MJCF link offsets. I did not run FK, because the instructions say no sims. The real factory pose is unknown
  until Monday. The elbow-down case is plausible enough that the plan must decide it, not leave it to the day.
- Fix (plan text plus a small Track C item):
  - (a) Track C adds an FK clearance check (`mj_forward` only, no stepping) in the volleyball table scene. It covers `to_ready` and
    `to_start` from hanging poses with an elbow range of about 0 to 1.6 rad at the nominal stop, and reports the minimum fingertip-to-table
    clearance and the table distance that keeps 5 cm.
  - (b) Exits from the policy phase go through `BALL_READY_POSE` first (the pose that the demos start and end in, verified clear in sim),
    and only then down to q0.
  - (c) Runbook M4-M6 get a precondition from (a): "the recorded factory elbow angle is below X, else place the table Y cm further back,
    or skip the table steps" (cut item 6 already exists for this case).
  - Until (a) has run, M4-M6 at the table are not cleared.

**RV6-2. The factory pose is recorded in Part B, possibly in a different robot mode than Part C uses. Part C can then lock itself out, and
`release` aims at a stale pose.**
- Trigger: Part B step 0 records the pose "before anything moves", in whatever mode the robot is in. Part C runs "in the arm_sdk mode agreed
  with the owner (Q8/Q10)". Issues #156/#170 show the usable mode depends on the firmware (for example R2+A instead of R1+X), so a mode switch
  between B and C is plausible, and the controller's arm hold pose may change with it.
- Impact:
  - If the hold pose moves by more than 0.10 rad, `86 ready` refuses with "run `86 release` first".
  - `release` moves the arms at w = 1 to the **old** recorded pose and ramps out. The factory controller then pulls them back to the new hold
    pose. That is an unrequested swing of up to the pose difference within 2 s, which is the N2 mechanism again.
  - `ready` then refuses again. The operators are stuck in a loop.
  - The only way out is `--overwrite-factory-pose`. The conflict message does not offer it, and non-authors cannot judge whether it is safe.
  - Part C is lost for the day.
- Fix:
  - Add Part C step M0: after the owner has set the agreed arm_sdk mode and before the first 86 command of the day, run
    `84 --steps joints --record-factory-pose`.
  - If it reports a conflict and no 86 command has run today (so nothing of ours can be latched), the runbook allows
    `--overwrite-factory-pose` once, with the owner's confirmation that the arms are in the controller's hold pose.
  - The record stores the mode answer (Q8) as text.
  - 86's refusal message names M0.

**RV6-3. The push-1 timeline contradicts the plan's own effort numbers, and the C2 fallback is ordered the wrong way.**
- Evidence:
  - Section 6.1 gives A and E the build slot 16:15-18:15 (2 h). The tracks state 3.5 h (A) and 4 h (E).
  - The workflow (`robot-day-build.workflow.js:107-118`) starts a track's review only after its build agent returns, then runs up to three
    fix/review rounds in series.
  - Clock now 15:25, plan review round 1 in progress. Build start is realistically about 16:00-16:45, which puts A's first review at about
    20:15-21:00 and E's at about 20:45-21:30. "A and E GO by 19:30" therefore needs both builds at about 55 % of their estimate and a
    first-round GO from a reviewer told to treat unexplained constants as majors.
  - The C2 fallback "push 1 = E only" is not faster: E is the longer track.
  - Section 6.6's "about 2 h slack before C5" assumes B2 ends at 20:45 and has one fix round. With a 16:30 start and two rounds, B2 lands at
    about midnight. Integration, the dry walk and the judge (about 6 h) then end at about 06:00-07:00, so C5 at 06:30 has almost no slack.
- Impact: push 1 realistically comes at about 21:00-22:30, not 20:00, and the operators' Sunday setup shifts late. The failure modes are safe
  (motion stays locked), but the user's milestones are stated as if they were likely.
- Fix:
  - State push 1 as "when A and E are GO, expected 21:00-22:00".
  - Move E's non-push-1 items (runbook Part C, the 89 `86 gate` line, `replay_demo.npz`) after its first review, so E reaches review sooner.
  - Drop the "E only" fallback, or base it on whichever track is actually GO.
  - Say that C5 has about 0-30 min slack, and that push 2 may move to about 08:00 with the operators pulling at 08:30.

### MINOR
- **m6-1** `docs/volleyball_natural_phases.png` is the M2 comparison image, but `.gitignore:7` (`docs/*.png`) ignores it and it is not tracked
  (`git check-ignore`). The operators will not have it. Fix: E copies it to `docs/robot_day/` (not ignored) and the runbook points there.
- **m6-2** E's verify runs `$PY tests/helpers/make_robot_day_data.py` "(sim env)". The sim env has no `pyarrow` and no `datasets`
  (`ModuleNotFoundError`), and the dataset stores its images as PNG bytes inside the parquet under `observation.images.camera1`, not
  `head_image` (`meta/info.json`). Fix: run it with `$SV`, which has pyarrow 26, PIL and cv2, and state the mapping camera1 -> head_image.
- **m6-3** `scripts/run_tests.sh` (the workflow's integration step 1) also runs pytest in the project `.venv`, which has no cyclonedds,
  unitree_sdk2py or cv2. New test files that import real_io, real_camera, real_send or motion_gate at module level will fail to collect there,
  and that breaks "everything that passed before must still pass". Fix: each such test file starts with
  `pytest.importorskip("unitree_sdk2py")` (or `cv2`), as the existing tests handle `needs_g1ext`.
- **m6-4** The factory record expires at midnight (`load_factory_pose` returns None for another date). Integration runs 23:00-01:30, so a sim
  record written before 00:00 makes every later e2e 86 refuse. Fix: every e2e test records its own sim factory pose in its fixture.
- **m6-5** G2 never proves that the replay advances through the demo. If 86 sends `tick=0` on every request, the replay loops over the first 50
  actions and G2 still passes (tracking, envelope and message matching are all fine). Fix: in the contract, `infer` tick = `int(mark_requested())`.
  G2 asserts that the policy-phase `policy_actions` equal `replay_demo.npz` actions at those ticks. Also, at `--speed 0.25` the default
  `--max-seconds 60` covers only 450 of the 726 demo frames (M3). Say this in the runbook, or default replay to the full length.
- **m6-6** Replay without hand state: `state27` puts NaN in the hand slots, `check_observation` refuses NaN (`policy_wire.py:130`), and the run
  ends in a `policy_stale` soft exit. That contradicts "replay warns". Fix: replay fills missing hand slots with the last hand command and
  logs `hand_state_synthetic`, as `--no-hands` does.
- **m6-7** The envelope is 27-D and includes hands 0..0.4, but `SafetyFilter.step` sees only the 17 SDK motors, and `hand_targets` caps at 0.6.
  The contract does not say whether hands are envelope-clipped or count towards `envelope_streak`. Fix: state "hands: HAND_CAP only".
- **m6-8** `MAX_WEIGHT_STEP` has no owner, and `release`'s first message is a deliberate jump to w = 1. Fix: state that the step limit applies to
  ramp_in/ramp_out and is checked in the runner, and that release's first message is the documented exception (Unitree example line 135).
- **m6-9** The soft-exit cap of 6 s at 0.3 rad/s covers 1.8 rad. The envelope reaches shoulder pitch -1.72, so a real factory shoulder pitch
  above about +0.08 makes the ramp-out start short of q0 (a swing). Fix: cap = largest gap / 0.3 + 1 s.
- **m6-10** `caffeinate -i` (rule 15) exists only on macOS. Fix: give the Ubuntu equivalent
  (`systemd-inhibit --what=sleep:idle:handle-lid-switch`) and the rule "never close the lid".
- **m6-11** The unlock recipe has no check before the push. Fix: commit every code change first, check that `git status --porcelain -- g1sim
  scripts` is empty, pin, commit the JSON, then require that `python scripts/86_real_run.py gate` prints `MOTION: UNLOCKED` before `git push`.
  Also decide whether the gate checks `date == today`; otherwise the unlock silently carries over to the next session.
- **m6-12** Five track reviewers append to one file, `.planning/reviews/real-robot-code.md`, at the same time, so they can clobber each other.
  The user's rule is one findings file per gate. Fix: use `real-robot-code-<track>.md`.
- **m6-13** E's `test_runbook_commands_match_script_flags` checks 84's flags. 84 is A's file and is absent while the tracks build in parallel,
  and only 86 has an "absent" skip. Fix: the same skip for every missing script.

## Verdict B: are the tracks disjoint, the contracts precise, the verify commands runnable here?
- **Disjoint: yes.** The five frontmatter file lists do not overlap. Shared outputs are `.planning/STATUS.md` (appends only) and the review
  file (m6-12).
- **Contracts:** precise enough for B1/B2/C, with the gaps m6-5, m6-6, m6-7 and m6-8. Every track imports `real_io` from A. The plan says
  "write real_io.py first", but nothing stops B2, C or E from running their tests before it exists. They must mark those steps "pending
  integration", which the workflow prompt allows.
- **Verify commands on this Mac:**
  - A, B1, B2, C: runnable.
  - E: `make_robot_day_data` fails in `$PY` (m6-2). The setup and release steps are runnable (uv 0.12.13 present; release tar present;
    78 GiB free).
  - Integration: `run_tests.sh` hits the collection errors in m6-3.

## Verdict C: does the timeline fit?
The overnight window fits the final gate only with almost no slack, and push 1 at 20:00 does not fit (RV6-3). All the fallbacks fail safe
(motion stays locked).

## Environment (not plan content, but it decides the overnight run)
- **At 15:2x the laptop is on battery: 62 %, discharging, "6:14 remaining" at light load** (`pmset -g batt`). Two `caffeinate -dimsu` processes
  are running. With five agents plus a sim it will not last the night. The main session must get the charger plugged in. This keeps the run
  at risk until confirmed.

## Checks I could not run
- FK clearance of the scripted paths against the table (RV6-1). It is estimated from the MJCF offsets, not computed.
- The real factory pose, the mode-dependent hold pose (RV6-2), Unitree's latch/blend behaviour, whether the owner's remote damping overrides
  arm_sdk, and the Ubuntu/CUDA setup path. Only the robot day or the operators' self-test show these.

## Overall (revision 6, round 1): **NO-GO**
There are no blockers. Three majors need plan-text changes before the build:
- RV6-1: table clearance of the scripted paths, exit via the ready pose, and an M4-M6 precondition.
- RV6-2: record the factory pose at the start of Part C, in the mode Part C uses.
- RV6-3: honest push-1 timing and a corrected fallback.

The minors can be folded into the track tasks without another review round. A quick round 2 that checks only these three should be enough.

---

# Plan review rev 6 - round 2

Reviewer: independent plan reviewer (opus), pre-build stage, round 2 on PLAN.md revision 6.1 (924 lines). I did not write the plan and saw
none of the planner's reasoning. Started 2026-10-18 16:50. I ran no sims and edited no plan or code. Read-only checks I did run:
`table_scene.py`, `action_space.py`, `table_env.py`, `policy_wire.py`, `81_smolvla_server.py`, the arm7 SDK example, `pytest.ini`,
`tests/conftest.py`, `.gitignore` + `git check-ignore` on every planned new path, imports in the sim and SmolVLA envs (`python -I`),
`pmset`, `pgrep`, `sysctl`, and the workflow script `~/dev/g1-ext/release/robot-day-build.workflow.js` (modified 16:50).
Line numbers below are PLAN.md rev 6.1 line numbers.

## Facts re-derived for this round
| Claim (plan) | Result | Evidence |
|---|---|---|
| 2.7b table geometry: `TABLE_TOP 0.74`, volleyball front edge x = 0.20, `R_STOP 0.458` -> pelvis 0.2455 m from the edge, `GROOT_STANCE` z 0.7505, ball (0.344, 0.241), stand r 0.05 / wall 0.006 / h 0.08, basket (0.344, -0.241) | Supported | `table_scene.py:28-29, 45-55, 61-80, 98-113` |
| `BALL_READY_POSE` = right arm (-0.25, -0.05, 0, -0.8, 0, 0.6, 0) mirrored, >= 4.9 cm clear at the nominal walk stop | Supported (as a code comment; FK not re-run by me) | `action_space.py:49-59` |
| 87 is buildable as specified: `TableEnv(scene="volleyball_walk").reset(randomize=False)` puts the pelvis at `WALK_STANCE`; `mj_geomDistance` exists | Supported | `table_env.py:51-54, 108-126, 177`; sim env: mujoco 3.13.0, `hasattr(mujoco, "mj_geomDistance")` True |
| Replay server: hello `mode == "replay"`, `reset(episode_stem)`, `infer(obs, tick)` serves `actions[tick:tick+50]`; `--episode replay_demo` matches the file stem | Supported | `policy_wire.py:141-147, 166-197`; `81_smolvla_server.py:154-176` |
| `check_observation` refuses non-finite state (m6-6 premise) | Supported | `policy_wire.py:119-130` |
| 86's import chain stays numpy-only + SDK (robot env, Python 3.10) | Supported | `g1sim/__init__.py` imports only `pathlib`; `episode` -> `action_space` -> `brainco` -> numpy |
| Unitree's arm7 example commands waist 12-14 through arm_sdk with weight 1 from the first message | Supported | `g1_arm7_sdk_dds_example.py:85-103, 141-148` (waist targets 0, kp 60); the plan's "13/14 held at q0" is the safer variant |
| pytest markers `integration` / `e2e` exist | Supported | `pytest.ini` markers unit/integration/e2e |
| New tracked files are not git-ignored | Supported | `git check-ignore` prints nothing for `tests/data/*.npz`, `docs/robot_day/*.png`, `tests/helpers/sim_host.py`; only `runs/` is ignored (`.gitignore:9`), which is right for factory records and logs |
| `$SV` has pyarrow/PIL/cv2 (m6-2 fix) | Supported | `pyarrow 26.0.0`, PIL, cv2 import in `$SV` |
| Workflow: review only after build returns, then up to 2 fix + re-review rounds | Supported | `robot-day-build.workflow.js:127-138` (the plan says "three", it is review r1 + two fix/review rounds) |

## Verdict 1 (plan correctness): safety design
Every earlier fix is still in the text, checked against sections 3.3, 3.4, 4.1-4.3, 5 and 7 (not against the response tables): B1
(`ENVELOPE_PHASES = ("policy",)`), M1 (`release_plan`, rule 13), M2 (`fault_targets`), M3/n1 (`LOWSTATE_HZ`), M4/n3 (sample-based
`active_writers`, `rt/lowcmd` warns), M5/n4 (`--waist-max`/`--waist-hold`, waist limit carried into M4), M6 (50.0 deg), N1 (`ramp_in(q0)`
frozen, sagging-arm mutation test, e2e < 0.03 rad), N2 (move at 0.3 rad/s then ramp, lead only at w == 1, e2e peak < 0.35 rad/s), N4 +
RV6-2 (dated record, tolerance, conflict, sim file, M0 in the Part C mode, mode note), n2 (G2 max lead < 0.15, Monday abort > 0.10), n5,
R4-1 (all loaded files + JSON vs the pinned commit), R4-2 (gantry gate (d), no kill mode), R4-3/R4-4 (static + import + guard,
`robot_net_present`), R4-5 (3.10), R4-6 (never box), r5-1..r5-5, RV6-1 (87 + exit through the ready pose + rule 16), m6-1..m6-13.
13/14 are held at q0 with `SIM_GAINS` in every message; hands: cap 0.6, soft exit opens them, fault holds, `hands-open` exists because the
BrainCo service has no timeout. "Our program dies" is covered by latch-or-timeout -> `release` from the measured angles -> owner's remote
box. One gap remains, and it is the RV6-1 mechanism again on the paths RV6-1's fix did not cover.

### MAJOR

**R2-1. Exits that start from a policy pose at the table are not covered by 87, although section 3.5 says they are.**
- Text: 3.5 (line 352-355): "`to_start`, `release` and the fault ramp-out run the same segment backwards or towards the hanging pose, so one
  check covers all of them". That is only true when the exit starts at `BALL_READY_POSE`. Three exits start somewhere else:
  1. **Fault ramp-out from the `policy` phase** (rule 12, line 405-407): contact fault, NaN, second signal, stale LowState and the `finally`
     path ramp w 1 -> 0 in 2 s at the measured (or last) angles. The factory controller then pulls the arm from wherever the policy left it
     (hand at the ball, above the table top, about 0.15 m behind the front edge) to the hanging pose (hand below table-top height, in front
     of the thighs). That arc crosses the plane of the table's front edge on its way down.
  2. **`86 release` after a crash, kill, cable pull or fault during M4-M6**: `release_plan(measured, factory)` (line 252-253) is a straight
     joint line from the latched policy pose to the factory pose. It does **not** go through the ready pose.
  3. **The `to_ready` leg of the soft exit** (`soft_exit_plan`, line 254-258): a straight joint line from the policy pose to
     `BALL_READY_POSE`. Shoulder pitch swings back while the elbow lifts, so the hand can dip on the way. No check covers it.
- Reachable: M4 (replay) and M6 (VLA live) at the table. Contact faults are the expected failure of a sim-only policy near a stand (the plan's
  own m10/n5 history: the little finger brushing the stand). A stale-LowState fault from a GIL or GC pause on an operator laptop is
  possible, and the 60 ms gap gate is only measured on this Mac.
- Impact: the Revo2 fingers can strike the table edge under the factory controller in about 2 s (case 1), or under our 0.3 rad/s line
  (cases 2 and 3; a contact there trips the lead fault, which leads into case 1). This is hardware damage on the owner's robot, not a risk
  to people (gantry, 1.5 m rule). The plan's text tells the user that 87 rules this out, and it does not.
- Evidence: plan lines 252-258, 352-355, 402-407; `table_scene.py:61, 74-80` (hips at table height, edge 0.2455 m in front of the pelvis);
  `action_space.py:49-56` (the ready pose was chosen by FK grid search *because* lower hands touched things). Not computed by FK (no sims
  allowed), so the crossing is an estimate, the same evidence level RV6-1 had.
- Fix (plan text, small Track C + E2 additions; no new motion logic):
  - (a) Correct 3.5: 87 covers ready <-> hanging only.
  - (b) 87 adds a second report block, `policy_exits`. For every 10th frame of `tests/data/replay_demo.npz` (a stand-in for policy poses),
    it reports the minimum clearance along `move(frame, BALL_READY_POSE)` (case 3) and along the straight line frame -> h(e, s) for the
    lookup grid (an approximation of cases 1 and 2). Sample every 5th tick. Same 5 cm margin.
  - (c) Runbook Part C (E2) + rule 16: after any exit 4, crash, kill or cable pull during M4-M6, `86 release` runs only after the owner
    has decided whether the table (with stand and basket) can be pulled back first. If not, use the owner's remote (existing box).
  - (d) A ruling R15 for the user: if (b) shows that the policy-pose -> hanging lines do not clear the table, M6 (VLA live) at the table
    accepts that a fault ramp-out may touch the table edge, or M6 is skipped at the table (M4 replay is a known trajectory and stays).
    Default recommendation: skip M6 at the table unless (b) is clear.
  - Not required tonight, but name it for the next session: a "fault hold" (measured angles at w = 1 until the operator confirms, then
    `release_plan` via the ready pose) would remove case 1. It is new motion logic and would need its own review.

### MINOR
- **R2-2 Factory-pose overwrite: a dead end after a mode change mid-day, and the "motion ran today" signal is written too late.**
  - After any 86 run, 84 refuses `--overwrite-factory-pose` (line 210). But 86's gap refusal tells the operators to "re-record it (runbook
    M0)" whenever the owner changed the mode (line 329-330). The plausible path is: an error, the owner damps and re-stands the robot,
    and tries another arm_sdk mode (issue #170). Then `ready` refuses, M0 is refused, and `release` aims at the old pose (the RV6-2 swing),
    so `ready` refuses again. Part C is lost for the day, and the operators get contradictory instructions.
  - Also, `motion_ran_today` reads sidecars. If 86 writes its sidecar only at exit, a SIGKILL'd or power-lost run (the case that matters)
    leaves no trace. A run refused with exit 2 before any writer existed does leave one.
  - Fix:
    - (i) Allow the overwrite when the newest motion sidecar of today is a `release` with exit 0, plus the owner's confirmation. After a
      clean release nothing of ours is latched.
    - (ii) 86 writes a `<stamp>_<mode>.started` marker (or the sidecar with `exit_code: null`) **before its first publish**.
      `motion_ran_today` counts those markers and ignores runs that never created a writer.
    - (iii) 86's gap message offers M0 only when 84 would accept it, and "run `86 release` first" otherwise.
    - (iv) A: one more test.
- **R2-3 The 87 lookup keys only on the elbow, but its grid assumes shoulder roll, shoulder yaw and the wrists are 0.** Unitree-style
  default arm poses often have a shoulder roll of about +-0.2-0.3 rad (unverified for this firmware). Fix:
  - `factory_arm_summary` also prints shoulder roll/yaw and the largest wrist angle.
  - The runbook lookup says "skip the table steps" when any of these is more than 0.15 rad from 0, or when the shoulder pitch is outside
    the grid (-0.1..0.3). Alternatively, add roll 0/0.3 to the grid.
  - Optional: `setback_max_policy_m`, the largest setback that keeps the ball inside the training `reach_m` (0.33-0.52, `table_scene.py:101`).
    Beyond it, M6 is not a meaningful attempt.
- **R2-4 Timeline arithmetic and workflow seams.**
  - (a) The 6.1 table goes from "fix to ~22:15" straight to "C2 22:15 A and E1 GO". The script needs a re-review after every fix
    (`workflow.js:132-135`), and the reviewer prompt makes unexplained constants majors, so a first-round GO is unlikely. A realistic
    estimate is A GO ~22:45-23:00, E1 ~22:15-22:30, then the 20-min push-1 check, so push 1 lands ~23:00-23:15. That is inside "latest
    23:30" but with almost no margin. State "expected ~23:00".
  - (b) Three of the four seams the plan hands to the main session (line 919-922) are **not** in the script as of 16:50:
    - review files are still the single `real-robot-code.md` (lines 115, 118, 155, 176), so five concurrent reviewers can clobber each
      other's gate records (m6-12);
    - there is no push-1 milestone signal inside a combined build stage;
    - the dry walk is one post-integration stage over the whole runbook "against the loopback sim" (line 159-169), not Parts A+B after push
      1. By the plan's own words (line 690, 756-757) C5 slack is then about 0.
  - Fix: update the script before the build launches (per-gate review files, run A+E1 as their own build stage so push 1 is signalled,
    split the dry walk).
  - (c) Part A (Sunday setup) is used by the operators before its dry walk. The E1 verify + push-1 check cover the commands, not the
    wording. Accept, or have the main session read Part A once before push 1.
- **R2-5 "Hangs in the gantry" is undefined for a robot that must stand under its own controller for arm_sdk.** Gate (d) and page 1 say
  "hangs"; the table formula (line 771-772) assumes it stands on its feet. Fix: define it in the runbook. The robot stands on its feet in
  the owner's arm_sdk mode, and the harness is attached to the gantry with only a few cm of slack, so a damp/collapse is caught. The
  operators type HANGING only for that state.
- **R2-6 E1's verify contradicts section 0, and its background install is fragile.**
  - Section 0 (line 87-88) says the CPU policy self-test runs only in the main session's push-1 check. E1's verify (line 637-639) runs it
    inside the build.
  - `setup_robot_laptop.sh --policy-env ... &` inside an agent's Bash call can be reaped when the call returns. It also runs at the same
    time as `--release-file` against the same prefix.
  - Fix: allow the CPU self-test in E1 explicitly (CPU only, no MuJoCo), start the install detached with its PID in
    `runs/real_e_setup_policy.pid`, and run `--release-file` before it or after it.

### Environment (not plan content, but it decides the overnight run)
At 16:5x `pmset -g batt` again shows **Battery Power, 100 %, discharging** (as STATUS 16:48 already noted). The plan's premise is "on AC
power" (line 65). Five agents, MuJoCo, a torch install and a 40k MPS run will not last until 06:30 on battery. The main session must
confirm AC power before it launches the build. Two `caffeinate -dimsu` processes are running.

## Verdict 2: tracks, contracts, verify commands
- **Disjoint: yes.** The six frontmatter lists do not overlap. E2 edits only E1's files, after E1 is GO (the script chains it,
  `workflow.js:139-142`). Shared outputs are `.planning/STATUS.md` (append only), `runs/` (ignored), C's `runs/arm_path_clearance.json`
  (consumed by E2 and integration), and the review file (R2-4b). `pytest.ini`/`conftest.py` need no edits: the markers exist and the repo
  root is on `sys.path`.
- **Contracts:** precise enough to build in parallel. Real_io <-> real_safety layout is pinned by a B2 test. The 86 log/sidecar names are
  pinned and are now a safety input for A (R2-2 asks for one more field). The CLI and server hello checks match `policy_wire`/81. 87's
  inputs (`TableEnv`, `mj_geomDistance`, B1's `move`) exist or are contracted. The R2-1 addition only changes C's 87 and E2's runbook text.
- **Verify commands runnable on this Mac tonight:**
  - A, B1, B2: yes.
  - C: yes. It starts sims, which is allowed. The clearance tests wait for B1's `real_safety.py`.
  - E1: yes. `$SV` is right for the data script, uv and gh are present, the tar is present, there is 79 GiB free and 24 GB RAM. R2-6
    applies to the background install.
  - E2: yes.
  - Push-1 check and integration: yes, given the m6-3 `importorskip` lines.

## Verdict 3: timeline
- The overnight plan fits, but only just.
  - B2 is the critical path: build ~22:00, then review and up to two fix/re-review rounds, ending ~00:45-01:30, inside the C3 cutoff.
  - Integration runs to ~03:00-04:00, then the Part C walk, then the judge.
  - C5 at 06:30 has about 30 min of slack only if the script splits the dry walk (R2-4b); otherwise it has about 0.
- Push 1 is realistically ~23:00 (R2-4a).
- Every fallback fails safe: motion stays locked, Parts A+B still run.
- If this round's NO-GO costs one planner round (~30-45 min), push 1 drifts to about 23:30-23:45, so the "only E1 GO" fallback becomes
  likely. To avoid that:
  - R2-1 changes only Track C's 87 and E2's runbook text. Tracks A, B1, B2, E1 (and C's 85/sim_host/e2e) can start on rev 6.1 now while
    the planner patches 3.5, rule 16, Part C and R15 (rev 6.2), provided C's 87 work and E2 read rev 6.2.
  - The R2-2 additions to A and B2 are small enough to fold into their first fix round.
  - That decision belongs to the main session.

## Checks I could not run
- FK of the policy-pose exit paths against the table (R2-1). It is estimated from the scene geometry; no sims allowed.
- The real factory arm pose, including shoulder roll (R2-3); Unitree's latch/blend/timeout; whether a mode change moves the hold pose; the
  real laptop's LowState jitter under Python 3.10. Only Monday shows these.
- Whether the main session updates the workflow seams before launch (R2-4b).

## Overall (revision 6.1, round 2): **NO-GO**
- No blockers. RV6-2 and RV6-3 are fixed. RV6-1 is fixed for the ready <-> hanging paths.
- One major remains, R2-1: exits that start from a policy pose at the table (fault ramp-out, `release` after a crash, the `to_ready` leg)
  are claimed as covered and are not.
- The fix is plan text plus a small 87 report block, two runbook lines and a ruling. No new motion logic is needed.
- The minors R2-2..R2-6 can be folded into the tracks without another round.
- Separately, AC power must be confirmed before the build starts.

---

# Plan review rev 6 - round 3

Reviewer: independent plan reviewer (opus), pre-build stage, round 3 on PLAN.md revision 6.2 (1074 lines, written 17:29). I did not write
the plan and saw none of the planner's reasoning. Started 2026-10-18 17:30. I ran no sims and edited no plan or code. Read-only checks I
ran: `action_space.py`, `table_scene.py`, `table_env.py`, `policy_wire.py`, `81_smolvla_server.py`, the arm7 SDK example and
`core/channel.py` at `65691c8`, `pytest.ini`, `tests/conftest.py`, `scripts/run_tests.sh`, `$DATASET/meta/{stats,info}.json` (with `$SV -I`),
the release tar listing, the HF cache on disk, DDS domain ids in the sim and scripts, PyPI wheel lists, a scratch git repo for the gate
primitive, `pmset`, `ioreg`, `pgrep`, and `.planning/robot-day-build.workflow.js`. Line numbers are PLAN.md rev 6.2 line numbers.

## Facts re-derived for this round
| Claim (plan) | Result | Evidence |
|---|---|---|
| arm7 example: weight in slot 29, kp 60 / kd 1.5, 50 Hz, ramp-out of the weight | Supported | `g1_arm7_sdk_dds_example.py` @ 65691c8: `kNotUsedJoint = 29 # NOTE: Weight`, `control_dt_ 0.02`, `kp 60.`, `kd 1.5`, stage 4 `(1 - ratio)` |
| SDK publishes through the module-level `DataWriter` | Supported | `core/channel.py:8` import, `:127` writer |
| Gains 12-14 kp 250 / kd 5; arms 50,50,80,80,40,40,40 / 3,3,3,3,1.5,1.5,1.5 | Supported | `table_env.py:42-43` |
| `BALL_READY_POSE` (-0.25, -0.05, 0, -0.8, 0, 0.6, 0) mirrored, >= 4.9 cm clear at the nominal walk stop | Supported (code comment) | `action_space.py:49-59` |
| Table geometry 2.7b; `reach_m` 0.33-0.52 | Supported | `table_scene.py:28-29, 45-55, 61-82, 101` (`reach_m` sits on the `volleyball` scene, not `volleyball_walk`; 87 must read it from there) |
| Envelope: shoulder pitch -1.62..-0.25, waist -0.71..0.68, thumbs <= 0.3, fingers <= 0.15; 300 episodes x 726 frames at 30 fps | Supported, plus: elbow -0.80..+1.42 | `meta/stats.json`, `meta/info.json` (217800 frames) |
| 81: `--device mps|cpu`, `--self-test` needs `--checkpoint`; hello `mode`/`chunk_size`/`action_dim`/`image_shape` | Supported | `81_smolvla_server.py:101-103, 155-170` |
| `PolicyClient(host, port, timeout_s=120, connect_wait_s=60)`; `check_observation` refuses NaN state | Supported | `policy_wire.py:119-130, 267` |
| The release tar unpacks to `pretrained_model/...` at its top level | Supported | `tar -tf` (so the setup must create `$P/smolvla-runs/run1/checkpoints/040000/` itself, as E1's verify assumes) |
| Robot-env pins have wheels: cyclonedds 0.10.2 cp310 (macOS arm64/x86_64, manylinux x86_64), opencv-python-headless 4.13.0.92 abi3 | Supported | PyPI JSON (opencv macOS arm64 wheel needs macOS 13+) |
| 87 is buildable: `mj_geomDistance` in both envs | Supported | `.venv` mujoco 3.12.0 and sim env 3.13.0: `hasattr` True |
| Workflow: review only after build, then 2 fix/re-review rounds; E2 chained after E1 GO; per-gate review files; per-track GO log line | Supported | `robot-day-build.workflow.js:127-142, 113-118`; `log("track X: GO")` at line 136 gives the push-1 signal |
| "On AC power" (Situation, line 65) | **Refuted at 17:36 and 17:43** | `pmset -g batt`: "Now drawing from 'Battery Power' ... discharging"; `pmset -g adapter`: "No adapter attached"; `ioreg`: `ExternalConnected = No` |

## Verdict 1 (plan correctness): safety design
The core design is sound and every earlier fix I checked in the plan text (not in the response tables) is still there: B1 (`ENVELOPE_PHASES`),
M1 (`release_plan`, rule 13), M2 (`fault_targets`), M3/n1 (`LOWSTATE_HZ`), M4/n3 (sample-based `active_writers`), M5 (`--waist-max` /
`--waist-hold`), M6 (50.0 deg), N1 (frozen q0 + sagging-arm mutation test + e2e < 0.03 rad), N2 (move at 0.3 rad/s, lead only at w == 1,
e2e peak < 0.35 rad/s), N4/RV6-2 (dated record, tolerance, conflict, sim file, M0, mode note), n2, n5, m2, m6-m8, R4-1, R4-2, R4-3/R4-4,
R4-5, r5-1..r5-3, r5-5, RV6-1, R2-1 (87 block 2, rule 16 (d), R15), R2-2 (sidecar before the first writer; `arms_may_be_held_by_us`), R2-3,
R2-5. 13/14 are held at q0 with `SIM_GAINS` in every message; hands: cap 0.6, soft exit opens them, fault holds, `hands-open` (the BrainCo
service has no timeout). "Our program dies" is covered by latch-or-timeout -> factory-gap refusal -> `86 release` -> owner's remote box, and
by the early sidecar for the overwrite rule. Two earlier fixes are only partly preserved (n4 in R3-2; m11, r5-4 and R4-6 in R3-6).

### MAJOR

**R3-1. The "fresh prefix" policy check cannot fail on this Mac, so the operators' policy path is never really tested tonight.**
- Mechanism: `81_smolvla_server.py:37, 67-68` sets `HF_HUB_CACHE` by `setdefault` to `~/dev/g1-ext/smolvla-test/hf_cache/hub` and
  forces `HF_HUB_OFFLINE=1`. On this Mac that directory holds the 2.7 GB SmolVLM2 backbone and `lerobot/smolvla_base` (`du`: 2.7G blobs).
  The checkpoint needs the backbone (`config.json`: `vlm_model_name` SmolVLM2-500M-Video-Instruct, `load_vlm_weights: true`).
- Trigger: E1's verify (lines 728-731) and the push-1 check (line 815, 89 with `--policy-device cpu`) run 81 "from the fresh prefix".
  The plan does not say where the setup puts the backbone or that `$P/policy-env.sh` exports `HF_HUB_CACHE` (2.7, line 174, says only
  "backbone cache 2.7 GB, `HF_HUB_OFFLINE=1`"). If `policy-env.sh` leaves `HF_HUB_CACHE` unset, both checks pass here from Finnegan's
  old cache. On the operators' laptops that path does not exist and the hub is offline, so 81 cannot load the model.
- Impact: M5 (shadow) and M6 (the VLA attempt the user wants) are lost. The operators' 89 (`SELFTEST: FAIL`) would show it, but only
  Sunday night or Monday 08:00, when Finnegan may not be able to fix it. Replay (M3/M4) still works, because the replay server needs no
  torch. No safety impact.
- Fix (E1 contract and verify, two lines): the setup's policy-env block downloads both repos (online) into `$P/hf_cache/hub`, and
  `$P/policy-env.sh` exports `HF_HUB_CACHE=$P/hf_cache/hub`. E1's verify and the push-1 check assert that
  `$P/hf_cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct` exists and that the self-test log prints that cache path. For
  example, run the self-test with `HF_HUB_CACHE` set explicitly from `policy-env.sh` and grep it. Then the self-test fails here exactly
  when it would fail on their laptop.

**R3-2. 87 checks the exits from policy poses only at waist 0.6, but M4 and M6 may run at the table at waist 0.15, at 0.35 or with
`--waist-hold`. That drops part of the earlier fix n4.**
- Text: 87 block 2 takes `state[26]` "clipped to `ROBOT_WAIST_MAX_CEILING` 0.6" (line 392). M3 escalates 0.15 -> 0.35 -> 0.6 "only after a
  clean replay and the owner's yes" and says "stay at the previous limit or use `--waist-hold`" when the robot sways (lines 894-896,
  R10 line 936). M4 then runs "with the waist limit reached in M3" (line 899), and M6 uses the "same waist limit" (line 908). Rule 16 (b)
  and (c) gate M4 and M6 on `replay_exits_clear` and `fault_exits_clear`, but those were computed for other poses.
- Reachable: the owner says no to 0.6 or the robot sways at a waist step, which the runbook itself routes to. Then
  `SafetyFilter` clips the waist (rule 6b). The arm joints follow the demo, but the hands land at a different azimuth. Example: 0.15
  instead of 0.61 is a 0.46 rad turn, about 19 cm at the 0.42 m ball radius. Their distance to the table's front edge, the stand and the
  basket is then not what 87 measured. The fault drop (case 1), `86 release` after a crash (case 2) and the `to_ready` leg (case 3)
  start from these unchecked poses.
- Evidence: plan lines 391-399, 472-482, 894-908. Rev-2 n4 (this file, lines 194-196) required "if that is < 0.8, do only the no-ball
  clearance pass at that limit". Rev 6.2 has no such rule (`grep` finds no "no-ball"). The direction of the effect is my estimate (no FK
  run). The gap itself is plain from the text.
- Impact: the same hardware risk as R2-1 (Revo2 fingers on the table edge under the factory controller, or under our 0.3 rad/s line),
  but behind a "yes" in the lookup. People are not at risk (gantry, 1.5 m rule).
- Fix (C + E2, no new motion logic):
  - Give 87 a `--waist-limits 0.0,0.15,0.35,0.6` option. Block 2 clips `state[26]` to each limit (0.0 = `--waist-hold`, waist at q0 = 0
    in the nominal layout) and reports `replay_exits_clear` / `fault_exits_clear` per waist limit.
  - E2's lookup row is per (elbow, waist limit). Rule 16 (b)/(c) read the column of the limit reached in M3. If that column is missing,
    M4-M6 at the table run only at the limit 87 checked (otherwise skip).
  - Mind the runtime (R3-8).

### MINOR

**R3-3. Motion gate (a): `git diff --quiet HEAD -- docs/ROBOT_DAY_MOTION.json` passes an untracked, hand-made unlock file.**
- Evidence: in a scratch repo, an untracked `docs/ROBOT_DAY_MOTION.json` gives `git diff --quiet HEAD -- <file>` exit 0,
  `git ls-files --error-unmatch` exit 1 and `git cat-file -e HEAD:<file>` exit 128.
- Plan 4.3 (a) (line 491-494) says "tracked and unmodified" but names only the diff primitive. Lens 4 (line 977) catches the same gap
  for (b), but not for (a). Then a JSON written on site with `commit` = HEAD and today's date unlocks motion (R4-1's "one helpful edit").
- Fix: (a) also requires `git cat-file -e HEAD:docs/ROBOT_DAY_MOTION.json`, and the gate reads the JSON content from
  `git show HEAD:docs/ROBOT_DAY_MOTION.json`, not from the working tree. B2's `test_gate_refuses_uncommitted_motion_json` uses an
  **untracked** file and gets [mutation: drop the cat-file check].

**R3-4. The push-1 check and Track C's sims share DDS domain 0 on lo0 at the same time.**
- Everything here initialises domain 0: `g1_brainco_sim/config.yaml:5` `DOMAIN_ID: 0`, LeRobot `unitree_g1.py:362`,
  `71_real_probe.py:37`, `72_real_check.py:44`, `52_brainco_hand_dds.py:29`.
- The push-1 check (6.2, ~23:30-00:15) starts `loopback_lowstate_pub` (rt/lowstate at q = 0 + hand state) and 84 `probe` (which counts
  `rt/arm_sdk` samples). At that time Track C is in review and fix (~23:50-01:00), and its reviewer and fixer re-run 85 plus
  `publish_test_arm_sdk` on lo0.
- 84 can then see foreign `rt/arm_sdk` samples, two LowState publishers and a doubled rate (a false FAIL that delays push 1 into the 1a
  fallback). Or it reads the sim's LowState and passes for the wrong reason. C's tests can also see the helper's q = 0 LowState.
- Fix: before the push-1 check, the main session checks `pgrep -fl "85_arm_sdk_sim|run_sim|g1_brainco_sim|sim_host"` and waits until
  it is empty. Track C (builder, reviewer, fixer) adds `loopback_lowstate_pub|84_real_bringup|89_robot_day_selftest` to its pgrep
  (line 640) before starting 85. Alternatively, a loopback-only `--domain` for the test tools, though that is more code.

**R3-5. Two safety inputs are cwd-relative paths, and one of them fails open.**
- `factory_pose_path` returns `runs/...` and `RUN_LOG_DIR = "runs/real"` (lines 215, 222-226). `arms_may_be_held_by_us()` returns
  False when it finds no sidecar.
- If 86 runs from another cwd (a new terminal, `python ~/UnitreeG1-Robot/scripts/86_real_run.py ...`), or with an `--out` the runbook
  adds, 84 later finds no sidecar and allows `--overwrite-factory-pose` while our run may hold the arms. That is the N4 / M1 snap path.
  The factory record itself fails closed: 86 refuses without a record.
- Fix: anchor both paths to the repo root (`pathlib.Path(__file__).resolve().parents[1] / "runs"`, as 81 anchors `REPO`). 84 scans
  `RUN_LOG_DIR` recursively, or 86 refuses an `--out` outside `RUN_LOG_DIR` off loopback. Add one A test with a sidecar written from a
  different cwd.

**R3-6. Parts of earlier fixes now live only in "as revision 5", and revision 5 no longer exists.**
- `git log --all -- .planning/real-robot/PLAN.md` shows only 8038eaf (rev 6.1). Rev 5 was never committed, and no copy exists in the repo
  or `~/dev/g1-ext/release`. Lines 235, 420, 535, 700, 702 and 925 still say "as revision 5".
- Carried only in the response table, not in E1's task text: m11 (macOS application firewall: allow incoming connections for the
  robot-env python, or DDS samples from the robot never arrive; 89's loopback test cannot see this) and r5-4 (operators outside arm
  reach during owner motion; the label records the owner's tool and mode). Lines 1011 and 1026 are the table entries.
- R4-6: `docs/START_PROMPT.md:100` still names `70_real_goto.py` as the next real-robot step.
- Fix: E1 step 5 names m11 (Part A: `/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate`; if it is on, allow the
  robot-env python) and r5-4 (Part B step 4). E1 step 2 names the setup's outputs that other parts use: `$P/robot-env.sh`,
  `$P/policy-env.sh`, the unpack target `$P/smolvla-runs/run1/checkpoints/040000/`, and the HF cache (R3-1). The main session's push-1
  docs mark 70, 72 `arm` and 72 `hand --bewegen` as "never on the robot day" in `START_PROMPT.md` and `PROGRESS.md`.

**R3-7. 87's straight-line stand-in for the fault ramp-out holds only if the blend weights every joint equally.**
- Assumption (line 123-126): under a linear blend the ramp-out target "lies exactly on the straight joint line". That holds if the
  controller blends q alone.
- 85 blends q, kp and kd (line 371-373). A torque-level blend weights joint i by `w*kp_s / (w*kp_s + (1-w)*kp_f)`. With our gains
  (50/50/80/80/40...) against unknown factory gains, that weight differs per joint. Then shoulder and elbow progress at different rates
  and the hand path bows away from the straight line. Neither blend type is published.
- Fix (cheap): 87 also samples the two "corner" orderings of each `to_hanging` exit (shoulder joints first, then elbow and wrist, and
  the reverse) and reports the worst of the three. Alternatively, the plan states that `fault_exits_clear` assumes a q-only blend, and
  R15 tells the user so.

**R3-8. 87 may not fit its 10-minute budget as specified.**
- Rough count for block 2: about 73 frames x (51 grid lines x 20 samples + about 70 `to_ready` samples), or about 80k samples per
  setback, times up to about 4 setbacks.
- Each sample needs one `mj_forward` plus a `mj_geomDistance` call for every arm/hand geom x every table/stand/basket geom. A static XML
  count gives 28+ arm/hand geoms and 22 object geoms (stand 16 segments, basket 5, table 1), so about 600+ pairs. That is about
  200M Python-level calls, and R3-2 would multiply it.
- Fix:
  - State the pre-filter: per sample, use a numpy bounding-sphere test (`data.geom_xpos`, `model.geom_rbound`) and call
    `mj_geomDistance` only for pairs within `distmax`.
  - Use collision geoms only.
  - Make the budget a reported number, not a done criterion (a slow 87 costs time; a fast but wrong 87 costs safety).

**R3-9. The M0 record can capture an arm pose that the owner's tool still holds.**
- Part B step 4 lets the owner run their own arm motion (`--label owner_motion`, Q12). If their tool stops without ramping its weight
  to 0, a latching controller keeps their last pose. Unitree's arm7 example has no signal handling, so a Ctrl+C mid-motion leaves its
  last weight.
- M0 then records that pose as the factory pose. The preflight sees no samples, and the gap check passes. The first ramp-in message
  at w ~ 0 hands the arms back to the controller's real hold pose in one tick (the M1 snap, from a foreign writer).
- Fix (runbook + one print in 84): M0 compares the new record with the Part B step 0 FACTORY ARMS line, which was taken before any owner
  motion. If they differ by more than `FACTORY_POSE_TOL` and the owner did not change the mode, the owner re-sets the robot (or
  confirms that their tool ended at weight 0) before recording.

**R3-10. The residual risks of M4 at the table are slightly understated.**
- (a) Rule 16 (b) ignores 87's "frames that start inside an object" count (line 399-400). If the nominal layout puts demo frames inside
  the table top, the replay itself presses into the table. Fix: M4 at the table also needs that count = 0 for the table, at the chosen
  setback and waist limit. Stand contacts stay a brush (light stand).
- (b) R15's "faults there are rare" (line 953) has no evidence. A grasp of a real ball offset by the episode's randomization is the
  likeliest lead-fault trigger, so the user should see the stricter option (M4 at the table also needs `fault_exits_clear`) as the
  natural default when the cost is low.
- (c) 86's preflight checks foreign samples only on `rt/arm_sdk` / `rt/lowcmd` (line 209). A foreign writer on `rt/brainco/<side>/cmd`
  (the owner's hand tool) would fight our hand commands. Fix: also count those two topics and warn.

**R3-11. Mutation runs of the refusal and guard tests must not create real DDS writers.** `test_loopback_pub_refuses_robot_network`
[mutation] and `test_publisher_guard_blocks_channel_publisher` [mutation]: with the check removed, a real helper or `ChannelPublisher`
would initialise DDS. The helper would then publish fake `rt/lowstate` on whatever interface the test passed, and during the build
Track C's sims run on lo0. Fix: both tests fake `ChannelFactoryInitialize` / the writer class, so a broken copy fails without touching
DDS.

### Environment (not plan content, but it decides the overnight run)
**At 17:36 and again at 17:43 the laptop runs on battery** ("No adapter attached", `ExternalConnected = No`, 100 %, discharging,
"13:46 remaining" at idle). The plan's Situation line 65 states "on AC power (`pmset` 17:20)". Five agents, MuJoCo, a torch install and
a 40k MPS run will not last until 07:15 on battery. The main session must see `Now drawing from 'AC Power'` before it launches the build.
This is the fourth time this has flipped today.

## Verdict 2: tracks, contracts, verify commands
- **Disjoint: yes.** The six frontmatter lists do not overlap. E2 touches only E1's files plus one new image, after E1 is GO
  (`workflow.js:139-142`). Shared outputs are STATUS (append only), `runs/` (ignored; distinct names per track), C's
  `runs/arm_path_clearance.json` (read by E2 and integration) and one review file per gate. A split of a file past ~300 lines creates a
  new file inside the same track. That is fine, because gate (b) and the unlock recipe's `git status --porcelain -- g1sim scripts`
  catch an uncommitted one.
- **Contracts:** precise enough for A, B1 and B2 (layout pinned by a B2 test; sidecar fields pinned in 3.4 and read by A with fakes; CLI
  and hello checks match `policy_wire`/81). Not precise enough in four places, all fixed above: E1's setup outputs and HF cache (R3-1,
  R3-6), 87's waist dimension (R3-2), gate (a)'s primitive (R3-3), the path anchoring of the two safety files (R3-5).
- **Verify commands on this Mac tonight:** A, B1, B2, C and E2 can run (`$SV` has pyarrow; uv 0.12.13, gh, the tar and its sha256
  present; 79 GiB free; 24 GB RAM). E1 can run too, but its policy self-test is not evidence until R3-1 is fixed. The push-1 check
  needs R3-4. Integration can run: `run_tests.sh` uses `.venv` (mujoco 3.12, `mj_geomDistance` present) and C's DDS tests follow the
  existing subprocess pattern (`conftest.run_python`).

## Verdict 3: timeline
- The plan's own clock (section 6) is internally consistent: build from ~18:05; A GO 23:15-23:45; push 1 ~23:45 (23:15-00:15); B2's
  last re-review ~02:15 (cutoff 02:30); integration to ~04:45; Part C walk to ~05:30; final gate to ~07:15; push 2 ~07:45.
- That clock is **later than the targets in this review's brief** (push 1 ~22:30, latest 23:30; final gate ~06:30; push 2 07:30-08:00).
  Push 1 at ~23:45 means the operators most likely set up on Monday at 08:00. The plan says so honestly (6.1 fallbacks); the user should
  hear it plainly.
- **Slack before push 2 is about 0.** Every miss fails safe (motion stays locked; Parts A+B still run).
- "Latest 09:30" (lines 72-76) needs Finnegan reachable until 09:30 on Monday. Line 64 says to assume he cannot be reached during the day.
  He should confirm the 09:30 window, or the latest push is whenever he goes offline.
- The finish stage (integration, walk, judge) must be launched by the main session after both build runs return (STATUS 17:18). That
  is a hand-off at ~02:15 in an unattended session. If the session is not re-invoked, push 2 carries no motion.
- A NO-GO round costs about 30-40 min (rev 6.2's planner took 10 min, this review about 25). Spent on the critical path, that pushes B2
  past the 02:30 cutoff. Hence the fast path below.

## Checks I could not run
- FK of any arm path (no sims allowed; 87 does not exist yet). The R3-2 direction and the R3-8 runtime are estimates.
- Unitree's blend type (q-only vs torque, R3-7), latch vs timeout, the factory gains, the real factory pose, the firmware's arm_sdk mode,
  and whether an operator laptop has the macOS firewall on. Only Monday shows these.
- Whether the main session plugs in the charger and re-invokes the finish stage overnight.

## Overall (revision 6.2, round 3): **NO-GO**, with a fast path
- No blockers. R2-1..R2-6 are fixed in the plan text. Two majors remain, and both are contract text with no design change:
  - **R3-1:** E1's setup must put the backbone in the prefix and export `HF_HUB_CACHE`, so the fresh-prefix check can fail.
  - **R3-2:** 87 and the lookup must cover the waist limits that Monday actually uses (restores n4).
- Fast path (the main session decides):
  - Tracks **A, B1 and B2** are unaffected by R3-1/R3-2 and are the critical path (B2). They can start now on rev 6.2. Hand R3-3 to B2
    and R3-5 / R3-11 to A as build notes.
  - The planner patches E1 step 2 + verify (R3-1, R3-6), 87's waist dimension + pre-filter (R3-2, R3-8), E2's lookup columns, rule 16
    (b)/(c) and R3-10 (a). A round-4 review checks only those items (about 15 min). C and E1 start after it, and C has slack (C ends
    ~23:05 vs the 02:15 critical path).
  - R3-4, R3-7, R3-9, R3-10 (b)/(c) and R3-11 can be folded in without another review round.
- Plug in the charger first.

---

# Plan review rev 6 - round 4

Reviewer: independent plan reviewer (opus), pre-build stage, round 4 on PLAN.md revision 6.3 (1317 lines, written ~18:07). I did not write
the plan and saw none of the planner's reasoning. Started 2026-10-18 18:08. I ran no sims and edited no plan or code. Read-only checks I
ran: the arm chain in `~/dev/g1-ext/g1_brainco_sim/assets/g1_29dof_no_hand.xml`, `table_scene.py`, `81_smolvla_server.py:37-68`,
`tests/data/replay_demo.npz` (numpy only), `runs/planner_fk_bench.log`, the import graph of every new module, `motion_gate.py` as built
so far, `.planning/robot-day-build.workflow.js`, STATUS, `pmset`, `pgrep`, and one reproduction in the scratchpad (a copy of HEAD plus only
the frontmatter push-1 files; nothing in the repo was touched). Line numbers are PLAN.md rev 6.3 line numbers.

## Facts re-derived for this round
| Claim (plan) | Result | Evidence |
|---|---|---|
| 81 sets `HF_HUB_CACHE` only by `setdefault` to Finnegan's cache (R3-1 premise) | Supported | `81_smolvla_server.py:37, 67-68` |
| `reach_m` lives on `SCENES["volleyball"]`, not `volleyball_walk` | Supported | `table_scene.py:104-121` |
| 87 block 2 size: 73 frames x (51 grid lines x 60 samples + ~30) = ~226k samples per (setback, L) | Supported | `replay_demo.npz`: 726 frames -> 73 at step 10; grid 17 elbows x 3 pitches = 51; 3 orderings x 20 |
| Planner's FK probe: 53 + 22 collision geoms, 70 us forward, 182 us per sample | Supported | `runs/planner_fk_bench.log` |
| Episode 0 joint ranges (the policy-pose stand-ins) | New fact | left shoulder pitch -1.15..-0.18, left elbow -0.77..+1.30, waist -0.50..+0.46 (max |waist| 0.50 < 0.6, so at L = 0.6 no frame is clipped; 0.15 and 0.35 clip part of the episode) |
| Arm length (for 87's sampling density) | New fact | shoulder pitch -> elbow ~0.20 m, elbow -> wrist yaw 0.184 m (`g1_29dof_no_hand.xml:261-318`), plus the Revo2 hand: ~0.5 m shoulder to fingertip |
| Ball | Supported, plus | size-5 volleyball, r 0.105 m, two-hand teacher (`table_scene.py:52-53, 114`) |
| Push 1 = "Track A and E1 files (frontmatter)" (6.2, line 1002) works | **Refuted** | 84 imports `g1sim/real_bringup.py` (`84_real_bringup.py:31`), which is not in the frontmatter; reproduction below |
| B1/B2 start ~18:20 on rev 6.3; "B2's rev-6.3 minors are in its task text" (lines 72, 613) | **Refuted** | STATUS: B2 started 17:47, B1 17:49, both on rev 6.2 (rev 6.3 was written 18:07) |
| "On AC power" before B1/B2/C launch (Situation, line 74-76) | **Refuted at 18:13** | `pmset -g batt`: "Battery Power, 100 %, discharging" while B1/B2 already built |
| Earlier fixes B1, M1-M6, N1, N2, N4, n1-n6, R4-1..R4-6, r5-1..r5-5, RV6-1..3, m6-1..13, R2-1..R2-6, R3-1..R3-11 present in the text | Supported | checked in sections 0, 2.7, 3.1-3.6, 4.1-4.3, 5 and 7 (not in the response tables): frozen `ramp_in(q0)`, `release_plan`, lead only at w == 1, `fault_targets`, `LOWSTATE_HZ`, sample-based `active_writers` incl. hand topics, `--waist-max/--waist-hold`, dated factory record + mode note + M0 + overwrite rule + early sidecar, gate (a) `cat-file -e HEAD:` + `git show`, anchored `REPO` paths, prefix HF cache, 87 per waist limit + corner orderings + pre-filter, `frames_inside`, stricter R15, one-DDS-run rule, faked DDS in mutation tests, firewall + r5-4 in E1 step 5, Q1-Q12 written out |

## Verdict 1 (plan correctness): safety design
The design is sufficient for non-author operators in the parts that protect people: arm_sdk only, a motion gate that fails closed (committed
JSON read from HEAD, pinned commit, every loaded file, today's date, two names, HANGING every command), gantry + owner's remote as the last
resort, frozen q0, slow release to a recorded and protected factory pose, 13/14 held at q0, rate limits in every phase, envelope in the
policy phase, stale/NaN/second-signal handling, measured-angle fault targets, hands capped with an explicit `hands-open` (the BrainCo
service has no timeout). "Our program dies" is covered by latch-or-timeout -> factory-gap refusal -> `86 release` (or the table box first,
rule 16 (e)) -> owner's remote. Every earlier fix is preserved (table above). One new hole sits in the evidence behind the table steps
(R4-1), which is exactly the 87 contract this round was asked to check before C starts.

### MAJOR

**R4-1. 87's corner orderings are sampled too coarsely: a fault drop that passes the table edge at 1.5-3 cm can be reported as >= 5 cm,
and that "yes" unlocks M4 and M6 at the table.**
- Text: 3.5 block 2 (lines 436-441): `to_hanging` = three paths "20 samples each", the corners "10 + 10 samples". `fault_exits_clear` is the
  rule-16 (c)/(d) and R15 default condition for both M4 and M6 at the table (lines 566-571).
- Arithmetic from the code: from an episode-0 pose (shoulder pitch down to -1.15, elbow down to -0.77) to h(1.6, 0.3) the shoulder pitch
  travels ~1.45 rad and the elbow ~2.4 rad; in `proximal_first` the waist yaw (up to 0.5 rad) moves in the same leg. With 10 samples per
  leg, the fingertip (~0.5 m from the shoulder, ~0.33 m from the elbow) moves about 8-10 cm between consecutive samples. For a hand that
  sweeps past the table's front edge, the sampled minimum is about sqrt(d_true^2 + (step/2)^2): a true 1.5 cm pass with a 10 cm step reads
  5.2 cm, a true 3 cm pass with an 8 cm step reads 5.0 cm. Both pass the 5 cm rule. The straight ordering (19 intervals, ~4-6 cm per step)
  eats ~1 cm of the margin. The `to_ready` exit is fine (every 5th tick at 0.3 rad/s, ~1.5 cm per step).
- Why it matters: the 5 cm margin is stated as the allowance for real-vs-sim pose error, tracking lag and sag of a 2 s ramp (lines 143-144,
  464-465). Discretization can consume most of it, and the corner orderings exist precisely because the factory blend may bow the path
  (R3-7). Lens 7 says 87 is "the only evidence for every exit that starts at the table".
- Impact: the same hardware risk class as R2-1/R3-2 (Revo2 fingers on the table edge under the factory controller), behind a "yes" in the
  lookup. People are not at risk (gantry, 1.5 m rule, "contact with the table edge" abort). Not computed by FK (no sims allowed); the step
  sizes come from the MJCF offsets and the demo's joint ranges.
- Fix (87 contract + one C test, no new motion logic): sample every path so that no arm/hand collision geom centre moves more than
  `MAX_SAMPLE_STEP_M` (named constant, e.g. 0.02 m) between consecutive samples, measured from `data.geom_xpos` and subdividing an interval
  where it does (adaptive, so the runtime grows only where the arm moves fast); report the largest step used in the summary; add
  `test_fault_exit_sampling_cannot_skip_a_close_pass` (a planted corner leg whose true pass at a box edge is 1 cm but whose 10 even samples
  all read >= 5 cm must fail) [mutation: fixed 10 samples per leg]. If the runtime then exceeds 30 min, the existing `--frame-step 20`
  fallback applies.

**R4-2. Push 1 as specified ships an 84 that cannot start: `g1sim/real_bringup.py` (and three A test/helper files) are not in the
frontmatter, and the push-1 check cannot see a missing file.**
- Text: 6.2 line 1002: "Push 1 files: Track A and E1 files (frontmatter ...)". Frontmatter lines 11-19 list `real_factory_pose.py` but not
  `g1sim/real_bringup.py`, `tests/helpers/real_fakes.py`, `tests/test_unit_real_bringup.py`, `tests/test_unit_real_factory_pose.py` (A's
  ~300-line splits; A's reviewer flagged the push list at 18:13 as "3 new test files").
- Reproduction (scratchpad copy of `git archive HEAD` + exactly the frontmatter A and E1 files, sim env python):
  `84_real_bringup.py --dry-run` -> exit 1, `ImportError: cannot import name 'real_bringup' from 'g1sim'`; collecting
  `tests/test_unit_real_io.py` -> ImportError (`runs/frontmatter_84_dryrun.log`, `runs/co.log` in that scratch folder). 89's `bringup`
  check runs `84 --dry-run`, so the operators' Sunday `SELFTEST` fails, and Monday's Part B (all of it is 84) cannot start.
- Why nothing catches it: the push-1 check (6.2) runs 89 and 84 from this working tree, where the untracked file exists. The unlock
  recipe's `git status --porcelain -- g1sim scripts` guards push 2 only; push 1 deliberately leaves B1/B2/C files untracked, so "must be
  empty" cannot be used there. The same gap exists for B2's splits (`real_preflight.py`, `real_run_log.py`, `real_run_modes.py`,
  `real_run_policy.py`, `tests/helpers/fake_real_run.py`), which are also missing from the frontmatter; push 2 is protected by the recipe,
  but the final gate's per-file list, 4.1 (b)'s import ban and 89's `FORBIDDEN_MODULES` are derived from the same incomplete lists.
- Urgency: A and E1 are both GO at 18:13 (STATUS), so push 1 can happen within the hour.
- Fix (plan text + one command): add every split file to its track in the frontmatter (A: the four above; B2: the five above); state that a
  track's push set = frontmatter + any file the track reports as a split; and make the push-1 check prove the commit, not the working
  tree: after Finnegan's local commit and before `git push`, `git clone -q . <tmp>` (a local clone of the committed state) and run
  `89 --policy-device cpu` and `84 --dry-run` from it with `$P/robot-env.sh` (output to `runs/push1_clean_*.log`). Same clean-clone check
  before push 2.

### MINOR

**R4-3. Section 6's clock is stale by hours, and two statements in it are now false.** Observed pace (STATUS): builds 5-13 min, reviews
5-6 min, fix rounds 5-8 min; A and E1 GO at 18:13. So push 1 is realistic ~18:45-19:30 (after R4-2's fix and the 20-min check), not
"~00:00", and "the operators most likely run the setup on Monday at 08:00" (line 951-952) is wrong in the direction that matters to the
user: they can set up Sunday evening, which is the safer path (the Monday 08:00 install over venue Wi-Fi is the risk). B1/B2 started on
rev 6.2 at 17:47/17:49, so "B2's rev-6.3 minors are in its task text" (line 613) is false; B2's code at 18:08 has no `--out` refusal and
no `foreign_hand_cmd`, and gate (a) reads the JSON from the working tree (`motion_gate.py:59-70`; `ls-files --error-unmatch` + `diff
--quiet HEAD` is equivalent for the untracked case but not the contract's primitive). Fix: re-time 6.1 from STATUS; extend main-session
seam (3) to B1/B2 ("rev-6.3 items arrive as review findings"); tell Finnegan to message the operators that setup can start tonight.

**R4-4. The safety state that decides "may our code still hold the arms" is per laptop, but the hazard is per robot.** The factory record,
86's sidecars and the Part B reading (`earliest_factory_arms_today`) live in `runs/` of one clone. If the robot laptop dies (power, panic)
during an 86 run and the operators continue on the second laptop, M0 there sees no sidecar (`arms_may_be_held_by_us` False) and no Part B
reading (no warning), so it can record a latched policy pose as the factory pose (the N4/M1 snap path). `86 release` on that laptop is
refused (no record), which routes to the owner, but M0 is a runbook step and comes first in the operators' minds. Fix: runbook page 1
"84 M0 and every 86 command run on one laptop all day; if it dies while 86 runs: owner's remote, owner re-stands the robot, then M0 on the
new laptop"; 84 prints "no Part B reading found on this laptop - M0 cannot compare" when `earliest_factory_arms_today()` is None.

**R4-5. 87 ignores the ball held between the hands.** The volleyball has r 0.105 m and the grasp is two-handed (`table_scene.py:52-53, 114`).
For demo frames after the lift, the `to_ready` leg and the fault drop (hands hold on a fault) carry the ball, which reaches up to ~10 cm
beyond the hand geoms that 87 measures. The ball is light (0.27 kg) and soft, so the likeliest outcome is the ball pressed against the
table edge between the descending hands. Fix: name it as a residual risk in R15 and the runbook ("a fault while the ball is lifted may
press the ball onto the table edge; abort as contact"), or add a 0.105 m sphere between the palms for frames whose hands are closed.

**R4-6. The M0 lookup rounds the larger factory elbow up to the next 0.1 row and assumes clearance falls monotonically with the elbow.**
Nothing pins that monotonicity (the fault drop is a 3-D sweep past an edge). Fix: the runbook row uses both neighbouring rows (floor and
ceil) and says "yes" only if both do; E2's report-vs-runbook test checks it.

**R4-7. The one-DDS-run `pgrep` pattern (section 0, 6.2) misses DDS traffic inside pytest.** `sim_host.py` is a library (3.5), so
`publish_test_arm_sdk` and `start_standing_g1` run inside a `pytest` process whose command line does not contain `sim_host`; between two
85 instances, C's tests can publish `rt/arm_sdk` on lo0 while the pattern prints nothing. Fix: add
`pytest.*(test_e2e_real_adapter_sim|test_integration_arm_sdk_sim)` to the pattern.

### Environment (not plan content, but it decides the overnight run)
At 18:13 the laptop is on battery again ("Battery Power, 100 %, discharging, 11:06 remaining"), and B1/B2 were launched without the "AC
Power" check the plan requires. C (MuJoCo, DDS), the integration's 40k MPS run and the judge will not last until 07:30 on battery.

## Verdict 2: tracks, contracts, verify commands
- **Disjoint: yes in practice, no on paper.** No two tracks write the same file (checked on disk: `real_fakes.py` is A's, `fake_real_run.py`
  and the four `real_run*`/`real_preflight` modules are B2's, by their docstrings and importers). But the frontmatter, which push lists,
  the final gate and the import bans are built from, is incomplete (R4-2).
- **Contracts:** precise enough for C to start: 85/sim_host/e2e are unchanged since round 3; 87's inputs exist (`replay_demo.npz` 726
  frames, B1's `move`/`sdk_targets`, `TableEnv`, `mj_geomDistance`); the per-waist and per-ordering report shape is fully specified. The
  one gap is the sampling density (R4-1).
- **Verify commands on this Mac tonight:** all runnable (A, B1, E1 already ran theirs; C's need the sim env, mujoco 3.13 and DDS on lo0;
  E2's need only pytest and git). The push-1 check runs, but proves the working tree, not the commit (R4-2).

## Verdict 3: timeline
- Fits with large slack. The plan's clock (push 1 ~00:00, push 2 ~08:00 with 0 slack) assumed 3-5 h builds; the observed pace is about
  10x faster, A and E1 are GO at 18:13, and B1/B2 are in review. Even with one more planner pass, C is unlikely to finish later than
  ~21:00-22:00, which leaves the integration, the Part C walk and the final gate well before 06:30 and push 2 at 07:30-08:00 comfortable.
- Every miss still fails safe (motion locked, Parts A+B run).
- The real timeline risks are power (above) and the main session being re-invoked for the finish stage overnight.

## Checks I could not run
- FK of any 87 path (no sims; 87 does not exist yet). R4-1's step sizes are from MJCF offsets and the demo's joint ranges.
- Unitree's blend type, latch vs timeout, factory gains, the real factory pose, the firmware's arm_sdk mode. Only Monday shows these.
- Whether Finnegan's push-1 commit will contain the split files (R4-2 is about the plan text that directs it).

## Overall (revision 6.3, round 4): **NO-GO**, with a fast path
- No blockers. Two majors, both plan text and cheap:
  - **R4-1:** 87 must sample by geom displacement (adaptive, `MAX_SAMPLE_STEP_M`), report the step, and test it with a mutation.
  - **R4-2:** add the split files to the frontmatter and prove push 1 (and push 2) from a clean local clone of the commit. This one is
    urgent: push 1 can happen within the hour.
- Fast path (the main session decides): C can start now. 85, `sim_host`, the integration and e2e files are untouched by R4-1, and 87 can
  be built with R4-1's sampling rule as a build note; C's code reviewer then checks it. R4-2 needs no review round, only the frontmatter
  edit and the clean-clone command before push 1. R4-3..R4-7 fold in without another plan review.
- Plug in the charger.

# Plan review rev 6 - round 5

Reviewer: independent plan reviewer (opus), pre-build stage, round 5. I did not write the plan and saw none of the planner's reasoning.
The task named "revision 6.x"; the file on disk at 18:30 is **revision 7** (frontmatter "7 (2026-10-18 18:28 ...)", 1547 lines), which
revises 6.3 after round 4, so this round reviews revision 7. Started 18:30, written 18:39. Nothing in the repo was edited except
this appended section and two STATUS lines. No sims, no DDS, no robot. Read-only checks I ran: `git status/diff/log`, the Track A
follow-up diff (`g1sim/real_io.py`, `scripts/84_real_bringup.py`, `tests/test_unit_real_io.py`, uncommitted), `runs/push1_84*.log`,
`runs/real_a_fu_*.log`, `runs/real_a_followup_84b_*.log`, `motion_gate.py`, `real_preflight.py`, `86_real_run.py` (grep),
`72_real_check.py` (imports), the runbook working copy vs `HEAD`, `real-robot-runbook-walk-AB.md`, `real-robot-code-E2.md`, `.gitignore`,
the arm7 example, `policy_wire.PolicyClient`, `$P` contents, `pgrep` with the section-0 pattern (prints nothing, exit 1), `pmset`, and
two FK-only numpy/MuJoCo kinematics computations on `tests/data/replay_demo.npz` (`mj_kinematics` of the bare G1 MJCF, no stepping,
no scene, no DDS). Line numbers are PLAN.md rev 7 line numbers.

## Facts re-derived for this round
| Claim (plan) | Result | Evidence |
|---|---|---|
| R4-1 planted test: 0.5 m arm, 2.4 rad in 10 even samples (13.3 cm), edge 1 cm off the arc -> even samples ~6.8 cm, densified <= 1.5 cm | Supported | law of cosines: 6.80 cm; the first halving inserts the sample exactly at the edge angle -> 1.0 cm |
| `MAX_SAMPLE_STEP_M` 0.02 overstates by sqrt(d^2+0.01^2)-d ~ 1 mm at 5 cm | Supported | 0.99 mm; `|dc| + rbound*angle` is a true bound on any point of a rigid geom (`|(R_b-R_a)v| = 2 sin(theta/2)|v_perp| <= theta*r`) |
| Smoothstep over 100 ticks peaks at 0.015 per message | Supported | max slope of 3t^2-2t^3 = 1.5 -> 1.5/100 |
| 726-frame demo: 97 s at 0.25, 49 s at 0.5 | Supported | 726/7.5 = 96.8, 726/15 = 48.4 |
| Arm7 example: weight in slot 29, kp 60 / kd 1.5, 50 Hz; 13/14 "INVALID ... with waist locked" | Supported | `g1_arm7_sdk_dds_example.py` lines 40-75, 135-168 (commanding 13/14 at q0 is harmless on a locked waist) |
| 86 passes `timeout_s=2, connect_wait_s=10` (the client's own defaults are 120/60) | Supported | `policy_wire.py:267`; `86_real_run.py:33, 201-202` |
| `docs/ROBOT_DAY_MOTION.json` and `docs/robot_day/*.png` are not git-ignored | Supported | `.gitignore` has `docs/*.png` (direct children only); `git check-ignore` prints nothing for both |
| Push-1 check prerequisites `$P/robot-env.sh`, `$P/hf_cache/hub/models--*` exist | Supported | `ls ~/dev/g1-ext/robot-day-test` |
| Planner probe: a clean clone of `c0898cf` passes `84 --dry-run` and 55 unit tests, "what remains is the DDS part" (6.2, lines 1167-1168) | Supported but **incomplete** | the DDS part was run at 18:16-18:20 and found an intermittent 84 crash: `runs/push1_84.log` 0 bytes (died before any output), `runs/push1_84_rep1.log` passes; isolated reproduction `runs/real_a_fu_drop_reader_old{1,2,3}.log` all "Fatal Python error: Segmentation fault", cause in `runs/real_a_fu_drop_reader_why.log` |
| Push-1 runbook = E1's committed runbook (`f897b0c`) | **Superseded** | paper walk NO-GO at 18:22 (4 majors + 1 conditional), fixed in the working copy at 18:33 incl. a new major F21 (table height 10 cm too low), in the same file as E2's uncommitted Part C |
| Section 7 Part B 3 "table top 0.74 m + (pelvis height - 0.7505 m)" | Ambiguous | 0.7505 is the pelvis body origin (`table_scene.py:69-71`); the hip pitch axis is 0.1027 m lower (`g1_29dof_no_hand.xml:72`), 0.648 m; the committed runbook told operators to measure to the hip axis |
| Demo start-pose spacing at `--frame-step 10` (new) | New fact | FK of a point 0.15 m beyond each wrist-yaw link: hand moves between consecutive checked frames max **11.8 cm**, p90 5.5 cm, median 1.5 cm; at `--frame-step 20` max 14.2 cm, p90 10.7 cm; 102 of 716 10-frame windows move > 5 cm, all with the hand over the table (pelvis-relative x 0.26-0.52 m, beyond the 0.2455 m front edge) |
| Whole-demo hand path (new) | New fact | 1.70 m (larger of both hands) -> about **86** start poses at 2 cm spacing, vs 73 at frame-step 10 |
| Battery | Unchanged | `pmset -g batt` 18:38: "Battery Power, 94 %, discharging, 7:05 remaining"; C has not started (no `scripts/85*`, `87*`, `sim_host.py` on disk) |
| Earlier fixes B1, M1-M6, N1, N2, N4, n1-n6, R4-1..6 (rev 4), r5-1..5, RV6-1..3, m6-1..13, R2-1..6, R3-1..11, R4-1..7 (round 4), A-review and B1-review planner items | Supported | present in sections 0, 1, 2.7, 3.1-3.6, 4.1-4.3, 5, 6.2-6.5, 7, 10, not only in the response tables (frozen `ramp_in(q0)`, `release_plan`, lead at w == 1, `fault_targets`, `LOWSTATE_HZ`, sample-based `active_writers` incl. hand topics, waist max/hold, dated + protected factory record in M0 with mode note, `arms_may_be_held_by_us` + early sidecar, gate (a) `cat-file -e HEAD:` + `git show`, `REPO` anchors, prefix HF cache, 87 per waist limit + corner orderings + `densify`, `frames_inside`, stricter R15, one-DDS-run pattern incl. pytest, faked DDS in mutation tests, firewall, Q1-Q12, one-laptop rule, ball residual risk, both lookup rows, `M3_LEAD_MAX`, clean-clone push checks) |

## Verdict 1 (plan correctness): safety design
Sufficient for two non-author operators in everything that protects people: arm_sdk only (never Damp/ZeroTorque/lowcmd, static test),
a motion gate that fails closed (committed JSON read from HEAD, full-commit pin, every loaded repo file unchanged, today's date, two
names + START, HANGING with a defined meaning for every motion command incl. `release`), gantry + owner's remote as the last resort,
frozen q0 on ramp-in, smoothstep ramps <= 0.02 per message, slow release to a dated, protected factory pose recorded in the Part C
mode, 13/14 held at q0 (harmless on a locked waist), rate limits in every phase, envelope in the policy phase only, NaN -> fault before
send, stale LowState -> fault on the last command, camera/hand/policy stale -> soft exit through the ready pose, second signal -> fault,
SIGHUP = Ctrl+C, `finally` ramp-out, hands capped with an explicit `hands-open` (the BrainCo service has no timeout). "Our program dies"
is covered by latch-or-timeout -> early sidecar -> factory-gap refusal -> `86 release` (table box first, rule 16 (e)) -> owner's remote.
Every earlier fix is preserved (table above). Two holes, both in the evidence and process around the table steps and push 1, not in
the person-safety chain:

### MAJOR

**R5-1. 87 samples the fault/soft exits densely along each path (R4-1) but picks the START poses by frame count, and the gap between
two checked start poses is up to 11.8 cm of hand travel (14.2 cm with the plan's own `--frame-step 20` fallback). A fault from an
unchecked pose between them can pass the table edge several cm closer than any checked row, behind a "yes".**
- Text: 3.5 block 2 (line 485-486) "start poses = every `--frame-step`-th frame of `--demo`"; line 517 and lens 9 (line 1395-1396):
  "re-runs it with `--frame-step 20` ... the fallback halves frames, never the sampling density"; R15 (line 1326-1327): "every path is
  sampled to 2 cm ..., so a 'yes' is not an artefact of coarse sampling".
- Evidence (FK only, re-derived above): at frame-step 10 the hand moves max 11.8 cm / p90 5.5 cm between consecutive checked frames,
  and every fast window has the hand over the table (x 0.26-0.52 m from the pelvis, front edge at 0.2455 m). A drop from an unchecked
  pose differs from the nearest checked drop by up to half that spacing at the start (~6 cm), shrinking towards the common goal; the
  closest approach to the front edge happens early in the drop, so 3-4 cm of the 5 cm margin can be eaten - the same magnitude R4-1
  was rated major for. `frames_inside.table` has the same blind spot (a frame between two checked ones that starts inside the table
  top is not counted). For M4 these are exactly the poses the replay passes through; for M6 the policy wanders further anyway.
- Trigger: any fault (second Ctrl+C, stale LowState, lead/contact, NaN) during the fast reach/lift phases of an M4 replay or M6 run at
  the table, at a factory elbow whose lookup row says "fault exits clear".
- Impact: Revo2 fingers / forearm on the table edge under the factory controller (hardware risk on the owner's robot; people stay
  protected by the gantry and the 1.5 m rule; the "contact with the table" abort stays).
- Fix (87 contract only; C has not started, so a build note is enough - round-4 fast path): choose start poses by arm movement, not by
  frame count: walk the demo frames and keep a start pose whenever `step_of(last_kept, frame)` would exceed `MAX_SAMPLE_STEP_M`,
  inserting joint-space interpolated poses where two consecutive raw frames are already farther apart (max 2.6 cm per frame
  measured); report `max_start_step_m` next to `max_sample_step_m` and assert it in C's verify and 6.3. Cost: ~86 start poses instead
  of 73 (whole-demo hand path 1.70 m), so the runtime estimate stays. Replace the `--frame-step 20` fallback with "fewer setbacks per
  run, or one waist limit per run, merged into one report" (never coarser start spacing); correct the R15 sentence. One C test, pure
  numpy like `test_fault_exit_sampling_cannot_skip_a_close_pass`: a planted demo whose dip lies between frames 10 apart must fail
  [mutation: fixed frame step].

**R5-2. Push 1 as the plan defines it (the local commits `c0898cf` + `f897b0c`, A's files "frozen until push 1") is stale: two defects
found after the plan's evidence are fixed in uncommitted, unreviewed edits of A's and E1's files by writers the plan does not name,
and the runbook file has had three writers in 20 minutes.**
- (a) 84 in `c0898cf` crashes intermittently. The main session's push-1 DDS run (18:16-18:20) got an empty `runs/push1_84.log` (the
  process died before printing) while repeats passed; the cause is a reader channel dropped by `step_hands` and freed by the cyclic GC,
  after which cyclonedds 0.10.2 skips `dds_delete` and calls a freed callback (`runs/real_a_fu_drop_reader_why.log`: SIGSEGV, 3 of 3
  isolated runs). A second defect: `rate_hz` divides by the full window, so 84 printed "slow hand state (left 4 Hz, right 4 Hz)" for a
  healthy ~78-100 Hz stream (`runs/push1_84_rep1.log`), which on Monday sends the operators after a healthy hand service. 89's
  `bringup` check runs only `84 --dry-run` (no readers), so the operators' SELFTEST passes and the crash shows up in Part B.
  The "Track A follow-up (push-1 check)" (STATUS 18:20) fixes both in the working tree (`_open_channels`, `close_dds` at `atexit`,
  new `rate_hz`, two tests, `faulthandler` in 84), i.e. it edits files the plan freezes until push 1 (section 5, line 696-697), has no
  findings file in 0's list, and changes `real_io` - a shared consumer (lens 5) that 86 (B2), `sim_host` (C) and the loopback helper
  load and that motion gate (b) pins. Plan 6.2 runs 84 **once** from a clean clone of the commit, so against an intermittent crash it
  can pass by luck and push the crashing 84.
- (b) The runbook in `f897b0c` failed the operator paper walk (`real-robot-runbook-walk-AB.md`, 18:22: F1 no way to find the camera
  number, F2 SSH password never mentioned, F3 table position undefined, F4 Ubuntu `gh`, F5 "robot laptop" undefined - the last one
  decides Part A tonight) and F21 found during the fix: the table-height formula tells operators to measure to the hip axis but uses
  the pelvis-origin height, so the table ends up ~10 cm too low (re-derived: 0.7505 - 0.1027 = 0.648 m). The fixes (18:33) live in
  the same working-copy file as E2's Part C (E2 GO round 1 with 6 minors still to fold in) and are under re-review since 18:34. The
  plan still schedules the "Parts A+B dry walk" after push 1 (6.1 row ~19:30-20:15) and lists no runbook writer besides E1/E2.
- Impact: followed literally, push 1 gives the operators tonight a runbook judged NO-GO for Part A and an 84 that may segfault on
  Monday; followed with the fixes, push 1 carries code and text no gate in the plan reviewed, and B2/C keep building and testing
  against the old `real_io` until someone tells them.
- Fix (plan text + main-session steps, no new code): (1) define the push-1 fix set = the Track A follow-up edits + the runbook working
  copy (Parts A+B fixes, and E2's Part C after E2's fix round, which is inert while `86 gate` says LOCKED - or Part C reverted to the
  E1 stub for push 1); (2) each gets a review gate with its own file (`real-robot-code-A-followup.md`; the walk file's re-review already
  runs) before Finnegan commits it; (3) one writer at a time on `docs/ROBOT_DAY_RUNBOOK.md` (E2 fix round, then nobody else until the
  push-1 commit), and A2 starts only after the follow-up is committed; (4) 6.2 runs 84 at least 5 times from the clean clone (all exit
  0, each log ends with `OWN WRITERS: 0`), and its hand-state line must show no "slow hand state" warning against the 100 Hz helper;
  (5) B2's re-review and C's e2e run against the follow-up `real_io` (the `atexit` `close_dds` now runs in 86 after its `finally`
  ramp-out) and the integration log says so; (6) section 7 Part B 3 uses the hip-axis formula (0.74 m + (hip height - 0.648 m)).

### MINOR

**R5-3. An exit 4 for `lowstate_stale` (the typical signature of a pulled or flaky robot cable) is treated as "weight 0 delivered",
though the ramp-out may never have reached the robot.** Rule 16 (e) (lines 646-647) says "after an exit 4 the ramp-out has already
handed the arms to the factory controller (weight 0 sent), so `86 release` should move nothing", and `arms_may_be_held_by_us` reads
`last_weight_sent` 0 as free (3.1, lines 284-287). With the link down, the controller may still latch our last policy pose over the
table: the table box is skipped (86 printed a final line), and 86's gap refusal says "our code does not hold the arms ... re-record the
factory pose (runbook M0)". The owner's confirmation at M0 and 87's straight ordering (= the release path from demo poses) limit the
damage, so minor. Fix: treat a newest sidecar with `exit_reason` `lowstate_stale` like an unfinished run (A2: one line in
`arms_may_be_held_by_us` + one test), and in the rule 16 (e) box "86 printed no final line **or** `FAULT EXIT lowstate_stale`".

**R5-4. The runbook fixer made `72_real_check.py kamera --suchen` a runbook command; the plan's "never" box and rule 4.1 (1)(a) cover
72 only as a forbidden tool.** `kamera` is SSH-only (lazy imports; the publishers sit inside `arm`/`hand`, `72_real_check.py:101-111,
186-206`), so it is read-only, but the plan should sanction it explicitly (section 0 and 7: "the only allowed 72 command is
`kamera --suchen`") or move the lookup into 84 via A2. The walk's five proposed script changes (Q2 "which /dev/videoN", Q5 SSH user in
`real_bringup.QUESTIONS`, ...) belong in A2's list, and the walk's open user questions (may the table move to match the camera picture;
the shared folder name) belong in section 10.

**R5-5. The push-2 proof does not catch a missing test file.** B2 reported two more splits at 18:34 (`tests/test_unit_real_preflight.py`,
`tests/test_unit_real_send.py`, untracked); the push-set rule covers them through STATUS, but the unlock recipe's
`git status --porcelain -- g1sim scripts tests/helpers` (line 674) and the clean-clone lines (gate + 89) do not look at `tests/test_*`.
Fix: add them to the frontmatter and widen the recipe's status check to `tests`, or run B1/B2's unit tests once in the clean clone.

### Environment (not plan content)
At 18:38 the laptop is still on battery (94 %, 7:05 remaining). C has not started; the plan correctly holds C, the integration and the
final gate until `AC Power` is seen, so an unplugged charger now delays the whole motion path, not only the overnight run.

## Verdict 2: tracks, contracts, verify commands
- **Disjoint in the frontmatter: yes** (A, B1, B2, C, E1 own different files; E2 and A2 are serialised after E1 and push 1). **In
  practice: no** - since 18:15 the runbook had E2 and the A+B fixer as writers, and A's `real_io`/84/test file have the push-1
  follow-up as a writer the plan does not list (R5-2). C's six files are untouched and nobody else writes them.
- **Contracts precise enough to build in parallel: yes** for 85, `sim_host`, the e2e file and 87 including `densify`; one gap in 87's
  start-pose selection (R5-1), fixable as a build note before C writes 87. B2 has a few rev-7 items still open by its own STATUS
  (`MAX LEAD` not built); the plan names the runbook stand-in.
- **Verify commands runnable on this Mac tonight: yes.** `$P` has both envs and the prefix cache; the `pgrep` pattern works (exit 1,
  no self-match); C's commands need the sim env and DDS on lo0 (one run at a time); E2's need pytest and git; 6.2 and the 4.3 clone
  lines run from a local clone. 6.2's single 84 run is too weak against an intermittent crash (R5-2 (4)).

## Verdict 3: timeline
- Fits, with hours of slack. A and E1 were GO at 18:13; B1 and B2 are in short test-only fix loops (5-10 min per round); E2 is GO
  round 1. Folding R5-2 in (one review of a ~40-line diff, the runbook re-review already running, 5 x 84 from a clean clone) moves push
  1 from the plan's 18:45-19:30 to about **19:30-20:15**, still far ahead of the old 23:30 latest. C is the critical path: started
  after AC and the R5-1 build note, ~30-60 min build + 87 run of 10-25 min (unchanged by R5-1), review/fix to ~21:00-21:30;
  integration, Part C walk and the final gate fit well before the 06:30 target and the 07:30 cutoff; the unlock waits for 00:00 by
  design; push 2 ~07:30-08:00 and the operators' 08:00 pull hold.
- Every miss stays fail-safe (motion locked, Parts A+B run). The real risks are power and the finish stage being re-invoked.

## Checks I could not run
- FK with the real 87 geom set and `step_of` (87 does not exist; my spacing numbers use a point 0.15 m beyond the wrist-yaw link, so
  the real collision-geom spacing is somewhat larger, which strengthens R5-1).
- Whether the Track A follow-up fully removes the crash (its own 4-5 repeats pass; no reviewer has judged it yet).
- Unitree's blend type, latch vs timeout, factory gains, the real factory pose, the firmware's arm_sdk mode, whether the link loss in
  R5-3 really leaves the arms latched. Only Monday shows these.

## Overall (revision 7, round 5): **NO-GO**, with a fast path
- No blockers; the person-safety chain is sound and every earlier fix is preserved. Two majors:
  - **R5-1:** 87 must pick start poses by arm movement (<= `MAX_SAMPLE_STEP_M`), report `max_start_step_m`, drop the
    `--frame-step 20` fallback, test it with a mutation. C has not started, so this goes to C as a build note now.
  - **R5-2:** redefine push 1 as the commits plus a reviewed push-1 fix set (84 crash + rate fix, runbook A+B fixes incl. the 10 cm
    table height, Part C after E2's fix round or reverted to the stub), one runbook writer at a time, 84 repeated 5x in the clean-clone
    check, B2/C told about the new `real_io`. Urgent: push 1 is the next step.
- Fast path (the main session decides): both majors are plan text plus main-session steps; neither needs another full plan round if
  C's code reviewer checks R5-1 and the follow-up's reviewer and the 6.2 log check R5-2. R5-3..R5-5 fold in via A2/E2.
- Plug in the charger before C.

---

# Plan review rev 6 - round 6 (focused on R5)

Reviewer: independent plan reviewer (opus), pre-build stage, round 6. I did not write the plan and saw none of the planner's reasoning.
Reviewed: PLAN.md **revision 7.1** (frontmatter "7.1 (2026-10-18 18:56 ...)", 1861 lines). Scope, as tasked: only whether R5-1..R5-5
are fixed correctly, whether the new facts (motorized head, on-site SSH login, table on its tape marks) agree across sections, and
whether the "Build tracks" section is still consistent. Started 18:57, written 19:05. I edited nothing except this appended section
and two STATUS lines. No sims, no DDS, no robot. Read-only checks: `git log/status/show --stat/reflog`, `real_io.py` (grep),
`real_run.py`/`86_real_run.py`/`84_real_bringup.py`/`real_bringup.py` (grep for the exit and log strings 6.2 and rule 16 (e) rely on),
`real_run_exits.py` imports, the two ban lists (`test_unit_no_publisher.MOTION_MODULES`, `89 FORBIDDEN_MODULES`), `72_real_check.py`
(kamera path), `payload.py:1-15`, `table_scene.py:60-85`, `g1_29dof_no_hand.xml` (both copies under `~/dev/g1-ext`), the pushed
runbook (grep), `real-robot-runbook-walk-AB.md` (diff check + main-session fixes), `real-robot-code-B2.md` (planner notes), STATUS,
`runs/push1c_*`, `runs/push1x_*`, `runs/push1_git.log`, and one FK-only computation (`mj_kinematics` on the `volleyball_walk`
TableEnv model, no stepping) of `select_start_poses` with a `step_of` = |dc| + rbound * angle over the 53 arm/hand collision geoms
(script and log: scratchpad `startpose_fk.py` / `startpose_fk.log`). Line numbers are PLAN.md rev 7.1 line numbers.

## Facts re-derived for this round
| Claim (plan) | Result | Evidence |
|---|---|---|
| R5-1 test: 0.5 m arm, 2.4 rad in 21 frames = 6 cm per frame; edge 1 cm off the arc midway between frames 14 and 15; frames 0/10/20 read ~27 cm | Supported | chord 2 * 0.5 * sin(0.06) = 6.00 cm; nearest of frames 0/10/20 to the point at 1.74 rad, r 0.51 = frame 10: 26.96 cm; movement selection halves 6 -> 3 -> 1.5 cm and the first halving lands exactly on the edge angle -> 1.0 cm |
| `select_start_poses` keeps every skipped frame within `MAX_SAMPLE_STEP_M` of the pose kept before it; consecutive kept poses <= the constant | Supported | the rule "keep the previous frame when the next one would exceed" plus densify of a too-long raw pair gives exactly that; my FK implementation of the 3.5 text: max kept step 2.00 cm for all four waist limits |
| "Expected ~86 start poses ... add ~15 %" (3.5, lens 10 (a)) | **Partly off** (runtime only) | FK with the real 53 collision geoms (max rbound 0.105 m): **59 / 77 / 101 / 115** start poses at L = 0 / 0.15 / 0.35 / 0.6 (raw max step per frame 2.96 cm, not 2.6 cm: the rbound * angle term); sum 352 vs 4 x 73 = 292 = **+21 %** before the cross-L reuse, which now hits less often because each L selects its own poses. Runtime stays a reported number with a merge fallback, so no safety effect (minor R6-3) |
| Hip formula: table top = 0.74 + (hip height - 0.648 m), 0.648 = 0.7505 - 0.1027 | Supported | `g1_29dof_no_hand.xml:72/149` hip pitch body `pos="0 +-0.064452 -0.1027"` (x offset 0, so the hip axis is directly below the pelvis origin; the -0.96 deg pelvis pitch shifts it 1.7 mm); `table_scene.py:69-71` pelvis z 0.7505; 0.6478 m |
| "24.5 cm in front of the hip axis" = 87's nominal layout | Supported | 87 uses `volleyball_walk` at `WALK_STANCE` (pelvis 0.2455 m from the front edge, `table_scene.py:78-82`); hip axis x = pelvis x; stand 14.4 cm back = 0.344 - 0.20 |
| Head: XL330 pan +-50 deg, tilt below it, training pose pan 0 / tilt 0.85 rad | Supported | `payload.py:5-7`, `table_scene.py:39`; 48.70 + 2.29 - 0.96 = 50.03 deg (2.4) |
| "Nothing in our code commands the head" | Supported | grep head/xl330/dynamixel/set_head over `g1sim/real_*.py` (incl. the new `real_run_exits.py`), `motion_gate.py`, 84, 86: only image/camera/reference-frame names |
| `72 kamera --suchen` is read-only | Supported | `72_real_check.py:225-243`: lazy `base64`/`pathlib`/`subprocess`, SSH only; the DDS publishers sit in `cmd_arm` (101-111) and `cmd_hand` (186-206) |
| R5-3 string the runbook box relies on | Supported | `real_run.py:220` raises `ExitRequest("fault", "lowstate_stale")`; `86_real_run.py:18, 37` print `86 <MODE>: FAULT EXIT <reason>` |
| 6.2 strings | Supported | `84_real_bringup.py:269` "OWN WRITERS: ... (must be 0)"; `real_bringup.py:194` "slow hand state"; `real_io.py:68, 166, 176, 180` `_open_channels`, `atexit.register(close_dds)`, `def close_dds` |
| R5-5 unlock status check over `tests` | Supported | 4.3 line 761 `git status --porcelain -- g1sim scripts tests`; today it prints the 16 untracked B2 files, so it would stop an unlock that forgot any of them |
| Frontmatter vs files on disk | **Gap** (minor R6-2) | on disk and untracked: `g1sim/real_run_exits.py` (18:47), `tests/test_unit_real_run_exits.py` (18:50), `tests/test_unit_real_run_options.py` (18:50) - B2's 18:56 split, named in STATUS 18:56 and in `real-robot-code-B2.md:361-363`; not in the frontmatter, B2 `<files>`/verify, or 6.4's explicit test list |
| Repo state vs plan | **Overtaken** (R6-1) | `git reflog origin/main`: push 1 = `733ade8` at **19:00:41** (`runs/push1_git.log`), on top of `3336c3e` (AF, 18:53) and `733ade8` (runbook A+B+C, 89, publisher, E2 files, 18:54); clean-clone check `runs/push1c_check.log` (18:55-18:57): unit 87 passed / 2 skipped, SELFTEST PASS with the prefix cache, 84 **5 of 5** exit 0, OWN WRITERS 0, no slow-hand warning. `real-robot-code-E2.md` has only "Track E2 - round 1" (no fix round, no round 2) |
| Battery / AC | Not checked this round | out of focus |

## Verdict 1 (implementation fidelity of the revision to the R5 findings)
- **R5-1: fixed correctly.** 3.5 replaces the frame step by `select_start_poses` (pure, same `step_of` as `densify`, per waist limit on
  the clipped 17-vectors, interpolated poses for long raw pairs, fractional `frame`), adds `max_start_step_m` and `start_poses` to the
  summary, asserts them in C's verify (line 1151) and 6.3 (line 1409), counts `frames_inside` over the dense poses, removes
  `--frame-step` everywhere except as a "there is no" statement and in history, replaces the fallback by fewer setbacks / one waist
  limit per run + `87 --merge` with a pure `merge_reports` that refuses mixed settings, corrects R15 (line 1580) and lenses 9/10, and
  puts the C test with the mutation "fixed frame step 10" into C's task text as a build note (C started 18:57 on rev 7.1). Numbers
  in the test re-derived (table above). Only the start-pose count estimate is low (R6-3).
- **R5-2: fixed correctly in the plan text**, all six parts: (1) push 1 = the three commits + a fix set with named tracks AF/RB/E2;
  (2) own gates named in section 0; (3) one runbook writer at a time, A2 after the fix-set commit; (4) 6.2 runs 84 five times from
  the clean clone with per-run exit, OWN WRITERS and slow-hand checks (the lens-4 per-file loop is right); (5) B2/C/integration
  record the AF `real_io` hash; (6) section 7 Part B 3 uses the hip-axis formula. The main session has since executed (4): the
  `push1c` logs match 6.2's pass criteria. What did not follow the plan is the E2 part (R6-1).
- **R5-3: fixed correctly.** 3.1 contract, A2 item 6 + test with mutation, rule 16 (e) "no final line **or** `FAULT EXIT
  lowstate_stale`" / "any **other** exit 4", E2 fix round, section 7. The pushed runbook does not have the wording yet (grep finds no
  `lowstate_stale`); that is inert while 86 is not pushed and is scheduled through E2's fix round.
- **R5-4: fixed correctly.** Section 0 and section 7 page 1 sanction exactly `kamera --suchen` with re-derived evidence; walk
  proposals 1, 2, 4, 5 are A2 items 7-9; the refutation of proposal 3 holds (72 is committed German code that section 0 keeps
  untouched, `27708e9`); R19 (a)/(b) carry the open questions.
- **R5-5: fixed in mechanism**, stale in one list (R6-2): the frontmatter/B2 add the two 18:34 test files, the unlock status check
  covers `tests`, 6.4 runs B1/B2's unit tests from the clean clone.

## Verdict 2 (plan correctness of the changes and cross-section consistency)
- **New facts are consistent across sections.** Motorized head: section 0, assumption, 2.4, rule 4.1 (3), RB item 4, E2 1b, section 7
  (Part B 3 head bullet, page-1 rule, M0 repeat, M4/M5 re-check), Q3, section 11 risk - all say "owner sets pan 0 / 50 deg down with
  the head's own control, operators only verify (phone 49-51 deg + 84 overlay), nothing moves it after Part B 3, 84 camera re-run
  before M4, before M5 and after a mode change or reboot". On-site SSH login: section 0, assumption, RB item 5, section 7, Q5 agree
  (never written into a file, 84 answer or sheet; `ConnectTimeout=5`). Tape marks: assumption, RB item 4, section 7 Part B 3/M4,
  R19 (a) agree, and the pushed runbook matches (B3 278-279/333-336: correct a wrong 24.5 cm by re-taping during B3, but never move
  the table off its marks to match the camera picture; Part C 480-495). Hip formula, landmark and 0.60-0.72 m range agree between
  RB item 2, section 7 and the pushed runbook - except the cap (R6-4).
- **Safety chain unchanged.** Nothing in rev 7.1 touches the person-safety design; R5-1 and R5-3 only make the lookup and the
  "may be held" rule stricter.

### MAJOR
None.

### MINOR

**R6-1. The plan is overtaken by push 1, and the E2 part of the "Build tracks" section no longer matches the repo.** Push 1 landed at
19:00 as `733ade8` (Finnegan's word, STATUS 19:00) and already contains E2's round-1 files (Part C, 89 motion line,
`tests/test_unit_runbook.py`, the phases image) plus three runbook edits the main session made at 18:53 after the RV diff check (D1
table cap 0.81 m + skip, D2 bullet order, D4 phone held; `real-robot-runbook-walk-AB.md:587-591` "unreviewed text edits - final gate
re-checks them"). Rev 7.1 (18:56) still says: push 1 "expected ~19:30-20:15" (Situation, 6.1); E2's files join push 1 only after
"Track E2 - round 2" is GO (section 0, 5 intro, E2 `push=` and 1b, 6.2 header and "Push 1 files"); E2's fallback "restores Part C to
E1's stub ... E2's files go to push 2" (line 1283-1286); A2 waits for "the fix-set commit" (it exists: `3336c3e` + `733ade8`); RB
owns "Part A, Part B, end of day only" and E2 1b re-lists RV-1/RV-2/RV-4 although RB already wrote them into Part C (pushed runbook
401-402, 480-495, 507; diff check D5 "E2's fix round must start from this working tree ... and not overwrite them").
- Trigger: the next workflow stages read the plan - E2's fix round (adds duplicate RV lines; on two failed rounds its fallback would
  revert pushed operator text to a stub in push 2), the final gate (its push-2 inventory, 6.4, does not name E2's fix round or the
  three unreviewed 18:53 edits, which operators now hold for Parts A+B tonight).
- Impact: process only. Pushed Part C is inert (86 not pushed, 89 prints `CHECK motion: INFO`); D1/D2/D4 follow the diff reviewer's
  own proposed text and all point the safe way (D1 is stricter than the plan). No person- or hardware-safety effect, so minor.
- Fix (status sync, plan text only, no new round needed): record push 1 = `733ade8` with the `runs/push1c_check.log` evidence (6.2
  done, 5/5); E2 1b becomes a push-2 item that starts from the pushed runbook, keeps RB's Part C lines (RV-1/2/4 = verify only), adds
  R5-3's box wording and E2 r1 minors 1-6, and its round-2 diff check also covers the 18:53 main-session edits (D1/D2/D4); drop the
  "restore E1's stub" fallback (if E2's fix round fails, push 2 keeps the pushed Part C and the final gate rules); A2 may start now;
  6.4 lists E2's fix round and the D-edits; mark the C2 push-1 fallback rows as moot.

**R6-2. B2's 18:56 split is missing from the push lists that enumerate files.** `g1sim/real_run_exits.py`,
`tests/test_unit_real_run_exits.py`, `tests/test_unit_real_run_options.py` exist (untracked) and are named in STATUS 18:56 and
`real-robot-code-B2.md:361-363`; rev 7.1 has them in neither the frontmatter, B2 `<files>`, B2's verify (4 of 6 test files), nor 6.4's
explicit clean-clone test list (line 1426-1427). The push-set rule and the unlock's `git status -- ... tests` still stop an unlock that
forgets them, so nothing unsafe reaches Monday; but 6.4's own clean-clone test run would pass with the two test files missing.
`real_run_exits.py` imports only `real_safety` (no publisher), so its absence from the ban lists (A2 item 3, E2 item 2) is cosmetic.
Fix: add the three files to the frontmatter and B2 `<files>`; B2's verify and 6.4 run all six B2 test files (or a glob
`tests/test_unit_real_*.py tests/test_unit_motion_gate.py`); optionally add `g1sim.real_run_exits` to both ban lists.

**R6-3. 87's start-pose count is under-estimated per waist limit.** FK with the real collision geoms gives 59 / 77 / 101 / 115 start
poses (L = 0 / 0.15 / 0.35 / 0.6), not ~86 each; aggregate +21 % over 4 x 73 before reuse, and the cross-L reuse shrinks because each L
selects its own poses. Runtime stays a reported number with the merge fallback, so this only affects the clock (maybe +5-10 min).
Fix: say "~60-115 per waist limit (FK, round 6)" in 3.5 and lens 10 (a); expect 87 nearer the upper end of 10-25 min.

**R6-4. The table cap in the plan (0.84 m) disagrees with the pushed runbook (0.81 m + skip M4-M6).** RB item 2 (line 913) and section
7 Part B 3 (line 1472) say "never above 0.84 m"; the runbook (B3 270-272) says "still above 0.72 m ... never set the table above
0.81 m, and skip M4-M6 at the table". The runbook value is the one the formula gives at the plan's own 0.72 m bound (0.74 + 0.072 =
0.812), and the diff reviewer's D1 shows a straight-legged G1 hip axis cannot exceed ~0.685 m, so 0.84 is never right. Fix: plan
text to 0.81 m + skip (matches the runbook; nothing to change in pushed text). Also 2.4/section 7 "phone flat on the D455 top face"
-> "held lightly flat against" (D4, as pushed).

### Environment / not plan content
- `runs/push1x_84_2.log` (18:59, the check of the alternative commit `1761e29` without B1) holds only "STEP net: NOT RUN" (41 bytes) -
  no "Fatal Python error" line, so most likely the check was stopped when the main session chose `733ade8`; I could not confirm why.
  The pushed commit's own check (`push1c`) is 5/5 clean and AF's reviewer saw 10/10.

## Verdict 3: tracks and timeline
- Tracks: C started 18:57 on rev 7.1 with the R5-1 build note - correct. B2 round 3 is under review against the AF `real_io`. The only
  inconsistency is R6-1 (E2 / A2 / push-1 wording) and the file list R6-2. Timeline: push 1 landed ~30-75 min earlier than the plan
  expected, so every downstream row gains slack; C3/C4/C5 and the cutoffs are unaffected.

## Checks I could not run
- 87 itself (not built yet): the real `select_start_poses`/`step_of` may differ slightly from my FK re-implementation of the 3.5 text.
- Whether `push1x_84_2.log` was a stopped run or a crash (no trace in the file).
- The 18:53 main-session runbook edits were read (grep), not reviewed in depth; that is E2 round 2's or the final gate's job (R6-1).

## Overall (revision 7.1, round 6): **GO** (plan for the remaining build), with four minor text fixes
- R5-1..R5-5 are fixed correctly; the load-bearing facts (R5-1 test geometry, movement-based selection, hip-axis formula, 24.5 cm
  layout, head and SSH facts, `kamera --suchen`, the strings 6.2 and rule 16 (e) rely on) re-derive from code.
- No major. R6-1 is a status sync after push 1 (most important: E2's fix round goes to push 2, starts from the pushed runbook, keeps
  RB's Part C lines, its diff check covers the 18:53 edits, and no "restore stub" fallback); R6-2 file lists; R6-3 runtime estimate;
  R6-4 cap 0.81 m. All are plan text the main session can fold in before launching E2's fix round and A2; no further plan round
  needed.
