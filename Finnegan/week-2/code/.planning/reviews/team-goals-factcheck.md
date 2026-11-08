# Fact-check: docs/TEAM_GOALS_PROPOSAL.md (2026-10-06)

Reviewer: fresh-context fact-checker (opus). Document not edited.
Sources: repo files (read 2026-10-06), web where noted. Claims already verified on 2026-10-06 were checked for wording only.

## Most important fixes (in priority order)

1. **L60 "Every part of the sequence builds on something we have" is WRONG.** SAP task in, SAP report out, carrying and putting down have no code in the repo. Say which steps are new.
2. **L60 the walking and grasping results leave out the conditions.** Walking 10/10 used the position from the sim. The grasp had the hip fixed in place, so the robot could not fall. Both conditions matter for the real-robot attempt.
3. **Red-mask conflict (new risk).** The only object finder in the repo looks for red pixels (`g1sim/perception.py:15`). A fire extinguisher is red, so a red ball or a red bottle in the same room would be confused with it.
4. **L71 + timeline: the real robot needs its own position source.** L1/L2 in sim take the position from the sim. Putting LiDAR "out" is fine only if odometry plus the camera is planned "in" for the real attempt.
5. **L92–96 targets are stricter than the repo's own acceptance criteria.** Walk + grasp is ≥ 8/10 (LEARNING_PATH L3), placing is ≥ 8/10 (L3a), and the full run is ≥ 7/10 (L4). The doc asks for 9/10 and 8/10.
6. **L117 extinguisher weights 6.5–10.5 kg do not match the verified 6.8–10.6 kg.**
7. **L104 the effort estimate** cannot be derived from the repo. LEARNING_PATH §5 gives only small/medium/large. Label it as a gut feeling.

## Claim-by-claim

### Header / Section 1

- **Quote:** "for our meeting on Wed 7 Oct 2026, 11:00"
  **Verdict:** CORRECT
  **Evidence:** 2026-10-07 is a Wednesday (computed).
  **Proposed wording:** unchanged.

- **Quote:** "Right now the three of us work on different things at the same time."
  **Verdict:** UNVERIFIABLE (team fact, not in the repo). It is consistent with memory `g1-team.md` (22.09: MuJoCo + Isaac Lab, Finnegan = tests/grasping).
  **Proposed wording:** unchanged.

### Section 2: the sequence

- **Quote:** "the robot checks the fire extinguisher ... and the emergency exit ... → this is the safety check you already built."
  **Verdict:** UNVERIFIABLE. There is no trace of an extinguisher or exit check anywhere in the repo (grep "extinguisher|Feuerl|emergency|Notausgang" over all *.md, with the German words for fire extinguisher and emergency exit included: 0 hits outside this doc). The doc also does not say whether the team's check runs in sim or on the real robot. It does not say whether it already covers "sign visible" and "door state", or only "extinguisher present".
  **Proposed wording:** "→ this builds on the safety check you already built (in sim / on the real robot – please correct me). If it does not cover the sign or the door state yet, we add that."

- **Quote:** "door in its required state, e.g. closed?"
  **Verdict:** CORRECT as an example. In practice, emergency exits must above all be free and openable. "Closed" applies to fire doors.
  **Proposed wording:** "(free and not blocked? door in its required state, e.g. a fire door closed?)"

- **Quote:** "Find: the robot looks around the room and finds the ball."
  **Verdict:** IMPRECISE (hidden dependency). The only detector in the repo is a red-pixel mask (`g1sim/perception.py:15` `red_mask`, LEARNING_PATH §4.1 "red mask only"). Fire extinguishers are red. A red target object would collide with the red-mask detector. "Looks around" (turning the head or body to search) is also not built yet; the head pan servo exists in sim (`g1sim/payload.py`, PROGRESS.md:480).
  **Proposed wording:** keep the step, and add to "Which object?": "Not red: our current object finder looks for red pixels, and the fire extinguisher is red."

### Business case

- **Quote:** "Today, people check by hand whether fire extinguishers are in place and emergency exits are free."
  **Verdict:** CORRECT in substance (routine visual checks are manual). The robot would replace the visual round, not the legally required expert maintenance of extinguishers.
  **Proposed wording:** optional addition: "(the robot does the visual round, not the official maintenance)".

- **Quote:** "SAP EHS ... has an API to report incidents and safety observations, so a real SAP integration is possible."
  **Verdict:** CORRECT (pre-verified: API_EHS_REPORT_INCIDENT_SRV covers incident / near miss / safety observation). The doc leaves out the condition that it needs an S/4HANA system with EHS. That condition is exactly meeting question 2.
  **Proposed wording:** "SAP EHS (Environment, Health and Safety) has an API to report incidents and safety observations. It needs an SAP S/4HANA system with EHS, so a real integration is possible if we get access to one."

- **Quote:** "SAP already works with inspection robots (ANYbotics ANYmal, SAP News 03/2026)."
  **Verdict:** CORRECT (pre-verified, SAP News 2026-03-30).
  **Proposed wording:** unchanged.

- **Quote:** "It also has an ongoing proof of concept with a Unitree G1 ... (at Arçelik-LG, SAP News 11/2025)."
  **Verdict:** IMPRECISE. The source is from 2025-11-05. Whether the proof of concept is still "ongoing" in Oct 2026 is not supported by that source.
  **Proposed wording:** "In Nov 2025, SAP announced a proof of concept with a Unitree G1 doing warehouse tasks guided by SAP EWM (at Arçelik-LG, SAP News 11/2025)."

### Which object?

- **Quote:** "A ball is harder to grasp than a bottle (it rolls in the hand and off the table), while our bottle grasp already works in sim."
  **Verdict:** IMPRECISE. The bottle grasp works, but the bottle hangs ~40° tilted because the fingers are too short for its 6.6 cm diameter (PROGRESS.md:42, :462–463; LEARNING_PATH L3a). A 6–7 cm ball is the same width, so the same finger-length limit is likely. That makes the ball test a fair one-day check, but "works" should carry the tilt caveat.
  **Proposed wording:** "...while our bottle grasp already works in sim (the bottle still hangs a bit tilted; we are fixing that now)."

- **Quote:** "test a soft ball (~6–7 cm) in sim for one day"
  **Verdict:** IMPRECISE. The size is the same as the bottle diameter that is already at the limit of the fingers (PROGRESS.md:460). The color matters too (see red mask above).
  **Proposed wording:** "test a soft, non-red ball (try ~5 and ~7 cm) in sim for one day."

### Section 3: constraints table

- **Quote:** "The robot has a built-in Jetson Orin NX (16 GB)."
  **Verdict:** CORRECT (pre-verified; docs/MASTERPLAN.md:103, PLAN.md:338).
  **Proposed wording:** unchanged.

- **Quote:** "The backpack is probably a Jetson AGX Orin (64 GB) – still to check, because our MASTERPLAN names a different computer."
  **Verdict:** CORRECT as hedged. The repo models it as Unitree's "AGX Backpack", 1.9 kg (PROGRESS.md:479–480, `g1sim/payload.py:1`). MASTERPLAN names a "Thunderobot ... Intel Core i7 + NVIDIA GeForce RTX" (docs/MASTERPLAN.md:104). Unitree's README does not name the module or its memory (github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description). The G1 AGX Orin dock sold by resellers is rated 275 TOPS (robotopian.com/products/unitree-g1-jetson-agx-orin-dock), and 275 TOPS is the AGX Orin **64 GB** module (the 32 GB module has 200 TOPS). So "64 GB" is a reasonable inference, not a confirmed fact.
  **Proposed wording:** unchanged, or "probably a Jetson AGX Orin (likely 64 GB)".

- **Quote:** "Big open models need ~200–800 GB."
  **Verdict:** CORRECT (pre-verified: DeepSeek-R1 ~689 GB; Llama 405B 203–810 GB). The wording leaves out "of memory".
  **Proposed wording:** "Big open models need ~200–800 GB of memory."

- **Quote:** "Big models do **not** run on the robot. Small ones (3–8B) could."
  **Verdict:** IMPRECISE (true only with conditions). An 8B model runs on an Orin NX 16 GB only compressed to 4-bit. NVIDIA reports Llama 3.1 8B INT4 at ~20 tokens/s on Orin NX 16 GB (JetPack 6.2 benchmarks; developer.nvidia.com/embedded/jetson-benchmarks, jetson-ai-lab.com/models). It shares the 16 GB with everything else running on that computer. "Could" without these conditions oversells it.
  **Proposed wording:** "Small ones (about 3–8B, compressed) could run, but slowly (about 20 words per second for an 8B model) and they share the 16 GB with our other programs."

- **Quote:** "Sim (MuJoCo) on our laptops. Training and Isaac Lab on a rented NVIDIA GPU. Big models via AWS."
  **Verdict:** CORRECT as a plan. GPU access is still an open blocker in the repo (PROGRESS.md "Blocker 3", :671–683; LEARNING_PATH §1 "GPU access (Blocker 3)").
  **Proposed wording:** unchanged. Optionally add "(not set up yet)" after "rented NVIDIA GPU".

- **Quote:** "Cloud adds some delay. Fine for 'thinking' ... Walking and balance always stay on the robot."
  **Verdict:** CORRECT (LEARNING_PATH §3: the walking controller owns legs and waist; the language model only picks skills).
  **Proposed wording:** unchanged.

- **Quote:** "AWS Bedrock ... (~$3–6 per 1,000 'look at this photo and decide' calls)."
  **Verdict:** CORRECT (pre-verified for the Claude Haiku/Sonnet class).
  **Proposed wording:** unchanged. Optionally add "(with a small or mid-size model)".

- **Quote:** "Our own GPU server 24/7: ~$700–1,700 per month."
  **Verdict:** CORRECT in numbers ($1.006 × 730 h = $734; $2.327 × 730 h = $1,699; EC2 g6/g6e Frankfurt, pre-verified). The wording is IMPRECISE: "our own" reads like buying hardware.
  **Proposed wording:** "A rented cloud GPU server running 24/7 (for our own model): ~$700–1,700 per month."

- **Quote:** "Rented GPU for training: ~$2.30/h."
  **Verdict:** CORRECT (g6e Frankfurt $2.327/h, pre-verified). This is the class that fits LoRA training (> 22.5 GB GPU memory, LEARNING_PATH §2).
  **Proposed wording:** unchanged.

- **Quote:** "The room: One room: a table, a ball, a fire extinguisher, a door with an emergency-exit sign."
  **Verdict:** UNVERIFIABLE (real room, not in the repo). Note that the current sim world is two rooms with a door and one table (`g1sim/rooms.py:1`). The timeline already plans "our room in sim".
  **Proposed wording:** unchanged.

- **Quote:** "Team: the robot walks through the room, checks the fire extinguisher and the exit door, and reports back."
  **Verdict:** UNVERIFIABLE (not in the repo; sim or real not stated).
  **Proposed wording:** "Team: the robot walks through the room (in sim / on the real robot), checks ... and reports back."

- **Quote:** "Finnegan (in sim): G1 with BrainCo hands walks to a point and between rooms (10/10)"
  **Verdict:** IMPRECISE (a condition is missing). It is 10/10 in both checks (PROGRESS.md:38, LEARNING_PATH L1/L2), but the robot's position came from the sim ("For now, the sim provides the position", LEARNING_PATH L1; PROGRESS.md:578). A real robot does not get that position for free.
  **Proposed wording:** "walks to a point and between rooms (10/10; the sim tells it where it is)"

- **Quote:** "grasps a bottle while standing still (95–100 % within arm's reach)"
  **Verdict:** IMPRECISE. The figures are right (100 % with ±5 cm offsets, 95 % with all random variations; PROGRESS.md:122, :603). "Standing still" is misleading: the hip is fixed in place in the grasp scene, and the robot cannot fall (LEARNING_PATH §1: "the pelvis is bolted down ... the free joint is deleted"). That is exactly why "walking + grasping together" is the big missing piece. Also, 30 of 50 random starts were out of reach (LEARNING_PATH §1).
  **Proposed wording:** "grasps a bottle with its hip fixed in place, so it cannot fall (95–100 % when the bottle is within arm's reach)"

- **Quote:** "Every part of the sequence builds on something we have."
  **Verdict:** WRONG (overstated). Per step:
  - SAP task in: nothing in the repo.
  - Inspect: team work, not checkable.
  - Find: red-mask detector only, no search behavior.
  - Pick up: grasp exists, but only with the hip fixed.
  - Carry: not built (PROGRESS.md:152, Stage 3.4 open).
  - Put down: not built (PROGRESS.md:121 "Open: placing"; LEARNING_PATH L3a part 2).
  - Check again: perception can be reused.
  - Report to SAP: nothing in the repo.
  **Proposed wording:** "Most robot steps build on something we have. New: walking + grasping together (the big one), carrying, putting down, and the SAP connection in and out."

### Section 4: in / out

- **Quote:** "A learned AI grasp (π₀.₅) – needs a GPU and ~2–4 extra weeks"
  **Verdict:** UNVERIFIABLE estimate. The repo rates it "large" and GPU-bound (LEARNING_PATH §5 table, L6 + D4–D8). It is consistent in direction.
  **Proposed wording:** "...needs a GPU and roughly 2–4 extra weeks (my guess)".

- **Quote:** "Carrying the real fire extinguisher (too heavy, see risks)"
  **Verdict:** CORRECT.
  **Proposed wording:** unchanged.

- **Quote:** "LiDAR-based localization (only if the room is too big for simple odometry)"
  **Verdict:** IMPRECISE. The deciding factor is not room size but how accurately the robot must stop at the table: the right hand only reaches 0.30–0.46 m in front of the hip (LEARNING_PATH L3, PROGRESS.md:517–518), a window of about ±8 cm. Odometry drift grows with distance walked and turns. The sim results never used odometry; they used the sim's position. The repo plans the camera-based "align" step (line up from the object's position) for the final approach (LEARNING_PATH L3). Whether the G1 exposes a usable built-in odometry was not checked in this review.
  **Proposed wording:** In the "In" column: "Real robot: find its position with odometry + camera (line up at the table using the object)". In the "Out" column: "LiDAR-based localization (only if odometry + camera drift too much)".

### Section 5: success measures

- **Quote:** "target: 8 out of 10 runs" / "Grasp ≥ 9/10" / "Carry + place ≥ 9/10"
  **Verdict:** IMPRECISE (stricter than the repo's own acceptance criteria). LEARNING_PATH sets walk + grasp in one world at ≥ 8/10 (L3 "Acceptance check"), placing at ≥ 8/10 upright (L3a "Acceptance check") and the full fetch run at ≥ 7/10 (L4 "Acceptance check"). The only measured grasp rate (95 %) is with the hip fixed. Higher targets are a valid choice, but the team should know they exceed the plan already written down.
  **Proposed wording:** keep the numbers if wanted and add: "(stricter than our current step plan, which uses 8/10 per step and 7/10 for the full run)". Alternatively align to 8/10 per step and 7/10 overall.

- **Quote:** "(If grasp and place each reach 9/10, the full sequence lands at about 8/10 ...)"
  **Verdict:** IMPRECISE. 0.9 × 0.9 = 0.81 holds only if every other step works every time. 10/10 in a 10-run test does not prove 100 %.
  **Proposed wording:** "(If grasp and place each reach 9/10 and all other steps always work, the full sequence lands at about 8/10.)"

- **Quote:** "Walk | ... 10/10"
  **Verdict:** CORRECT as a target (matches L1/L2 10/10 in sim with the sim's position).
  **Proposed wording:** unchanged.

### Section 6: timeline

- **Quote:** "~6–12 weeks of work for one person, ~4–7 weeks for the three of us."
  **Verdict:** UNVERIFIABLE. LEARNING_PATH §5 gives only sizes (small/medium/large), no durations, so the numbers cannot be derived from it. Recorded pace: 7 work days (09-07, 09-09, 09-14, 09-15, 09-16, 09-22, 10-02 in PROGRESS.md log) delivered Stage 0–1, L1, L2 and D1. L1 + L2 + D1 were all done on 2026-10-02. Remaining sim work per LEARNING_PATH is L3a part 2 (medium), L3 (large, "riskiest step"), and L4 (medium), plus new items not in LEARNING_PATH: the room scene, ball, inspection, SAP. At that pace the sim part fits the estimate. The real-robot part (Stage 5) has no size in the repo and is the real unknown. Three people do not divide this by three, because L3 → L4 is a chain.
  **Proposed wording:** "My rough guess: ~6–12 weeks of work for one person, ~4–7 weeks for the three of us (not 3× faster, because walking + grasping must come before the full task flow)."

- **Quote:** "Weeks 1–2: ... grasp + put down the object at a table ... first connection to the real robot + walking"
  **Verdict:** IMPRECISE (missing preconditions). The repo makes a sim-to-sim safety test mandatory before real hardware (CLAUDE.md:52 "Sim2Sim is the mandatory safety test"; PLAN.md:259–266). Day one on hardware is read-only (PLAN.md:268–270). The BrainCo hand driver is Linux-only (PROGRESS.md:674–676). Putting down is not built yet (L3a part 2).
  **Proposed wording:** "first connection to the real robot (read-only, safety check first) + walking with Unitree's own controller"

- **Quote:** "Weeks 3–4: Walk + grasp in one world · carry + put down at the second table · the full task flow · SAP task in / report out"
  **Verdict:** IMPRECISE (dense). This packs L3 (large, riskiest), Stage 3.4 carrying, and L4 (medium) into two weeks. LEARNING_PATH L3 names Plan B and Plan C (whole-body controller GEAR-SONIC, untested on the Mac) in case balance fails. The current next-step list in PROGRESS.md:41–52 puts dataset work D2, D4–D7 before L3. Agreeing to this goal implies moving D4–D7 behind L3/L4.
  **Proposed wording:** keep it and add a buffer note: "Weeks 3–4 are the riskiest. If walking + grasping takes longer, the sim demo moves to week 6 and the real attempt stays in week 7."

### Section 7: risks

- **Quote:** "A standard one weighs 6.5–10.5 kg; the G1 EDU arm carries at most ~3 kg."
  **Verdict:** IMPRECISE. The verified full weights are ~6.8–10.6 kg. The ~3 kg arm limit is CORRECT (unitree.com/g1).
  **Proposed wording:** "A standard one weighs about 7–11 kg; the G1 EDU arm carries at most ~3 kg."

- **Quote:** "When the arm moves, the robot's balance shifts. Not built yet."
  **Verdict:** CORRECT (LEARNING_PATH L3 "Why it is hard").
  **Proposed wording:** unchanged.

- **Quote:** "Putting things down and carrying while walking are not built yet. Only grasp + lift is."
  **Verdict:** CORRECT (PROGRESS.md:121, :152).
  **Proposed wording:** "...Only grasp + lift is (with the hip fixed)."

- **Quote:** "A grasp that works in sim often fails on the real robot at first."
  **Verdict:** CORRECT. The repo's own expectation is harsher: "Manipulation Sim2Real: hard; contact-rich tasks often reach only 5-10% success" (CLAUDE.md, section "Managing expectations").
  **Proposed wording:** "A grasp that works in sim often fails on the real robot at first. Contact-heavy tasks often start at only 5–10 % success."

- **Quote:** "If it is not clear within ~2 weeks, we build a simple stand-in (a web page that acts like SAP)"
  **Verdict:** CORRECT as a plan; it is consistent with the timeline weeks 1–2.
  **Proposed wording:** unchanged.

- **Quote:** "If our G1 is the 23-DoF version (the wrist cannot nod up and down), the current grasp does not work and needs to be redone."
  **Verdict:** IMPRECISE. "Does not work" is CORRECT: with the wrist locked it was 0/5 (`runs/stufe1_4_greifen_23dof.json`: passed_trials 0; hand 10.2 mm off, lift 3.4 cm < 5 cm; PROGRESS.md:463). That test was 5 trials on 2026-09-15, before later improvements. The IK still reaches the pose at 3.2 mm (PROGRESS.md:453). The 23-DoF arm also lacks side-to-side wrist turning, and the waist has 1 joint instead of 3 (LEARNING_PATH §10.11). "Redone" overstates it; "adapted and re-tested" fits the evidence.
  **Proposed wording:** "If our G1 is the 23-DoF version (the wrist cannot nod or turn sideways), our current grasp fails in sim (0 of 5) and has to be adapted."

- **Missing risk (new):** "Our object finder only sees red."
  **Verdict:** n/a (gap). `g1sim/perception.py:15`. A fire extinguisher is red.
  **Proposed wording:** add "**Colors.** Our object finder looks for red, and the fire extinguisher is red too. So the target object must not be red, or we use a better object finder."

- **Missing risk (new):** "The real robot does not know where it is."
  **Verdict:** n/a (gap). Sim walking used the sim's position (LEARNING_PATH L1, L7).
  **Proposed wording:** add "**Position on the real robot.** In sim the robot knows exactly where it is. On the real robot it has to estimate that (odometry + camera), which is less exact."

- **Missing risk (new):** "Hands on the real robot need Linux."
  **Verdict:** n/a (gap). The BrainCo hand stack (RS-485 + ROS 2) is Linux-only (PROGRESS.md:674–676).
  **Proposed wording:** add to the hardware check: "We need a Linux computer to control the real BrainCo hands."

### Section 8: questions

- **Quote:** "7. Hardware check: which computer is in the backpack? 23 or 29 DoF?"
  **Verdict:** CORRECT (both open: MASTERPLAN vs Unitree "AGX Backpack"; DoF open per LEARNING_PATH D3 item 2).
  **Proposed wording:** optionally add the 10-second test: "Can the hand nod at the wrist, powered? Yes = 29 DoF (LEARNING_PATH §10.11)."

## Checked and fine (wording only, no change needed)

Jetson Orin NX 16 GB; ~$3–6 per 1,000 Bedrock image calls; $2.30/h rented GPU; ANYbotics SAP News 03/2026; walking 10/10 (numbers); grasp 95–100 % (numbers); G1 EDU arm ~3 kg; "Walking and balance always stay on the robot"; SAP stand-in fallback.

## Sources used beyond the pre-verified set

- NVIDIA Jetson benchmarks (Llama 3.1 8B INT4 on Orin NX 16 GB ≈ 20.5 tok/s, JetPack 6.2): https://developer.nvidia.com/embedded/jetson-benchmarks, https://www.jetson-ai-lab.com/models/
- Unitree g1_description README ("How to Add AGX Backpack?", 1.9 kg, module not named): https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
- G1 Jetson AGX Orin dock, 275 TOPS: https://robotopian.com/products/unitree-g1-jetson-agx-orin-dock
