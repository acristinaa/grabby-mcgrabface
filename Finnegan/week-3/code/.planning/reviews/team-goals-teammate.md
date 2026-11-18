# Teammate-lens review: docs/TEAM_GOALS_PROPOSAL.md

Reviewer role: Cristina or Saad. We built the room safety check (patrol, fire extinguisher position, exit door open or closed, report back) and we work in Isaac Lab on a rented GPU. We have not seen Finnegan's repo. Date: 2026-10-06. The document was not edited.

## Short answers

1. **Do I know what is proposed and what I must decide?** Mostly. The one-sentence goal and the 7 steps are clear. What I must decide on Wednesday is not clear: §8 lists 9 questions, and half of them are homework, not decisions.
2. **Unclear terms:** DoF, π₀.₅, Bedrock, odometry, LiDAR-based localization, Jetson Orin NX / AGX Orin, "3–8B", "MASTERPLAN", "BrainCo hands", "programmed grasp", EHS / EWM, "sim" (which one?).
3. **One-sided?** Yes, a little. 4 of the 7 steps are grasp work (Finnegan's area). Our work gets one vague line ("Team: ..."), while Finnegan's gets numbers. Our Isaac Lab setup and our real-robot experience do not appear anywhere in the plan.
4. **Missing before I agree:** which simulator the full sequence runs in, and how our safety check gets into it. Also: the pitch deadline, and why a safety round fetches a ball.
5. **Length:** about 1,450 words. It can lose about 35 % (mostly §3 compute and cost rows, "Which object?", and duplicated risks) and keep its meaning.

## Findings (most important first)

### 1. The plan never says which simulator, so our work has no place in it
"Sim (MuJoCo) on our laptops. Training and Isaac Lab on a rented NVIDIA GPU." Our safety check lives in Isaac Lab (or on the real robot). Finnegan's walking and grasping live in MuJoCo. Step 2 says "this is the safety check you already built", but the 8/10 sim target needs all 7 steps in **one** world. Nobody says which world, or who moves which part. That is the biggest open question, and it decides how much of our work gets reused.
- **Add to §3 table, new row:**
  `| **One sim for the full sequence** | Walking + grasping run in MuJoCo (Finnegan). The safety check runs in Isaac Lab / on the real robot (Cristina, Saad). | We must pick one sim for the 8/10 demo, or connect both through the same task and report format. To decide on Wednesday. |`
- **Add to §8 as question 2:** "**Simulator:** do we run the full sequence in MuJoCo or in Isaac Lab? How does our existing safety check get into it?"

### 2. The story has two halves that do not connect (safety round + returning a ball)
A misplaced ball is not a safety problem. A reader (and later a pitch jury) will ask why a safety robot tidies up toys. The fix is easy and makes our inspection and Finnegan's grasp **one** story: the object is a **trip hazard in the walkway or in front of the emergency exit**.
- §2 one sentence. Old: "The robot checks the room, finds a misplaced object, brings it back to the right place" → New: "The robot does a safety round, finds an object that blocks the way to the emergency exit, moves it to a safe place"
- Step 1. Old: *"Safety round: check the room and bring the ball back to Table B."* → New: *"Safety round: check the fire extinguisher and the exit, and clear the escape route."*
- Step 3. Old: "the robot looks around the room and finds the ball." → New: "the robot finds an object lying in the escape route."
- Step 7. Old: *"Ball was misplaced → returned to Table B."* → New: *"Object blocked the exit route → moved to Table B."*
- Business case. Old: "the robot fixes small things itself." → New: "the robot also fixes the most common small problem itself: something left in the escape route."

### 3. Our work is undervalued in "What exists already" and in the timeline
Our row says "**Team:** the robot walks through the room, checks ... and reports back." It does not say real robot or sim, and it gives no numbers. Finnegan's row has "10/10" and "95–100 %". It also reads as if Finnegan is not part of "Team". In week 1–2, "first connection to the real robot + walking" sounds like nobody has done it yet. If our patrol already runs on the real robot, our navigation is the best starting point for the real attempt in week 6–7.
- Old: "**Team:** the robot walks through the room, checks the fire extinguisher and the exit door, and reports back." → New: "**Cristina + Saad ([real robot / Isaac Lab], please fill in):** the robot patrols the room, checks the fire extinguisher and the exit door, and reports back. [How often does it work?]"
- Old: "**Finnegan (in sim):**" → keep, but put it as the second line, after ours.
- Old "What it means" cell: "Every part of the sequence builds on something we have." → New: "Every part builds on something we have: inspection and real-robot walking from Cristina + Saad, walking-to-a-point and grasping in sim from Finnegan. The big missing piece: walking + grasping together."
- Week 1–2. Old: "first connection to the real robot + walking" → New: "reuse the real-robot patrol (Cristina, Saad) as the base for the real attempt" (only if that is true; ask first).

### 4. It is not clear what we must decide on Wednesday
Nine questions is too many for one meeting. Some are decisions (goal, object, success, scope). Others are things someone has to find out (backpack computer, DoF, AWS, SAP system, deadline).
- Replace the intro of §8 with: "**To decide on Wednesday:** 1 goal · 2 simulator · 3 object · 4 success definition · 5 π₀.₅ optional. **To find out by [date], one owner each:** SAP system · second table · backpack computer + 23/29 DoF · AWS budget · pitch date."
- Also put one line under the title: "**What I need from you on Wednesday:** a yes, a change, or a different goal, plus answers to the 5 decisions in §8."

### 5. Fact check: two of Finnegan's numbers sound stronger than the repo shows
Source: PROGRESS.md, entries from 2026-09-15 and 2026-10-02.
- Walking 10/10 (L1 go-to-point, L2 room change with 0 wall contacts) is real. But the robot gets its position **from the simulator**. Finding its own position (localization) is not built (PROGRESS M8). We would expect our real-robot navigation to fill exactly that gap.
- The grasp is **95 % (19/20)** when the bottle is within arm's reach, with all random variations. 100 % only happens per single variation group. When the robot starts ±10 cm off, 12 of 50 bottles are out of reach, and stepping closer ("align") is not built. The bottle also sits about 40° tilted in the hand, which matters for putting it down.
- Old: "walks to a point and between rooms (10/10); grasps a bottle while standing still (95–100 % within arm's reach)." → New: "walks to a point and between two rooms (10/10, but the sim tells the robot where it is); grasps and lifts a bottle while standing still (95 % when the bottle is within arm's reach; stepping closer and putting down are not built yet)."

### 6. Jargon that non-robotics or non-native readers will stop at
Explain in brackets the first time, or remove:
- "23-DoF version" → "23-joint version (DoF = degrees of freedom, the number of joints that can move)"
- "A learned AI grasp (π₀.₅)" → "A learned AI grasp (π₀.₅, an open AI model that learns grasping from recorded examples)"
- "AWS Bedrock" → "AWS Bedrock (Amazon's service where you pay per request to use big AI models)"
- "simple odometry" → "simple step counting (odometry: the robot estimates its position from its own steps)"
- "LiDAR-based localization" → "laser-scanner positioning (LiDAR)"
- "Small ones (3–8B)" → "Small models (3–8 billion parameters)"
- "our MASTERPLAN names a different computer" → "Finnegan's planning document names a different computer" (we have never seen MASTERPLAN)
- "G1 with BrainCo hands" → "G1 with our BrainCo hands (the five-finger hands on our robot)"
- "SAP EWM" → "SAP EWM (warehouse software)"
- "a programmed grasp" → "a grasp written by hand in code (not learned)"

### 7. §3 "Compute" and "Cloud cost" rows are too detailed for a goal decision
"Big open models need ~200–800 GB", "~$3–6 per 1,000 ... calls", "~$700–1,700 per month", "~$2.30/h" have no sources. They will start a debate about prices that does not change the goal. Only one sentence of these rows matters for the decision.
- Replace both rows with one row: `| **Compute + cloud** | Big AI models do not fit on the robot's computer. | "Thinking" (planning, finding objects) runs in the cloud, pay per use. Walking and balance always stay on the robot. A GPU is rented only for training. Prices: in the appendix / see link. |`

### 8. The timeline has no end date
"Rough timeline" says 7 weeks, but §8 asks "when is the pitch?". If the pitch is in 5 weeks, the plan changes a lot. Put the unknown up front.
- Old: "## 6. Rough timeline (if all three of us work on it)" → New: "## 6. Rough timeline (if all three of us work on it; pitch date still unknown, see §8)"
- Old: "My estimate: **~6–12 weeks of work for one person, ~4–7 weeks for the three of us.**" → New: "My estimate: **~4–7 weeks for the three of us.** If the pitch is earlier, we cut the real-robot attempt first, not the sim demo." (Or the other way round. Either way the doc should say what gets cut first.)

### 9. The SAP claims need links, and the SAP part needs an owner
We have not seen the sources for "SAP EHS has an API", "ANYbotics ANYmal, SAP News 03/2026" or "Unitree G1 ... Arçelik-LG, SAP News 11/2025". If we repeat them in the pitch, we need the links. Also, the risk says "if not clear within ~2 weeks" but nobody owns it.
- Old: "(ANYbotics ANYmal, SAP News 03/2026)" → New: "(ANYbotics ANYmal, [SAP News 03/2026](link))". Do the same for the Arçelik-LG item and the EHS API (link to the API page on api.sap.com).
- Old risk: "If it is not clear within ~2 weeks, we build a simple stand-in" → New: "If [name] has no answer by [date, ~21 Oct], we build a simple stand-in"

### 10. §1 can be shorter, and one sentence reads strangely
"We are all developers and could all do the same work. So we need one shared goal first." This sounds as if our skills are the same as yours and you could do our part. Since the role split was removed, this sentence is not needed.
- Old §1 (whole section) → New: "Right now the three of us work on different things. We have not agreed yet on **what** the robot should do in the end, **what** we try on the way, and **how** we measure success. This doc is my suggestion for that."

### 11. Success targets for "Inspect" and "Find" need a scene that does not exist yet
"Correct report for fire extinguisher and exit, 10/10 test scenes." Who builds the 10 test scenes, and in which sim? Can MuJoCo show a readable exit sign to the camera at all? Today our check works where we built it. Also "'Almost works' is already a success" is vague next to the clear rule in the next paragraph.
- Inspect row. Old: "10 / 10 test scenes" → New: "10 / 10 test scenes (extinguisher there / missing, door open / closed), built in the sim we pick in §8"
- Old: "It does not have to work perfectly. "Almost works" is already a success." → delete. The "On the real robot:" paragraph below already defines success.

### 12. Cut list to get under ~1,000 words
- "Which object?" (3 lines) → one line: "**Object:** a soft ball is harder to grasp than a bottle. We test it in sim for one day, else we use a bottle, can or bean bag." (With finding 2, the object becomes "anything in the escape route", so this choice matters less.)
- Risks: merge "Walking + grasping together is the hardest step" and "Putting things down and carrying while walking are not built yet" into one bullet: "**Not built yet:** walking + grasping together (balance shifts when the arm moves), carrying while walking, putting down."
- Risks: delete "We need a second table or a marked spot / bowl." It is already §8 question 6.
- §4 Out: "Carrying the real fire extinguisher (too heavy, see risks)" duplicates the first risk. Keep it in one place only.
- §5: delete "(If grasp and place each reach 9/10 ...)". Keep only "9/10 each, so the full run lands at about 8/10" in the table header or a short note.
