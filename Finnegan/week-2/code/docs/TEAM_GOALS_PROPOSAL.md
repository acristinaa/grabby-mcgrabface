# Team Goals – Proposal for our G1 Project

*From Finnegan · for our meeting on Wed 7 Oct 2026, 11:00*

**What I need from you on Wednesday:** a yes, a change, or a different goal – plus answers to the 5 decisions in §8.

---

## 1. Why this doc

Right now the three of us work on different things. We have not agreed yet on **what** the robot should do in the end, **what** we try on the way, and **how** we measure success. This doc is my suggestion for that.

---

## 2. The proposal in one sentence

**SAP sends the robot a task. The robot does a safety round in our room, finds a misplaced item, brings it back to its place – fully on its own – and reports everything back to SAP.**

### The sequence

Between SAP and the robot sits a **small bridge program** on our laptop. It receives the task from SAP, starts the robot's step-by-step program, and sends the result back. In week 1 we agree on a short task and report format, so the SAP side and the robot side can be built at the same time.

1. **Task from SAP:** e.g. *"Safety round: check the room and return misplaced items."*
2. **Inspect (report only):** the robot goes to two fixed viewpoints (fire extinguisher, exit door), takes a photo of each and answers fixed yes/no questions: extinguisher in place? sign visible? exit path free? fire door closed? It does not touch anything. *→ This is your safety check. Open question for you: where does it run today (sim or real robot), which camera does it use, and what does it return?*
3. **Find:** the robot finds the misplaced item.
4. **Pick up:** it walks to the item, stops, grasps it and lifts it.
5. **Carry + put down:** it walks to the second table and puts the item into a box there (a box stops a ball from rolling away).
6. **Check again:** is the item now in the box?
7. **Report to SAP:** e.g. *"Extinguisher OK. Exit OK. Item was misplaced → returned. Done."*

**If a step fails:** every step has a time limit and one retry. If it still fails, the robot stops, puts the item down if it is holding one, and reports *"failed at step X, because Y"*. A failed run with a correct report is still a correct report.

### The business case

Today, people check by hand whether fire extinguishers are in place and emergency exits are free. That is regular, manual work. The idea: **a robot does this safety round, SAP starts it and receives the result, and the robot fixes small things itself** – later, for example, returning light safety items like a first-aid kit to their place. A real fire extinguisher is too heavy for the robot (about 7–11 kg vs. max ~3 kg per arm), so the robot only checks and reports it. (The robot does the visual round, not the official maintenance.)

This fits what SAP already does:
- SAP's EHS software (Environment, Health and Safety) has an [API to report incidents and safety observations](https://api.sap.com/api/API_EHS_REPORT_INCIDENT_SRV) – our report could create a real entry there, if we get access to an SAP S/4HANA system with EHS.
- SAP works with the inspection robot ANYmal ([SAP News, 30 Mar 2026](https://news.sap.com/2026/03/anybotics-industrial-inspections-into-business-insights/)) and in Nov 2025 announced a proof of concept with a Unitree G1 doing warehouse tasks ([SAP News, 5 Nov 2025](https://news.sap.com/2025/11/sap-physical-ai-partnerships-new-robotics-pilots/)).

### Which item?

A soft ball is harder to grasp than a bottle (it rolls), and our fingers are already at their limit around the 6.6 cm bottle. We test a ball (about 5 and 7 cm) in sim for one day; if it is too hard, we use a bottle, a can or a bean bag. **The item must not be red**, because our current object finder looks for red – and the fire extinguisher is red.

---

## 3. What we already know

| Topic | What we know | What it means |
|---|---|---|
| **What exists** | **Cristina + Saad** *(please correct/fill in: sim or real robot? how often does it work?)*: the robot patrols the room, checks the fire extinguisher and the exit door, and reports back. **Finnegan (MuJoCo sim):** the G1 with our BrainCo hands (the five-finger hands on our robot) walks to a point and between two rooms (10/10 – but the sim tells the robot where it is); it grasps and lifts a bottle with its hip fixed in place, so it cannot fall (about 95 % when the bottle is within arm's reach). | Most robot steps build on something we have. New: walking to an item and grasping it in one world (the big one), carrying, putting down, the SAP connection, and the real robot. |
| **One simulator?** | Walking + grasping run in MuJoCo (on our laptops). Isaac Lab runs on a rented NVIDIA GPU. | For the full-sequence demo we need **one** sim, or both connected through the same task/report format. → decision in §8. |
| **Our robot** | Unitree G1 EDU, **29 DoF** · BrainCo Revo2 hands · cameras: RealSense D435i (built in) + D455 on a movable 2-axis head · Livox Mid-360 LiDAR · built-in NVIDIA Jetson Orin NX 16 GB · backpack: Thunderobot PC (Intel Core i7 + NVIDIA GeForce RTX). | 29 DoF means our current grasp works on the real robot's arm. The RTX GPU in the backpack may be able to run the grasp AI on the robot itself (depends on its GPU memory). The LiDAR lets the robot find its position in the room later (Level 4). |
| **Compute + cloud** | Big AI models do not fit on the robot's computers (they need hundreds of GB of memory). | "Thinking" (planning, reading photos) runs in the cloud, e.g. AWS Bedrock (Amazon's pay-per-request service for big AI models). Walking and balance always stay on the robot. A GPU is rented only for training. |
| **The room** | A table, a ball, a fire extinguisher, a door with an emergency-exit sign. | We also need a second table (or a box on a marked spot). |

---

## 4. What is in – and what is out

| In | Out (only if time is left) |
|---|---|
| Full sequence in sim, with a grasp written in code | A learned AI grasp (π₀.₅ – an AI model that learns grasping from recorded examples; needs a GPU and roughly 2–4 extra weeks, my guess) |
| SAP: start the task, receive the report – **via a stand-in web page from week 1**, the real SAP system as soon as we have access | Picking things up from the floor |
| A real-robot track **from week 1**, step by step: connect → walk to a point → move arm, hand, camera → grasp at a fixed table → full attempt | Realistic extinguisher/door models in sim: **in sim we fake the inspection result** to test the flow; the real check runs on real photos from our room |
| Finding the tables on the real robot: rough way from the robot's own step counting (odometry), **the camera for the last metre** (e.g. a printed marker on each table) | A laser-scanner (LiDAR) map |

---

## 5. How we measure success

**Main goal**

1. **Simulation:** the full sequence works in **at least 7 out of 10 runs** (random start positions), and **every failed run names the step and the reason**.
2. **Real robot:** at least **3 full attempts**, filmed. It does not have to work perfectly – for each attempt we write down which step passed, which failed, why, and what we would change.
3. **Pitch:** sim demo + real attempts + lessons learned + the business case.

**Per step (in sim)** – these help us find the weak step. The full run is lower than each step, because the failures of all steps add up.

| Step | Success means | Target |
|---|---|---|
| Task from SAP | The task reaches the robot's program | 10 / 10 |
| Inspect | 10 prepared scenes (5 OK, 5 with one fault: extinguisher missing, exit blocked, door open). Report matches the known answer. | 10 / 10 |
| Find | Item found within 60 s, position error < 5 cm | ≥ 9 / 10 |
| Walk + grasp | Walks to the item, stops, lifts it; no fall | ≥ 8 / 10 |
| Carry + put down | Item inside the box and still for 3 s | ≥ 8 / 10 |
| Report to SAP | Report matches the run log (also for failed runs) | 10 / 10 |

---

## 6. Rough timeline

Pitch date still unknown (see §8). My rough guess: **~4–7 weeks for the three of us** (not 3× faster than alone, because some steps must come one after another).
The **sim chain** is the critical path – it does not get shorter with more people. The other lanes run next to it.

| Week | Sim chain (critical path) | Real robot | SAP + inspection |
|---|---|---|---|
| 1 | Our room in sim · put the item into a box at a fixed table | Connect (read-only first) · safety checklist | Task/report format · SAP stand-in page · inspection test scenes |
| 2–3 | Walk to the item + grasp in one world (hardest step) | Walk to a point · arm, hand, camera | Real SAP if we have access · inspection on real photos |
| **End of 3** | **Decision:** walk + grasp ≥ 5/10? If not → plan B (slower arm, look again before closing the hand), and the sim demo moves to week 6 | | |
| 4 | Full flow with carry, put down, failure handling | Grasp at a fixed table | Bridge connects SAP ↔ robot program |
| 5 | **Sim demo ≥ 7/10** | First full attempts | Report checked against run log |
| 6 | Buffer · pitch | More attempts · lessons learned | |

If the pitch is earlier, we shorten the real-robot track first, not the sim demo.

---

## 7. Honest risks

- **Not built yet:** walking to an item and grasping it in one world (the robot's balance shifts when the arm moves), carrying while walking, putting down.
- **Real robot ≠ sim.** The real robot has never been connected to our code yet. In sim it walks with a different walking program than Unitree's own one on the real robot. A grasp that works in sim often fails on the real robot at first (contact-heavy tasks often start at only 5–10 % success) – that is why our real goal is "attempts + lessons learned".
- **Safety on the real robot:** never alone; a second person always holds the remote; harness or spotter for every new movement; slow speeds; 3 m cleared. We write a 5-line safety checklist before the first session.
- **SAP access.** Without an SAP test system the most special part is missing. If *[name]* has no answer by *~21 Oct*, we keep the stand-in page and connect the real system later.

---

## 8. Meeting

**To decide on Wednesday**

1. **Goal:** this one, a changed one, or a different one?
2. **Simulator:** does the full sequence run in MuJoCo or in Isaac Lab? How does your safety check get into it?
3. **Item:** ball, or whatever is easiest to grasp (bottle, can, bean bag – not red)?
4. **Success:** 7/10 in sim + 3 real attempts + lessons learned – OK?
5. **Scope:** the learned AI grasp (π₀.₅) is optional – OK?

**To find out (one owner each, by *[date]*)**

- **SAP:** which system **starts** the task, which API **receives** the report? Who asks?
- **Robot access:** where is it, who may run it, how many hours per week, who is the second person?
- **Hardware:** which RTX GPU is in the backpack and how much GPU memory does it have? Which operating system runs on it (Linux or Windows)? How heavy is the backpack (our sim still uses a lighter 1.9 kg backpack)? Which Linux computer controls the real BrainCo hands (their driver needs Linux)?
- **Room:** second table + box?
- **AWS:** who sets it up, what budget?
- **Pitch date:** does the timeline fit?
