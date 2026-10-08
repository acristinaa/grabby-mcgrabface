# Grabby McGrabface

## Autonomous Visual Pick and Place with the Unitree G1 EDU

Grabby McGrabface is a robotics project exploring autonomous manipulation using the Unitree G1 EDU humanoid robot

The goal is to develop a system that allows the G1 to perceive its workspace, identify objects, plan and execute grasps, move objects to designated locations, verify successful placement, and recover from common manipulation failures

The initial system will operate with the robot standing in a fixed position. This allows us to focus on perception, manipulation, and task planning before introducing humanoid locomotion

## Project Goal

The main objective is:

> Develop and evaluate a vision guided manipulation system that enables the Unitree G1 EDU to autonomously identify, grasp, transport, and place objects into designated locations while handling variation in object position and recovering from failed manipulation attempts


## Phase 1: Controlled Pick and Place

The robot operates in a controlled environment where the object type, approximate position, and destination are known.

The objective is to establish reliable low level manipulation.

```text
Home
  |
  v
Approach object
  |
  v
Grasp
  |
  v
Lift
  |
  v
Move to destination
  |
  v
Release
  |
  v
Return home
```

## Phase 2: Vision Based Picking

The object's position is no longer provided directly.

The robot uses its camera and other available sensors to locate the object.

```text
Camera
  |
  v
Object detection
  |
  v
Depth estimation
  |
  v
3D localization
  |
  v
Coordinate transformation
  |
  v
Grasp pose generation
  |
  v
Motion planning
  |
  v
Grasp
```

The system should be able to locate objects at different positions within the workspace

## Phase 3: Task Level Pick and Place

The robot receives a high level task instead of an explicit sequence of movements

For example:

> Put all the red objects into the red box.

The robot must determine which objects are relevant, which object to manipulate, where to grasp, where to place the object, and whether the manipulation was successful

```text
Detect scene
    |
    v
Select object
    |
    v
Plan grasp
    |
    v
Pick
    |
    v
Verify grasp
    |
    v
Place
    |
    v
Verify placement
    |
    v
Select next object
```

## Phase 4: Robust Manipulation

The final stage introduces environmental variation and manipulation failures.

Possible challenges include:

1. Different object positions
2. Different object orientations
3. Multiple objects
4. Distractor objects
5. Imperfect object localization
6. Failed grasps
7. Objects slipping
8. Objects being dropped
9. Changes in the workspace

The system should detect failures and attempt recovery instead of terminating the task.

