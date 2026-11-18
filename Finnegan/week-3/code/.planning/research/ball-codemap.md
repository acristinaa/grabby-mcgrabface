# Bottle-to-ball switch: everything in the code that assumes a bottle, plus a small design and where the box goes

Checked on 2026-10-07 against commit `ec59653`. The working tree is clean apart from an untracked `.DS_Store`. Repo root, called REPO below: `/Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot`.

## 0. Key findings

1. **A ball lying on the table is mostly out of reach for the current D2 grasp.** I replayed the exact `54 --randomize` starts (seed 0 and seed 1, 50 starts each) and put the object centre at the bottle's x/y.
   - At bottle height: 25/50 and 35/50 starts reachable. This matches the D2 runs exactly (`runs/d2_54_seed0.json` and `seed1.json`), so the probe reproduces the real pipeline.
   - Ball of radius 3.5 cm on the table: 12/50 and 12/50.
   - Ball of radius 5 cm: 12/50 and 18/50. Radius 7 cm: 15/50 and 20/50. Ball raised 10 cm: 20/50 and 28/50.
   - The limit is the arm itself with this hand orientation, not the 2 cm table-clearance rule. Setting that rule to 0 in the probe changed nothing.
   - The bottle's start spot (0.40, −0.16) is out of reach for a ball on the table (about 18 mm reach error).
2. **The unchanged D2 grasp can pick up a 3.5 cm-radius ball** (red sphere, one trial per case, fixed stand) once `GRASP_BELOW_TOP = r`, so the grasp point is the ball's centre:
   - Perception error 2.5 mm, hand error 0.4–0.5 mm.
   - **0.3 kg and 0.5 kg balls: held, lift 10.0–10.5 cm.**
   - **0.058 kg (tennis-ball weight) and 0.15 kg balls: slipped out during the lift.** The ball ended back on the table.
   - The ball rolls 6–17 mm while the fingers close. Turning on rolling friction did not change that during closing.
   - These are single trials, not success rates.
3. **A ball rolls away with the current contact setting.** I pushed a 0.15 kg ball at 0.2 m/s in a small standalone MuJoCo 3.13 model:
   - With the current setting (`condim=4`) it still moves at 0.14 m/s after 2 s and has travelled 28.5 cm.
   - With `condim=6` and rolling friction 0.001 it stops within 5.6 cm.
   - (`condim` sets which friction types a contact uses; only 6 includes rolling friction.)
4. **The ball looks 2–5 times smaller in the dataset camera image** (head camera at 320×200): 155–337 pixels versus 765 for the bottle. The `Locator` still finds it at 640×400 (614–1356 pixels against a 50-pixel minimum). One existing test needs more than 500 pixels.
5. **Velocities of a free-floating object in MuJoCo:** the first three numbers are linear velocity in world coordinates, the last three are angular velocity in the object's own frame. I checked this in MuJoCo 3.13. This matters for the "still for 3 s" check.

How I measured: all probes are read-only scripts under `/private/tmp/claude-501/-Users-finneganadam-Documents-Projekt-Tim-and-Lukas-Cursor-Software-Engineering-SAP-Robot-Project-First-Ideas/c4393878-48d4-4476-8b1e-4e89a44b6cd7/scratchpad/probe/`. They load the existing `~/dev/g1-ext/g1_brainco_sim/assets/scene_29dof_brainco_table.xml`, change the bottle shape in memory only, and write nothing to the repo or to the sim folder.

## 1. Every bottle assumption, by file

Columns (a), (b), (c) say whether the item matters for (a) the scripted grasp at the fixed table, (b) placing into the box, (c) demo recording, dataset and openpi. "doc" means it is only a comment, docstring or UI text.

### g1sim/table_scene.py
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| 8 | Docstring: "rote Flasche (0,5 kg)" | Text | doc |
| 21, 123–134 | One scene file (`SCENE_FILE`), rewritten by every script and test | One file per object (e.g. `..._table_ball.xml`). Otherwise a bottle run and a ball run in parallel load the wrong scene. The atomic write only prevents half-written files | a, c |
| 32–37 (row 34) | `red_bottle`: cylinder, size [0.033, 0.105], red, 0.50 kg, at (0.40, −0.16) | A ball entry: sphere, [r], red, a mass to decide, and a start spot inside the reachable band | a, b, c |
| 46–48 | `object_rest_height`: cylinder uses `size[1]`, everything else uses `size[2]` | A sphere branch using `size[0]`. Today a sphere crashes with IndexError | a |
| 106–119 | Line 114: anything that is not a cylinder becomes a **box**. Line 115: size. Lines 118–119: `condim=4`, friction [1.2, 0.05, 0.001] | Sphere type and size [r, 0, 0]. Today a sphere entry silently becomes a box. Decide `condim` 6 or not (rolling friction only works with 6) | a |
| 95–104 | Table only, no box | Insertion point for the box (section 4) | b |

### g1sim/table_env.py
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| 69–70 | `object_qadr` and `scene_bodies` = table + OBJECTS | Add the box to `scene_bodies` so contacts with it are reported. Store each object's velocity index too, for the stillness check | b |
| 107–111 | Saved variations are keyed by object name (`v["objects"][name]`) | Replaying an old run whose variation says `red_bottle` raises KeyError if the bottle is not in OBJECTS. This is the main reason to keep the bottle reproducible | c |
| 127–133 | `_start_is_clear` checks only objects and the table | Add the box parts | b |
| 199–201 | `lift_height` defaults to `"red_bottle"` | Default to the selected target | a, c |
| 203–207 | `object_tilt` defaults to `"red_bottle"` | Meaningless for a ball: it only measures how far the ball has rolled. Skip it or report "none" | a, c |

### g1sim/randomization.py and configs/randomization.json
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| 23–24, json:35 | Config key `bottle_rgb_jitter` (the loader requires exact key names) | Keep the name (old configs keep loading) or rename both together | a, c |
| 27 | `TARGET = "red_bottle"` | Selected target name | a, c |
| 100–103, 179–185 | Only the target is red; everything else is drawn "not red" | Same rule for the ball. The box colour must not be red | a, b |
| 188–191 | `footprint`: cylinder uses `size[0]`, everything else `hypot(size[0], size[1])` | A sphere branch. Today: IndexError | a |
| 194–211, json:7–13 | Offset ±0.15 m around (0.40, −0.16) | Too wide for a ball (finding 1). Needs a ball start spot and offset of its own. Also keep objects out of the box | a, b |
| 86, 142–144 | `mass_scale` per object name | Works as is | – |

### g1sim/grasp.py
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| 1–12, 27–29 | Side grasp tuned against bottle tilt (roll −30°) | Keep for now. It worked on the ball in the probe | a |
| 32–35 | `BOTTLE_IN_HAND` = [0.066, 0, 0.090], the bottle axis in hand coordinates | Same numbers when r ≈ 0.033–0.035. Rename or move into the per-object grasp settings. Script 63 overrides it by this exact name | a |
| 36–38 | `GRASP_BELOW_TOP` = 0.06, grasp height below the cap | Equals r for a ball (grasp at the centre) | a |
| 41–45 | `LIFT` 0.10 and `LOAD_SAG` 0.04, tuned for 0.5 kg (about 2.5 cm arm sag) | A lighter ball sags less and lifts higher. Harmless (probe: 10–10.5 cm) | a |
| 47–53 | `CLOSED`: ring and pinky stay open; the reason given is bottle-specific | Light balls slipped (finding 2). Probably needs a ball-specific sweep with script 63 | a |
| 59–63 | `LOOK_OFFSET` tuned with "bottle ≥ 770 pixels visible" | Re-check the pixel count for the ball | a |
| 65 | `FORBIDDEN` = table, cup, block | Decide how contact with the box is treated | b |
| 72–75, 89–91 | `grasp_pose(bottle_top)`; `bottle_top` = position + `OBJECTS["red_bottle"][1][1]` (half height) | Top of the ball = centre + r. Generalise to `object_top` | a |
| 94–106 | `reach_error` against the true bottle; reach window x ≈ 0.30–0.46 (comment, line 97) | Measured: at ball height the window shrinks to x ≈ 0.25–0.35/0.40 | a |
| 131–180 | Phases end with lift and a 5 s hold. Line 175 reports `bottle_tilt_deg` | Add carry, lower, release and wait. Tilt has no meaning for a ball | a, b |
| 183–189 | `_look` error text "Flasche"; compares with `bottle_top` | Generalise | a |

### g1sim/perception.py
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| 1–6, 11, 31–50 | Returns the centre of the flat cap: mean of the points within 4 mm of the highest red points | Works unchanged as "top of ball" (centre xy, about 2 mm low), measured error 2.5 mm. Then centre = top − r | a |
| 15–19 | `red_mask` | The ball must be red (or the mask changes) | a |
| 12 | `MIN_PIXELS` = 50 | Fine at 640×400 | – |

### g1sim/grasp_ik.py and g1sim/action_space.py
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| grasp_ik.py 17, 76–78 | Forearm keeps 2 cm from `table_top` only | Measured: the lowest finger sits 0.9 cm (1.55 cm with all fingers closed) below the grasp point, so a ball with r ≥ 0.03 respects the 2 cm limit. Add the box geometry to this clearance rule | b |
| action_space.py 40–44 | Start pose comment mentions the bottle | No change needed: a lower object is further from the hand | doc |

### g1sim/teleop.py and g1sim/teleop_page.html
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| teleop.py 21, 23, 230 | Grasp constants and `DEFAULT_TASK` | Selected object and task | c |
| teleop.py 46, 294–295 | Teleop success = lift ≥ 5 cm and no forbidden contact (its own copy of the rule) | Add the in-box rule | b, c |
| teleop.py 48, 238–245 | Only scenes with a reachable bottle | Fewer reachable draws (finding 1); `MAX_DRAWS` 50 is probably still enough | c |
| teleop.py 281 | Vibration on new contact with `"red_bottle"` | Selected target | c |
| teleop.py 299–302 | Episode JSON stores `bottle_tilt_deg`, no object description | Add the object spec | c |
| teleop_page.html 50, 65, 88–89 | UI text "Flasche" | Text | doc |

### g1sim/openpi_env.py, g1sim/episode.py, openpi_g1/
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| openpi_env.py 15 | `DEFAULT_TASK = "pick up the red bottle"`, used in 64:32/48/76, teleop.py:23/230, 60:131 | A task text per object and task, e.g. "put the red ball in the box" (needs a ruling) | c |
| openpi_g1/config.py 28 | `REPO_ID = "g1_brainco/red_bottle_sim"` | A separate dataset name so bottle and ball data never mix | c |
| openpi_g1/policy.py 28 | `make_example` default prompt | Optional | c |
| episode.py 52–55, 98–99 | The `.npz` stores only task, fps and success | The object is identified only by the task text and the side JSON. Add it to the side JSON | c |

### scripts/
| Line | What it assumes | What a ball needs | a/b/c |
|---|---|---|---|
| 53:4, 53:85 | Check `flasche_0_5_kg` on the `red_bottle` body | Check per object | a |
| 54:1–10, 41–47, 54–66 | `evaluate()`: `flasche_in_reichweite`, `kopfkamera_findet_flasche`, …, `flasche_5_cm_angehoben_5_s_gehalten`. Scripts 63 and 64 reuse this function | New final check "in the box and still for 3 s"; keep lift ≥ 5 cm as an intermediate check | a, b, c |
| 54:93–94, 114 | Start position and video follow `"red_bottle"` | Selected target | a |
| 54:121–129, 154–157 | Prints tilt; criteria list `hold_s` 5.0 | Update | a |
| 55:51–52, 56:15, 57:194 | Print and help text | Text | doc |
| 58:38–39, 60–65 | Simulated operator plan from `bottle_top`, `GRASP_BELOW_TOP`, `BOTTLE_IN_HAND`; plan ends with lift and hold | Generalise, then add placing | a, b, c |
| 60:75, 86–87, 131 | Object drift over all objects; `--task` | Task only | c |
| 63:1–11, 48–50, 62–73 | Overrides grasp constants **by their exact names** | Any rename must update `KEYS` here | a |
| 63:92–96, 113–115, 120, 130–140, 198 | `"red_bottle"` and tilt statistics | Target name; drop tilt for a ball | a |
| 64:3–5, 56–59, 76–82 | Out-of-reach reason `flasche_in_reichweite`; metadata stores `bottle_tilt_deg`, `GRASP_BELOW_TOP`, `BOTTLE_IN_HAND` | Add the object spec to the metadata | c |
| 10, 12, 13, 23 | Old scene (`scripts/10_build_scene.py:75`), separate from the table scene | Out of scope | – |

Other mentions are comments only: `g1sim/rooms.py:5`. `g1sim/liveview.py:121` draws a cylinder; I assume it is an unrelated marker, not checked. In `g1sim/payload.py` and `tests/test_unit_payload.py`, "tilt" means the head servo and is unrelated.

## 2. Tests that would break if the bottle is replaced outright (ball, r = 0.035, at the bottle's spot)

- `tests/test_integration_table_env.py:63-68`, the counter-check that the fingers sit inside the bottle in the zero pose. **Fails**: measured, no contact with the ball.
- `tests/test_integration_grasp_ik.py:27-41`, IK reaches the grasp pose (needs < 1 cm). **Probably fails**: the grid point (0.40, −0.15) at ball height gave 18 mm. It passes if the ball starts inside the reachable band.
- `tests/test_integration_grasp_ik.py:97-143`, which needs more than 500 pixels of `red_bottle_geom` at 320×200. **Probably fails**: the ball shows 155–337 pixels (measured in the zero arm pose, not after the hand moves away). It also breaks on the name alone.
- `tests/test_integration_grasp_ik.py:73`, `tests/test_integration_randomization.py:105-110`, and `scripts/53` line 85: KeyError if the object is renamed.
- `tests/test_integration_randomization.py:113-129`: IndexError in `footprint` for a sphere unless a sphere branch is added.
- `tests/test_unit_openpi_policy.py:34` breaks only if the default prompt in `policy.py` changes.
- Not broken: `test_integration_openpi.py:25` and `test_integration_openpi_env.py:77` (they compare against their own task text); `test_integration_table_env.py:46-55` (measured 0 mm drift for the ball); `test_integration_table_scene.py` (uses the old scene).
- **With a flag that defaults to the bottle, no existing test breaks.** The ball needs tests of its own.

## 3. Design recommendation: an object setting chosen by `--object ball|bottle`, bottle as the default for now

What it would look like (a sketch, not an implementation):
- **Object list:** a small table in `table_scene.py`, e.g. `TARGETS = {"bottle": (cylinder, [0.033, 0.105], red, 0.50, (0.40, −0.16)), "ball": (sphere, [r], red, mass, start spot)}`.
- **Scene:** `OBJECTS` = the selected target plus the distractors, one scene file per object. Never put both targets in one scene: both are red and the red mask would find both.
- **Target name in one place:** move `TARGET` (`randomization.py:27`) into `table_scene`. Replace the literal `"red_bottle"` in the code with it (`table_env.py:199/203`, `grasp.py:91/175`, `teleop.py:281`, `54:93-94`, `63:92-120`). Keep the bottle's body name `red_bottle` so old variation files still replay.
- **Grasp settings per object:** e.g. `GRASPS = {"bottle": {D2 values}, "ball": {GRASP_BELOW_TOP: r, ...}}`. Script 63 sets these by name, so either keep the names or update its `KEYS` (63:48).
- **Sphere branches** in `object_rest_height`, `footprint`, and the geom type/size lines (114–115).
- **Where the flag goes:** scripts 53, 54, 55, 57, 58, 63, 64 pass it into `table_scene.write(object=...)` and `TableEnv(object=...)`. `DEFAULT_TASK` and `REPO_ID` follow the object and task.

Why I recommend the flag over replacing the bottle:
1. The bottle results stay reproducible. Saved variations are keyed `red_bottle` (`table_env.py:107-108`; the run files in `runs/d2_*`, 64 metadata, teleop JSONs).
2. Levels 2 and 3 need several objects anyway ("red vs yellow bottle", "give me the yellow bottle"), so an object list is needed regardless.
3. The existing tests stay valid as a bottle regression check.

Trade-off: replacing the bottle is a smaller change today (one row in OBJECTS plus about 12 literals). But it breaks the tests in section 2, makes old runs impossible to replay, and the bottles come back at Level 2. I would switch the default to ball once the ball passes the acceptance test (your call).

## 4. Adding the box and checking "inside and still for 3 s"

**Where in the code:** in `table_scene.build_spec`, right after the table (after `table_scene.py:104`, before the object loop at 106). Add a `box` body directly under the world with no joint, which makes it static. It gets 5 box-shaped parts: a floor and 4 walls. Use a colour that is not red, `condim=4`, and friction like the table. Store the measurements as constants (centre, inner half-width, wall height, wall thickness, floor thickness).

It must also be added in:
- `table_env.scene_bodies` (line 70), so contacts with it are reported;
- `_start_is_clear` (lines 130–133);
- the forearm clearance rule in `grasp_ik.py:76-78`;
- `randomization._sample_objects` (194–211), as a no-spawn zone;
- the contact policy in `FORBIDDEN` (`grasp.py:65`).

A static box adds nothing to the joint state vector, so recorded joint states keep their layout.

**Where on the table** (inferred from the reach map at the nominal stand; the stand variation of ±10 cm and ±10° is not measured):
- Carrying height (ball centre 0.15 m above the table) is reachable within 1–3 mm over x 0.25–0.45, y −0.30 to +0.05.
- A ball lying on the table is reachable within 10 mm only at about x 0.25–0.35 (up to 0.40 near y −0.15), y −0.25 to 0.0.
- Proposal: **box centre ≈ (0.33, 0.00), inside ≈ 0.14 × 0.14 m, walls ≈ 0.06 m high.** Ball start spot ≈ (0.31, −0.16) with a smaller random offset, e.g. ±0.06 m (a guess).
- This collides with the blue block's start area ((0.48, 0.0) ± 0.15), so the block has to be kept out of the box area or moved.
- Alternative: box on the right at (0.30, −0.32). That is at the edge of the reach area (1–14 mm), so it is weaker once the stand varies.
- The head camera sits at y ≈ 0, so the box would be beside the ball, not between camera and ball. Occlusion is not measured.

**The check, using MuJoCo data,** evaluated every control tick after the hand has released the ball:
- Ball position `p` (from its free joint), relative to the box centre `c`:
  - |p_x − c_x| ≤ inner half-width x − r + tolerance, and the same for y;
  - p_z ≤ table top + floor thickness + r + 0.01 (lying on the floor, not held above it).
- Stillness: linear speed (velocity entries 0–2, world frame) < about 0.01 m/s, and angular speed (entries 3–5, ball frame) < about 0.3 rad/s. These thresholds are guesses.
- No robot contact with the ball (the target missing from `env.robot_contacts()`). Optionally also require the ball to be touching the box parts.
- Success when all of this holds for **90 control ticks in a row** (3 s × 30 Hz).
- `table_env` stores each object's position index but not its velocity index (line 69); one more lookup is needed.

## 5. What this means for Level 1

- A ball lying flat on the table cuts the reachable randomized starts roughly in half to a third: 24/100 versus 60/100 for the bottle. The current random offset (±0.15) and start spot were chosen for the bottle.
- The acceptance target "≥ 8/10 of 50 randomized trials" needs a decided denominator: script 54 counts out-of-reach starts as failures, while script 64 filters them out first.
- The current grasp holds a 3.5 cm ball of 0.3–0.5 kg but not 0.06–0.15 kg (one trial each). The ball's size and mass decide whether `CLOSED`, the roll angle, and `BOTTLE_IN_HAND` need a ball sweep with script 63.

## 6. Decisions for you
1. Ball size and mass. Ideally match a real ball you will buy; a tennis ball has r ≈ 0.033 m but weighs about 0.058 kg, which slipped in the probe.
2. How to deal with the reach problem: bigger ball, raised stand or holder, narrower start area for the ball, or a different grasp from above.
3. `condim=6` with rolling friction for the ball (realistic, but a real table may roll the ball more freely).
4. Whether the box may be touched while placing.
5. The task text and dataset name. This also settles the open conflict in `PLAN.md:60` and `:89`: dataset v1 as "pick up" only, or as pick-and-place.
6. Default object once the ball passes.

## 7. Still uncertain
- The ball-grasp results are single trials at the fixed stand; there are no success rates yet.
- Reach at the proposed box spot under stand variation is not measured.
- Whether the box blocks the camera's view of the ball, or the carrying arm, is not measured.
- Pixel counts were taken in the zero arm pose, not after the hand moves out of view.
- Why the light balls slip is not investigated (my guess: the closing fingers squeeze them out).
- The thresholds for "still" and the tolerances in section 4 are proposals, not measurements.