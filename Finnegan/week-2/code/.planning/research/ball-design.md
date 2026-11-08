# Ball pick-and-place design spec (Level 1, right hand, fixed pelvis)

**Recommendation in short:** use a 6.4–6.7 cm red ball weighing at least 57 g. Put it on a 3 cm ring holder and grasp it **top-down** with the palm facing down, closing only thumb, index and middle. Ring and pinky stay open, as in D2. The box sits on the same table, inside the reach window.

All design choices below are recommendations; the user decides.

---

## 0. How this was checked (and what was not)

**Read from the model and code:**
- Hand geometry comes from loading `~/dev/g1-ext/g1_brainco_sim/assets/scene_29dof_brainco_table.xml`, built 2026-10-07 (commit ec59653).
- Reach was computed with the repo's own `g1sim.grasp_ik.ArmIK` (mink 1.3.0 / daqp 0.9.1, installed only in a scratch venv).

**Simulated, hand only:** I loaded `models/revo2/xml_right/brainco-righthand-v2.xml` with the scene's settings: elliptic cone, impratio 10, dt 1/300, condim 4, motors limited to `actfrcrange`, and the same PD (proportional-derivative) control as `table_env.py:155-159` (KP 20, KD 0.3). The hand is fixed in space. The ball sits on a table or holder, the fingers close, then the table is lowered 10 cm (equivalent to a 10 cm lift) and held for 3 s.

**Not run:** the full `TableEnv` with arm, IK sag, perception and approach motion. All success numbers are therefore an upper bound.

**Environment gap:** the project venv `.venv` (pointing to `.venv.nosync`) has mujoco 3.12.0 but no `mink`, `daqp` or `qpsolvers`. So `g1sim/grasp_ik.py` cannot be imported there right now (install line: `COMMANDS.md:262`).

**Scratch scripts** are in `/private/tmp/claude-501/-Users-finneganadam-Documents-Projekt-Tim-and-Lukas-Cursor-Software-Engineering-SAP-Robot-Project-First-Ideas/c4393878-48d4-4476-8b1e-4e89a44b6cd7/scratchpad/` (`hold.py`, `holder.py`, `robust*.py`, `reach*.py`, `box.py`).

## 1. Revo2 facts

Hand-base frame: x = palm normal, y = thumb side, z = along the fingers.

**Measured from the model:**
| Quantity | Value | Model / source |
|---|---|---|
| Palm face | x = 0.0217 m | |
| Palm width | y −0.0371…+0.0373, so 7.4 cm | official 76 mm |
| Length to middle fingertip | 0.159 m | official 160 mm |
| Finger base joints (MCP) | z ≈ 0.079–0.085 | xml:95,116,137,158 |
| Open finger pads | x ≈ 0.02, z 0.129–0.150 | |
| Rotated-thumb pad centroid | (0.086, 0.019, 0.041) | |
| Thumb–index pad distance, open | 11.9 cm | official max opening 100 mm thumb–index |
| Fully closed fingertips | curl back to z ≈ 0.05 (a fist) | |
| Thumb rotation torque | 0.5 Nm | xml:70 |
| Thumb aux torque | 1.1 Nm | xml:75 |
| Finger torque | 2 Nm each | xml:97… |
| Distal joint coupling | 1.0 / 1.155 | xml:199-203 |

**Official BrainCo spec ([parameters page](https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/parameters.html)):** fist grip ≥ 50 N, pinch ≥ 15 N, open/close ≤ 0.65 s, joint ranges thumb flexion 0–59°, thumb abduction 0–90°, fingers 0–81°.

**§10.10 mechanism confirmed for the ball:** with all five fingers closed, the thumb rotation is driven back from 1.0 to 0.48 and the ball is lost. This happened in 31 of 41 failures (side pose) and 33 of 42 (top pose). With ring and pinky open, the thumb stays at 0.99–1.0. Closing all five gives 6–8 of 48 grid points; the D2 closure gives 36–37 of 48.

## 2. Two blockers on a flat table, and why the holder is needed

**Lowest hand point relative to a 6.5 cm ball resting on the table (measured):**
- Side pose (D2): the pinky ends 0.3–0.8 cm above the table.
- Top pose: the rotated thumb ends 0.5 cm *inside* the table.

**IK cannot reach it.** The IK keeps the whole forearm and hand 2 cm above the table (`TABLE_CLEARANCE = 0.02`, `grasp_ik.py:17,70,76-78`). The bottle needed ±5 mm accuracy (`grasp.py:32-34`).

**IK position error, pelvis fixed, x = 0.30…0.46 m, y = 0…−0.25 m:**
| Pose | Flat table | 3 cm holder |
|---|---|---|
| Side (D2 roll −30°) | ≥ 0.9 cm everywhere, up to 10.7 cm | ≤ 0.8 cm only for x ≤ 0.38 |
| Top (palm down) | not usable (thumb hits table) | ≤ 0.4 cm for x 0.30–0.42, y −0.05…−0.25 |

**The ball rolls freely in the current setup.** Scene objects use condim 4 (`table_scene.py:118`); condim is MuJoCo's contact-dimension setting, and 4 includes no rolling friction. Rolling resistance needs condim 6 ([MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)).

**Holder:** a ring or short tube, 2 cm ring radius, 3 cm tall. The ball rests on the rim with its centre about 5.6 cm above the table.
- In sim: 16 vertical capsules (radius 3 mm) on a 2 cm circle, or one mesh. Make it a static body and add it to `FORBIDDEN` (`grasp.py:65`).
- Real (suggestion, not tested): a 30 mm piece of roughly 40 mm pipe, or a 3D-printed ring.

**Is a holder allowed for Level 1?** That is the user's call. My evidence says the alternative is lowering the finger clearance plus a partial thumb rotation, which I did not test.

## 3. Ball

**Sim ball:**
- Sphere radius 0.032–0.0335 (match the real ball), mass equal to the real ball.
- `condim=6`, `friction=[1.0, 0.005, 0.0005]`, `priority=1`.
- `solref=[0.02, 1]` (default), `solimp=[0.95, 0.99, 0.001, 0.5, 2]`.
- Scene: elliptic cone and impratio 10 as already set (`table_scene.py:69-70`).

**Measured, ±5 mm ball offsets (27 per case), 3 cm holder, D2 closure:**
| Ball | Default contact parameters | Recommended parameters |
|---|---|---|
| 6.5 cm / 57 g | 24/27 | — |
| 6.8 cm / 57 g | 24–26/27 | — |
| 7.0 cm / 57 g | 27/27 | — |
| Any size / 100–150 g | 27/27 | — |
| 30 g | 1–5/27 (squeezed out of the hand) | 27/27 |
| 6.4 cm / 145 g | — | 27/27 (side and top) |
| 6.7 cm / 57 g, μ 1.0 or 0.8 | — | 27/27 (side and top) |

**Friction (μ) with default contact parameters:**
| Ball friction μ | Result |
|---|---|
| 0.7–0.8 at ≥ 100 g | 27/27 |
| 0.7 at 65 g | 18–20/27 |
| 0.5 | 12–14/27 |

**Diameter band:**
- On the flat table, 5.5–6.0 cm mostly failed (penetration).
- On the holder, 6.0 cm gives 15–21/27.
- The usable band is about 6.4–7.0 cm. I tested nothing above 7.0 cm.

**Contact settings:**
- A damping ratio below 1 (e.g. 0.2) leaves the ball jittering at about 3 mm/s, so "still for 3 s" becomes unreliable. Keep dampratio = 1.
- A warning in the spirit of §10.10: the stiffer `solimp` changes contact softness, not friction. Randomize both settings, and pick a ball of at least 57 g so the default contact settings also pass.

**Real ball (prices and availability not checked):**
1. **Red lacrosse ball.** NOCSAE ND049: 6.27–6.47 cm, 5.0–5.25 oz (142–149 g), bounce coefficient (COR) 0.60–0.70, smooth solid rubber ([NOCSAE](https://nocsae.org/wp-content/uploads/2018/05/ND049-19-Std-perf-spec-for-new-lacrosse-balls.pdf)). It is in the success band, heavy enough to be robust, and red passes `red_mask` (`perception.py:15-19`).
2. **Standard tennis ball.** ITF types 1–3: 6.54–6.86 cm (type 3: 7.00–7.30 cm), 56.0–59.4 g, rebound 135–147 cm from 254 cm, so COR about 0.73–0.76 ([ITF Rules 2026, App. I](https://www.itftennis.com/media/7221/2026-rules-of-tennis-english.pdf)). It is yellow, so it needs a red cover or paint for the current mask.
   - ITF Stage 2 orange: 6.00–6.86 cm, 36–46.9 g. Orange probably fails `r > 2g` (inference).
   - Stage 3 red: 7–8 cm (felt) or 8–9 cm (foam), which is above the tested band.
3. **Not recommended:** ping-pong (4 cm, 2.7 g) is far outside the size band and too light. A foam stress ball has unknown mass (probably under 40 g, a guess) and only passes in sim with the stiffer contact settings.

## 4. Grasp strategy (top-down, three fingers)

**Ball centre in hand-base coordinates (m):**
`BALL_IN_HAND = (0.022 + r + 0.0075, +0.015, 0.0825)`, which for a 6.5 cm ball is (0.062, 0.015, 0.0825).
- In words: the palm sits 7.5 mm above the top of the ball, the ball is 1.5 cm toward the thumb side, and 8.25 cm along the fingers. For comparison the bottle was (0.066, 0, 0.090) (`grasp.py:35`).
- The success region is wide: gap 0–10 mm, y 10–20 mm, z 75–90 mm.
- A 7 cm ball needs z ≥ 0.08, or the thumb collides with the ball at the start.

**Orientation:** palm normal straight down, fingers pointing forward (+x), thumb side to the left (+y). Columns: x_h = (0, 0, −1), y_h = (0, 1, 0), z_h = (1, 0, 0).

**Sequence:**
1. Pregrasp 8 cm straight above the grasp pose, hand open, thumb **not** rotated.
2. Descend straight down. The last about 3 cm should be slow, say ≤ 3 cm/s (my guess).
3. Settle and measure sag as today (`grasp.py:147,152`).
4. Rotate the thumb (`THUMB_ROTATED`, 2 s), then close (1, 1, 1, 1, 0, 0).

Rotating the thumb late gave the same 24/27 as pre-rotating it. A 1 s closing ramp did not help (21–24/27). Pushing the ball before contact is not modelled, since the hand-only probe does not move.

**Side grasp:** it holds the ball just as well in the hand, but reach is worse on the holder (see the IK table in §2), so I do not recommend it.

**LOAD_SAG 0.04** (`grasp.py:45`) was tuned for the 0.5 kg bottle. Re-measure it with the ball (inference: it will be less).

## 5. Box and placement

- **Box:** static, inner 0.14 × 0.14 m, walls 0.06 m high and 5 mm thick, floor 5 mm, not red. It sits on the same table, which spans x 0.25–0.85 and y ±0.45 (`table_scene.py:24-26`).
- **Positions:** box centre (0.34, −0.26), holder at (0.36, −0.06). The holder and the box wall are about 11 cm apart.
- **Release:** palm down above the box centre, with the ball bottom 2–3 cm above the wall top (ball centre about 0.11–0.12 m above the table).
- **IK error over the box area** (y −0.20…−0.30, x 0.30–0.38):
  - Release height: ≤ 0.2 cm.
  - Carry height (0.16 m): ≤ 0.1 cm.
  - Error grows from x = 0.42 and from y = −0.35.
- **Drop test (sim, 18 drops):** no bounce-out with lateral speed up to 0.3 m/s. The ball comes to rest after 0.55–0.7 s with condim 6 versus 1.7–2.8 s with condim 4.
- **Real bounce (inference):** the sim contact is nearly inelastic. A real COR of 0.6–0.76 from about an 8 cm drop gives roughly 3–4.6 cm rebound, which is still below the 6 cm wall.
- **Release order:** not tested. Suggestion: open index, middle and thumb flexion first, rotate the thumb out last, wait about 1 s, then retract upward.

## 6. Success criteria (proposed)

**Grasp:**
- The minimum ball height above its rest-on-holder height during a 3 s hold is ≥ 0.05 m.
- The ball touches at least 2 hand bodies.
- No hand contact with the table, holder or box.

**Place:**
- Within 10 s of release, the ball centre is inside the box: |dx|, |dy| ≤ 0.07 m and z ≤ floor + r + 0.01.
- The ball speed stays below 0.01 m/s on every tick for 3 s.
- No hand contact with the ball during that window.

**Trial target:** ≥ 40/50 randomized trials (the PLAN's ≥ 8/10).

**Code changes these criteria need:**
- `object_rest_height`, `lift_height` and `object_tilt` only handle cylinders and boxes (`table_scene.py:46-48`, `table_env.py:199-207`). Add a sphere case and drop tilt for the ball.
- Randomize the holder and ball as one pair. The current ±0.15 m object offset (`configs/randomization.json`) is wider than the reach window.

## 7. Top three risks

1. **The hand-only probe is not the full robot.** It leaves out arm sag, IK lag, the approach pushing the ball, and stand randomization of ±0.10 m and ±10°.
   *Mitigation:* run a `63_grasp_sweep`-style sweep in `TableEnv` on the same 50 starts, re-measure `LOAD_SAG`, and keep the final descent slow.
2. **Perception of a sphere.** `Locator` takes the 99th-percentile top cap (`perception.py:11,49-50`). On a sphere that cap is only about 1.6 cm wide, and depth noise is 2 mm std, so it may be biased (inference). Also, a hand hovering above the ball will block the head camera at the second look (inference). Note: `Locator` defaults to 640×400 (`perception.py:23`), not 320×200.
   *Mitigation:* estimate the centre from all red surface points plus the known radius along the viewing ray, and take the second look from a pose that is offset back and outward.
3. **Sim-to-real contact.** Success depends strongly on μ (0.5 gives 12–14/27), on `solimp`, and on ball mass. Real Revo2 fingertip friction and the real thumb rotation torque (0.5 Nm in the model) are unknown.
   *Mitigation:* randomize μ 0.7–1.2 and both `solimp` settings, use a ball of at least 57 g, and test the real ball on the real hand by teleop before recording demos.

## Open uncertainties
- Full-arm grasp success has not been measured.
- I did not check whether fingers touch the holder during closing; only the start pose was checked and it was clean.
- Release dynamics are not simulated.
- Whether the user accepts a holder for Level 1.
- How the real ball's friction compares to the 0.7–1.0 tested.
- MuJoCo's rule for combining `solref`/`solimp` when geom priorities differ: the docs I read only confirm the rule for equal priorities. The measured effect of the change still stands.
- Missing `mink` in `.venv`.

**Sources:**
- [BrainCo Revo2 parameters](https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/parameters.html)
- [PR Newswire Revo2 launch](https://www.prnewswire.com/news-releases/brainco-launches-revo2-hand-for-humanoid-robotics-302557714.html)
- [ITF Rules of Tennis 2026, Appendix I](https://www.itftennis.com/media/7221/2026-rules-of-tennis-english.pdf)
- [NOCSAE ND049 lacrosse ball spec](https://nocsae.org/wp-content/uploads/2018/05/ND049-19-Std-perf-spec-for-new-lacrosse-balls.pdf)
- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo modeling, contact parameter mixing](https://mujoco.readthedocs.io/en/stable/modeling.html)