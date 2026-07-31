# Spin the Bottle — NAO Robot Game

A ROS package that makes an [Aldebaran/SoftBank NAO](https://en.wikipedia.org/wiki/Nao_(robot)) robot referee a game of spin-the-bottle: it watches a real bottle with its camera, works out where it's pointing once it stops spinning, turns its head to look at that spot, and asks whoever it sees to take their turn.

Originally built by **Diego Alejandro Gómez Pardo** in 2015 as a project for the Humanoid Robotics Praktikum at the University of Bonn — the original report is in [`spin_the_bottle/documentation/Report.pdf`](spin_the_bottle/documentation/Report.pdf). This repo is now a personal, ongoing rework of that project.

The repo now contains two things:

- **`spin_the_bottle/`** — the original 2015 ROS (Indigo) / OpenCV package that ran on the physical robot.
- **`vision_standalone/`** — a from-scratch, ROS-free reimplementation of the vision pipeline, started in 2026, that ports the working ideas from `spin_the_bottle` into modern C++17/CMake so they can be developed and tested without a robot or a ROS install. See [Ongoing and future work](#ongoing-and-future-work-vision_standalone) below.

## The game

Up to 5 players sit in a half-circle facing NAO, with a bottle on the floor between them and the robot. Whoever is directly in front of NAO spins the bottle first. NAO watches it spin, and once it settles:

1. It works out which of 5 regions the bottle is pointing at.
2. It turns its head to look at that region.
3. If it sees a face there, it asks that person to spin next; if not, it asks the group to spin again.

NAO can't turn its neck through the 225°–315° arc behind it, which is exactly why the players sit in a half-circle rather than a full one.

## `spin_the_bottle/` — the ROS package (2015)

### Development steps

The project was scoped as 5 sequential deliverables, built roughly in this order over February–July 2015:

1. **Bottle detection and pointing** (Feb–May) — detect whether a bottle is fully in frame, then compute the center and pointing angle. This step alone went through 4 vision strategies before landing on the shipped one — see [Vision pipeline](#vision-pipeline-what-shipped-and-what-was-tried-first) below.
2. **Movement detection** (Mar, May) — built alongside step 1, since a pointing angle is only meaningful once the bottle has stopped spinning.
3. **Pixel → world coordinate transformation** (Jun) — convert the pointing angle from image-pixel space into NAO head pitch/yaw, so the robot can physically turn to look at it.
4. **People detection** (Jul) — Haar-cascade face detection once NAO's head is turned, to check whether it's actually looking at a person.
5. **NAO speaking messages** (Jul) — the spoken prompts that keep the game moving at each of the above steps (no bottle found, bottle off-center, nobody in the pointed region, etc.).

The sections below describe the system that resulted, not the chronological build order.

### Runtime flow

Everything is driven by one node, `subscriber` (built from [`src/my_subscriber.cpp`](spin_the_bottle/src/my_subscriber.cpp)), which subscribes to NAO's camera topic and runs a state machine per frame:

```
camera/image_raw
      │
      ▼
 PreGame::BottleDetected ──(no bottle / off-center)──▶ AC_SAY asks player to add/center it
      │ (bottle centered)
      ▼
 PreGame::MovementDetected ──(still spinning)──▶ keep waiting
      │ (stopped)
      ▼
 LineProjectionE::FindPointingArea + drawPointingLine  → pointing angle, in pixel space
      │
      ▼
 WorldCoordinates::getWorldCoordinates / getPitchYawHead → NAO head pitch/yaw
      │
      ▼
 ActionNaoNeck (via /joint_trajectory) turns NAO's head, then switches to the top camera
      │
      ▼
 DetectFace::detectAndDisplay (Haar cascade) ──found──▶ AC_SAY: "your turn"
                                              ──not found──▶ AC_SAY: "spin again", restart
```

At startup the node also triggers a Choregraphe behavior (`behaviors/bottlehello`) over `qicli`/`ALBehaviorManager` for NAO's opening greeting, and enables idle "breathing" on the body while disabling it on the head so the head-turn calculations aren't thrown off by idle motion.

### Classes

| Class | File | Responsibility |
|---|---|---|
| `PreGame` | `PreGame.cpp` | Is a bottle in frame and roughly centered? Has it stopped moving? |
| `LineProjectionE` | `LineProjectionE.cpp` | Given a centered, still bottle, find its center and the angle/line it's pointing along. |
| `WorldCoordinates` | `WorldCoordinates.cpp` | Convert that pixel-space angle into the head pitch/yaw NAO needs to look at it. |
| `DetectFace` | `DetectFace.cpp` | Haar-cascade face detection on the top camera once NAO is looking at a region. |
| `ActionClientSay` (`ac_say`) | `ActionClientSay.cpp` | Thin actionlib client wrapping NAO's `/speech_action`. |
| `ActionNaoNeck` | `ActionNaoNeck.cpp` | Thin actionlib client wrapping NAO's `/joint_trajectory` to move the neck. *Written but never wired into `my_subscriber.cpp`* — the subscriber talks to `/joint_trajectory` directly instead. |
| `FaceDetection` | `FaceDetection.cpp` | Wrapper around NAO's own onboard `/nao_vision/faces_detected` topic. Explicitly unused (see file header) — `DetectFace`'s OpenCV Haar cascade was used instead because NAO's built-in detector can't report face count/position. |
| `actionLib` | `ActionLib.h` | Generic actionlib helper; superseded by the two clients above. |

### Vision pipeline: what shipped, and what was tried first

Four bottle-orientation strategies were tried before landing on the one in `LineProjectionE.cpp`:

1. **Canny edges + probabilistic Hough lines + k-means** — worked on high-res phone photos, fell apart on NAO's low-res camera because the bottle's edges became too discontinuous for Hough to find full-length lines.
2. **Grayscale threshold + PCA** — found the bottle's principal axis reliably in low, even light, but glass reflections under bright/artificial light created spurious high-variance regions that threw the principal axis off.
3. **Single green-channel zero-pixel mask** (shadows/reflections read as nearly pure black in the green channel, so masking them out isolates the bottle) **+ PCA** — rejected because PCA's axis then ran from the bottle's occluded base rather than its true center, biasing the angle.
4. **Single green-channel mask + `fitEllipse` + `convexHull` + `fillConvexPoly`** — the shipped approach. The convex hull patches over the parts of the contour occluded by reflections, `fillConvexPoly` gives a clean solid silhouette immune to remaining brightness artifacts, and `fitEllipse` on that silhouette gives a stable center and orientation.

Movement detection similarly went through two approaches: NAO's own onboard motion-detection node never registered the bottle spinning at all (confirmed with `rostopic echo`), so it was replaced with a plain frame-to-frame difference of the green-channel threshold masks, with a small noise floor to absorb camera sensor noise.

Pixel→world coordinates uses a **linear mapping of pixel position to NAO head angles** (calibrated against known camera height and field of view) rather than inverting NAO's camera intrinsics matrix — an intentional accuracy-for-simplicity tradeoff, since the game only needs NAO to look at one of 5 discrete regions, not at an exact point.

### Known gaps

- `DetectFace.cpp` loads its cascade from `ros::package::getPath("bottle_recognition")`, but the package is named `spin_the_bottle` — this path resolution looks like a leftover from an earlier package name/split and would need fixing to actually find the cascade at runtime. It also looks for `lbpcascade_frontalface.xml`, while the cascade actually checked into the repo is `src/haarcascade_frontalface_alt.xml`.
- `ActionNaoNeck` and `FaceDetection` are dead code — present, compiled into the `motion_orientation` library, but never instantiated from `my_subscriber.cpp`.
- The robot hostname (`optimusprime.local`) and the Choregraphe install path are hardcoded in `main()` — expected for a one-robot deployment, but worth knowing if you're pointing this at different hardware.

### Building and running

This is a [catkin](http://wiki.ros.org/catkin) package written against **ROS Indigo** (the `CV_BGR2HSV`-style OpenCV 2 macros and the plain `catkin_make` layout date it precisely). You'll need a NAO-compatible ROS stack that provides:

- `naoqi_msgs` / `naoqi_bridge_msgs` (`JointTrajectoryAction`, `SpeechWithFeedbackAction`, `RunBehaviorAction`)
- `nao_interaction_msgs` (`FaceDetected`)
- `cv_bridge`, `image_transport`, `sensor_msgs`, `std_msgs`, `actionlib`, OpenCV 2.x

```bash
# from a catkin workspace with spin_the_bottle/ under src/
catkin_make
source devel/setup.bash

# with NAO's ROS bridge already running and publishing camera/image_raw:
rosrun spin_the_bottle subscriber
```

Without a real NAO (or a simulator publishing the same topics/actions), the node won't do anything useful — the whole pipeline is driven by the `camera/image_raw` subscription, and the head/speech steps depend on NAO's `naoqi_bridge` action servers being up.

## Ongoing and future work: `vision_standalone/`

`vision_standalone/` re-implements the vision half of the pipeline — bottle detection, movement detection, pointing-angle calculation, and pixel→head-pose conversion — as a **plain C++17/CMake project with no ROS and no catkin dependency**, so it can be built, run and unit-tested on any machine with OpenCV 4, independent of a robot or a ROS Indigo install.

| Module | Ports | Status |
|---|---|---|
| `vision_common` | Shared hue-based bottle mask + convex-hull helper, factored out so `pre_game` and `line_projection` don't duplicate it | Done |
| `pre_game` | `PreGame::BottleDetected` / `MovementDetected` | Done — reworked to threshold on **Hue** (not brightness/saturation) so detection survives lighting changes better than the 2015 version did |
| `line_projection` | `LineProjectionE::FindPointingArea` / `drawPointingLine` | Done — `find_pointing_area`, `compute_pointing_line` and `draw_pointing_line` port the `fitEllipse` + `convexHull` + `fillConvexPoly` approach from `LineProjectionE.cpp` onto the `vision_common` mask |
| `world_coordinates` | `WorldCoordinates::getWorldCoordinates` / `getPitchYawHead` | Done — `to_world_coordinates`, `find_target_point` and `compute_head_pose`, including the 5-region head-yaw snapping from the original |

`src/main.cpp` is a throwaway harness that runs the whole pipeline end-to-end against a synthetic frame (a colored ellipse standing in for a real NAO camera frame) so the modules can be exercised without a camera, printing each stage's output and writing out `saturation_mask.png` / `pointing_line.png` for a visual sanity check.

`pre_game`, `line_projection` and `world_coordinates` each also have a dedicated test binary under `tests/`, wired into `ctest`:

- **`world_coordinates_tests`** — checks against values worked out by hand from the underlying geometry (line/circle intersection, angle bucketing), independent of the image pipeline.
- **`line_projection_tests`** — checks against synthetic shapes with known geometry (an asymmetric "bottle" silhouette, a rotated ellipse), with looser tolerances since `fitEllipse`'s output isn't something derivable by hand.
- **`pre_game_tests`** — checks `bottle_detected`'s threshold buckets against synthetic frames with known bottle-pixel counts, and `movement_detected`'s diff-threshold logic against identical vs. shifted frames. `BottleState::FullFrame` isn't covered: `threshold_mask` always zeroes out a border band, so a mask covering 100% of the frame can't actually happen through the public API.

`vision_common` doesn't have its own test binary — it's exercised indirectly through the `pre_game` and `line_projection` tests.

```bash
cd vision_standalone
mkdir -p build && cd build
cmake ..
cmake --build .
./vision_standalone   # runs the synthetic-frame harness
ctest                 # runs pre_game_tests, world_coordinates_tests and line_projection_tests
```

The bottle-color hue band in `vision_common.cpp` (`kBottleHueLow`/`kBottleHueHigh`) is calibrated for a green bottle, matching the wine/Sprite bottle the original project used — recalibrate it if you use a different colored bottle.

### Future work

Roughly in the order it's expected to happen:

1. **Port face detection** — re-add the Haar-cascade step, this time loading the cascade path correctly (see the [known gap](#known-gaps) in the 2015 code).
2. **Re-integrate with ROS** — now that `pre_game`, `line_projection` and `world_coordinates` are all ported and tested standalone, wire them back into a ROS node (or a newer ROS 2 / non-ROS control layer) that talks to NAO, replacing `my_subscriber.cpp`'s monolithic callback with calls into these modules.
3. **Replace the synthetic-frame harness** — `main.cpp`'s hand-drawn ellipse is a stand-in; once real NAO camera frames (or recordings) are available again, swap them in and add regression tests against them.

## Repository layout

```
spin_the_bottle/                      catkin package (2015, ROS Indigo)
├── include/spin_the_bottle/          class headers
├── src/                              node + class implementations, Haar cascade XML
├── behaviors/bottlehello/            Choregraphe behavior played at game start
├── documentation/Report.pdf          original 2015 project report
└── package.xml, CMakeLists.txt

vision_standalone/                    ROS-free modern rework (2026-)
├── include/                          pre_game.h, line_projection.h, world_coordinates.h, vision_common.h
├── src/                              implementations + throwaway test harness (main.cpp)
├── tests/                            pre_game_tests.cpp, line_projection_tests.cpp, world_coordinates_tests.cpp (wired into ctest)
└── CMakeLists.txt
```
