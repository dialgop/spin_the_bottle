# Spin the Bottle — NAO Robot Game

A ROS package that makes an [Aldebaran/SoftBank NAO](https://en.wikipedia.org/wiki/Nao_(robot)) robot referee a game of spin-the-bottle: it watches a real bottle with its camera, works out where it's pointing once it stops spinning, turns its head to look at that spot, and asks whoever it sees to take their turn.

Originally built by **Diego Alejandro Gómez Pardo** in 2015 as a project for the Humanoid Robotics Praktikum at the University of Bonn. That original ROS (Indigo) / OpenCV package is archived on the [`nao-2015`](../../tree/nao-2015) branch, including the project report. This repo is now a personal, ongoing rework of that project: `vision_standalone/` is a from-scratch, ROS-free reimplementation of the vision pipeline, started in 2026, that ports the working ideas from the 2015 code into modern C++17/CMake so they can be developed and tested without a robot or a ROS install.

## The game

Up to 5 players sit in a half-circle facing NAO, with a bottle on the floor between them and the robot. Whoever is directly in front of NAO spins the bottle first. NAO watches it spin, and once it settles:

1. It works out which of 5 regions the bottle is pointing at.
2. It turns its head to look at that region.
3. If it sees a face there, it asks that person to spin next; if not, it asks the group to spin again.

NAO can't turn its neck through the 225°–315° arc behind it, which is exactly why the players sit in a half-circle rather than a full one.

## Ongoing and future work: `vision_standalone/`

`vision_standalone/` re-implements the vision half of the pipeline — bottle detection, movement detection, pointing-angle calculation, pixel→head-pose conversion, and face detection — as a **plain C++17/CMake project with no ROS and no catkin dependency**, so it can be built, run and unit-tested on any machine with OpenCV 4, independent of a robot or a ROS Indigo install.

| Module | Ports | Status |
|---|---|---|
| `vision_common` | Shared hue-based bottle mask + convex-hull helper, factored out so `pre_game` and `line_projection` don't duplicate it | Done |
| `pre_game` | `PreGame::BottleDetected` / `MovementDetected` | Done — reworked to threshold on **Hue** (not brightness/saturation) so detection survives lighting changes better than the 2015 version did |
| `line_projection` | `LineProjectionE::FindPointingArea` / `drawPointingLine` | Done — `find_pointing_area`, `compute_pointing_line` and `draw_pointing_line` port the `fitEllipse` + `convexHull` + `fillConvexPoly` approach from `LineProjectionE.cpp` onto the `vision_common` mask |
| `world_coordinates` | `WorldCoordinates::getWorldCoordinates` / `getPitchYawHead` | Done — `to_world_coordinates`, `find_target_point` and `compute_head_pose`, including the 5-region head-yaw snapping from the original |
| `face_detection` | `DetectFace::detectAndDisplay` | Done — `load_cascade`/`detect_face` port the Haar-cascade check onto the frame's central half, this time loading the cascade from a compile-time path (`FACE_CASCADE_PATH`, pointing at `data/haarcascade_frontalface_alt.xml`) instead of the broken `ros::package::getPath` lookup the 2015 code used. Validated against real footage - see [Known limitations](#known-limitations) below for a gap that found |

`src/main.cpp` is a throwaway harness that runs the whole pipeline end-to-end, either against a synthetic frame (a colored ellipse standing in for a real NAO camera frame) or, if given a video file path as its first argument, against real recorded frames via `cv::VideoCapture` — printing each stage's output per frame and writing out `saturation_mask.png` / `pointing_line.png` for a visual sanity check. Below is that output from a real recorded frame (`data/bottle_examples/`), not the synthetic ellipse:

| `saturation_mask.png` | `pointing_line.png` |
|---|---|
| ![Hue-based bottle mask isolating a real bottle's silhouette](vision_standalone/docs/saturation_mask_example.png) | ![Fitted ellipse and pointing-direction arrow drawn on a real bottle photo](vision_standalone/docs/pointing_line_example.png) |

`pre_game`, `line_projection`, `world_coordinates` and `face_detection` each also have a dedicated test binary under `tests/`, wired into `ctest`:

- **`world_coordinates_tests`** — checks against values worked out by hand from the underlying geometry (line/circle intersection, angle bucketing), independent of the image pipeline.
- **`line_projection_tests`** — checks against synthetic shapes with known geometry (an asymmetric "bottle" silhouette, a rotated ellipse), with looser tolerances since `fitEllipse`'s output isn't something derivable by hand.
- **`pre_game_tests`** — checks `bottle_detected`'s threshold buckets against synthetic frames with known bottle-pixel counts, and `movement_detected`'s diff-threshold logic against identical vs. shifted frames. `BottleState::FullFrame` isn't covered: `threshold_mask` always zeroes out a border band, so a mask covering 100% of the frame can't actually happen through the public API.
- **`face_detection_tests`** — checks `load_cascade`'s success/failure return value against a missing file and the real checked-in cascade, and checks that `detect_face` returns `nullopt` both with no cascade loaded and against a blank frame. It also runs against real recorded footage in `data/faces_examples/`: a synthetic AI-generated human face ([thispersondoesnotexist.com](https://thispersondoesnotexist.com/)), a dog, and an empty background ("Small cacti with a white wall background" via [rawpixel.com](https://www.rawpixel.com/)) — confirming a real face is actually found, and that an empty background isn't a false positive. See [Known limitations](#known-limitations) below for what this footage found.

`vision_common` doesn't have its own test binary — it's exercised indirectly through the `pre_game` and `line_projection` tests.

```bash
cd vision_standalone
mkdir -p build && cd build
cmake ..
cmake --build .
./vision_standalone              # runs the synthetic-frame harness
./vision_standalone path/to.mp4  # runs the pipeline against a real video file instead
ctest                             # runs pre_game_tests, line_projection_tests, world_coordinates_tests and face_detection_tests
```

The bottle-color hue band in `vision_common.cpp` (`kBottleHueLow`/`kBottleHueHigh`) is calibrated for a green bottle, matching the wine/Sprite bottle the original project used — recalibrate it if you use a different colored bottle.

### Known limitations

Running `face_detection` against real recorded footage (see `face_detection_tests` above) surfaced a real weakness: the Haar cascade (`haarcascade_frontalface_alt.xml`) **false-positives on a dog's face** in `data/faces_examples/Dog_happy_nao.mp4`. This isn't a bug in `detect_face`'s own logic — it's a known limitation of Haar cascades in general, whose gradient-based features can match a front-on animal face closely enough to trigger. This is deliberately left as a documented, un-asserted case in `face_detection_tests` rather than worked around, since the real fix is a better classifier (see Future work below).

### Future work

Roughly in the order it's expected to happen:

1. **Replace `face_detection`'s Haar cascade with an ONNX-based model** — motivated directly by the dog false-positive above; a proper CNN-based face detector (e.g. OpenCV's YuNet, run via its ONNX model) should be far more discriminating between human and animal faces. Planned as a separate follow-up merge, not bundled with the real-footage testing work.
2. **Re-integrate with ROS** — now that `pre_game`, `line_projection`, `world_coordinates` and `face_detection` are all ported and tested standalone, wire them back into a ROS node (or a newer ROS 2 / non-ROS control layer) that talks to NAO.

## Repository layout

```
vision_standalone/                    ROS-free modern rework (2026-)
├── include/                          pre_game.h, line_projection.h, world_coordinates.h, vision_common.h, face_detection.h
├── src/                              implementations + throwaway test harness (main.cpp)
├── tests/                            pre_game_tests.cpp, line_projection_tests.cpp, world_coordinates_tests.cpp, face_detection_tests.cpp (wired into ctest)
├── data/                             haarcascade_frontalface_alt.xml (cascade used by face_detection)
│   ├── bottle_examples/              real recorded bottle-spin videos, for use with ./vision_standalone path/to.mp4
│   └── faces_examples/               real face/dog/background videos, used by face_detection_tests
└── CMakeLists.txt
```