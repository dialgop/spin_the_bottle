# Spin the Bottle — NAO Robot Game (2015 ROS package)

A ROS package that makes an [Aldebaran/SoftBank NAO](https://en.wikipedia.org/wiki/Nao_(robot)) robot referee a game of spin-the-bottle: it watches a real bottle with its camera, works out where it's pointing once it stops spinning, turns its head to look at that spot, and asks whoever it sees to take their turn.

Originally built by **Diego Alejandro Gómez Pardo** in 2015 as a project for the Humanoid Robotics Praktikum at the University of Bonn — the original report is in [`spin_the_bottle/documentation/Report.pdf`](spin_the_bottle/documentation/Report.pdf).

This branch, `nao-2015`, is a frozen archive of that original ROS (Indigo) / OpenCV package as it ran on the physical robot. Ongoing development has moved to a ROS-free C++17/CMake rework — see the [`master`](../../tree/master) branch's `vision_standalone/`.

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

## Repository layout

```
spin_the_bottle/                      catkin package (2015, ROS Indigo)
├── include/spin_the_bottle/          class headers
├── src/                              node + class implementations, Haar cascade XML
├── behaviors/bottlehello/            Choregraphe behavior played at game start
├── documentation/Report.pdf          original 2015 project report
└── package.xml, CMakeLists.txt
```
