# `libs/` — third‑party SLAM / optimization / visualization

This directory contains **vendored third‑party libraries** used by this repository’s SLAM stack. It is **not** a ROS 2 package/workspace by itself (note the `COLCON_IGNORE` file) — the ROS 2 code that *consumes* these libraries lives under `slam/src/`.

The most important integration is: **Tello camera frames → ORB‑SLAM2 wrapper node → map/pose outputs for visualization & downstream control**.

---

## What’s in here

### `libs/ORB_SLAM2/`
An in-tree fork of **ORB‑SLAM2** (monocular SLAM) with its own third‑party dependencies:

- **DBoW2** (`libs/ORB_SLAM2/Thirdparty/DBoW2`): bag‑of‑words place recognition for relocalization and loop detection.
- **g2o** (`libs/ORB_SLAM2/Thirdparty/g2o`): nonlinear least squares optimization (bundle adjustment, pose graph, Sim(3) alignment).

In this repo, ORB‑SLAM2 is built as a shared library (`libORB_SLAM2.so`) and then linked into a ROS 2 wrapper node (`slam/src/orbslam2`).

Key upstream modules you will see referenced throughout the algorithm:

- `include/System.h` / `src/System.cc`: high-level orchestration, threads, public API (`TrackMonocular`, shutdown, trajectory saving).
- `include/Tracking.h` / `src/Tracking.cc`: per-frame tracking + relocalization logic.
- `include/LocalMapping.h` / `src/LocalMapping.cc`: keyframe insertion, map point creation/culling, local bundle adjustment.
- `include/LoopClosing.h` / `src/LoopClosing.cc`: loop detection + correction + pose graph optimization / global BA triggers.
- `include/Optimizer.h` / `src/Optimizer.cc`: g2o formulations and solvers for BA / pose graph steps.
- `include/ORBextractor.h` / `src/ORBextractor.cc`: multi-scale ORB feature extraction.
- `include/ORBmatcher.h` / `src/ORBmatcher.cc`: descriptor matching (Hamming distance) and geometric checks.
- `include/Sim3Solver.h` / `src/Sim3Solver.cc`: Sim(3) estimation for loop closure scale/rotation/translation alignment.
- `include/Initializer.h` / `src/Initializer.cc`: monocular initialization (two-view geometry + triangulation).

### `libs/g2o/`
A standalone checkout of g2o. Some parts of this repo link against g2o indirectly via ORB‑SLAM2’s own Thirdparty copy; other packages may prefer the standalone version.

### `libs/Pangolin/`
Pangolin is a lightweight OpenGL visualization / UI library frequently used by academic SLAM projects (including upstream ORB‑SLAM2) for real‑time map rendering.

In **this repository’s build**, ORB‑SLAM2 is compiled **without** its Pangolin viewer (the file `libs/ORB_SLAM2/src/Viewer.cc` includes Pangolin headers, but it is not part of the library sources listed in `libs/ORB_SLAM2/CMakeLists.txt`). Instead, visualization is done via ROS 2 topics published by the wrapper node (annotated frames + RViz markers).

---

## How `libs/` integrates with the ROS 2 code

### Build-time integration (linking the wrapper against ORB‑SLAM2)

The ROS 2 wrapper package lives at `slam/src/orbslam2/` and builds an executable called `mono`.

- **Discovery**: `slam/src/orbslam2/CMakeModules/FindORB_SLAM2.cmake` looks for:
  - a project root containing ORB‑SLAM2 “Thirdparty” headers (so relative includes work), and
  - an include directory containing `System.h` (or `ORB_SLAM2/System.h`), and
  - a library called `libORB_SLAM2.*`

- **Control knob**: `slam/src/orbslam2/CMakeLists.txt` has `option(ORB_SLAM2_ENABLE ...)`.
  - If ORB‑SLAM2 is **not** found, it prints a warning and **skips** building the wrapper node so the overall workspace can still build.

- **Expected environment variable**: set `ORB_SLAM2_ROOT_DIR` to the ORB‑SLAM2 directory you want to link against (often `.../libs/ORB_SLAM2` after you build it).

#### Building ORB‑SLAM2 from `libs/ORB_SLAM2`

If you want to build the in-tree ORB‑SLAM2 so the ROS 2 wrapper can link against it:

```bash
# From the repo root
cd libs/ORB_SLAM2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j

# Point the ROS2 wrapper at this build
export ORB_SLAM2_ROOT_DIR="$PWD/.."
```

Notes:

- The ORB‑SLAM2 library output is configured to land under `libs/ORB_SLAM2/lib/` (see `libs/ORB_SLAM2/CMakeLists.txt`).
- `FindORB_SLAM2.cmake` also searches common locations like `build/lib` / `lib64`, so either an in-tree build or a separate install can work as long as headers + library are discoverable.

### Runtime integration (topic wiring)

At runtime, the dataflow is:

- `workspace/src/tello/tello/node.py` publishes:
  - camera frames on `image_raw`
  - camera intrinsics on `camera_info`
  - IMU on `imu` (orientation + linear acceleration)

- The ORB‑SLAM2 wrapper node (`slam/src/orbslam2/src/monocular/monocular-slam-node.cpp`) subscribes to an image topic named `camera` and calls:
  - `ORB_SLAM2::System::TrackMonocular(image, timestamp)`

There are two launch files in-tree, and they differ in naming/remapping:

- `workspace/src/tello/launch/tello.launch.py`: publishes the camera on the standard `/image_raw`.
- `workspace/src/launch.py`: remaps `('/image_raw', '/camera')` for the driver (so downstream nodes can subscribe to `/camera`).

If you run the SLAM wrapper, ensure your launch/remaps make its subscription (`camera`) receive the Tello frames (either publish to `camera`, or remap `camera:=/image_raw`).

---

## ORB‑SLAM2: the algorithm and pipeline (deep dive)

ORB‑SLAM2 is a **feature‑based monocular SLAM system**. Conceptually it builds and maintains a sparse 3D map (keyframes + 3D points) while estimating the camera trajectory. Internally it is a multi-thread pipeline:

- **Tracking** (real-time, every frame): estimate the current camera pose \(T_{cw}\) and decide when to create a new keyframe.
- **Local mapping** (background): triangulate new 3D points, cull bad points/keyframes, run local bundle adjustment.
- **Loop closing** (background): detect revisits, estimate a Sim(3) loop constraint, correct drift, and optimize the pose graph.

### 1) Frame ingest and feature extraction

**Where in code**

- Wrapper calls `ORB_SLAM2::System::TrackMonocular(...)` from `slam/src/orbslam2/src/monocular/monocular-slam-node.cpp`.
- ORB feature extraction is implemented in `libs/ORB_SLAM2/include/ORBextractor.h` and `libs/ORB_SLAM2/src/ORBextractor.cc`.

**What happens**

- The image is converted to grayscale (internally) and processed with an image pyramid.
- ORB keypoints are detected per pyramid level (FAST-like corner detector) and described with a rotated BRIEF-style binary descriptor.
- Matching uses **Hamming distance** on binary descriptors, which is fast and robust for real‑time.

**Why it matters**

- The choice of ORB features is a performance/robustness trade-off: fast enough for CPU-only real-time, but less robust than learned features in extreme lighting/texture conditions.

### 2) Monocular initialization (bootstrap)

**Where in code**

- `libs/ORB_SLAM2/include/Initializer.h` / `libs/ORB_SLAM2/src/Initializer.cc`
- invoked from tracking when the system state is `NOT_INITIALIZED` (`libs/ORB_SLAM2/include/Tracking.h` / `src/Tracking.cc`).

**What happens**

Monocular SLAM starts with **two-view geometry**:

- Find feature correspondences between an initial reference frame and a new frame.
- Hypothesize a geometric model:
  - **Homography** (planar / low-parallax) vs
  - **Fundamental / essential matrix** (general 3D motion)
- Recover relative pose \((R, t)\) up to scale and triangulate initial 3D points.

**Key constraint**

- With a single camera, absolute scale is unobservable. ORB‑SLAM2 establishes an arbitrary initial scale; trajectories/maps are **up to a similarity transform** (rotation, translation, scale).

### 3) Tracking (pose estimation every frame)

**Where in code**

- `libs/ORB_SLAM2/include/Tracking.h` / `libs/ORB_SLAM2/src/Tracking.cc`
- descriptor matching logic: `libs/ORB_SLAM2/include/ORBmatcher.h` / `src/ORBmatcher.cc`

**What happens**

Once initialized, each incoming frame goes through:

- **Pose prediction** (motion model): predict \(T_{cw}\) from the last state to reduce search.
- **Map point projection + matching**:
  - project local map points into the current frame using the predicted pose,
  - match descriptors within a search window, reject outliers.
- **Pose optimization**:
  - solve a PnP-like problem and refine pose by minimizing reprojection error of matched points.
  - robust kernels reduce the effect of outliers.

**Relocalization path (when tracking is LOST)**

- Use DBoW2 to retrieve candidate keyframes by appearance.
- Perform geometric verification (PnP + inlier checks).
- If successful, tracking resumes with a recovered pose.

### 4) Keyframes, map points, and local mapping

**Where in code**

- `libs/ORB_SLAM2/include/LocalMapping.h` / `libs/ORB_SLAM2/src/LocalMapping.cc`
- map structures:
  - `libs/ORB_SLAM2/include/KeyFrame.h` / `src/KeyFrame.cc`
  - `libs/ORB_SLAM2/include/MapPoint.h` / `src/MapPoint.cc`
  - `libs/ORB_SLAM2/include/Map.h` / `src/Map.cc`

**What happens**

When tracking decides a new keyframe is needed:

- Insert a **KeyFrame** into the map and update the covisibility graph (which keyframes share observations).
- Create new **MapPoints** by triangulating matches between the new keyframe and neighboring keyframes.
- **Culling**:
  - Remove recently created points that are poorly observed or inconsistent.
  - Remove redundant keyframes to keep the map compact.

**Local Bundle Adjustment (Local BA)**

The local mapping thread runs a nonlinear optimization over:

- camera poses for a window of local keyframes, and
- the 3D positions of map points observed by them,

minimizing reprojection error. This is where g2o does most of the heavy lifting.

### 5) Loop closing and global consistency

**Where in code**

- `libs/ORB_SLAM2/include/LoopClosing.h` / `libs/ORB_SLAM2/src/LoopClosing.cc`
- place recognition DB:
  - `libs/ORB_SLAM2/include/KeyFrameDatabase.h` / `src/KeyFrameDatabase.cc`
  - vocabulary: `libs/ORB_SLAM2/include/ORBVocabulary.h`
- similarity alignment:
  - `libs/ORB_SLAM2/include/Sim3Solver.h` / `src/Sim3Solver.cc`

**What happens**

- DBoW2 proposes loop candidates (visually similar keyframes).
- Geometric verification computes a **Sim(3)** transform (rotation + translation + scale) between current keyframe and loop candidate.
  - Sim(3) is used because monocular SLAM accumulates scale drift; loop closure must correct scale as well.
- If accepted, the system:
  - adds loop constraints,
  - performs **pose graph optimization** to distribute correction through the trajectory,
  - updates/corrects map points and keyframes to enforce global consistency.

### 6) Optimization engine (g2o)

**Where in code**

- `libs/ORB_SLAM2/include/Optimizer.h` / `libs/ORB_SLAM2/src/Optimizer.cc`
- g2o types used by ORB‑SLAM2 live under `libs/ORB_SLAM2/Thirdparty/g2o/...`

**Mental model**

ORB‑SLAM2 repeatedly solves problems of the form:

\[
\min_{\mathbf{x}} \sum_i \rho\left(\left\lVert \mathbf{r}_i(\mathbf{x}) \right\rVert^2\right)
\]

- \(\mathbf{x}\) are the states (camera poses, 3D points, sometimes Sim(3) nodes).
- \(\mathbf{r}_i\) are reprojection residuals (predicted pixel − observed pixel).
- \(\rho\) is a robust loss (Huber-like) to downweight mismatches/outliers.

g2o provides:

- graph representation (vertices = states, edges = residuals),
- sparse linear solvers,
- iterative nonlinear solvers (Gauss‑Newton / Levenberg‑Marquardt).

---

## What the ROS 2 wrapper publishes (what you can consume)

The monocular wrapper node (`slam/src/orbslam2/src/monocular/monocular-slam-node.cpp`) publishes:

- `annotated_frame` (`sensor_msgs/Image`): the incoming image with tracked features overlaid (useful for debugging tracking quality).
- `ORB_SLAM_map` (`visualization_msgs/Marker`): RViz markers representing:
  - sparse map points,
  - reference points,
  - (optionally) keyframes / graphs (stubs exist, some publishers are currently empty).

Internally the wrapper pulls:

- current tracking state via `m_SLAM->GetTrackingState()`,
- the “current frame” via `m_SLAM->GetCurrentFrame()`,
- map structures via `GetAllKeyFrames()`, `GetAllMapPoints()`, `GetReferenceMapPoints()`.

---

## Practical notes / gotchas (engineer-facing)

- **Timestamping**: the wrapper currently passes `msg->header.stamp.sec` (integer seconds) into `TrackMonocular(...)`. For accurate motion model and time-based logic, ORB‑SLAM2 typically expects higher-resolution timestamps (floating seconds). If you see degraded tracking stability, this is one of the first integration points to fix.
- **Monocular scale**: pose/map are only defined up to a similarity transform. If your downstream control needs metric scale, you need an external scale source (stereo, depth, IMU fusion, known-size fiducials, barometer constraints, etc.).
- **IMU topic is not fused**: ORB‑SLAM2 (as included here) is visual-only. The Tello IMU is published by the driver, but the SLAM pipeline does not currently use it (no VIO/IMU preintegration in ORB‑SLAM2).
- **Calibration matters**: camera intrinsics/distortion parameters strongly affect feature reprojection error. Ensure `camera_info` matches the actual stream resolution and lens distortion.

