# SLAM Map Evaluation Protocol — `rtabmap_drone_pkg`

How to turn *"the map looks about right in the GCS"* into a repeatable, numeric
pass/fail for this project's **2.5 cm stereo-inertial occupancy grid**.

Every threshold below is checked automatically by
[`scripts/diagnostics/map_eval.py`](../scripts/diagnostics/map_eval.py) against a run
captured by [`scripts/diagnostics/map_recorder.py`](../scripts/diagnostics/map_recorder.py).
`map_eval.py` exits non-zero when a graded metric is outside limits, so it can gate a
release the same way a test suite does.

> **Why this exists.** `gotalldone.md`'s own "known, unaddressed loose ends" records that
> no real flight has validated the map and that drift has never been measured. Nothing in
> this repo could previously answer *"is the map good?"* with a number. This closes that.

---

## 0. What is being evaluated

| Product | Producer | Artifact | Format |
|---|---|---|---|
| **Raw 2D occupancy grid** | RTAB-Map `Grid/*` (`launch/rtabmap_slam.launch.py`) | `map.npy` + `map.png` | int8 `(H, W)`, ROS row-major |
| **Wall skeleton** | `scripts/map_thinning_node.py` | `map_thin.npy` + `map_thin.png` | int8 `(H, W)`, single-pixel walls |
| **Trajectory** | RTAB-Map `stereo_odometry` → `/odom` | `pose.csv` | `t_s,x,y,z,yaw_deg,cov_xx,lost` |

### Grid encoding — read this before writing any analysis

This is **`nav_msgs/OccupancyGrid`, not a log-odds grid.** There is no sigmoid step.

| Stored value | Meaning |
|---|---|
| `-1` | unknown — never observed |
| `0 … 100` | occupancy probability, in percent |

```python
import numpy as np
grid = np.load("out_mapping/run_20260918_1204/map.npy")   # int8 (H, W)

unknown  = grid < 0
occupied = grid >= 65      # --occ-thresh
free     = (~unknown) & (grid <= 25)   # --free-thresh
# 26..64 is observed-but-undecided and belongs to NONE of the three.
```

Folding the undecided band into `free` is the single easiest way to make a map look
safer than it is, which is why `map_eval.py` keeps it as its own class.

**Orientation.** Artifacts are stored ROS row-major: row indexes Y, column indexes X,
origin at `meta.json`'s `origin_x/origin_y`. Cell coordinates read straight off `map.png`
are the `COL,ROW` that `map_eval.py`'s `--landmark` / `--wall` / `--corner` flags expect.
*(The GCS's `ros2_map_listener.py` transposes to `(W, H)` for rendering — that is a
display convention only and does not apply here.)*

### Pipeline parameters the metrics depend on

From `launch/rtabmap_slam.launch.py` — if these change, the evaluation changes with them:

| Parameter | Value | Consequence for evaluation |
|---|---|---|
| `Grid/CellSize` | `0.025` | 1 cell = 2.5 cm; wall-straightness limits are in cells |
| `Grid/RangeMin` / `RangeMax` | `0.3` / `3.5` | Coverage is only scored within **3.5 m of the flown path** (`--sweep-radius-m`) |
| `Grid/MinObstacleHeight` | `0.30` | **Anything below 30 cm is invisible by design** — exclude it from obstacle-recall ground truth or you will fail a working map |
| `Grid/MaxObstacleHeight` | `2.0` | Same, above 2 m |
| `Grid/NormalsSegmentation` | `true` | Ground plane is filtered out; floor is *not* an obstacle |
| `Rtabmap/DetectionRate` | `2.0` | Map updates at ~2 Hz; a run shorter than ~30 s has too few updates to judge |

---

## 1. Fix the test set first

No number means anything until the inputs are frozen. For each recorded run, log in
`--notes` (it lands in `meta.json`):

- **Sensor + profile** — D435i, stereo IR resolution, whether IMU was fused.
- **Trajectory type** — *loop* (returns to start), *out-and-back*, *straight corridor*,
  *exploratory*. Drift behaves differently in each; one run of each is the minimum set.
- **Motion** — walked/flown, speed, whether yaw rotations were included.
- **Scene** — lighting, texture, dynamic objects (people walking through).

> Capture at least **one loop**, **one out-and-back** and **one long straight run** per
> environment. Keep them as a frozen set and re-run them after every SLAM change —
> a threshold only earns trust once you have seen it move.

### Ground truth, cheapest first

1. **Loop-closure self-consistency** — if the path returns to its start, the start↔end
   gap *is* the drift. **No external ground truth needed.** Start here.
2. **Tape/laser-measured landmark distances** — two features you can identify in the
   grid, and the real distance between them.
3. **Floor plan / CAD** — wall positions and room dimensions to overlay.
4. **Reference scan** — LiDAR or photogrammetry, if you ever get access to one.

---

## 2. Capturing a run

On the Radxa, with the pipeline already up (`./camera.sh`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rtabmap_drone_pkg map_recorder.py --notes "loop, lab, slow walk, lights on"
# ... fly or walk the planned trajectory ...
# Ctrl-C to finish and write the artifacts
```

Recording is read-only: it subscribes to `/map`, `/map_thin` and `/odom` and never
publishes or commands anything, so it is safe to run during a real flight.

**Before the trajectory starts**, do the standard tracking-init move — a slow yaw rotation
until VIO locks. `gotalldone.md` dev log #25 notes heading-estimate stability does not
survive a tracking reset, so this has to happen on *every* run, not once per boot. A run
that begins with lost odometry produces a map that fails for a reason that has nothing to
do with the mapping code.

---

## 3. Metrics, thresholds and what a failure means

Run everything the recorded artifacts support:

```bash
./scripts/diagnostics/map_eval.py out_mapping/run_20260918_1204 \
    --loop --true-path-m 38.5 \
    --landmark 412,300 412,460 --landmark-true-m 4.00 \
    --wall 400,290 425,470 \
    --corner 400,290 425,470 600,300 780,325 \
    --json report.json
```

### 3.1 Geometric accuracy

| Metric | How it is computed | Limit | A failure means |
|---|---|---|---|
| **Scale error** | Distance between two landmark cells (`--landmark`) vs. surveyed truth | **≤ 2 %** | The metric scale of VIO is wrong — stereo baseline/calibration, or IMU scale |
| **Wall straightness** | Total-least-squares line fit through occupied cells in `--wall`; RMS perpendicular residual | **≤ 1 cell** (2.5 cm) | Pose noise is being baked into geometry; walls bend where the trajectory wobbled |
| **Squareness** | Angle between two fitted walls at a known 90° corner (`--corner`) | **±3°** | Yaw drift within a single room — worse than it looks, since it compounds per loop |

Straightness and squareness are measured on **`map_thin`** when present: the skeleton is
one cell wide, so residuals describe wall *geometry* rather than wall *thickness*.

### 3.2 Trajectory / drift

| Metric | How it is computed | Limit | A failure means |
|---|---|---|---|
| **Loop-closure gap** | `‖pose[last] − pose[0]‖ ÷ path length`, on a `--loop` run | **≤ 2 %** of path | Accumulated VIO drift. The single most informative number here |
| **Yaw drift** | `yaw[last] − yaw[0]`, wrapped to (−180, 180] | **≤ 5°** | Heading estimate is walking; expect squareness to fail too |
| **Path scale error** | Odometry path length vs. `--true-path-m` | **≤ 3 %** | Consistent over/under-estimation of travel — a scale problem, not a noise problem |

### 3.3 VO health — *why* a map is good or bad

These grade the process, not the product. A map that fails §3.1 with a healthy §3.3 is a
mapping problem; a map that fails with an unhealthy §3.3 is a tracking problem, and no
amount of grid tuning will fix it.

| Metric | Source | Limit |
|---|---|---|
| **VIO lost fraction** | `cov_xx ≥ 9999.0` in `/odom` — RTAB-Map's own "tracking lost" sentinel (dev log #6) | **< 5 %** of samples |
| **Inlier ratio** | `inliers ÷ matches`, when captured | **≥ 0.50** sustained |

### 3.4 Occupancy quality

| Metric | How it is computed | Limit | Why |
|---|---|---|---|
| **Completeness** | Cells classified (`occupied ∨ free`) within `--sweep-radius-m` of the path | **≥ 90 %** | Holes inside sensor range mean the grid is not integrating what the camera saw |
| **Free-space precision** | Of cells marked free, the fraction truly traversable (`--gt-traversable`) | **≥ 95 %** | **Asymmetric on purpose** — a false *free* cell flies the drone into a wall; a false *occupied* cell only costs a detour |
| **Obstacle recall** | Of real obstacles, the fraction marked occupied (`--gt-obstacle`) | **≥ 90 %** | Build the ground-truth mask from obstacles **in the 0.30–2.0 m height band only** |

Coverage is deliberately scored only near the flown path. Scoring the whole grid would
punish the map for the far side of a wall it never flew past, which says nothing about
SLAM quality.

### 3.5 Qualitative, recorded but not graded

- **Ghosting / smearing** — people walking through the scene leaving occupied trails.
  Count them and note it; there is no automatic check.
- **Double walls** — the classic drift signature: the same wall mapped twice, offset.
  If you see it, §3.2 should already have failed. If it did not, the run was too short.

---

## 4. Reading the report

```
METRIC                         VALUE  UNIT             LIMIT  RESULT
------------------------------------------------------------------------------
scale_error                    1.240  %              <= 2     PASS
wall_straightness              0.640  cells          <= 1     PASS
squareness_error               4.100  deg            <= 3     FAIL
completeness                  93.800  %              >= 90    PASS
loop_closure_gap               3.400  % of path      <= 2     FAIL
yaw_drift                      6.200  deg            <= 5     FAIL
vo_lost                        1.100  % of samples   <= 5     PASS
```

Read it top-down as a causal chain, not as seven independent verdicts. The example above
is one fault, not three: yaw drift → squareness error → loop-closure gap, with healthy VO
underneath. The fix is in heading estimation (IMU fusion, or the yaw-init procedure), not
in the occupancy grid.

`SKIP` means the input was not supplied — e.g. `scale_error` without `--landmark`. Skips
are not failures, but a run where everything skipped has proven nothing.

---

## 5. Regression use

Freeze a run directory, commit its `report.json`, and re-run `map_eval.py` after any
change to:

- `launch/rtabmap_slam.launch.py` (`Grid/*`, `Reg/*`, `Optimizer/*`)
- `launch/stereo_inertial_odom.launch.py` (VIO parameters)
- `scripts/map_thinning_node.py` (skeletonisation, noise purge)
- the D435i profile in `launch/d435i_stereo_imu.launch.py`

Because `map_eval.py` is pure numpy — no rclpy, no Qt — it runs anywhere, including CI
against a synthetic grid (see `tests/test_map_eval.py`).

---

## 6. Honest limits of this protocol

- **The geometry metrics need you to identify landmarks by hand.** There is no automatic
  wall detector here; `--wall` and `--corner` take boxes you read off `map.png`. That is a
  deliberate trade: an automatic detector would need its own validation.
- **Loop-closure gap flatters a stationary run.** A drone that barely moved has a tiny gap
  and a tiny path length; always read it beside `path_length`.
- **Ground-truth masks are the expensive part.** Free-space precision and obstacle recall
  are the two metrics that genuinely need surveying work. Everything else is free.
- **This grades the map, not the flight.** A map can pass every threshold here and still
  be unsafe to fly if `obstacle_distance_bridge.py` is not wired into the launch file
  (currently it is not — see `gotalldone.md` dev log #17).
