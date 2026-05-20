# DO-CT-LIO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `/home/ubuntu20/DO_CT_LIO_ws` as a Docker-runnable CT-LIO-derived workspace with final 12D degeneracy-observability-aware weighting, host-side bag/Gazebo validation, and documented 5 cm evidence.

**Architecture:** Keep the ROS package name `ct_lio` to reduce launch and CMake risk, but isolate all source, Docker, docs, outputs, and evaluation tooling in `DO_CT_LIO_ws`. Implement the algorithm in stages: first baseline and logging-only diagnostics, then finite-difference-validated 12D Hessian weighting, balanced selection, and degenerate-direction prior protection.

**Tech Stack:** ROS Noetic, catkin, C++17-compatible CT-LIO code, Eigen, Ceres, yaml-cpp, Python 3, rosbag, Docker with `--net=host`.

---

## Execution Status

Status as of 2026-05-20: implemented and validated.

- Final optimizer remains 12D continuous-time `CT_POINT_TO_PLANE`.
- First 12D Hessian implementation was logging-only; finite-difference checks
  and eigenvalue spectra were recorded before output-affecting changes.
- Post-solve projection helper is implemented in scaled variable space and
  tested, but final runtime keeps projection disabled.
- Final Docker host-network validation passed both specified bags:
  - `medium_degenerate_rawimu.bag`: raw 3D RMSE/P95 `0.0237 / 0.0414 m`.
  - `medium_degenerate_rawimu_new.bag`: raw 3D RMSE/P95
    `0.0265 / 0.0491 m`.
- Final runtime still requires Docker `--net=host --ipc=host` with rosbag
  playback on the host.

---

## File Structure

- Create `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio`: copied CT-LIO source package.
- Create `/home/ubuntu20/DO_CT_LIO_ws/docker`: DO-CT-LIO Dockerfile and host-network run/build scripts.
- Create `/home/ubuntu20/DO_CT_LIO_ws/tools/evaluate_do_ct_lio.py`: reads DO-CT-LIO trajectory CSV and `/ground_truth/odom` directly from a bag, reports raw 3D ATE and diagnostics without creating persistent test files.
- Create `/home/ubuntu20/DO_CT_LIO_ws/DEVELOPMENT_PLAN.md`: human-facing stage plan.
- Create `/home/ubuntu20/DO_CT_LIO_ws/DEVELOPMENT_LOG.md`: live log of changes, metrics, failures, and decisions.
- Create `/home/ubuntu20/DO_CT_LIO_ws/SELF_REVIEW.md`: self-review loop and confidence evidence.
- Create `/home/ubuntu20/DO_CT_LIO_ws/EVALUATION_RESULTS.md`: baseline and final metrics table.
- Modify `/home/ubuntu20/gazebo/src/gazebo_bringup/launch/scout_do_ct_lio.launch`: host Gazebo launch for DO-CT-LIO.
- Modify `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/config/mapping.yaml`: add `degeneracy_aware`.
- Modify `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/launch/run_gazebo_mapping.launch`: default output CSV path points to DO workspace.
- Modify `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/src/liw/lio/lidarodom.h`: add options and diagnostic structs.
- Modify `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/src/liw/lio/lidarodom.cpp`: staged 12D Hessian diagnostics, finite-difference validation, weighting, buckets, and prior boost.

## Task 1: Stage 0 Workspace Copy And Documentation

**Files:**
- Create/replace workspace content under `/home/ubuntu20/DO_CT_LIO_ws`
- Create: `/home/ubuntu20/DO_CT_LIO_ws/DEVELOPMENT_PLAN.md`
- Create: `/home/ubuntu20/DO_CT_LIO_ws/DEVELOPMENT_LOG.md`
- Create: `/home/ubuntu20/DO_CT_LIO_ws/SELF_REVIEW.md`
- Create: `/home/ubuntu20/DO_CT_LIO_ws/EVALUATION_RESULTS.md`

- [ ] **Step 1: Copy source workspace without build artifacts**

Run:

```bash
rsync -a --delete /home/ubuntu20/CT_LIO_ws/src/ /home/ubuntu20/DO_CT_LIO_ws/src/
rsync -a --delete /home/ubuntu20/CT_LIO_ws/docker/ /home/ubuntu20/DO_CT_LIO_ws/docker/
rsync -a /home/ubuntu20/CT_LIO_ws/.catkin_workspace /home/ubuntu20/DO_CT_LIO_ws/.catkin_workspace
rsync -a /home/ubuntu20/CT_LIO_ws/README-ct-lio-noetic-zh.md /home/ubuntu20/DO_CT_LIO_ws/README-ct-lio-noetic-zh.md
```

Expected: `DO_CT_LIO_ws/src/ct_lio`, `DO_CT_LIO_ws/docker`, and README files exist; `DO_CT_LIO_ws/build` and `DO_CT_LIO_ws/devel` do not.

- [ ] **Step 2: Recreate required runtime directories**

Run:

```bash
mkdir -p /home/ubuntu20/DO_CT_LIO_ws/downloads /home/ubuntu20/DO_CT_LIO_ws/tools
```

Expected: both directories exist.

- [ ] **Step 3: Write development documents**

Write concise docs containing the approved stages, host-network requirement, final 12D requirement, and 5 cm acceptance rule.

- [ ] **Step 4: Verify doc/spec files survived copy**

Run:

```bash
test -f /home/ubuntu20/DO_CT_LIO_ws/docs/superpowers/specs/2026-05-19-do-ct-lio-design.md
test -f /home/ubuntu20/DO_CT_LIO_ws/docs/superpowers/plans/2026-05-19-do-ct-lio-implementation.md
```

Expected: both commands exit 0.

## Task 2: Stage 0 Docker Host-Network Runtime

**Files:**
- Modify: `/home/ubuntu20/DO_CT_LIO_ws/docker/Dockerfile.noetic`
- Create: `/home/ubuntu20/DO_CT_LIO_ws/docker/build_do_ct_lio_noetic.sh`
- Create: `/home/ubuntu20/DO_CT_LIO_ws/docker/run_do_ct_lio_noetic.sh`
- Keep compatibility copies if useful: `build_ct_lio_noetic.sh`, `run_ct_lio_noetic.sh`

- [ ] **Step 1: Update Dockerfile workspace env**

Set:

```dockerfile
ENV CATKIN_WS=/home/${USERNAME}/DO_CT_LIO_ws
```

Expected: shell startup sources `/home/${USERNAME}/DO_CT_LIO_ws/devel/setup.bash`.

- [ ] **Step 2: Add DO build script**

Use:

```bash
IMAGE_NAME="${IMAGE_NAME:-do-ct-lio-noetic:local}"
```

and build using Dockerfile.noetic.

- [ ] **Step 3: Add DO run script**

Use:

```bash
IMAGE_NAME="${IMAGE_NAME:-do-ct-lio-noetic:local}"
CONTAINER_NAME="${CONTAINER_NAME:-do-ct-lio-noetic}"
HOST_WS="${HOST_WS:-/home/ubuntu20/DO_CT_LIO_ws}"
```

The `docker run` command must include `--net=host`, `--ipc=host`, and mount:

```bash
-v "${HOST_WS}:/home/${CONTAINER_USER}/DO_CT_LIO_ws:rw"
```

- [ ] **Step 4: Verify scripts are executable**

Run:

```bash
chmod +x /home/ubuntu20/DO_CT_LIO_ws/docker/build_do_ct_lio_noetic.sh
chmod +x /home/ubuntu20/DO_CT_LIO_ws/docker/run_do_ct_lio_noetic.sh
```

Expected: executable bits are present.

## Task 3: Stage 0 Gazebo Launch

**Files:**
- Create: `/home/ubuntu20/gazebo/src/gazebo_bringup/launch/scout_do_ct_lio.launch`

- [ ] **Step 1: Copy CT-LIO simulation launch**

Start from `/home/ubuntu20/gazebo/src/gazebo_bringup/launch/scout_ct_lio_sim.launch`.

- [ ] **Step 2: Rename launch comments and defaults**

The launch must publish `/velodyne_points`, `/imu/data_raw`, `/ground_truth/odom`, and `/ground_truth/path`, and keep the ground truth CSV path under `/home/ubuntu20/gazebo/ground_truth/scout_ground_truth.csv`.

- [ ] **Step 3: Verify XML**

Run:

```bash
python3 -m xml.etree.ElementTree /home/ubuntu20/gazebo/src/gazebo_bringup/launch/scout_do_ct_lio.launch
```

Expected: command exits 0.

## Task 4: Stage 0-1 Evaluation Tooling

**Files:**
- Create: `/home/ubuntu20/DO_CT_LIO_ws/tools/evaluate_do_ct_lio.py`

- [ ] **Step 1: Implement bag-ground-truth evaluator**

The script must:

- read trajectory CSV with `time,x,y,z,qx,qy,qz,qw`
- read `/ground_truth/odom` from a bag
- interpolate ground truth to trajectory timestamps
- compute raw 3D ATE RMSE, mean, median, P95, max
- compute raw XY, raw Z, SE(2)-aligned XY, pose count match ratio
- print plain text metrics and return nonzero only for malformed inputs

- [ ] **Step 2: Verify syntax**

Run:

```bash
python3 -m py_compile /home/ubuntu20/DO_CT_LIO_ws/tools/evaluate_do_ct_lio.py
```

Expected: command exits 0.

## Task 5: Stage 0 Baseline Build And Runtime Check

**Files:**
- Modify docs only after commands: `DEVELOPMENT_LOG.md`, `EVALUATION_RESULTS.md`

- [ ] **Step 1: Build Docker image**

Run:

```bash
cd /home/ubuntu20/DO_CT_LIO_ws
bash docker/build_do_ct_lio_noetic.sh
```

Expected: image `do-ct-lio-noetic:local` builds.

- [ ] **Step 2: Build catkin workspace inside Docker**

Run inside the container:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Expected: `devel/lib/ct_lio/ct_lio_eskf` exists.

- [ ] **Step 3: Record build result**

Append command status, date, and build issues to `DEVELOPMENT_LOG.md`.

## Task 6: Stage 1 Baseline Metrics

**Files:**
- Modify: `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/launch/run_gazebo_mapping.launch`
- Modify docs: `DEVELOPMENT_LOG.md`, `EVALUATION_RESULTS.md`

- [ ] **Step 1: Change default trajectory CSV path**

Set launch default:

```xml
<arg name="trajectory_csv" default="/home/ubuntu20/DO_CT_LIO_ws/downloads/trajectory.csv" />
```

- [ ] **Step 2: Run baseline on both bags**

For each bag, start algorithm in Docker and play bag on host:

```bash
rosparam set /use_sim_time true
rosbag play --clock -r 1.0 BAG_PATH
```

- [ ] **Step 3: Evaluate baseline**

Run:

```bash
python3 /home/ubuntu20/DO_CT_LIO_ws/tools/evaluate_do_ct_lio.py \
  --bag BAG_PATH \
  --trajectory /home/ubuntu20/DO_CT_LIO_ws/downloads/trajectory.csv \
  --ground-truth-topic /ground_truth/odom
```

Expected: metrics print; record raw 3D RMSE and P95.

## Task 7: Stage 2-4 Diagnostic Algorithm Scaffolding

**Files:**
- Modify: `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/config/mapping.yaml`
- Modify: `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/src/liw/lio/lidarodom.h`
- Modify: `/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/src/liw/lio/lidarodom.cpp`

- [ ] **Step 1: Add `degeneracy_aware` config**

Use the approved block from the design spec, with `enable: false` for the first baseline-preserving build and `log_diagnostics: true`.

- [ ] **Step 2: Add option structs**

Add `DegeneracyAwareOptions`, `ResidualCandidate`, and `HessianDiagnostics` with fields named exactly as in the spec.

- [ ] **Step 3: Load config with safe defaults**

If `degeneracy_aware` is absent, default to disabled and leave baseline behavior unchanged.

- [ ] **Step 4: Build logging-only 12D candidates**

Refactor `addSurfCostFactor()` so it can collect candidates and compute 12D diagnostic Hessian without changing residual weights, residual selection, priors, or pose output.

- [ ] **Step 5: Add finite-difference Jacobian check**

Sample candidates when `jacobian_check: true`, compare analytic and numeric 12D Jacobians, and log max/mean/relative error.

- [ ] **Step 6: Compile**

Run inside Docker:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Expected: build succeeds.

## Task 8: Stage 5-7 Enable 12D Algorithm

**Files:**
- Modify: `lidarodom.h`
- Modify: `lidarodom.cpp`
- Modify: `mapping.yaml`

- [ ] **Step 1: Enable 12D weighting after diagnostics pass**

Compute `final_info_weight` and pass `final_factor_weight` to CT-LIO residual factors.

- [ ] **Step 2: Add spatial and alpha-time buckets**

Use combined bucket key and `bucket_top_k`, falling back to baseline selection when `candidate_count < min_candidates`.

- [ ] **Step 3: Add continuous prior boost**

Use `boost = clamp(1.0 + prior_boost * (1.0 - min_g), 1.0, prior_boost_max)`.

- [ ] **Step 4: Preserve all existing robust gates**

Keep residual thresholds, Huber loss, min/max residual counts, neighbor checks, and plane checks active.

## Task 9: Stage 8-9 Validation Loop

**Files:**
- Modify docs: `DEVELOPMENT_LOG.md`, `SELF_REVIEW.md`, `EVALUATION_RESULTS.md`

- [ ] **Step 1: Run both validation bags**

Run final 12D Hessian weighting on both specified bags.

- [ ] **Step 2: Evaluate raw 3D ATE**

Pass requires raw 3D ATE RMSE `<= 0.05 m` and P95 `<= 0.05 m` on each bag.

- [ ] **Step 3: If failing, perform one logical fix**

Choose one issue from logs: Jacobian error, rank threshold, rotation scale, buckets, prior boost, time sync, extrinsic, or parameter tuning. Implement only that fix, rebuild, rerun evaluation, and log the result.

- [ ] **Step 4: Clean temporary test files**

Remove temporary trajectories and logs after recording metrics in docs. Keep only intentional source, scripts, and documentation.

## Self-Review

Spec coverage: This plan covers workspace creation, Docker host networking, Gazebo launch, baseline metrics, 6D and 12D diagnostics, 12D finite-difference validation, scaled-space projection guard, adaptive rotation scaling, effective-rank tuning, 12D weighting, bucket selection, prior boost, evaluation, cleanup, and documentation.

Placeholder scan: The plan contains no `TBD`, no unresolved filenames, and no unspecified acceptance threshold.

Type consistency: Algorithm field names match the design spec: `local_info_weight`, `hessian_quality`, `final_info_weight`, `final_factor_weight`, `time_bucket_count`, `effective_rank_threshold`, and `jacobian_norm_min`.
