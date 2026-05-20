# LIO-SAM Config Drift Remediation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make DO-CT-LIO run the `2026-03-27-16-55-59.bag` dataset from raw LiDAR/IMU using the LIO-SAM reference configuration instead of MID360 or Gazebo-overfit settings.

**Architecture:** Add self-contained Livox CustomMsg support, select the LiDAR subscriber from YAML `lidar_type`, and isolate the LIO-SAM-referenced runtime in a separate config/launch file. Preserve the existing Gazebo validation config.

**Tech Stack:** ROS Noetic, catkin, C++17, Livox custom ROS messages, Docker image `do-ct-lio-noetic:local`.

---

### Task 1: Self-Contained Livox Message Package

**Files:**
- Create: `src/livox_ros_driver2/CMakeLists.txt`
- Create: `src/livox_ros_driver2/package.xml`
- Create: `src/livox_ros_driver2/msg/CustomMsg.msg`
- Create: `src/livox_ros_driver2/msg/CustomPoint.msg`
- Modify: `src/ct_lio/CMakeLists.txt`
- Modify: `src/ct_lio/package.xml`
- Modify: `src/ct_lio/cmake/packages.cmake`

- [x] Add the minimal message package matching bag type `livox_ros_driver2/CustomMsg`.
- [x] Add `livox_ros_driver2` as a catkin dependency of `ct_lio`.
- [x] Build in Docker and confirm generated headers are available.

### Task 2: Livox Cloud Conversion

**Files:**
- Modify: `src/ct_lio/src/preprocess/cloud_convert/cloud_convert.h`
- Modify: `src/ct_lio/src/preprocess/cloud_convert/cloud_convert.cc`
- Create: `src/ct_lio/src/apps/test_cloud_convert_livox.cpp`
- Modify: `src/ct_lio/src/apps/CMakeLists.txt`

- [x] Add `CloudConvert::Process(const livox_ros_driver2::CustomMsg::ConstPtr&, ...)`.
- [x] Convert `offset_time` from nanoseconds to seconds.
- [x] Preserve `reflectivity`, `line`, per-point timestamp, `relative_time`, and `alpha_time`.
- [x] Add a unit test for Livox timestamp conversion.
- [x] Run `test_cloud_convert_livox` inside Docker.

### Task 3: Runtime Subscriber And Config Selection

**Files:**
- Modify: `src/ct_lio/src/apps/main_eskf.cpp`
- Modify: `src/ct_lio/launch/run_eskf.launch`
- Create: `src/ct_lio/launch/run_liosam_livox.launch`

- [x] Use private param `~config_yaml` or `--config_yaml` instead of always using `config/mapping.yaml`.
- [x] Subscribe to `livox_ros_driver2::CustomMsg` when `preprocess/lidar_type: 1`.
- [x] Subscribe to `sensor_msgs::PointCloud2` for the existing non-Livox paths.
- [x] Add `imu_initialization.acceleration_scale` and apply it in `imuHandler`.
- [x] Skip `/cmd_vel` subscription when the topic is empty.
- [x] Build `ct_lio_eskf` inside Docker.

### Task 4: LIO-SAM Referenced Dataset Config

**Files:**
- Create: `src/ct_lio/config/liosam_livox.yaml`

- [x] Use `/livox/lidar` and `/livox/imu`.
- [x] Use LIO-SAM extrinsic translation `[-0.173896233, -0.110653711, -0.249088050]`.
- [x] Use identity extrinsic rotation.
- [x] Use gravity and acceleration scale `9.80511`.
- [x] Disable Gazebo output offsets, command priors, and fixed world-Z reference.
- [x] Run the target bag with this config in Docker.

### Task 5: Validation And Cleanup

**Files:**
- Modify: `DEVELOPMENT_LOG.md`
- Modify: `DEVELOPMENT_PLAN.md`
- Modify: `EVALUATION_RESULTS.md`
- Modify: `SELF_REVIEW.md`

- [x] Record build and test evidence.
- [x] Run the target bag and record endpoint pose metrics.
- [x] Remove temporary bags, CSVs, diagnostics, and logs produced during tests.
- [x] Record only the test results and self-review in development docs.
