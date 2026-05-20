# LIO-SAM Config Drift Remediation Design

## Context

The dataset `/home/ubuntu20/下载/2026-03-27-16-55-59.bag` must be treated with
the pose/extrinsic reference from
`/home/ubuntu20/LIOSAM_ws/src/LIO-SAM-master/config/params.yaml`, not with the
FAST-LIO2 MID360 config.

The bag exposes raw LiDAR as `/livox/lidar` with type
`livox_ros_driver2/CustomMsg` and IMU as `/livox/imu`. The existing DO-CT-LIO
runtime subscribed only to `sensor_msgs/PointCloud2`, rejected `lidar_type: 1`,
and its default config carried Gazebo/Scout-specific output offsets, Z
references, and command priors. Those defaults are overfit to the previous
Gazebo validation bags and must not be used for this dataset.

## Root-Cause Hypotheses

1. Raw LiDAR was not ingested correctly because the online node had no
   Livox CustomMsg subscriber.
2. Runs that used `/map_scan` were using a derived mapping topic rather than
   raw sensor scans, so scan time and frame semantics were wrong for CT-LIO.
3. The bag IMU acceleration norm is about `1 g`; DO-CT-LIO propagation expects
   `m/s^2`, so acceleration must be scaled by the LIO-SAM gravity value
   `9.80511`.
4. Previous Gazebo offsets and motion priors can make the earlier validation
   pass while damaging generalization on this LIO-SAM-referenced dataset.

## Design

- Add a minimal `livox_ros_driver2` message package inside `DO_CT_LIO_ws` so the
  Docker build is self-contained.
- Extend `CloudConvert` with a Livox CustomMsg path. Convert `offset_time`
  nanoseconds to seconds, preserve reflectivity/line, compute `alpha_time` in
  the same frame time base, and keep the existing PointCloud2 paths unchanged.
- Update `ct_lio_eskf` so `lidar_type: 1` subscribes to raw
  `livox_ros_driver2/CustomMsg`; other LiDAR types continue using PointCloud2.
- Allow `run_eskf.launch` to pass a `config_yaml` private parameter.
- Add `config/liosam_livox.yaml` using LIO-SAM extrinsic translation,
  identity rotation, gravity `9.80511`, acceleration scale `9.80511`,
  `/livox/lidar`, and `/livox/imu`.
- Do not enable Gazebo output offsets, command velocity priors, fixed world-Z
  references, or MID360 extrinsics in this config.
- Keep 12D continuous-time optimization and 12D Hessian diagnostics. Projection
  remains disabled.

## Validation

- Build inside Docker image `do-ct-lio-noetic:local` with host networking
  available for runtime.
- Run unit tests:
  - `test_cloud_convert_livox`
  - `test_do_ct_diagnostics`
- Run the target bag with Docker algorithm and host-side rosbag playback.
- Use `/tf` (`camera_init -> aft_mapped`) and `/chatter_LD360` only as available
  in-bag references unless a true ground-truth topic is found. Do not claim
  factual 2 cm ground-truth accuracy without a validated ground-truth source.
