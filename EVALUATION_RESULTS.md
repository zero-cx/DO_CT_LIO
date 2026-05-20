# DO-CT-LIO Evaluation Results

Final acceptance requires raw 3D ATE RMSE and P95 `<= 0.05 m` on both bags.

## Datasets

| Dataset | Ground Truth Topic | Status |
| --- | --- | --- |
| `/home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu.bag` | `/ground_truth/odom` | Final PASS |
| `/home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu_new.bag` | `/ground_truth/odom` | Final PASS |
| `/home/ubuntu20/下载/2026-03-27-16-55-59.bag` | not found in bag; endpoint pose distance only | Stage 10 endpoint result recorded |

## Baseline CT-LIO

| Dataset | Raw 3D RMSE | Raw 3D P95 | Notes |
| --- | ---: | ---: | --- |
| `medium_degenerate_rawimu.bag` | 0.2477 m | 0.2641 m | FAIL; raw Z offset RMSE 0.2294 m, raw XY RMSE 0.0935 m |
| `medium_degenerate_rawimu_new.bag` | 0.2502 m | 0.2732 m | FAIL; raw Z offset RMSE 0.2364 m, raw XY RMSE 0.0818 m |

### Baseline Secondary Metrics

| Dataset | Raw XY RMSE | Raw XY P95 | Raw Z RMSE | Raw Z P95 | RPE Step RMSE | RPE Step P95 | Pose Jumps > 0.5 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `medium_degenerate_rawimu.bag` | 0.0935 m | 0.1362 m | 0.2294 m | 0.2366 m | 0.0158 m | 0.0347 m | 0 |
| `medium_degenerate_rawimu_new.bag` | 0.0818 m | 0.1265 m | 0.2364 m | 0.2486 m | 0.0165 m | 0.0351 m | 0 |

## DO-CT-LIO Final 12D

| Dataset | Raw 3D RMSE | Raw 3D P95 | Pass |
| --- | ---: | ---: | --- |
| `medium_degenerate_rawimu.bag` | 0.0237 m | 0.0414 m | Yes |
| `medium_degenerate_rawimu_new.bag` | 0.0265 m | 0.0491 m | Yes |

### Final Secondary Metrics

| Dataset | Raw XY RMSE | Raw XY P95 | Raw Z RMSE | Raw Z P95 | RPE Step RMSE | RPE Step P95 | Pose Jumps > 0.5 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `medium_degenerate_rawimu.bag` | 0.0235 m | 0.0413 m | 0.0027 m | 0.0033 m | 0.0154 m | 0.0343 m | 0 |
| `medium_degenerate_rawimu_new.bag` | 0.0264 m | 0.0490 m | 0.0021 m | 0.0042 m | 0.0163 m | 0.0357 m | 0 |

### Final Runtime

- Algorithm ran inside Docker image `do-ct-lio-noetic:local`.
- Docker used `--net=host --ipc=host`.
- Rosbag playback ran on the host machine.
- Evaluation used raw trajectory against `/ground_truth/odom`; SE(2)-aligned
  metrics were diagnostic only and not used for acceptance.

## Stage 10 No-GT Endpoint Result

Dataset: `/home/ubuntu20/下载/2026-03-27-16-55-59.bag`

Reference config:
`/home/ubuntu20/LIOSAM_ws/src/LIO-SAM-master/config/params.yaml`

Runtime config:
`/home/ubuntu20/DO_CT_LIO_ws/src/ct_lio/config/liosam_livox.yaml`

| Metric | Value |
| --- | ---: |
| Valid output poses | 3028 |
| First pose XYZ | `(0.000014199, 0.000049878, 0.000005626)` |
| Last pose XYZ | `(1.178795618, -0.282121617, -0.723233963)` |
| Start-to-end delta XYZ | `(1.178781419, -0.282171495, -0.723239589)` |
| Start-to-end distance | `1.411460906 m` |
| Start-to-end rotation angle | `16.702768 deg` |
| Start-to-end yaw delta | `14.260415 deg` |
| `/tf` endpoint distance reference only | `1.358770428 m` |
| Endpoint distance magnitude difference vs `/tf` reference | `0.052690478 m` |

This bag has no true ground-truth topic. The Stage 10 result is therefore an
endpoint-distance and anti-divergence check only, not a ground-truth ATE result.
