# DO-CT-LIO Development Plan

Date: 2026-05-19

## Objective

Create DO-CT-LIO in `/home/ubuntu20/DO_CT_LIO_ws`, derived from
`/home/ubuntu20/CT_LIO_ws`, with final validation using a 12D
continuous-time Hessian weighting and degeneracy protection algorithm.

## Hard Requirements

- Algorithm runs inside Docker.
- Gazebo and rosbag playback run on the host.
- Docker uses host networking: `--net=host`.
- Final validation uses `/ground_truth/odom` from both validation bags.
- Final accepted metrics for each bag:
  - raw 3D ATE RMSE `<= 0.05 m`
  - raw 3D ATE P95 `<= 0.05 m`
- 6D Hessian is diagnostic only and cannot be final.
- First 12D Hessian implementation is logging-only.
- 12D Jacobian finite-difference validation is required before weighting.

## Stages

1. Stage 0: Create workspace, Docker runtime, Gazebo launch, and evaluator.
   Status: complete.
2. Stage 1: Build and run baseline CT-LIO, record both bag metrics.
   Status: complete; baseline failed.
3. Stage 2: Add logging-only 6D Hessian diagnostics.
   Status: complete.
4. Stage 3: Add logging-only 12D Hessian diagnostics with no output changes.
   Status: complete.
5. Stage 4: Add finite-difference validation for 12D Jacobian.
   Status: complete.
6. Stage 5: Enable 12D residual weighting.
   Status: complete.
7. Stage 6: Add spatial and `alpha_time` bucketed residual selection.
   Status: complete.
8. Stage 7: Add continuous degenerate-direction prior boost.
   Status: complete.
9. Stage 8: Add scaled-space post-solve projection only if required.
   Status: helper implemented and tested; final runtime disabled because tests
   showed projection worsened raw accuracy.
10. Stage 9: Validate both bags and iterate until the 5 cm target passes.
    Status: complete; both specified bags pass raw 3D RMSE/P95 <= 0.05 m.
11. Stage 10: Remediate drift on `/home/ubuntu20/下载/2026-03-27-16-55-59.bag`
    using the LIO-SAM reference config, not the MID360 config.
    Status: complete for Docker raw-Livox remediation and endpoint-distance
    reporting; no ground-truth accuracy claim is made for this no-GT bag.

## Final Runtime Configuration

- Algorithm runs inside Docker image `do-ct-lio-noetic:local`.
- Rosbag/Gazebo run on the host.
- Docker must use `--net=host --ipc=host`.
- Final Gazebo launch:
  `/home/ubuntu20/gazebo/src/gazebo_bringup/launch/scout_do_ct_lio.launch`.
- Final evaluation command reads `/ground_truth/odom` from the bag and compares
  raw 3D trajectory without alignment.

## References

- Design spec: `docs/superpowers/specs/2026-05-19-do-ct-lio-design.md`
- Implementation plan: `docs/superpowers/plans/2026-05-19-do-ct-lio-implementation.md`
- LIO-SAM drift remediation design:
  `docs/superpowers/specs/2026-05-20-liosam-config-drift-remediation-design.md`
- LIO-SAM drift remediation plan:
  `docs/superpowers/plans/2026-05-20-liosam-config-drift-remediation-implementation.md`
