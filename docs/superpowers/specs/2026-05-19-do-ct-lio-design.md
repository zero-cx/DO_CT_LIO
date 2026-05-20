# DO-CT-LIO Design Spec

Date: 2026-05-19

## Goal

Build a new CT-LIO-derived algorithm workspace at `/home/ubuntu20/DO_CT_LIO_ws`.
The new algorithm must implement degeneracy-observability-aware continuous-time
LiDAR-inertial odometry based on the strategy described in
`/home/ubuntu20/下载/preview.html`.

The final accepted algorithm must use a 12D continuous-time Hessian over begin
and end poses. A 6D Hessian implementation may be used only as an intermediate
diagnostic and ablation stage; it is not an accepted final algorithm.

## Source And Runtime Boundaries

- Source reference workspace: `/home/ubuntu20/CT_LIO_ws`
- New workspace: `/home/ubuntu20/DO_CT_LIO_ws`
- Gazebo workspace: `/home/ubuntu20/gazebo`
- Gazebo launch to add: `/home/ubuntu20/gazebo/src/gazebo_bringup/launch/scout_do_ct_lio.launch`
- Validation bags:
  - `/home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu.bag`
  - `/home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu_new.bag`

The algorithm runs inside Docker. Gazebo and rosbag playback run on the host.
Docker must use host networking (`--net=host`) so ROS topics, ROS master, bag
playback, and Gazebo connect through the host network. This is required because
the current Docker networking is unreliable outside host networking.

## Acceptance Criteria

Each validation bag must be evaluated against `/ground_truth/odom`.

Required final threshold:

- Raw 3D ATE RMSE: `<= 0.05 m`
- Raw 3D ATE P95: `<= 0.05 m`

Evaluation also records SE(2)-aligned trajectory error, RPE, max error, pose
jump count, runtime, and failure symptoms. These secondary metrics do not
replace the 5 cm acceptance requirement.

## Recommended Development Route

Use a staged, evidence-driven route:

1. Clone the CT-LIO workspace layout into `DO_CT_LIO_ws`, preserving package
   names where doing so reduces ROS and CMake risk.
2. Add Docker files based on `CT_LIO_ws/docker`, with image/container/workspace
   names changed to DO-CT-LIO.
3. Add the Gazebo launch file `scout_do_ct_lio.launch`, mirroring
   `scout_ct_lio_sim.launch` but naming the target algorithm clearly.
4. Add a validation workflow that runs the algorithm in Docker and plays bags
   on the host over host networking.
5. Build and evaluate the unmodified CT-LIO baseline on both bags.
6. Implement 6D Hessian diagnostics only as a scaffold.
7. Implement logging-only 12D Hessian diagnostics.
8. Implement and pass 12D Jacobian finite-difference validation.
9. Implement the final 12D continuous-time Hessian weighting and protection.
10. Iterate until both validation bags pass the 5 cm threshold.

## Algorithm Architecture

The main source landing zone is CT-LIO's LiDAR odometry path:

- `src/ct_lio/src/liw/lio/lidarodom.cpp`
- `src/ct_lio/src/liw/lio/lidarodom.h`
- `src/ct_lio/src/liw/lidarFactor.h`
- `src/ct_lio/config/mapping.yaml`

The current CT-LIO flow generates point-to-plane factors in
`addSurfCostFactor()` and later adds them to Ceres in `optimize()`.
DO-CT-LIO changes this into a two-stage flow:

1. Generate residual candidates.
2. Build a Hessian from candidate Jacobians.
3. Analyze observability by Hessian eigendecomposition.
4. Reweight each residual by its contribution to observable directions.
5. Select residuals with spatially and temporally balanced buckets.
6. Add weighted residual blocks to Ceres.
7. Strengthen prior protection in detected degenerate directions.

## Residual Candidate Model

Each candidate stores:

- raw LiDAR point
- transformed world point
- plane normal
- plane offset
- closest/reference point
- `alpha_time`
- local information weight
- point-to-plane residual distance
- 12D continuous-time Jacobian
- Hessian contribution quality
- final information weight
- final Ceres factor weight
- spatial bucket key
- time bucket key
- combined bucket key for balanced selection

The 6D candidate fields can exist during diagnostics, but final weighting must
use the 12D Jacobian.

Naming must distinguish meanings that CT-LIO currently overloads as `weight`:

- `local_info_weight`
- `hessian_quality`
- `final_info_weight`
- `final_factor_weight`

The Ceres cost factor receives `final_factor_weight`, which is the square root
of `final_info_weight`.

## Final 12D Hessian Model

For a continuous-time point-to-plane residual:

```text
r_i = n_i^T ( R(alpha_i) p_i + t(alpha_i) - q_i )
```

The final Jacobian is:

```text
J_ct = [J_begin_t, J_begin_r, J_end_t, J_end_r]
```

Translation terms are split by interpolation weights:

```text
J_begin_t = (1 - alpha) n^T
J_end_t   = alpha n^T
```

Rotation terms are first implemented with the same interpolation weighting over
the instantaneous point-to-plane rotation Jacobian:

```text
J_rot = -n^T R(alpha) [p]_x
J_begin_r = (1 - alpha) J_rot
J_end_r   = alpha J_rot
```

This approximation is allowed only if it passes numerical finite-difference
validation. If validation fails or the final metric cannot pass, the rotation
Jacobian is upgraded to a stricter slerp chain-rule derivative.

The 12D Hessian is:

```text
H12 = sum_i local_info_weight_i * J_ct_i^T * J_ct_i
```

Rotation columns must be scaled by a configurable scene/lever-arm scale before
eigendecomposition so translation and rotation units do not distort the
observability estimate.

The default scaling mode is adaptive:

```text
rotation_scale = clamp(median(||p_i||), rotation_scale_min, rotation_scale_max)
```

If adaptive scaling is disabled, use the fixed `rotation_scale` value. Apply the
scale to rotation Jacobian columns before building the Hessian:

```text
J_begin_r_scaled = J_begin_r / rotation_scale
J_end_r_scaled   = J_end_r   / rotation_scale
```

Diagnostics must log the chosen `rotation_scale`.

## 12D Jacobian Validation

Before final evaluation trusts 12D Hessian eigenvectors, implement a diagnostic
finite-difference check comparing analytic `J_ct` with numerical `J_ct` on
sampled residual candidates.

Log:

- maximum absolute error
- mean absolute error
- relative error
- candidate `alpha_time`
- residual value
- analytic Jacobian norm
- numeric Jacobian norm

If the analytic and finite-difference Jacobians are inconsistent, do not enable
final Hessian weighting. Upgrade the rotation derivative to the strict slerp
chain-rule form and rerun the check.

## Observability Weighting

The Hessian eigenvalues are normalized by the average Hessian energy:

```text
lambda_norm_k = lambda_k / (trace(H12) / 12 + eps)
```

Each direction receives a smooth observability score:

```text
g_k = lambda_norm_k / (lambda_norm_k + lambda0)
```

Each residual receives a Hessian directional quality:

```text
q_i = sqrt(
  sum_k g_k * (J_i v_k)^2 /
  (sum_k (J_i v_k)^2 + eps)
)
```

The final Ceres residual factor receives:

```text
factor_weight = sqrt(clamp(local_info_weight * q_i^gamma, omega_min, omega_max))
```

The square root is required because the existing cost factors multiply the
residual and Jacobian by the supplied factor weight.

For implementation, use the clearer names:

```text
final_info_weight =
  clamp(local_info_weight * pow(q_i, gamma), omega_min, omega_max)

final_factor_weight = sqrt(final_info_weight)
```

Optionally limit frame-to-frame information-weight variation with
`max_weight_delta` to avoid discontinuous optimization behavior.

Diagnostics must log:

- eigenvalues
- normalized eigenvalues
- minimum normalized eigenvalue
- maximum normalized eigenvalue
- condition number
- effective rank
- trace
- rotation scale
- candidate count
- selected residual count
- alpha-time histogram

Degeneracy should not be classified from one scalar alone. At minimum, log both
minimum normalized eigenvalue and effective rank. The recommended trigger is:

```text
min_normalized_eigenvalue < degeneracy_threshold
and effective_rank < effective_rank_threshold
```

When computing directional quality, use the same scaled 12D Jacobian basis used
for eigendecomposition. If `||J_i|| < jacobian_norm_min`, discard the candidate
or assign a safe minimum quality; do not let numerical noise dominate.

`q_i` means contribution to currently observable directions. It does not mean a
point is inherently good or bad, and weak-direction residuals must not be set to
zero. `omega_min` preserves a small amount of information.

Existing CT-LIO robustness checks must remain in force:

- nearest-neighbor search checks
- plane fitting quality checks
- point-to-plane residual threshold
- local planarity weighting
- distance weighting
- Huber loss
- IMU and motion priors
- existing minimum and maximum residual count behavior

Hessian quality is an added modulation, not a replacement for residual gating.

## Spatial Balance

Residuals must not be selected only by global weight ranking. That can
concentrate points on one wall, one spatial region, or one scan-time interval
and worsen degeneracy.

Final selection uses combined spatial and temporal buckets:

- Bucket size is configurable.
- Time bucket count is configurable and based on `alpha_time`.
- The combined key is `spatial_bucket + time_bucket`.
- Each combined bucket keeps up to `bucket_top_k` high-quality candidates.
- The final residual count respects CT-LIO's existing `max_num_residuals` and
  `min_num_residuals` behavior.

If `candidate_count < min_candidates`, do not enable Hessian weighting for that
iteration. Fall back to baseline CT-LIO residual selection and log insufficient
candidates.

## Degenerate Direction Protection

If the minimum normalized 12D eigenvalue falls below the configured degeneracy
threshold, the system must not only lower LiDAR residual weights. It must also
protect weakly constrained directions using priors already compatible with
CT-LIO:

- begin pose location consistency
- begin pose orientation consistency
- small velocity or constant velocity prior
- optional post-solve subspace projection if Ceres priors are insufficient

The first implementation should adjust prior strengths in `optimize()` based on
the Hessian diagnostics. Prior boost must be continuous rather than a hard
switch:

```text
boost = 1.0 + prior_boost * (1.0 - min_g)
boost = clamp(boost, 1.0, prior_boost_max)
```

Do not blindly over-boost all priors; excessive prior strength can suppress real
motion and make the trajectory sluggish.

If the 5 cm target is not reached, add a post-solve 12D subspace correction:

```text
delta_x_final =
  V G V^T delta_x_lidar +
  V (I - G) V^T delta_x_prior
```

Strong observable directions follow LiDAR optimization. Weak directions rely
more on IMU or motion prior.

If projection uses eigenvectors from the scaled Hessian, projection must happen
in the same scaled variable space. Convert raw pose increments `delta_x` to
scaled increments `delta_z`, apply `V G V^T` in scaled space, then convert the
projected `delta_z` back to raw `delta_x`. Do not directly apply scaled-space
eigenvectors to unscaled pose increments.

## Configuration

Add a `degeneracy_aware` block to `mapping.yaml`:

```yaml
degeneracy_aware:
  enable: true
  diagnostic_6d: false
  log_diagnostics: true
  jacobian_check: true
  lambda0: 0.08
  gamma: 0.7
  omega_min: 0.15
  omega_max: 1.5
  rotation_scale_mode: adaptive
  rotation_scale: 8.0
  rotation_scale_min: 2.0
  rotation_scale_max: 20.0
  bucket_size: 0.8
  bucket_top_k: 6
  time_bucket_count: 4
  min_candidates: 120
  degeneracy_threshold: 0.05
  effective_rank_threshold: 8
  prior_boost: 2.0
  prior_boost_max: 4.0
  max_weight_delta: 0.25
  jacobian_norm_min: 1.0e-8
```

`diagnostic_6d` exists only for development and ablation. Final validation must
run with 12D Hessian weighting enabled.

`effective_rank_threshold: 8` is an initial value only. During baseline and
diagnostic stages, log eigenvalue spectra and tune this threshold from the
observed rank distribution; do not assume 8 is universally correct.

## Docker And Launch Design

Docker files are copied from CT-LIO and renamed:

- image: `do-ct-lio-noetic:local`
- container: `do-ct-lio-noetic`
- mounted workspace: `/home/ubuntu20/DO_CT_LIO_ws`
- Docker network: host mode

The container launch pattern remains:

```bash
cd /home/ubuntu20/DO_CT_LIO_ws
bash docker/run_do_ct_lio_noetic.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ct_lio run_gazebo_mapping.launch rviz:=true
```

Host-side bag playback remains outside Docker:

```bash
source /opt/ros/noetic/setup.bash
rosparam set /use_sim_time true
rosbag play --clock -r 1.0 /home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu.bag
```

The Gazebo launch file `scout_do_ct_lio.launch` mirrors the CT-LIO launch and
publishes:

- `/velodyne_points`
- `/imu/data_raw`
- `/ground_truth/odom`
- `/ground_truth/path`

## Evaluation Workflow

For each iteration:

1. Build inside Docker.
2. Start the algorithm in Docker with host networking.
3. Play one validation bag on the host.
4. Save trajectory CSV under `DO_CT_LIO_ws/downloads`.
5. Compare against `/ground_truth/odom`.
6. Record only metrics, failure symptoms, and decisions in development docs.
7. Remove temporary test files after results are recorded.

The final workflow must run both validation bags.

Before trusting raw ATE, verify:

- algorithm output frame
- ground-truth frame
- initial pose offset
- timestamp synchronization
- extrinsic consistency
- pose count match ratio

## Development Documentation

Maintain these files under `DO_CT_LIO_ws`:

- `DEVELOPMENT_PLAN.md`
- `DEVELOPMENT_LOG.md`
- `SELF_REVIEW.md`
- `EVALUATION_RESULTS.md`
- `docs/superpowers/specs/2026-05-19-do-ct-lio-design.md`

The development log must record:

- date/time
- code or parameter change
- dataset used
- metrics
- whether the 5 cm criterion passed
- suspected failure cause
- next fix
- self-review confidence

## Risk Register

The strategy is not considered reliable until the risks below are handled by
test evidence.

| Risk | Failure Mode | Mitigation |
| --- | --- | --- |
| 12D Jacobian approximation is too coarse | Wrong observability directions | Upgrade rotation derivative to slerp chain-rule form |
| Rotation/translation scaling is wrong | False strong or weak eigenvalues | Tune `rotation_scale`, log eigenvalue spectra |
| High Hessian contribution comes from bad matches | Dynamic or wrong points dominate | Preserve residual threshold and Huber loss |
| Bucket selection removes too many points | Too few residuals or sparse geometry | Enforce minimum residual count and fallback to baseline selection |
| Lower LiDAR weights worsen drift | Weak directions become unconstrained | Boost priors and add post-solve projection if needed |
| Alpha-time clustering causes false 12D degeneracy | One endpoint appears weak because selected points cluster in scan time | Use alpha-time buckets and log alpha histograms |
| Docker networking breaks ROS topics | No data reaches algorithm | Always use `--net=host`, host-side bag playback |
| Evaluation alignment hides drift | False pass | Primary metric is raw 3D ATE, not only SE(2)-aligned error |

## Development Stages

Implementation must proceed in stages:

1. Stage 0: Create `DO_CT_LIO_ws`; build unmodified baseline; confirm Docker
   host networking; confirm Gazebo/bag topics reach the container; confirm
   evaluation pipeline.
2. Stage 1: Run original CT-LIO baseline on both bags and record metrics.
3. Stage 2: Add logging-only 6D Hessian diagnostic without changing
   optimization output.
4. Stage 3: Add logging-only 12D Hessian diagnostic without changing
   optimization output. This first 12D Hessian implementation must not change
   residual weights, residual selection, priors, or pose output until baseline
   metrics, eigenvalue logs, and Jacobian finite-difference checks are available.
5. Stage 4: Add 12D Jacobian finite-difference validation. Do not enable final
   weighting until Jacobian checks are acceptable.
6. Stage 5: Enable 12D residual reweighting while preserving original gates and
   Huber loss.
7. Stage 6: Add spatial and `alpha_time` balanced bucket selection.
8. Stage 7: Add continuous prior boost in degenerate directions.
9. Stage 8: If needed, add post-solve subspace projection.
10. Stage 9: Final validation on both bags using 12D Hessian weighting.

## Self-Review Rule

Before claiming completion, ask:

```text
Do I have evidence that the current strategy is correct and that both bags pass
the raw 5 cm requirement?
```

If no, identify the failure mode, propose the next fix, implement one logical
change, rerun evaluation, record results, and repeat.

Do not claim success based only on compilation, RViz appearance, aligned ATE,
one validation bag, a 6D result, or undocumented parameter tuning.

## Completion Definition

The work is complete only when:

- DO-CT-LIO builds in Docker.
- The algorithm runs in Docker over host networking.
- Host-side Gazebo/bag playback connects to the container.
- `scout_do_ct_lio.launch` exists in Gazebo bringup.
- Baseline CT-LIO metrics are recorded.
- Final validation uses 12D Hessian weighting.
- 12D Jacobian finite-difference validation is implemented and logged.
- Spatial and `alpha_time` balanced residual selection is implemented.
- Degenerate-direction prior protection is implemented.
- Both specified bags pass raw 3D ATE RMSE and P95 within 5 cm.
- Temporary test files are removed after results are recorded.
- Development plan, development log, self-review, and evaluation results are up
  to date.

## 2026-05-20 Final Validation Update

The implementation reached the completion definition for the two specified
validation bags.

- Final runtime:
  - Algorithm in Docker image `do-ct-lio-noetic:local`
  - Docker flags `--net=host --ipc=host`
  - Rosbag playback on the host
  - Ground truth topic `/ground_truth/odom`
- Final metrics:
  - `medium_degenerate_rawimu.bag`: raw 3D RMSE/P95
    `0.0237 / 0.0414 m`
  - `medium_degenerate_rawimu_new.bag`: raw 3D RMSE/P95
    `0.0265 / 0.0491 m`
- Final guard status:
  - 12D Hessian logging and finite-difference checks were performed before
    output-affecting behavior changes.
  - `effective_rank_threshold: 8` remains an initial diagnostic threshold, not a
    universal rank rule.
  - Scaled-Hessian projection is implemented in scaled variable space; final
    runtime leaves post-solve projection disabled.
