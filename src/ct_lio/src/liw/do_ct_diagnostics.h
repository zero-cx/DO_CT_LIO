#ifndef DO_CT_DIAGNOSTICS_H__
#define DO_CT_DIAGNOSTICS_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace do_ct_lio
{
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using RowVector12d = Eigen::Matrix<double, 1, 12>;

struct SpectrumClassification
{
     int effective_rank = 0;
     int configured_min_rank = 0;
     bool rank_pass = false;
     double max_eigenvalue = 0.0;
     double min_eigenvalue = 0.0;
};

struct CommandMotionSample
{
     double stamp = 0.0;
     double linear_x = 0.0;
     double angular_z = 0.0;
};

struct CommandMotionDelta
{
     bool valid = false;
     int samples_used = 0;
     double duration = 0.0;
     double forward = 0.0;
     double yaw = 0.0;
};

struct ResidualSelectionItem
{
     std::string bucket_key;
     double final_info_weight = 0.0;
};

struct OneShotScalarReference
{
     bool valid = false;
     double value = 0.0;
};

Matrix12d ScaleHessianRawToScaled(const Matrix12d &raw_hessian, const Vector12d &scale);

Vector12d ProjectRawDeltaWithScaledBasis(const Vector12d &raw_delta,
                                         const Vector12d &scale,
                                         const Matrix12d &scaled_eigenvectors,
                                         const Vector12d &gates);

Vector12d BlendProjectedRawDelta(const Vector12d &raw_delta,
                                 const Vector12d &scale,
                                 const Matrix12d &scaled_eigenvectors,
                                 const Vector12d &gates,
                                 double projection_strength);

SpectrumClassification ClassifySpectrum(const Vector12d &eigenvalues,
                                        int effective_rank_threshold,
                                        double relative_eigen_floor);

bool IsDegenerateSpectrum(double min_normalized_eigenvalue,
                          double degeneracy_threshold,
                          int effective_rank,
                          int effective_rank_threshold);

Vector12d MakePoseIncrementScale(double rotation_scale_meters);

double MakeMotionPriorWeight(int residual_count, double beta);

double ComputeLidarFrameBeginTime(double header_stamp,
                                  double scan_time_span,
                                  bool header_stamp_is_scan_end);

OneShotScalarReference UpdateOneShotScalarReference(const OneShotScalarReference &current,
                                                    double candidate_value);

bool ShouldLogDoCtHessianDiagnostics(bool enabled,
                                     bool logging_only,
                                     bool log_first_iteration_only,
                                     int frame_id,
                                     int iteration,
                                     int log_every_n_frames,
                                     bool has_candidate_residuals);

CommandMotionDelta IntegrateCommandMotion(const std::vector<CommandMotionSample> &samples,
                                          double begin_time,
                                          double end_time,
                                          double max_gap);

Eigen::Vector3d ApplyBodyTranslationOffset(const Eigen::Vector3d &translation,
                                           const Eigen::Quaterniond &orientation,
                                           const Eigen::Vector3d &body_offset,
                                           const Eigen::Vector3d &world_offset);

Vector12d NormalizeEigenvaluesByAverageEnergy(const Vector12d &eigenvalues);

Vector12d ComputeObservabilityGates(const Vector12d &eigenvalues, double lambda0);

double ComputeDirectionalQuality(const RowVector12d &scaled_jacobian,
                                 const Matrix12d &scaled_eigenvectors,
                                 const Vector12d &observability_gates,
                                 double jacobian_norm_min);

double ComputeFinalInformationWeight(double local_info_weight,
                                     double hessian_quality,
                                     double gamma,
                                     double omega_min,
                                     double omega_max);

std::vector<int> SelectBalancedResidualIndices(const std::vector<ResidualSelectionItem> &items,
                                               int bucket_top_k,
                                               int min_count,
                                               int max_count);

std::string FormatVector12Csv(const Vector12d &values);
}

#endif
