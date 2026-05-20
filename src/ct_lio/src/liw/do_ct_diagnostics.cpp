#include "do_ct_diagnostics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

namespace do_ct_lio
{
namespace
{
void ValidatePositiveScale(const Vector12d &scale)
{
     for (int i = 0; i < scale.size(); ++i)
     {
          if (!std::isfinite(scale(i)) || scale(i) <= 0.0)
               throw std::invalid_argument("DO-CT-LIO scale entries must be positive finite values");
     }
}
}

Matrix12d ScaleHessianRawToScaled(const Matrix12d &raw_hessian, const Vector12d &scale)
{
     ValidatePositiveScale(scale);
     const Eigen::DiagonalMatrix<double, 12> S(scale);
     return S * raw_hessian * S;
}

Vector12d ProjectRawDeltaWithScaledBasis(const Vector12d &raw_delta,
                                         const Vector12d &scale,
                                         const Matrix12d &scaled_eigenvectors,
                                         const Vector12d &gates)
{
     ValidatePositiveScale(scale);

     const Vector12d scaled_delta = raw_delta.cwiseQuotient(scale);
     const Vector12d projected_scaled_delta =
         scaled_eigenvectors * gates.asDiagonal() * scaled_eigenvectors.transpose() * scaled_delta;
     return projected_scaled_delta.cwiseProduct(scale);
}

Vector12d BlendProjectedRawDelta(const Vector12d &raw_delta,
                                 const Vector12d &scale,
                                 const Matrix12d &scaled_eigenvectors,
                                 const Vector12d &gates,
                                 double projection_strength)
{
     const double strength =
         std::max(0.0, std::min(1.0, std::isfinite(projection_strength) ? projection_strength : 0.0));
     if (strength <= 0.0)
          return raw_delta;

     const Vector12d projected =
         ProjectRawDeltaWithScaledBasis(raw_delta, scale, scaled_eigenvectors, gates);
     return raw_delta + strength * (projected - raw_delta);
}

SpectrumClassification ClassifySpectrum(const Vector12d &eigenvalues,
                                        int effective_rank_threshold,
                                        double relative_eigen_floor)
{
     SpectrumClassification result;
     result.configured_min_rank = std::max(0, effective_rank_threshold);
     result.max_eigenvalue = eigenvalues.maxCoeff();
     result.min_eigenvalue = eigenvalues.minCoeff();

     const double floor = std::max(0.0, relative_eigen_floor) * std::max(0.0, result.max_eigenvalue);
     for (int i = 0; i < eigenvalues.size(); ++i)
     {
          if (std::isfinite(eigenvalues(i)) && eigenvalues(i) > floor)
               ++result.effective_rank;
     }
     result.rank_pass = result.effective_rank >= result.configured_min_rank;
     return result;
}

bool IsDegenerateSpectrum(double min_normalized_eigenvalue,
                          double degeneracy_threshold,
                          int effective_rank,
                          int effective_rank_threshold)
{
     if (!std::isfinite(min_normalized_eigenvalue))
          return false;
     return min_normalized_eigenvalue < degeneracy_threshold &&
            effective_rank < effective_rank_threshold;
}

Vector12d MakePoseIncrementScale(double rotation_scale_meters)
{
     if (!std::isfinite(rotation_scale_meters) || rotation_scale_meters <= 0.0)
          throw std::invalid_argument("DO-CT-LIO rotation scale must be positive and finite");

     Vector12d scale = Vector12d::Ones();
     const double inverse_rotation_scale = 1.0 / rotation_scale_meters;
     scale.segment<3>(3).setConstant(inverse_rotation_scale);
     scale.segment<3>(9).setConstant(inverse_rotation_scale);
     return scale;
}

double MakeMotionPriorWeight(int residual_count, double beta)
{
     if (residual_count <= 0 || !std::isfinite(beta) || beta <= 0.0)
          return 0.0;
     return std::sqrt(static_cast<double>(residual_count) * beta);
}

double ComputeLidarFrameBeginTime(double header_stamp,
                                  double scan_time_span,
                                  bool header_stamp_is_scan_end)
{
     if (!std::isfinite(header_stamp))
          return header_stamp;
     if (!header_stamp_is_scan_end)
          return header_stamp;
     if (!std::isfinite(scan_time_span) || scan_time_span <= 0.0)
          return header_stamp;
     return header_stamp - scan_time_span;
}

OneShotScalarReference UpdateOneShotScalarReference(const OneShotScalarReference &current,
                                                    double candidate_value)
{
     if (current.valid)
          return current;

     OneShotScalarReference updated;
     if (std::isfinite(candidate_value))
     {
          updated.valid = true;
          updated.value = candidate_value;
     }
     return updated;
}

bool ShouldLogDoCtHessianDiagnostics(bool enabled,
                                     bool logging_only,
                                     bool log_first_iteration_only,
                                     int frame_id,
                                     int iteration,
                                     int log_every_n_frames,
                                     bool has_candidate_residuals)
{
     if (!enabled || !logging_only || !has_candidate_residuals)
          return false;
     if (log_first_iteration_only && iteration != 0)
          return false;
     const int log_every = std::max(1, log_every_n_frames);
     return frame_id <= 3 || frame_id % log_every == 0;
}

CommandMotionDelta IntegrateCommandMotion(const std::vector<CommandMotionSample> &samples,
                                          double begin_time,
                                          double end_time,
                                          double max_gap)
{
     CommandMotionDelta result;
     if (!std::isfinite(begin_time) || !std::isfinite(end_time) ||
         !std::isfinite(max_gap) || end_time <= begin_time || max_gap <= 0.0)
          return result;

     result.duration = end_time - begin_time;

     std::vector<CommandMotionSample> ordered;
     ordered.reserve(samples.size());
     for (const auto &sample : samples)
     {
          if (std::isfinite(sample.stamp) &&
              std::isfinite(sample.linear_x) &&
              std::isfinite(sample.angular_z) &&
              sample.stamp <= end_time &&
              sample.stamp >= begin_time - max_gap)
          {
               ordered.push_back(sample);
          }
     }

     if (ordered.empty())
          return result;

     std::sort(ordered.begin(), ordered.end(),
               [](const CommandMotionSample &left, const CommandMotionSample &right)
               { return left.stamp < right.stamp; });

     int current_index = -1;
     for (int i = 0; i < static_cast<int>(ordered.size()); ++i)
     {
          if (ordered[i].stamp <= begin_time)
               current_index = i;
          else
               break;
     }

     if (current_index < 0 || begin_time - ordered[current_index].stamp > max_gap)
          return result;

     int next_index = current_index + 1;
     double time = begin_time;
     result.samples_used = 1;

     constexpr double kTimeEpsilon = 1.0e-12;
     while (time < end_time - kTimeEpsilon)
     {
          const auto &current = ordered[current_index];
          const double next_sample_time =
              next_index < static_cast<int>(ordered.size())
                  ? ordered[next_index].stamp
                  : std::numeric_limits<double>::infinity();
          const double valid_until = current.stamp + max_gap;
          const double interval_end = std::min(end_time, std::min(next_sample_time, valid_until));

          if (interval_end <= time + kTimeEpsilon)
          {
               if (next_index < static_cast<int>(ordered.size()) &&
                   ordered[next_index].stamp <= time + kTimeEpsilon)
               {
                    current_index = next_index;
                    ++next_index;
                    ++result.samples_used;
                    continue;
               }
               return result;
          }

          const double dt = interval_end - time;
          result.forward += current.linear_x * dt;
          result.yaw += current.angular_z * dt;
          time = interval_end;

          if (next_index < static_cast<int>(ordered.size()) &&
              ordered[next_index].stamp <= time + kTimeEpsilon)
          {
               current_index = next_index;
               ++next_index;
               ++result.samples_used;
          }
          else if (time < end_time - kTimeEpsilon && time >= valid_until - kTimeEpsilon)
          {
               return result;
          }
     }

     result.valid = true;
     return result;
}

Eigen::Vector3d ApplyBodyTranslationOffset(const Eigen::Vector3d &translation,
                                           const Eigen::Quaterniond &orientation,
                                           const Eigen::Vector3d &body_offset,
                                           const Eigen::Vector3d &world_offset)
{
     return translation + orientation.normalized() * body_offset + world_offset;
}

Vector12d NormalizeEigenvaluesByAverageEnergy(const Vector12d &eigenvalues)
{
     const double average_energy =
         std::max(eigenvalues.sum() / static_cast<double>(eigenvalues.size()), 1.0e-12);
     Vector12d normalized = eigenvalues / average_energy;
     for (int i = 0; i < normalized.size(); ++i)
     {
          if (!std::isfinite(normalized(i)) || normalized(i) < 0.0)
               normalized(i) = 0.0;
     }
     return normalized;
}

Vector12d ComputeObservabilityGates(const Vector12d &eigenvalues, double lambda0)
{
     const double safe_lambda0 = std::max(lambda0, 1.0e-12);
     const Vector12d normalized = NormalizeEigenvaluesByAverageEnergy(eigenvalues);
     Vector12d gates = Vector12d::Zero();
     for (int i = 0; i < gates.size(); ++i)
          gates(i) = normalized(i) / (normalized(i) + safe_lambda0);
     return gates;
}

double ComputeDirectionalQuality(const RowVector12d &scaled_jacobian,
                                 const Matrix12d &scaled_eigenvectors,
                                 const Vector12d &observability_gates,
                                 double jacobian_norm_min)
{
     const double norm_min = std::max(jacobian_norm_min, 0.0);
     const double squared_norm = scaled_jacobian.squaredNorm();
     if (!std::isfinite(squared_norm) || squared_norm <= norm_min * norm_min)
          return 0.0;

     const RowVector12d projected = scaled_jacobian * scaled_eigenvectors;
     const double denominator = std::max(projected.squaredNorm(), 1.0e-18);
     double numerator = 0.0;
     for (int i = 0; i < projected.size(); ++i)
          numerator += observability_gates(i) * projected(i) * projected(i);

     const double ratio = std::max(0.0, std::min(1.0, numerator / denominator));
     return std::sqrt(ratio);
}

double ComputeFinalInformationWeight(double local_info_weight,
                                     double hessian_quality,
                                     double gamma,
                                     double omega_min,
                                     double omega_max)
{
     if (!std::isfinite(local_info_weight) || local_info_weight <= 0.0)
          return 0.0;

     const double safe_quality =
         std::max(0.0, std::min(1.0, std::isfinite(hessian_quality) ? hessian_quality : 0.0));
     const double safe_gamma = std::max(0.0, std::isfinite(gamma) ? gamma : 1.0);
     const double lower = std::max(0.0, std::min(omega_min, omega_max));
     const double upper = std::max(omega_min, omega_max);
     const double weighted = local_info_weight * std::pow(safe_quality, safe_gamma);
     return std::max(lower, std::min(upper, weighted));
}

std::vector<int> SelectBalancedResidualIndices(const std::vector<ResidualSelectionItem> &items,
                                               int bucket_top_k,
                                               int min_count,
                                               int max_count)
{
     if (items.empty())
          return {};

     const int item_count = static_cast<int>(items.size());
     const int target_min = std::min(item_count, std::max(0, min_count));
     int target_max = max_count > 0 ? std::min(item_count, max_count) : item_count;
     target_max = std::max(target_min, target_max);
     if (target_max <= 0)
          return {};

     auto finite_weight = [&](int index) -> double
     {
          const double weight = items[index].final_info_weight;
          return std::isfinite(weight) ? weight : -std::numeric_limits<double>::infinity();
     };

     std::map<std::string, std::vector<int>> buckets;
     for (int i = 0; i < item_count; ++i)
          buckets[items[i].bucket_key].push_back(i);

     std::vector<int> selected_indices;
     std::set<int> selected_set;
     const int keep_per_bucket = std::max(1, bucket_top_k);
     for (auto &entry : buckets)
     {
          auto &indices = entry.second;
          std::sort(indices.begin(), indices.end(), [&](int left, int right)
                    { return finite_weight(left) > finite_weight(right); });
          const int keep = std::min(keep_per_bucket, static_cast<int>(indices.size()));
          for (int i = 0; i < keep && static_cast<int>(selected_indices.size()) < target_max; ++i)
          {
               if (selected_set.insert(indices[i]).second)
                    selected_indices.push_back(indices[i]);
          }
     }

     std::vector<int> global_indices(item_count);
     std::iota(global_indices.begin(), global_indices.end(), 0);
     std::sort(global_indices.begin(), global_indices.end(), [&](int left, int right)
               { return finite_weight(left) > finite_weight(right); });

     for (int index : global_indices)
     {
          if (static_cast<int>(selected_indices.size()) >= target_max)
               break;
          if (selected_set.insert(index).second)
               selected_indices.push_back(index);
     }

     std::sort(selected_indices.begin(), selected_indices.end());
     return selected_indices;
}

std::string FormatVector12Csv(const Vector12d &values)
{
     std::ostringstream stream;
     for (int i = 0; i < values.size(); ++i)
     {
          if (i > 0)
               stream << ",";
          stream << values(i);
     }
     return stream.str();
}
}
