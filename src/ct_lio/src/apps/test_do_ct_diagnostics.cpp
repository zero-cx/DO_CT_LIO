#include "liw/do_ct_diagnostics.h"
#include "liw/lidarFactor.h"
#include "common/cloudMap.hpp"

#include <cmath>
#include <ceres/ceres.h>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>

namespace
{
using do_ct_lio::Matrix12d;
using do_ct_lio::Vector12d;

void ExpectNear(double actual, double expected, double tolerance, const std::string &label)
{
     if (std::abs(actual - expected) > tolerance)
     {
          throw std::runtime_error(label + " expected " + std::to_string(expected) +
                                   " got " + std::to_string(actual));
     }
}

double YawFromQuaternion(const Eigen::Quaterniond &q_in)
{
     const Eigen::Quaterniond q = q_in.normalized();
     return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                       1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void TestScaleHessianRawToScaled()
{
     Matrix12d raw = Matrix12d::Zero();
     raw(0, 0) = 1.0;
     raw(1, 1) = 4.0;

     Vector12d scale = Vector12d::Ones();
     scale(0) = 2.0;
     scale(1) = 3.0;

     const Matrix12d scaled = do_ct_lio::ScaleHessianRawToScaled(raw, scale);
     ExpectNear(scaled(0, 0), 4.0, 1e-12, "scaled hessian h00");
     ExpectNear(scaled(1, 1), 36.0, 1e-12, "scaled hessian h11");
}

void TestProjectionRunsInScaledSpace()
{
     Vector12d raw_delta = Vector12d::Zero();
     raw_delta(0) = 10.0;
     raw_delta(1) = 1.0;

     Vector12d scale = Vector12d::Ones();
     scale(0) = 10.0;
     scale(1) = 1.0;

     Matrix12d basis = Matrix12d::Identity();
     basis(0, 0) = 1.0 / std::sqrt(2.0);
     basis(1, 0) = 1.0 / std::sqrt(2.0);
     basis(0, 1) = -1.0 / std::sqrt(2.0);
     basis(1, 1) = 1.0 / std::sqrt(2.0);

     Vector12d gates = Vector12d::Zero();
     gates(0) = 1.0;

     const Vector12d projected =
         do_ct_lio::ProjectRawDeltaWithScaledBasis(raw_delta, scale, basis, gates);

     ExpectNear(projected(0), 10.0, 1e-12, "scaled projection x0");
     ExpectNear(projected(1), 1.0, 1e-12, "scaled projection x1");
}

void TestProjectionBlendUsesScaledSpace()
{
     Vector12d raw_delta = Vector12d::Zero();
     raw_delta(0) = 20.0;
     raw_delta(1) = 1.0;

     Vector12d scale = Vector12d::Ones();
     scale(0) = 10.0;
     scale(1) = 1.0;

     Matrix12d basis = Matrix12d::Identity();
     basis(0, 0) = 1.0 / std::sqrt(2.0);
     basis(1, 0) = 1.0 / std::sqrt(2.0);
     basis(0, 1) = -1.0 / std::sqrt(2.0);
     basis(1, 1) = 1.0 / std::sqrt(2.0);

     Vector12d gates = Vector12d::Zero();
     gates(0) = 1.0;

     const Vector12d unchanged =
         do_ct_lio::BlendProjectedRawDelta(raw_delta, scale, basis, gates, 0.0);
     const Vector12d projected =
         do_ct_lio::BlendProjectedRawDelta(raw_delta, scale, basis, gates, 1.0);
     const Vector12d halfway =
         do_ct_lio::BlendProjectedRawDelta(raw_delta, scale, basis, gates, 0.5);

     ExpectNear(unchanged(0), raw_delta(0), 1e-12, "projection blend strength 0 x0");
     ExpectNear(unchanged(1), raw_delta(1), 1e-12, "projection blend strength 0 x1");
     ExpectNear(projected(0), 15.0, 1e-12, "projection blend strength 1 x0");
     ExpectNear(projected(1), 1.5, 1e-12, "projection blend strength 1 x1");
     ExpectNear(halfway(0), 17.5, 1e-12, "projection blend strength 0.5 x0");
     ExpectNear(halfway(1), 1.25, 1e-12, "projection blend strength 0.5 x1");
}

void TestEffectiveRankUsesTunableMinimum()
{
     Vector12d eigenvalues = Vector12d::Zero();
     for (int i = 0; i < 12; ++i)
          eigenvalues(i) = static_cast<double>(i + 1);

     const auto rank8 = do_ct_lio::ClassifySpectrum(eigenvalues, 8, 1e-12);
     const auto rank10 = do_ct_lio::ClassifySpectrum(eigenvalues, 10, 1e-12);

     if (rank8.effective_rank != 12 || !rank8.rank_pass)
          throw std::runtime_error("rank8 classification should pass all positive eigenvalues");
     if (rank10.effective_rank != 12 || !rank10.rank_pass)
          throw std::runtime_error("rank10 classification should also pass all positive eigenvalues");

     eigenvalues(0) = 0.0;
     eigenvalues(1) = 0.0;
     eigenvalues(2) = 0.0;

     const auto rank_after_floor = do_ct_lio::ClassifySpectrum(eigenvalues, 10, 1e-12);
     if (rank_after_floor.effective_rank != 9 || rank_after_floor.rank_pass)
          throw std::runtime_error("effective rank must respect the configured minimum");
}

void TestDegeneracyClassificationRequiresWeakValueAndLowRank()
{
     if (do_ct_lio::IsDegenerateSpectrum(0.01, 0.05, 12, 8))
          throw std::runtime_error("weak minimum eigenvalue alone should not trigger prior boost");
     if (do_ct_lio::IsDegenerateSpectrum(0.10, 0.05, 6, 8))
          throw std::runtime_error("low effective rank alone should not trigger prior boost");
     if (!do_ct_lio::IsDegenerateSpectrum(0.01, 0.05, 6, 8))
          throw std::runtime_error("weak eigenvalue and low rank should trigger degeneracy");
}

void TestMotionPriorWeight()
{
     ExpectNear(do_ct_lio::MakeMotionPriorWeight(100, 0.25), 5.0, 1e-12,
                "motion prior weight");
     ExpectNear(do_ct_lio::MakeMotionPriorWeight(100, 0.0), 0.0, 1e-12,
                "zero beta prior weight");
     ExpectNear(do_ct_lio::MakeMotionPriorWeight(0, 1.0), 0.0, 1e-12,
                "zero residual prior weight");
     ExpectNear(do_ct_lio::MakeMotionPriorWeight(100, -1.0), 0.0, 1e-12,
                "negative beta prior weight");
}

void TestLidarFrameBeginTimeHonorsHeaderReference()
{
     ExpectNear(do_ct_lio::ComputeLidarFrameBeginTime(10.0, 0.1, false),
                10.0, 1e-12, "scan begin header frame begin");
     ExpectNear(do_ct_lio::ComputeLidarFrameBeginTime(10.0, 0.1, true),
                9.9, 1e-12, "scan end header frame begin");
     ExpectNear(do_ct_lio::ComputeLidarFrameBeginTime(10.0, -0.1, true),
                10.0, 1e-12, "invalid scan span fallback");
}

void TestDiagnosticLoggingScheduleDoesNotDependOnActiveWeighting()
{
     if (!do_ct_lio::ShouldLogDoCtHessianDiagnostics(true, true, true, 50, 0, 50, true))
          throw std::runtime_error("diagnostic logging should run on scheduled first iterations");
     if (!do_ct_lio::ShouldLogDoCtHessianDiagnostics(true, true, true, 2, 0, 50, true))
          throw std::runtime_error("diagnostic logging should run on startup frames");
     if (do_ct_lio::ShouldLogDoCtHessianDiagnostics(true, true, true, 50, 1, 50, true))
          throw std::runtime_error("diagnostic logging should respect first-iteration-only");
     if (do_ct_lio::ShouldLogDoCtHessianDiagnostics(true, true, true, 51, 0, 50, true))
          throw std::runtime_error("diagnostic logging should respect frame schedule");
     if (do_ct_lio::ShouldLogDoCtHessianDiagnostics(true, true, true, 50, 0, 50, false))
          throw std::runtime_error("diagnostic logging should require candidate residuals");
}

void TestPoseIncrementScaleUsesInverseRotationLength()
{
     const Vector12d scale = do_ct_lio::MakePoseIncrementScale(8.0);
     ExpectNear(scale(0), 1.0, 1e-12, "translation scale");
     ExpectNear(scale(3), 0.125, 1e-12, "begin rotation scale");
     ExpectNear(scale(9), 0.125, 1e-12, "end rotation scale");
}

void TestObservabilityGatesUseNormalizedEnergy()
{
     Vector12d eigenvalues = Vector12d::Ones();
     const Vector12d gates = do_ct_lio::ComputeObservabilityGates(eigenvalues, 1.0);
     for (int i = 0; i < gates.size(); ++i)
          ExpectNear(gates(i), 0.5, 1e-12, "observability gate");
}

void TestDirectionalQualityUsesScaledBasis()
{
     do_ct_lio::RowVector12d row = do_ct_lio::RowVector12d::Zero();
     row(0) = 1.0;
     row(1) = 1.0;

     Matrix12d basis = Matrix12d::Identity();
     Vector12d gates = Vector12d::Zero();
     gates(0) = 1.0;

     const double quality =
         do_ct_lio::ComputeDirectionalQuality(row, basis, gates, 1.0e-8);
     ExpectNear(quality, std::sqrt(0.5), 1e-12, "directional quality");
}

void TestInformationWeightClamp()
{
     ExpectNear(do_ct_lio::ComputeFinalInformationWeight(0.25, 0.5, 1.0, 0.15, 1.5),
                0.15, 1e-12, "minimum information weight");
     ExpectNear(do_ct_lio::ComputeFinalInformationWeight(2.0, 1.0, 1.0, 0.15, 1.5),
                1.5, 1e-12, "maximum information weight");
}

void TestBalancedSelectionFillsToMaxResidualBudget()
{
     std::vector<do_ct_lio::ResidualSelectionItem> items = {
         {"a", 0.9},
         {"a", 0.8},
         {"b", 0.7},
         {"b", 0.6},
         {"c", 0.5}};

     const std::vector<int> selected =
         do_ct_lio::SelectBalancedResidualIndices(items, 1, 2, 4);

     if (selected.size() != 4)
          throw std::runtime_error("balanced selection should fill up to max residual budget");

     std::set<int> selected_set(selected.begin(), selected.end());
     if (selected_set.size() != selected.size())
          throw std::runtime_error("balanced selection should not duplicate indices");
     if (!selected_set.count(0) || !selected_set.count(2) || !selected_set.count(4))
          throw std::runtime_error("balanced selection must preserve bucket top candidates first");
     if (!selected_set.count(1))
          throw std::runtime_error("balanced selection should globally fill after bucket pass");
}

void TestVoxelBlockTracksFrameAgeAndCanRefreshStalePoints()
{
     voxelBlock block(2);
     block.AddPoint(Eigen::Vector3d(1.0, 0.0, 0.0), 10);
     block.AddPoint(Eigen::Vector3d(2.0, 0.0, 0.0), 12);

     if (block.FirstFrameId() != 10 || block.LastFrameId() != 12)
          throw std::runtime_error("voxel block should track first and last frame ids");
     if (block.IsStale(20, 20))
          throw std::runtime_error("voxel block should not be stale inside configured age");
     if (!block.IsStale(40, 20))
          throw std::runtime_error("voxel block should be stale after configured age");

     block.ResetWithPoint(Eigen::Vector3d(3.0, 0.0, 0.0), 41);
     if (block.NumPoints() != 1)
          throw std::runtime_error("stale refresh should replace old voxel points");
     if (block.FirstFrameId() != 41 || block.LastFrameId() != 41)
          throw std::runtime_error("stale refresh should reset frame age metadata");
     ExpectNear(block.points.front().x(), 3.0, 1e-12, "stale refresh point");
}

void TestCtAutoDiffFactorAppliesExtrinsic()
{
     CT_ICP::CTPointToPlaneFunctor::q_il = Eigen::Quaterniond::Identity();
     CT_ICP::CTPointToPlaneFunctor::t_il = Eigen::Vector3d(0.5, 0.0, 0.0);

     std::unique_ptr<ceres::CostFunction> cost(
         CT_ICP::CTPointToPlaneFunctor::Create(Eigen::Vector3d(1.5, 0.0, 0.0),
                                               Eigen::Vector3d(1.0, 0.0, 0.0),
                                               Eigen::Vector3d(1.0, 0.0, 0.0),
                                               0.5,
                                               1.0));

     double begin_t[3] = {0.0, 0.0, 0.0};
     double end_t[3] = {0.0, 0.0, 0.0};
     double begin_q[4] = {0.0, 0.0, 0.0, 1.0};
     double end_q[4] = {0.0, 0.0, 0.0, 1.0};
     double const *parameters[4] = {begin_t, begin_q, end_t, end_q};
     double residual[1] = {999.0};

     if (!cost->Evaluate(parameters, residual, nullptr))
          throw std::runtime_error("ct autodiff factor evaluate failed");

     ExpectNear(residual[0], 0.0, 1e-12, "ct autodiff extrinsic residual");
}

void TestBodyFrameOutputOffsetRotatesWithPose()
{
     const Eigen::Quaterniond yaw_90(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
     const Eigen::Vector3d translated =
         do_ct_lio::ApplyBodyTranslationOffset(Eigen::Vector3d(1.0, 2.0, 3.0),
                                               yaw_90,
                                               Eigen::Vector3d(0.1, 0.0, -0.2),
                                               Eigen::Vector3d(0.0, 0.0, 0.3));

     ExpectNear(translated.x(), 1.0, 1e-12, "body offset rotated x");
     ExpectNear(translated.y(), 2.1, 1e-12, "body offset rotated y");
     ExpectNear(translated.z(), 3.1, 1e-12, "body offset plus world z");
}

void TestNonHolonomicFunctorUsesBodyFrameDisplacement()
{
     const Eigen::Quaterniond yaw_90(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
     CT_ICP::NonHolonomicMotionFunctor functor(2.0);

     double begin_t[3] = {0.0, 0.0, 0.0};
     double begin_q[4] = {yaw_90.x(), yaw_90.y(), yaw_90.z(), yaw_90.w()};
     double forward_end_t[3] = {0.0, 1.0, 0.0};
     double lateral_end_t[3] = {1.0, 0.0, 0.5};
     double residual[2] = {999.0, 999.0};

     if (!functor(begin_t, begin_q, forward_end_t, residual))
          throw std::runtime_error("nonholonomic forward evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "nonholonomic forward lateral");
     ExpectNear(residual[1], 0.0, 1e-12, "nonholonomic forward vertical");

     if (!functor(begin_t, begin_q, lateral_end_t, residual))
          throw std::runtime_error("nonholonomic lateral evaluate failed");
     ExpectNear(residual[0], -2.0, 1e-12, "nonholonomic lateral residual");
     ExpectNear(residual[1], 1.0, 1e-12, "nonholonomic vertical residual");
}

void TestRelativeOrientationFunctorConstrainsScanRotation()
{
     const Eigen::Quaterniond predicted_delta(
         Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()));
     CT_ICP::RelativeOrientationFunctor functor(predicted_delta, 2.0);

     const Eigen::Quaterniond begin_q = Eigen::Quaterniond::Identity();
     const Eigen::Quaterniond matching_end_q(
         Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond yaw_error_end_q(
         Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ()));

     double begin_params[4] = {begin_q.x(), begin_q.y(), begin_q.z(), begin_q.w()};
     double matching_end_params[4] = {matching_end_q.x(), matching_end_q.y(),
                                      matching_end_q.z(), matching_end_q.w()};
     double yaw_error_end_params[4] = {yaw_error_end_q.x(), yaw_error_end_q.y(),
                                       yaw_error_end_q.z(), yaw_error_end_q.w()};
     double residual[3] = {999.0, 999.0, 999.0};

     if (!functor(begin_params, matching_end_params, residual))
          throw std::runtime_error("relative orientation matching evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "relative orientation zero roll");
     ExpectNear(residual[1], 0.0, 1e-12, "relative orientation zero pitch");
     ExpectNear(residual[2], 0.0, 1e-12, "relative orientation zero yaw");

     if (!functor(begin_params, yaw_error_end_params, residual))
          throw std::runtime_error("relative orientation yaw error evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "relative orientation yaw error roll");
     ExpectNear(residual[1], 0.0, 1e-12, "relative orientation yaw error pitch");
     ExpectNear(residual[2], 0.2, 1e-3, "relative orientation yaw error residual");
}

void TestRelativeYawFunctorConstrainsYawOnly()
{
     CT_ICP::RelativeYawFunctor functor(0.10, 2.0);

     const Eigen::Quaterniond begin_q = Eigen::Quaterniond::Identity();
     const Eigen::Quaterniond matching_end_q(
         Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond yaw_error_end_q(
         Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond roll_error_end_q(
         Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitX()));
     const Eigen::Quaterniond roll_then_matching_yaw_end_q =
         Eigen::Quaterniond(Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitX())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()));

     double begin_params[4] = {begin_q.x(), begin_q.y(), begin_q.z(), begin_q.w()};
     double matching_end_params[4] = {matching_end_q.x(), matching_end_q.y(),
                                      matching_end_q.z(), matching_end_q.w()};
     double yaw_error_end_params[4] = {yaw_error_end_q.x(), yaw_error_end_q.y(),
                                       yaw_error_end_q.z(), yaw_error_end_q.w()};
     double roll_error_end_params[4] = {roll_error_end_q.x(), roll_error_end_q.y(),
                                        roll_error_end_q.z(), roll_error_end_q.w()};
     double roll_then_matching_yaw_params[4] = {
         roll_then_matching_yaw_end_q.x(), roll_then_matching_yaw_end_q.y(),
         roll_then_matching_yaw_end_q.z(), roll_then_matching_yaw_end_q.w()};
     double residual[1] = {999.0};

     if (!functor(begin_params, matching_end_params, residual))
          throw std::runtime_error("relative yaw matching evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "relative yaw zero residual");

     if (!functor(begin_params, yaw_error_end_params, residual))
          throw std::runtime_error("relative yaw error evaluate failed");
     ExpectNear(residual[0], 0.2, 1e-12, "relative yaw error residual");

     CT_ICP::RelativeYawFunctor zero_yaw_functor(0.0, 2.0);
     if (!zero_yaw_functor(begin_params, roll_error_end_params, residual))
          throw std::runtime_error("relative yaw roll-only evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "relative yaw ignores roll");

     CT_ICP::RelativeYawFunctor roll_then_matching_yaw_functor(
         YawFromQuaternion(roll_then_matching_yaw_end_q), 2.0);
     if (!roll_then_matching_yaw_functor(begin_params, roll_then_matching_yaw_params, residual))
          throw std::runtime_error("relative yaw mixed roll/yaw evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "relative yaw uses true heading");

     const Eigen::Quaterniond tilted_heading_q =
         Eigen::Quaterniond(Eigen::AngleAxisd(-0.78, Eigen::Vector3d::UnitX())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(0.79, Eigen::Vector3d::UnitZ())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(0.79, Eigen::Vector3d::UnitY()));
     CT_ICP::RelativeYawFunctor tilted_heading_functor(
         YawFromQuaternion(tilted_heading_q), 2.0);
     double tilted_heading_params[4] = {tilted_heading_q.x(), tilted_heading_q.y(),
                                        tilted_heading_q.z(), tilted_heading_q.w()};
     if (!tilted_heading_functor(begin_params, tilted_heading_params, residual))
          throw std::runtime_error("relative yaw tilted heading evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "relative yaw ignores roll/pitch tilt");
}

void TestScanTranslationFunctorConstrainsPredictedDisplacement()
{
     CT_ICP::ScanTranslationFunctor functor(Eigen::Vector3d(1.0, 0.0, 0.0), 2.0);

     double begin_t[3] = {0.0, 0.0, 0.0};
     double matching_end_t[3] = {1.0, 0.0, 0.0};
     double lateral_error_end_t[3] = {1.0, 0.1, -0.2};
     double residual[3] = {999.0, 999.0, 999.0};

     if (!functor(begin_t, matching_end_t, residual))
          throw std::runtime_error("scan translation matching evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "scan translation zero x");
     ExpectNear(residual[1], 0.0, 1e-12, "scan translation zero y");
     ExpectNear(residual[2], 0.0, 1e-12, "scan translation zero z");

     if (!functor(begin_t, lateral_error_end_t, residual))
          throw std::runtime_error("scan translation error evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "scan translation error x");
     ExpectNear(residual[1], 0.2, 1e-12, "scan translation error y");
     ExpectNear(residual[2], -0.4, 1e-12, "scan translation error z");
}

void TestFrameNonHolonomicFunctorUsesPreviousBodyFrame()
{
     const Eigen::Quaterniond previous_yaw_90(
         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
     CT_ICP::FrameNonHolonomicFunctor functor(Eigen::Vector3d(1.0, 2.0, 0.0),
                                              previous_yaw_90,
                                              3.0);

     double forward_end_t[3] = {1.0, 3.0, 0.0};
     double lateral_end_t[3] = {2.0, 2.0, -0.5};
     double residual[2] = {999.0, 999.0};

     if (!functor(forward_end_t, residual))
          throw std::runtime_error("frame nonholonomic forward evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "frame nonholonomic forward lateral");
     ExpectNear(residual[1], 0.0, 1e-12, "frame nonholonomic forward vertical");

     if (!functor(lateral_end_t, residual))
          throw std::runtime_error("frame nonholonomic lateral evaluate failed");
     ExpectNear(residual[0], -3.0, 1e-12, "frame nonholonomic lateral residual");
     ExpectNear(residual[1], -1.5, 1e-12, "frame nonholonomic vertical residual");
}

void TestWorldZConsistencyFunctorConstrainsEndHeightOnly()
{
     CT_ICP::WorldZConsistencyFunctor functor(1.25, 4.0);

     double matching_end_t[3] = {10.0, -3.0, 1.25};
     double shifted_end_t[3] = {10.0, -3.0, 1.20};
     double residual[1] = {999.0};

     if (!functor(matching_end_t, residual))
          throw std::runtime_error("world z matching evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "world z zero residual");

     if (!functor(shifted_end_t, residual))
          throw std::runtime_error("world z shifted evaluate failed");
     ExpectNear(residual[0], -0.2, 1e-12, "world z shifted residual");
}

void TestOneShotScalarReferenceInitializesOnlyOnce()
{
     do_ct_lio::OneShotScalarReference reference;
     reference = do_ct_lio::UpdateOneShotScalarReference(reference, 1.25);
     if (!reference.valid)
          throw std::runtime_error("one-shot reference should initialize from first finite value");
     ExpectNear(reference.value, 1.25, 1e-12, "one-shot reference initial value");

     reference = do_ct_lio::UpdateOneShotScalarReference(reference, 2.50);
     ExpectNear(reference.value, 1.25, 1e-12, "one-shot reference must not track later values");

     do_ct_lio::OneShotScalarReference invalid_reference;
     invalid_reference = do_ct_lio::UpdateOneShotScalarReference(invalid_reference,
                                                                 std::numeric_limits<double>::quiet_NaN());
     if (invalid_reference.valid)
          throw std::runtime_error("one-shot reference should ignore non-finite initial values");
}

void TestCommandMotionIntegrationUsesPiecewiseConstantSamples()
{
     std::vector<do_ct_lio::CommandMotionSample> samples = {
         {0.0, 1.0, 0.5},
         {0.1, 1.0, 0.5},
         {0.2, 2.0, -0.5},
         {0.3, 2.0, -0.5}};

     const auto delta =
         do_ct_lio::IntegrateCommandMotion(samples, 0.05, 0.25, 0.2);

     if (!delta.valid)
          throw std::runtime_error("command integration should be valid");
     ExpectNear(delta.duration, 0.20, 1e-12, "command integration duration");
     ExpectNear(delta.forward, 0.25, 1e-12, "command integration forward");
     ExpectNear(delta.yaw, 0.05, 1e-12, "command integration yaw");
}

void TestCommandMotionIntegrationRejectsLargeCommandGap()
{
     std::vector<do_ct_lio::CommandMotionSample> samples = {
         {0.0, 1.0, 0.0},
         {1.0, 1.0, 0.0}};

     const auto delta =
         do_ct_lio::IntegrateCommandMotion(samples, 0.1, 0.9, 0.2);

     if (delta.valid)
          throw std::runtime_error("command integration should reject large gaps");
}

void TestCommandMotionFunctorConstrainsPlanarForwardLateralAndYaw()
{
     CT_ICP::CommandMotionFunctor functor(1.0, M_PI / 2.0, 2.0, 5.0, 3.0);

     const Eigen::Quaterniond begin_q = Eigen::Quaterniond::Identity();
     const Eigen::Quaterniond begin_yaw_90(
         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond begin_tilted_yaw_90 =
         Eigen::Quaterniond(Eigen::AngleAxisd(0.40, Eigen::Vector3d::UnitY())) *
         begin_yaw_90;
     const Eigen::Quaterniond matching_end_q(
         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond tilted_matching_end_q =
         begin_tilted_yaw_90 *
         Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond yaw_error_end_q(
         Eigen::AngleAxisd(M_PI / 2.0 + 0.1, Eigen::Vector3d::UnitZ()));
     const Eigen::Quaterniond roll_then_matching_yaw_end_q =
         Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitX())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

     double begin_t[3] = {0.0, 0.0, 0.0};
     double matching_end_t[3] = {1.0, 0.0, 0.0};
     double forward_error_end_t[3] = {1.2, 0.0, 0.0};
     double lateral_error_end_t[3] = {1.0, 0.2, 0.0};
     double tilted_yaw_forward_end_t[3] = {0.0, 1.0, 0.5};
     double begin_params[4] = {begin_q.x(), begin_q.y(), begin_q.z(), begin_q.w()};
     double begin_yaw_90_params[4] = {begin_yaw_90.x(), begin_yaw_90.y(),
                                      begin_yaw_90.z(), begin_yaw_90.w()};
     double begin_tilted_yaw_90_params[4] = {
         begin_tilted_yaw_90.x(), begin_tilted_yaw_90.y(),
         begin_tilted_yaw_90.z(), begin_tilted_yaw_90.w()};
     double matching_end_params[4] = {matching_end_q.x(), matching_end_q.y(),
                                      matching_end_q.z(), matching_end_q.w()};
     double tilted_matching_end_params[4] = {
         tilted_matching_end_q.x(), tilted_matching_end_q.y(),
         tilted_matching_end_q.z(), tilted_matching_end_q.w()};
     double yaw_error_end_params[4] = {yaw_error_end_q.x(), yaw_error_end_q.y(),
                                       yaw_error_end_q.z(), yaw_error_end_q.w()};
     double roll_then_matching_yaw_params[4] = {
         roll_then_matching_yaw_end_q.x(), roll_then_matching_yaw_end_q.y(),
         roll_then_matching_yaw_end_q.z(), roll_then_matching_yaw_end_q.w()};
     double residual[3] = {999.0, 999.0, 999.0};

     if (!functor(begin_t, begin_params, matching_end_t, matching_end_params, residual))
          throw std::runtime_error("command motion matching evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "command motion zero forward");
     ExpectNear(residual[1], 0.0, 1e-12, "command motion zero lateral");
     ExpectNear(residual[2], 0.0, 1e-12, "command motion zero yaw");

     if (!functor(begin_t, begin_params, forward_error_end_t, yaw_error_end_params, residual))
          throw std::runtime_error("command motion error evaluate failed");
     ExpectNear(residual[0], 0.4, 1e-12, "command motion forward residual");
     ExpectNear(residual[2], 0.3, 1e-12, "command motion yaw residual");

     if (!functor(begin_t, begin_params, lateral_error_end_t, matching_end_params, residual))
          throw std::runtime_error("command motion lateral error evaluate failed");
     ExpectNear(residual[1], 1.0, 1e-12, "command motion lateral residual");

     if (!functor(begin_t, begin_yaw_90_params, tilted_yaw_forward_end_t,
                  tilted_matching_end_params, residual))
          throw std::runtime_error("command motion yaw-only translation evaluate failed");
     ExpectNear(residual[0], 0.0, 1e-12, "command motion yaw-only forward");
     ExpectNear(residual[1], 0.0, 1e-12, "command motion yaw-only lateral");

     CT_ICP::CommandMotionFunctor roll_then_matching_yaw_functor(
         1.0, YawFromQuaternion(roll_then_matching_yaw_end_q), 2.0, 5.0, 3.0);
     if (!roll_then_matching_yaw_functor(begin_t, begin_params, matching_end_t,
                                         roll_then_matching_yaw_params, residual))
          throw std::runtime_error("command motion mixed roll/yaw evaluate failed");
     ExpectNear(residual[2], 0.0, 1e-12, "command motion uses true heading");

     const Eigen::Quaterniond tilted_heading_q =
         Eigen::Quaterniond(Eigen::AngleAxisd(-0.78, Eigen::Vector3d::UnitX())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(0.79, Eigen::Vector3d::UnitY()));
     CT_ICP::CommandMotionFunctor tilted_heading_functor(
         1.0, YawFromQuaternion(tilted_heading_q), 2.0, 5.0, 3.0);
     double tilted_heading_params[4] = {tilted_heading_q.x(), tilted_heading_q.y(),
                                        tilted_heading_q.z(), tilted_heading_q.w()};
     if (!tilted_heading_functor(begin_t, begin_params, matching_end_t,
                                 tilted_heading_params, residual))
          throw std::runtime_error("command motion tilted heading evaluate failed");
     ExpectNear(residual[2], 0.0, 1e-12, "command motion ignores roll/pitch tilt");
}
}

int main()
{
     TestScaleHessianRawToScaled();
     TestProjectionRunsInScaledSpace();
     TestProjectionBlendUsesScaledSpace();
     TestEffectiveRankUsesTunableMinimum();
     TestDegeneracyClassificationRequiresWeakValueAndLowRank();
     TestMotionPriorWeight();
     TestLidarFrameBeginTimeHonorsHeaderReference();
     TestDiagnosticLoggingScheduleDoesNotDependOnActiveWeighting();
     TestPoseIncrementScaleUsesInverseRotationLength();
     TestObservabilityGatesUseNormalizedEnergy();
     TestDirectionalQualityUsesScaledBasis();
     TestInformationWeightClamp();
     TestBalancedSelectionFillsToMaxResidualBudget();
     TestVoxelBlockTracksFrameAgeAndCanRefreshStalePoints();
     TestCtAutoDiffFactorAppliesExtrinsic();
     TestBodyFrameOutputOffsetRotatesWithPose();
     TestNonHolonomicFunctorUsesBodyFrameDisplacement();
     TestRelativeOrientationFunctorConstrainsScanRotation();
     TestRelativeYawFunctorConstrainsYawOnly();
     TestScanTranslationFunctorConstrainsPredictedDisplacement();
     TestFrameNonHolonomicFunctorUsesPreviousBodyFrame();
     TestWorldZConsistencyFunctorConstrainsEndHeightOnly();
     TestOneShotScalarReferenceInitializesOnlyOnce();
     TestCommandMotionIntegrationUsesPiecewiseConstantSamples();
     TestCommandMotionIntegrationRejectsLargeCommandGap();
     TestCommandMotionFunctorConstrainsPlanarForwardLateralAndYaw();
     std::cout << "test_do_ct_diagnostics passed" << std::endl;
     return 0;
}
