#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "estimation/gauss_newton_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "factor_builders/depth_residual.hpp"
#include "factor_builders/imu_preintegration_residual.hpp"
#include "factor_builders/relative_pose_residual.hpp"
#include "sensor_models/so3.hpp"

using uw::estimation::GaussNewtonSolver;
using uw::estimation::PoseGraphProblem;
using uw::factor_builders::DepthResidual;
using uw::factor_builders::RelativePoseResidual;
using uw::sensor_models::Pose3;

// End-to-end check that PoseGraphProblem + GaussNewtonSolver correctly
// consume real ResidualBlock implementations from
// algorithms/factor_builders/* — not just synthetic test doubles. This is
// the concrete proof that the FactorBuilder -> ResidualBlock ->
// PoseGraphProblem -> solver chain described in the architecture doc
// actually composes.
TEST(PoseGraphSolver, ThreeKeyframeChainConvergesToTruth) {
  Pose3 true_kf0;  // identity
  Pose3 relative_01;
  relative_01.translation = Eigen::Vector3d(1.0, 0.2, -0.1);
  relative_01.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));
  Pose3 relative_12;
  relative_12.translation = Eigen::Vector3d(0.8, -0.3, 0.05);
  relative_12.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));

  const Pose3 true_kf1 = true_kf0 * relative_01;
  const Pose3 true_kf2 = true_kf1 * relative_12;
  const double true_depth = -true_kf2.translation.z();

  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", true_kf0, /*fixed=*/true);

  // Perturbed initial guesses for the free keyframes.
  Pose3 init_kf1 = true_kf1;
  init_kf1.translation += Eigen::Vector3d(0.3, -0.2, 0.15);
  Pose3 init_kf2 = true_kf2;
  init_kf2.translation += Eigen::Vector3d(-0.25, 0.35, -0.1);
  problem.AddKeyframe("kf1", init_kf1);
  problem.AddKeyframe("kf2", init_kf2);

  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_01, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf0", "kf1"});
  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_12, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf1", "kf2"});
  problem.AddResidualBlock(std::make_unique<DepthResidual>(true_depth, /*sqrt_information=*/5.0),
                           {"kf2"});

  GaussNewtonSolver solver;
  const auto summary = solver.Solve(problem);

  EXPECT_LT(summary.final_cost, summary.initial_cost);
  EXPECT_LT(summary.final_cost, 1e-6);

  const Pose3 solved_kf1 = problem.GetKeyframePose("kf1");
  const Pose3 solved_kf2 = problem.GetKeyframePose("kf2");
  EXPECT_NEAR((solved_kf1.translation - true_kf1.translation).norm(), 0.0, 1e-3);
  EXPECT_NEAR((solved_kf2.translation - true_kf2.translation).norm(), 0.0, 1e-3);
}

// Covers the backend-agnostic accessor that replaced the earlier
// `friend class GaussNewtonSolver` — any future solver adapter (e.g. Ceres)
// relies on it having exactly this shape: KeyframeOrder order, correct
// fixed flags, 7-wide pose blocks, and write-through params pointers.
TEST(PoseGraphProblem, MutableAllParameterBlocksMatchKeyframeOrderAndAreWritable) {
  PoseGraphProblem problem;
  Pose3 pose0;
  pose0.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  Pose3 pose1;
  pose1.translation = Eigen::Vector3d(4.0, 5.0, 6.0);
  problem.AddKeyframe("kf0", pose0, /*fixed=*/true);
  problem.AddKeyframe("kf1", pose1, /*fixed=*/false);

  const auto blocks = problem.MutableAllParameterBlocks();
  ASSERT_EQ(blocks.size(), problem.KeyframeOrder().size());
  for (std::size_t i = 0; i < blocks.size(); ++i) {
    EXPECT_EQ(blocks[i].ref.kind, PoseGraphProblem::ParameterKind::kPose);
    EXPECT_EQ(blocks[i].ref.keyframe_id, problem.KeyframeOrder()[i]);
    EXPECT_EQ(blocks[i].size, 7);
    EXPECT_EQ(blocks[i].fixed, problem.IsFixed(blocks[i].ref.keyframe_id));
    ASSERT_NE(blocks[i].params, nullptr);
  }

  // Mutating through the returned pointer is visible via GetKeyframePose —
  // this is exactly what a solver's optimization step relies on.
  auto* kf1_params = blocks[1].params;
  kf1_params[0] = 42.0;
  EXPECT_DOUBLE_EQ(problem.GetKeyframePose("kf1").translation.x(), 42.0);
  EXPECT_EQ(problem.NumInertialStates(), 0u);
}

// Inertial states come AFTER every pose in the block ordering, so adding
// one cannot shift an existing pose's columns.
TEST(PoseGraphProblem, InertialBlocksAreOrderedAfterAllPoses) {
  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", Pose3{});
  problem.AddKeyframe("kf1", Pose3{});
  problem.AddInertialState("kf0", PoseGraphProblem::InertialState{});
  problem.AddKeyframe("kf2", Pose3{});
  problem.AddInertialState("kf2", PoseGraphProblem::InertialState{});

  const auto blocks = problem.MutableAllParameterBlocks();
  ASSERT_EQ(blocks.size(), 5u);
  EXPECT_EQ(blocks[0].ref.keyframe_id, "kf0");
  EXPECT_EQ(blocks[1].ref.keyframe_id, "kf1");
  EXPECT_EQ(blocks[2].ref.keyframe_id, "kf2");
  for (int i = 0; i < 3; ++i) EXPECT_EQ(blocks[i].ref.kind, PoseGraphProblem::ParameterKind::kPose);
  EXPECT_EQ(blocks[3].ref.kind, PoseGraphProblem::ParameterKind::kInertial);
  EXPECT_EQ(blocks[3].ref.keyframe_id, "kf0");
  EXPECT_EQ(blocks[3].size, 9);
  EXPECT_EQ(blocks[4].ref.kind, PoseGraphProblem::ParameterKind::kInertial);
  EXPECT_EQ(blocks[4].ref.keyframe_id, "kf2");
}

TEST(PoseGraphProblem, InertialStateApiRejectsUnknownKeyframesAndUnbackedRefs) {
  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", Pose3{});
  EXPECT_THROW(problem.AddInertialState("nope", PoseGraphProblem::InertialState{}),
               std::out_of_range);
  EXPECT_THROW(problem.GetInertialState("kf0"), std::out_of_range);
  EXPECT_THROW(problem.SetInertialState("kf0", PoseGraphProblem::InertialState{}),
               std::out_of_range);
  // A ParameterRef naming an inertial state that was never added must be
  // refused at bind time, not silently produce a dangling parameter block.
  EXPECT_THROW(problem.AddResidualBlockOnParameters(
                   std::make_unique<DepthResidual>(1.0, 1.0),
                   {PoseGraphProblem::InertialRef("kf0")}),
               std::out_of_range);
}
