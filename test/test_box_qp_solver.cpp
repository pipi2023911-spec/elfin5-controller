#include "elfin5_qp_controller/box_qp_solver.h"

#include <gtest/gtest.h>

namespace elfin5_qp_controller
{

TEST(BoxQPSolver, SolvesBoundedQuadratic)
{
  // 基础用例：无半空间约束时，最优解应被截断在变量上下界上。
  BoxQPSolver solver;
  const Eigen::Matrix2d hessian = 2.0 * Eigen::Matrix2d::Identity();
  Eigen::Vector2d gradient;
  gradient << -4.0, 2.0;
  const Eigen::Vector2d lower = Eigen::Vector2d::Constant(-1.0);
  const Eigen::Vector2d upper = Eigen::Vector2d::Constant(1.0);

  const QPSolution solution = solver.solve(
      hessian, gradient, lower, upper, Eigen::Vector2d::Zero());

  ASSERT_TRUE(solution.success) << solution.message;
  EXPECT_NEAR(solution.x[0], 1.0, 1e-6);
  EXPECT_NEAR(solution.x[1], -1.0, 1e-6);
}

TEST(BoxQPSolver, EnforcesHalfspace)
{
  // 奇异规避用例：验证返回速度满足 a'*x >= b。
  BoxQPSolver solver;
  const Eigen::Matrix2d hessian = Eigen::Matrix2d::Identity();
  const Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  const Eigen::Vector2d lower = Eigen::Vector2d::Constant(-1.0);
  const Eigen::Vector2d upper = Eigen::Vector2d::Constant(1.0);
  Eigen::Vector2d normal;
  normal << 1.0, 1.0;

  const QPSolution solution = solver.solve(
      hessian, gradient, lower, upper, Eigen::Vector2d::Zero(), &normal, 0.5);

  ASSERT_TRUE(solution.success) << solution.message;
  EXPECT_GE(normal.dot(solution.x), 0.5 - 1e-7);
}

TEST(BoxQPSolver, RejectsConflictingBounds)
{
  // 安全用例：下界大于上界时必须拒绝求解，不能输出运动命令。
  BoxQPSolver solver;
  const Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(1, 1);
  const Eigen::VectorXd gradient = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd lower(1);
  Eigen::VectorXd upper(1);
  lower << 1.0;
  upper << -1.0;

  const QPSolution solution = solver.solve(
      hessian, gradient, lower, upper, Eigen::VectorXd::Zero(1));
  EXPECT_FALSE(solution.success);
}

}  // namespace elfin5_qp_controller
