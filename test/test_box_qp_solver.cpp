#include "elfin5_qp_controller/box_qp_solver.h"

#include <gtest/gtest.h>
#include <limits>

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
  // 接口参数类型为 const Eigen::VectorXd*，固定尺寸向量取地址无法隐式转换。
  Eigen::VectorXd normal(2);
  normal << 1.0, 1.0;

  const QPSolution solution = solver.solve(
      hessian, gradient, lower, upper, Eigen::Vector2d::Zero(), &normal, 0.5);

  ASSERT_TRUE(solution.success) << solution.message;
  EXPECT_GE(normal.dot(solution.x), 0.5 - 1e-7);
  // 最优解：min 0.5|x|^2 s.t. x1+x2>=0.5 -> (0.25, 0.25)
  EXPECT_NEAR(solution.x[0], 0.25, 1e-9);
  EXPECT_NEAR(solution.x[1], 0.25, 1e-9);
}

TEST(BoxQPSolver, FeasibleNarrowCornerIsNotRejected)
{
  // V2 交替投影的反例：可行域只剩 x1=1, x2>=0.99 的窄角，V2 误报不可行。
  BoxQPSolver solver;
  const Eigen::Matrix2d hessian = Eigen::Matrix2d::Identity();
  const Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  const Eigen::Vector2d lower = Eigen::Vector2d::Constant(-1.0);
  const Eigen::Vector2d upper = Eigen::Vector2d::Constant(1.0);
  Eigen::VectorXd normal(2);
  normal << 1.0, 0.01;

  const QPSolution solution = solver.solve(
      hessian, gradient, lower, upper, Eigen::Vector2d::Zero(), &normal, 1.0099);

  ASSERT_TRUE(solution.success) << solution.message;
  EXPECT_TRUE(solution.converged);
  EXPECT_NEAR(solution.x[0], 1.0, 1e-8);
  EXPECT_NEAR(solution.x[1], 0.99, 1e-8);
}

TEST(BoxQPSolver, RejectsTrulyInfeasibleHalfspace)
{
  BoxQPSolver solver;
  const Eigen::Matrix2d hessian = Eigen::Matrix2d::Identity();
  const Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  const Eigen::Vector2d lower = Eigen::Vector2d::Constant(-1.0);
  const Eigen::Vector2d upper = Eigen::Vector2d::Constant(1.0);
  Eigen::VectorXd normal(2);
  normal << 1.0, 1.0;  // 盒上 a'x 最大为 2 < 2.5

  const QPSolution solution = solver.solve(
      hessian, gradient, lower, upper, Eigen::Vector2d::Zero(), &normal, 2.5);
  EXPECT_FALSE(solution.success);
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

TEST(BoxQPSolver, ZeroNormalAndEquivalentScaling)
{
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(1,1);
  Eigen::VectorXd z=Eigen::VectorXd::Zero(1),u=Eigen::VectorXd::Ones(1),a(1);
  EXPECT_FALSE(BoxQPSolver().solve(H,z,z,u,z,&z,1).success);
  EXPECT_TRUE(BoxQPSolver().solve(H,z,z,u,z,&z,0).converged);
  for(double scale:{1e-200,5e-8,1.,1e200}) {
    a<<scale;const auto s=BoxQPSolver().solve(H,z,z,u,u,&a,.5*scale);
    ASSERT_TRUE(s.success&&s.converged)<<s.message;
    EXPECT_NEAR(s.x[0],.5,1e-9);EXPECT_LE(s.projected_gradient_norm,1e-7);
  }
}

TEST(BoxQPSolver, NonoptimalAlternatingProjectionCounterexample)
{
  Eigen::Matrix2d H=Eigen::Matrix2d::Identity();
  Eigen::Vector2d g(-2,0),l(0,0),u(1,1);Eigen::VectorXd a(2);a<<-1,1;
  const auto s=BoxQPSolver().solve(H,g,l,u,l,&a,0);
  ASSERT_TRUE(s.converged)<<s.message;
  EXPECT_NEAR(s.x[0],1,1e-9);EXPECT_NEAR(s.x[1],1,1e-9);
}

TEST(BoxQPSolver, WeakNullspaceCurvature)
{
  Eigen::Matrix2d H=Eigen::Matrix2d::Identity();H(1,1)=1e-10;
  Eigen::Vector2d g(0,-.4e-10),l(0,0),u(1,1);
  const auto s=BoxQPSolver().solve(H,g,l,u,l);
  ASSERT_TRUE(s.converged)<<s.message;EXPECT_NEAR(s.x[1],.4,1e-9);
}

TEST(BoxQPSolver, TotalIterationBudget)
{
  Eigen::Matrix2d H=2*Eigen::Matrix2d::Identity();
  Eigen::Vector2d g(-4,2),l(-1,-1),u(1,1),initial(0,0);
  const auto s=BoxQPSolver(1).solve(H,g,l,u,initial);
  EXPECT_LE(s.iterations,1);EXPECT_TRUE(s.success);EXPECT_FALSE(s.converged);
}

TEST(BoxQPSolver, EmptyAndNonfiniteInput)
{
  Eigen::MatrixXd empty(0,0);Eigen::VectorXd z(0);
  EXPECT_FALSE(BoxQPSolver().solve(empty,z,z,z,z).success);
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(1,1);
  Eigen::VectorXd l=Eigen::VectorXd::Zero(1),u=Eigen::VectorXd::Ones(1),bad(1);
  bad<<std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(BoxQPSolver().solve(H,bad,l,u,l).success);
}

}  // namespace elfin5_qp_controller
