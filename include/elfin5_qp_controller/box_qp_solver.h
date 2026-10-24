/*
 * 版权所有 (c) 2026
 *
 * Elfin5 笛卡尔控制器使用的盒约束 + 单半空间二次规划求解器。
 * 求解目标为 0.5*x'*H*x + g'*x，约束包括变量上下界，并可选择加入
 * 一个线性半空间约束 a'*x >= b。
 *
 * 为降低工控机部署依赖，本实现仅依赖 Eigen；ROS Melodic 目标机不需要额外
 * 安装 OSQP 或 qpOASES。
 */

#ifndef ELFIN5_QP_CONTROLLER_BOX_QP_SOLVER_H
#define ELFIN5_QP_CONTROLLER_BOX_QP_SOLVER_H

#include <Eigen/Core>

#include <string>

namespace elfin5_qp_controller
{

struct QPSolution
{
  // success只表示数值有效且在容差内可行，不证明最优或物理执行安全。
  // converged表示完整归一化KKT证书满足容差且盒子问题已求完。
  bool success{false};
  bool converged{false};
  int iterations{0};
  // 保留旧字段名以兼容调用；现为无量纲KKT残差，不与旧投影残差混比。
  double projected_gradient_norm{0.0};
  Eigen::VectorXd x;
  std::string message;
};

class BoxQPSolver
{
public:
  // 总有效集步数预算，所有乘子子问题共享；iterations不会超过此值。
  BoxQPSolver(int max_iterations = 80, double tolerance = 1e-7);

  QPSolution solve(const Eigen::MatrixXd& hessian,
                   const Eigen::VectorXd& gradient,
                   const Eigen::VectorXd& lower_bound,
                   const Eigen::VectorXd& upper_bound,
                   const Eigen::VectorXd& initial_guess,
                   const Eigen::VectorXd* halfspace_normal = nullptr,
                   double halfspace_offset = 0.0) const;

private:
  int max_iterations_;
  double tolerance_;
};

}  // namespace elfin5_qp_controller

#endif  // ELFIN5_QP_CONTROLLER_BOX_QP_SOLVER_H
