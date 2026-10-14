/*
 * 版权所有 (c) 2026
 *
 * Elfin5 笛卡尔控制器使用的 V1 盒约束二次规划求解器。
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
  // success 表示结果数值有效且满足全部约束，可以安全地下发给控制器。
  // converged 单独表示投影梯度是否达到设定容差，避免把“可行”误写成“已收敛”。
  bool success{false};
  bool converged{false};
  int iterations{0};
  double projected_gradient_norm{0.0};
  Eigen::VectorXd x;
  std::string message;
};

class BoxQPSolver
{
public:
  BoxQPSolver(int max_iterations = 80, double tolerance = 1e-7);

  QPSolution solve(const Eigen::MatrixXd& hessian,
                   const Eigen::VectorXd& gradient,
                   const Eigen::VectorXd& lower_bound,
                   const Eigen::VectorXd& upper_bound,
                   const Eigen::VectorXd& initial_guess,
                   const Eigen::VectorXd* halfspace_normal = nullptr,
                   double halfspace_offset = 0.0) const;

private:
  // 将候选解投影到盒约束及可选的奇异规避半空间中。
  Eigen::VectorXd project(const Eigen::VectorXd& value,
                          const Eigen::VectorXd& lower_bound,
                          const Eigen::VectorXd& upper_bound,
                          const Eigen::VectorXd* halfspace_normal,
                          double halfspace_offset) const;

  int max_iterations_;
  double tolerance_;
};

}  // namespace elfin5_qp_controller

#endif  // ELFIN5_QP_CONTROLLER_BOX_QP_SOLVER_H
