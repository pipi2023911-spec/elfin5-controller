/*
 * Elfin5 V1 控制器使用的投影梯度 QP 求解器。
 * H 必须为对称正定矩阵。每次迭代在关节盒约束与一个奇异规避半空间之间
 * 交替投影，从而保持输出速度满足约束。
 */

#include "elfin5_qp_controller/box_qp_solver.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace elfin5_qp_controller
{

BoxQPSolver::BoxQPSolver(const int max_iterations, const double tolerance)
  : max_iterations_(max_iterations), tolerance_(tolerance)
{
}

Eigen::VectorXd BoxQPSolver::project(
    const Eigen::VectorXd& value,
    const Eigen::VectorXd& lower_bound,
    const Eigen::VectorXd& upper_bound,
    const Eigen::VectorXd* halfspace_normal,
    const double halfspace_offset) const
{
  Eigen::VectorXd projected = value.cwiseMax(lower_bound).cwiseMin(upper_bound);

  if (halfspace_normal == nullptr || halfspace_normal->squaredNorm() < 1e-14)
  {
    return projected;
  }

  // 当前问题的可行域仅为六维盒与一个半空间的交集，交替投影即可满足 V1 需求。
  for (int pass = 0; pass < 8; ++pass)
  {
    const double deficit = halfspace_offset - halfspace_normal->dot(projected);
    if (deficit <= 0.0)
    {
      break;
    }

    projected += (deficit / halfspace_normal->squaredNorm()) * (*halfspace_normal);
    projected = projected.cwiseMax(lower_bound).cwiseMin(upper_bound);
  }
  return projected;
}

QPSolution BoxQPSolver::solve(
    const Eigen::MatrixXd& hessian,
    const Eigen::VectorXd& gradient,
    const Eigen::VectorXd& lower_bound,
    const Eigen::VectorXd& upper_bound,
    const Eigen::VectorXd& initial_guess,
    const Eigen::VectorXd* halfspace_normal,
    const double halfspace_offset) const
{
  QPSolution result;
  const Eigen::Index dimension = gradient.size();

  if (hessian.rows() != dimension || hessian.cols() != dimension ||
      lower_bound.size() != dimension || upper_bound.size() != dimension ||
      initial_guess.size() != dimension)
  {
    result.message = "QP 矩阵或向量维度不一致";
    return result;
  }
  if (halfspace_normal != nullptr && halfspace_normal->size() != dimension)
  {
    result.message = "QP 半空间约束维度不一致";
    return result;
  }
  if (!hessian.allFinite() || !gradient.allFinite() ||
      !lower_bound.allFinite() || !upper_bound.allFinite() ||
      !initial_guess.allFinite() || !std::isfinite(halfspace_offset) ||
      (halfspace_normal != nullptr && !halfspace_normal->allFinite()))
  {
    result.message = "QP 输入包含非有限数值";
    return result;
  }
  if (max_iterations_ <= 0 || !std::isfinite(tolerance_) || tolerance_ <= 0.0)
  {
    result.message = "QP 求解器配置无效";
    return result;
  }
  if ((lower_bound.array() > upper_bound.array()).any())
  {
    result.message = "QP 上下界不可行";
    return result;
  }

  const Eigen::MatrixXd symmetric_hessian = 0.5 * (hessian + hessian.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(symmetric_hessian);
  if (eigen_solver.info() != Eigen::Success || eigen_solver.eigenvalues().minCoeff() <= 0.0)
  {
    result.message = "QP Hessian 矩阵不是正定矩阵";
    return result;
  }

  // 凸二次问题在步长小于 2/L 时收敛；这里使用更保守的 1/L。
  const double lipschitz = eigen_solver.eigenvalues().maxCoeff();
  const double step = 1.0 / std::max(lipschitz, 1e-12);
  Eigen::VectorXd x = project(initial_guess, lower_bound, upper_bound,
                              halfspace_normal, halfspace_offset);

  for (int iteration = 0; iteration < max_iterations_; ++iteration)
  {
    const Eigen::VectorXd objective_gradient = symmetric_hessian * x + gradient;
    const Eigen::VectorXd next = project(x - step * objective_gradient,
                                         lower_bound, upper_bound,
                                         halfspace_normal, halfspace_offset);
    const double projected_gradient_norm = (next - x).norm() / step;
    x = next;

    result.iterations = iteration + 1;
    result.projected_gradient_norm = projected_gradient_norm;
    if (projected_gradient_norm < tolerance_)
    {
      result.converged = true;
      break;
    }
  }

  if (halfspace_normal != nullptr &&
      halfspace_normal->dot(x) + 1e-7 < halfspace_offset)
  {
    result.x = x;
    result.message = "奇异性半空间与关节上下界无法同时满足";
    return result;
  }

  const double constraint_tolerance = std::max(1e-7, tolerance_);
  const bool lower_bound_satisfied =
      (x.array() >= (lower_bound.array() - constraint_tolerance)).all();
  const bool upper_bound_satisfied =
      (x.array() <= (upper_bound.array() + constraint_tolerance)).all();
  result.success = x.allFinite() && lower_bound_satisfied && upper_bound_satisfied;
  result.x = x;
  if (!result.success)
  {
    result.message = "QP 返回无效解或不可行解";
  }
  else if (result.converged)
  {
    result.message = "QP 已收敛";
  }
  else
  {
    // 可行但未收敛的速度仍满足安全边界，可以作为降级结果使用；上层控制器
    // 会发布 QP_DEGRADED，而不是将它误报成完全求解成功。
    result.message = "QP 已达到迭代次数上限，当前解可行但尚未收敛";
  }
  return result;
}

}  // namespace elfin5_qp_controller
