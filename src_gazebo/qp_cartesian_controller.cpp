/*
 * Elfin5 笛卡尔 QP 控制器实现，V1。
 *
 * 本控制器只执行笛卡尔位置任务。QP 不使用几何雅可比中的旋转行，因此腕部
 * 姿态保持为自由冗余度，可用于奇异性规避和关节限位规避。
 */

#include "elfin5_qp_controller/qp_cartesian_controller.h"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <moveit/robot_model/joint_model.h>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace elfin5_qp_controller
{
namespace
{

constexpr double kDefaultVelocityLimit = 1.57;
constexpr double kDefaultAccelerationLimit = 12.56;
constexpr double kSmallNumber = 1e-9;

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

diagnostic_msgs::KeyValue makeKeyValue(const std::string& key, const double value)
{
  diagnostic_msgs::KeyValue item;
  item.key = key;
  std::ostringstream stream;
  stream << value;
  item.value = stream.str();
  return item;
}

bool isFinitePoint(const geometry_msgs::Point& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isFinitePose(const geometry_msgs::Pose& pose)
{
  return isFinitePoint(pose.position) &&
      std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
      std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

// 同时检查消息时间戳和本节点的接收时间。接收时间可以覆盖命令行测试中
// header.stamp 为零的情况，避免发布器中断后旧数据被无限期使用。
bool isInputStale(const ros::Time& now,
                  const ros::Time& message_stamp,
                  const ros::Time& receive_time,
                  const double timeout)
{
  if (timeout <= 0.0)
  {
    return false;
  }
  const bool receive_time_stale = receive_time.isZero() ||
      (now - receive_time).toSec() > timeout;
  const bool message_time_stale = !message_stamp.isZero() &&
      (now - message_stamp).toSec() > timeout;
  return receive_time_stale || message_time_stale;
}

}  // namespace

QPCartesianController::QPCartesianController(
    const ros::NodeHandle& node_handle,
    const ros::NodeHandle& private_node_handle)
  : node_handle_(node_handle)
  , private_node_handle_(private_node_handle)
  , qp_solver_(100, 1e-7)
{
  // 集中读取参数，便于在不改代码的情况下完成工控机现场调参。
  private_node_handle_.param<std::string>("planning_group", planning_group_, "elfin_arm");
  private_node_handle_.param<std::string>("base_link", base_link_, "elfin_base_link");
  private_node_handle_.param<std::string>("tip_link", tip_link_, "elfin_end_link");
  private_node_handle_.param<std::string>("absolute_frame", absolute_frame_, "world");
  private_node_handle_.param("control_rate", control_rate_, 50.0);
  private_node_handle_.param("position_gain", position_gain_, 1.5);
  private_node_handle_.param("maximum_cartesian_speed", maximum_cartesian_speed_, 0.15);
  private_node_handle_.param("goal_tolerance", goal_tolerance_, 0.005);
  private_node_handle_.param("position_limit_margin", position_limit_margin_, 0.03);
  private_node_handle_.param("velocity_limit_scale", velocity_limit_scale_, 0.7);
  private_node_handle_.param("acceleration_limit_scale", acceleration_limit_scale_, 0.7);
  private_node_handle_.param("nominal_damping", nominal_damping_, 1e-4);
  private_node_handle_.param("singularity_damping", singularity_damping_, 0.08);
  private_node_handle_.param("singularity_warning_threshold", singularity_warning_threshold_, 0.08);
  private_node_handle_.param("singularity_stop_threshold", singularity_stop_threshold_, 0.015);
  private_node_handle_.param("singularity_safe_threshold", singularity_safe_threshold_, 0.05);
  private_node_handle_.param("singularity_avoidance_gain", singularity_avoidance_gain_, 0.4);
  private_node_handle_.param("joint_centering_gain", joint_centering_gain_, 0.08);
  private_node_handle_.param("finite_difference_step", finite_difference_step_, 1e-4);
  private_node_handle_.param("input_timeout", input_timeout_, 1.0);
  private_node_handle_.param("command_horizon", command_horizon_, 0.14);
  private_node_handle_.param("qp_max_iterations", qp_max_iterations_, 100);
  private_node_handle_.param("qp_tolerance", qp_tolerance_, 1e-7);

  // 参数错误如果拖到控制循环中才暴露，通常会表现为NaN或机械臂突然停止。
  // 因此启动时一次性拒绝无效配置，让问题直接出现在启动日志中。
  if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
      !std::isfinite(position_gain_) || position_gain_ <= 0.0 ||
      !std::isfinite(maximum_cartesian_speed_) || maximum_cartesian_speed_ <= 0.0 ||
      !std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0 ||
      !std::isfinite(position_limit_margin_) || position_limit_margin_ < 0.0 ||
      !std::isfinite(velocity_limit_scale_) || velocity_limit_scale_ <= 0.0 ||
      velocity_limit_scale_ > 1.0 ||
      !std::isfinite(acceleration_limit_scale_) || acceleration_limit_scale_ <= 0.0 ||
      acceleration_limit_scale_ > 1.0 ||
      !std::isfinite(nominal_damping_) || nominal_damping_ <= 0.0 ||
      !std::isfinite(singularity_damping_) || singularity_damping_ < 0.0 ||
      !std::isfinite(singularity_avoidance_gain_) || singularity_avoidance_gain_ <= 0.0 ||
      !std::isfinite(joint_centering_gain_) || joint_centering_gain_ < 0.0 ||
      !std::isfinite(finite_difference_step_) || finite_difference_step_ <= 0.0 ||
      !std::isfinite(input_timeout_) || input_timeout_ <= 0.0 ||
      !std::isfinite(command_horizon_) || command_horizon_ <= 0.0 ||
      qp_max_iterations_ <= 0 ||
      !std::isfinite(qp_tolerance_) || qp_tolerance_ <= 0.0)
  {
    throw std::invalid_argument("控制参数必须为有限正数");
  }
  if (!(0.0 <= singularity_stop_threshold_ &&
        singularity_stop_threshold_ < singularity_safe_threshold_ &&
        singularity_safe_threshold_ <= singularity_warning_threshold_))
  {
    throw std::invalid_argument(
        "奇异性阈值必须满足 0 <= stop < safe <= warning");
  }

  // 求解器参数由YAML控制，实验时可以根据QP_DEGRADED比例调整，而无需改代码。
  qp_solver_ = BoxQPSolver(qp_max_iterations_, qp_tolerance_);

  if (!loadRobotModel())
  {
    throw std::runtime_error("Elfin5 MoveIt 机器人模型初始化失败");
  }

  // 输入与输出接口名称保持固定，方便视觉端和主控台提前联调。
  joint_state_subscriber_ = node_handle_.subscribe(
      "/joint_states", 1, &QPCartesianController::jointStateCallback, this);
  base_pose_subscriber_ = private_node_handle_.subscribe(
      "base_pose", 1, &QPCartesianController::basePoseCallback, this);
  target_point_subscriber_ = private_node_handle_.subscribe(
      "target_point", 1, &QPCartesianController::targetPointCallback, this);

  command_publisher_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(
      "/elfin_arm_controller/command", 1);
  current_point_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>(
      "current_point", 1);
  position_error_publisher_ = private_node_handle_.advertise<std_msgs::Float64>(
      "position_error", 1);
  minimum_singular_value_publisher_ = private_node_handle_.advertise<std_msgs::Float64>(
      "min_singular_value", 1);
  diagnostics_publisher_ = private_node_handle_.advertise<diagnostic_msgs::DiagnosticArray>(
      "state", 1, true);
  enable_server_ = private_node_handle_.advertiseService(
      "enable", &QPCartesianController::enableCallback, this);

  control_timer_ = node_handle_.createTimer(
      ros::Duration(1.0 / std::max(control_rate_, 1.0)),
      &QPCartesianController::controlTimerCallback, this);

  publishDiagnostics(ControllerState::IDLE, "控制器已初始化", 0.0, 0.0,
                     ros::Time::now());
  ROS_INFO_STREAM("Elfin5 QP 控制器已初始化，关节数：" << joint_names_.size()
                  << "；规划组：" << planning_group_
                  << "，基座：" << base_link_ << "，末端：" << tip_link_
                  << "，Jacobian 参考链接：" << jacobian_reference_link_model_->getName());
}

bool QPCartesianController::loadRobotModel()
{
  // RobotModelLoader 会同时读取 URDF、SRDF 以及 MoveIt 的关节限制参数。
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  robot_model_ = robot_model_loader_->getModel();
  if (!robot_model_)
  {
    ROS_ERROR("未能从 robot_description 加载 MoveIt 机器人模型，请检查参数及重映射");
    return false;
  }

  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
  if (joint_model_group_ == nullptr)
  {
    ROS_ERROR_STREAM("MoveIt 规划组不存在：" << planning_group_);
    return false;
  }

  tip_link_model_ = robot_model_->getLinkModel(tip_link_);
  if (tip_link_model_ == nullptr)
  {
    ROS_ERROR_STREAM("模型中不存在末端链接：" << tip_link_);
    return false;
  }

  base_link_model_ = robot_model_->getLinkModel(base_link_);
  if (base_link_model_ == nullptr)
  {
    ROS_ERROR_STREAM("模型中不存在基座链接：" << base_link_);
    return false;
  }

  // MoveIt的getJacobian()不是固定返回model frame坐标；它使用规划组第一个
  // 关节的父链接作为参考坐标。本项目的SRDF首关节是elfin_joint1，其父链接
  // 为elfin_base，而外部base接口使用elfin_base_link，两者相差一个固定旋转。
  const std::vector<const moveit::core::JointModel*>& group_joint_models =
      joint_model_group_->getJointModels();
  if (group_joint_models.empty())
  {
    ROS_ERROR_STREAM("MoveIt 规划组中没有关节：" << planning_group_);
    return false;
  }
  jacobian_reference_link_model_ = group_joint_models.front()->getParentLinkModel();
  if (jacobian_reference_link_model_ == nullptr)
  {
    ROS_ERROR_STREAM("无法确定规划组的 Jacobian 参考链接："
                     << planning_group_);
    return false;
  }

  joint_names_ = joint_model_group_->getVariableNames();
  const Eigen::Index count = static_cast<Eigen::Index>(joint_names_.size());
  minimum_positions_.resize(count);
  maximum_positions_.resize(count);
  maximum_velocities_.resize(count);
  maximum_accelerations_.resize(count);
  previous_velocity_command_ = Eigen::VectorXd::Zero(count);

  for (Eigen::Index index = 0; index < count; ++index)
  {
    // 位置和速度优先采用模型数据；缺失时才使用 Elfin5 的保守默认值。
    const moveit::core::VariableBounds& bounds =
        robot_model_->getVariableBounds(joint_names_[static_cast<std::size_t>(index)]);

    minimum_positions_[index] = bounds.position_bounded_
        ? bounds.min_position_ : -std::numeric_limits<double>::infinity();
    maximum_positions_[index] = bounds.position_bounded_
        ? bounds.max_position_ : std::numeric_limits<double>::infinity();
    maximum_velocities_[index] = bounds.velocity_bounded_
        ? std::abs(bounds.max_velocity_) : kDefaultVelocityLimit;

    // 部分旧版 MoveIt 不提供完整加速度边界，因此 V1 以 ROS 参数作为统一来源。
    maximum_accelerations_[index] = kDefaultAccelerationLimit;
  }

  std::vector<double> configured_velocity_limits;
  if (private_node_handle_.getParam("joint_velocity_limits", configured_velocity_limits) &&
      configured_velocity_limits.size() == joint_names_.size())
  {
    for (Eigen::Index index = 0; index < count; ++index)
    {
      maximum_velocities_[index] = configured_velocity_limits[static_cast<std::size_t>(index)];
    }
  }

  std::vector<double> configured_acceleration_limits;
  if (private_node_handle_.getParam("joint_acceleration_limits", configured_acceleration_limits) &&
      configured_acceleration_limits.size() == joint_names_.size())
  {
    for (Eigen::Index index = 0; index < count; ++index)
    {
      maximum_accelerations_[index] = configured_acceleration_limits[static_cast<std::size_t>(index)];
    }
  }

  if (!maximum_velocities_.allFinite() || !maximum_accelerations_.allFinite() ||
      (maximum_velocities_.array() <= 0.0).any() ||
      (maximum_accelerations_.array() <= 0.0).any())
  {
    ROS_ERROR("关节速度和加速度限值必须为有限正数");
    return false;
  }

  robot_state_.reset(new moveit::core::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  return true;
}

void QPCartesianController::jointStateCallback(
    const sensor_msgs::JointStateConstPtr& message)
{
  if (message->name.empty() || message->position.empty() ||
      !std::all_of(message->position.begin(), message->position.end(),
                   [](const double value) { return std::isfinite(value); }))
  {
    ROS_ERROR_THROTTLE(1.0, "拒绝 joint_states：数据缺失或包含非有限数值");
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_joint_state_ = *message;
  latest_joint_state_receive_time_ = ros::Time::now();
  have_joint_state_ = true;
}

void QPCartesianController::basePoseCallback(
    const geometry_msgs::PoseStampedConstPtr& message)
{
  if (!isFinitePose(message->pose))
  {
    ROS_ERROR_THROTTLE(1.0, "拒绝 base_pose：包含非有限数值");
    return;
  }
  // 零四元数无法归一化，会使绝对坐标到基座坐标的变换产生 NaN。
  const double quaternion_norm = std::sqrt(
      message->pose.orientation.x * message->pose.orientation.x +
      message->pose.orientation.y * message->pose.orientation.y +
      message->pose.orientation.z * message->pose.orientation.z +
      message->pose.orientation.w * message->pose.orientation.w);
  if (quaternion_norm < 1e-6)
  {
    ROS_ERROR_THROTTLE(1.0, "拒绝 base_pose：四元数长度为零");
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_base_pose_ = *message;
  latest_base_pose_receive_time_ = ros::Time::now();
  have_base_pose_ = true;
}

void QPCartesianController::targetPointCallback(
    const geometry_msgs::PointStampedConstPtr& message)
{
  if (!isFinitePoint(message->point))
  {
    ROS_ERROR_THROTTLE(1.0, "拒绝 target_point：包含非有限数值");
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_target_point_ = *message;
  latest_target_point_receive_time_ = ros::Time::now();
  have_target_point_ = true;
}

bool QPCartesianController::enableCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response)
{
  // 锁顺序固定为 control_mutex_ -> data_mutex_，与控制周期保持一致，避免死锁。
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  std::lock_guard<std::mutex> data_lock(data_mutex_);

  // disable始终允许执行。enable则是实际运动的最后一道门禁：缺少输入、
  // 坐标系不一致或输入过期时直接拒绝，不让实验人员误以为已进入闭环。
  if (request.data)
  {
    const auto reject_enable = [this, &response](const std::string& reason)
    {
      enabled_ = false;
      previous_velocity_command_.setZero();
      hold_command_pending_ = true;
      response.success = false;
      response.message = "启用请求被拒绝：" + reason;
      return true;
    };

    if (!have_joint_state_ || !have_base_pose_ || !have_target_point_)
    {
      return reject_enable("必须同时收到 joint_states、base_pose 和 target_point");
    }
    if (latest_base_pose_.header.frame_id != absolute_frame_ ||
        latest_target_point_.header.frame_id != absolute_frame_)
    {
      return reject_enable("基座位姿和目标点必须使用绝对坐标系 absolute_frame=" +
                           absolute_frame_);
    }

    const ros::Time now = ros::Time::now();
    if (isInputStale(now, latest_joint_state_.header.stamp,
                     latest_joint_state_receive_time_, input_timeout_) ||
        isInputStale(now, latest_base_pose_.header.stamp,
                     latest_base_pose_receive_time_, input_timeout_) ||
        isInputStale(now, latest_target_point_.header.stamp,
                     latest_target_point_receive_time_, input_timeout_))
    {
      return reject_enable("至少一类输入超过 input_timeout");
    }

    std::vector<double> joint_positions;
    std::string joint_state_error;
    if (!extractJointPositions(latest_joint_state_, joint_positions, joint_state_error))
    {
      return reject_enable(joint_state_error);
    }
  }

  enabled_ = request.data;
  previous_velocity_command_.setZero();
  hold_command_pending_ = !request.data;
  response.success = true;
  response.message = enabled_ ? "Elfin5 QP 计算已启用；真机输出仍须网关独立放行" : "Elfin5 QP 计算已关闭";
  return true;
}

bool QPCartesianController::updateRobotState(
    const std::vector<double>& joint_positions)
{
  if (joint_positions.size() != joint_names_.size())
  {
    return false;
  }
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  robot_state_->update();
  return true;
}

bool QPCartesianController::extractJointPositions(
    const sensor_msgs::JointState& joint_state,
    std::vector<double>& joint_positions,
    std::string& error_message) const
{
  // JointState 中关节顺序由发布器决定，不能假设它与MoveIt规划组顺序相同。
  // 这里先按名称建立查找表，再严格重排为 joint_names_ 的顺序。
  std::unordered_map<std::string, double> positions_by_name;
  for (std::size_t index = 0;
       index < joint_state.name.size() && index < joint_state.position.size(); ++index)
  {
    if (!std::isfinite(joint_state.position[index]))
    {
      error_message = "joint_states 中的关节位置不是有限数值";
      return false;
    }
    positions_by_name[joint_state.name[index]] = joint_state.position[index];
  }

  joint_positions.assign(joint_names_.size(), 0.0);
  for (std::size_t index = 0; index < joint_names_.size(); ++index)
  {
    const auto iterator = positions_by_name.find(joint_names_[index]);
    if (iterator == positions_by_name.end())
    {
      error_message = "joint_states 缺少关节：" + joint_names_[index];
      return false;
    }
    joint_positions[index] = iterator->second;
  }
  return true;
}

bool QPCartesianController::computeKinematics(
    const std::vector<double>& joint_positions,
    Eigen::Vector3d& end_point_in_base,
    Eigen::MatrixXd& translation_jacobian)
{
  if (!updateRobotState(joint_positions))
  {
    return false;
  }

  // getGlobalLinkTransform() 返回 model frame -> link。通过显式相对变换，将
  // 末端位置统一表达在配置的base_link坐标系，而不是依赖URDF根链接名称。
  const Eigen::Isometry3d model_from_base =
      robot_state_->getGlobalLinkTransform(base_link_model_);
  const Eigen::Isometry3d model_from_tip =
      robot_state_->getGlobalLinkTransform(tip_link_model_);
  end_point_in_base = (model_from_base.inverse() * model_from_tip).translation();

  Eigen::MatrixXd geometric_jacobian;
  if (!robot_state_->getJacobian(joint_model_group_, tip_link_model_,
                                 Eigen::Vector3d::Zero(), geometric_jacobian, false))
  {
    return false;
  }
  if (geometric_jacobian.rows() < 3 ||
      geometric_jacobian.cols() != static_cast<Eigen::Index>(joint_names_.size()))
  {
    return false;
  }
  // MoveIt返回的Jacobian表达在规划组首关节父链接中。Elfin5模型中该链接为
  // elfin_base，而接口基座默认为 elfin_base_link；实际固定变换从当前模型读取，
  // 不假定相差 90 度。这里只旋转速度坐标轴，使 Jv 与位置误差处于同一基座坐标系。
  const Eigen::Matrix3d model_from_jacobian_rotation =
      robot_state_->getGlobalLinkTransform(jacobian_reference_link_model_).linear();
  const Eigen::Matrix3d base_from_jacobian_rotation =
      model_from_base.linear().transpose() * model_from_jacobian_rotation;
  translation_jacobian =
      base_from_jacobian_rotation * geometric_jacobian.topRows(3);
  return true;
}

double QPCartesianController::computeMinimumSingularValue(
    const Eigen::MatrixXd& translation_jacobian) const
{
  Eigen::JacobiSVD<Eigen::MatrixXd> singular_value_decomposition(
      translation_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (singular_value_decomposition.singularValues().size() == 0)
  {
    return 0.0;
  }
  return singular_value_decomposition.singularValues().minCoeff();
}

Eigen::VectorXd QPCartesianController::computeSingularValueGradient(
    const std::vector<double>& joint_positions)
{
  // 常规情况下使用中央差分估计 sigma_min 对各关节的导数。精确奇异点处
  // sigma_min 可能不可微，正负扰动的函数值相同会让中央差分错误地变成零；
  // 因此同时保存两侧函数值，必要时用“哪一侧能提高sigma”构造恢复方向。
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(joint_positions.size()));
  Eigen::VectorXd positive_sigma = Eigen::VectorXd::Constant(
      gradient.size(), -std::numeric_limits<double>::infinity());
  Eigen::VectorXd negative_sigma = positive_sigma;
  Eigen::VectorXd positive_step = Eigen::VectorXd::Zero(gradient.size());
  Eigen::VectorXd negative_step = Eigen::VectorXd::Zero(gradient.size());

  Eigen::Vector3d ignored_point;
  Eigen::MatrixXd current_jacobian;
  if (!computeKinematics(joint_positions, ignored_point, current_jacobian))
  {
    return gradient;
  }
  const double current_sigma = computeMinimumSingularValue(current_jacobian);

  for (std::size_t index = 0; index < joint_positions.size(); ++index)
  {
    std::vector<double> positive = joint_positions;
    std::vector<double> negative = joint_positions;
    const Eigen::Index eigen_index = static_cast<Eigen::Index>(index);
    double step_up = finite_difference_step_;
    double step_down = finite_difference_step_;
    if (std::isfinite(maximum_positions_[eigen_index]))
    {
      step_up = std::min(step_up, maximum_positions_[eigen_index] - positive[index]);
    }
    if (std::isfinite(minimum_positions_[eigen_index]))
    {
      step_down = std::min(step_down, negative[index] - minimum_positions_[eigen_index]);
    }
    step_up = std::max(0.0, step_up);
    step_down = std::max(0.0, step_down);

    Eigen::MatrixXd positive_jacobian;
    Eigen::MatrixXd negative_jacobian;
    if (step_up > kSmallNumber)
    {
      positive[index] += step_up;
      if (computeKinematics(positive, ignored_point, positive_jacobian))
      {
        positive_sigma[eigen_index] = computeMinimumSingularValue(positive_jacobian);
        positive_step[eigen_index] = step_up;
      }
    }
    if (step_down > kSmallNumber)
    {
      negative[index] -= step_down;
      if (computeKinematics(negative, ignored_point, negative_jacobian))
      {
        negative_sigma[eigen_index] = computeMinimumSingularValue(negative_jacobian);
        negative_step[eigen_index] = step_down;
      }
    }

    if (positive_step[eigen_index] > 0.0 && negative_step[eigen_index] > 0.0)
    {
      gradient[eigen_index] =
          (positive_sigma[eigen_index] - negative_sigma[eigen_index]) /
          (positive_step[eigen_index] + negative_step[eigen_index]);
    }
    else if (positive_step[eigen_index] > 0.0)
    {
      gradient[eigen_index] =
          (positive_sigma[eigen_index] - current_sigma) / positive_step[eigen_index];
    }
    else if (negative_step[eigen_index] > 0.0)
    {
      gradient[eigen_index] =
          (current_sigma - negative_sigma[eigen_index]) / negative_step[eigen_index];
    }
  }

  if (gradient.norm() <= 1e-8)
  {
    Eigen::VectorXd recovery_direction = Eigen::VectorXd::Zero(gradient.size());
    for (Eigen::Index index = 0; index < gradient.size(); ++index)
    {
      const double positive_improvement = positive_sigma[index] - current_sigma;
      const double negative_improvement = negative_sigma[index] - current_sigma;
      if (positive_step[index] > 0.0 && positive_improvement > 1e-10 &&
          positive_improvement >= negative_improvement)
      {
        recovery_direction[index] = positive_improvement / positive_step[index];
      }
      else if (negative_step[index] > 0.0 && negative_improvement > 1e-10)
      {
        // 负向扰动能够提高sigma，因此恢复速度的关节方向也应为负。
        recovery_direction[index] = -negative_improvement / negative_step[index];
      }
    }
    gradient = recovery_direction;
  }

  // 有限差分会多次修改共享 RobotState，计算完成后恢复到真实关节状态。
  updateRobotState(joint_positions);
  return gradient;
}

Eigen::VectorXd QPCartesianController::computeJointCenteringVelocity(
    const Eigen::VectorXd& positions) const
{
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(positions.size());
  for (Eigen::Index index = 0; index < positions.size(); ++index)
  {
    if (!std::isfinite(minimum_positions_[index]) ||
        !std::isfinite(maximum_positions_[index]))
    {
      continue;
    }
    const double middle = 0.5 * (minimum_positions_[index] + maximum_positions_[index]);
    const double half_range = 0.5 * (maximum_positions_[index] - minimum_positions_[index]);
    if (half_range > kSmallNumber)
    {
      velocity[index] = joint_centering_gain_ * (middle - positions[index]) /
                        (half_range * half_range);
    }
  }
  return velocity;
}

bool QPCartesianController::buildVelocityBounds(
    const Eigen::VectorXd& positions,
    const double control_dt,
    const double command_horizon,
    Eigen::VectorXd& lower_bound,
    Eigen::VectorXd& upper_bound) const
{
  lower_bound.resize(positions.size());
  upper_bound.resize(positions.size());

  for (Eigen::Index index = 0; index < positions.size(); ++index)
  {
    // 先建立对称速度限制，再依次与位置预测限制和加速度限制求交集。
    const double velocity_limit = velocity_limit_scale_ * maximum_velocities_[index];
    double lower = -velocity_limit;
    double upper = velocity_limit;

    if (std::isfinite(minimum_positions_[index]))
    {
      lower = std::max(lower,
          (minimum_positions_[index] + position_limit_margin_ - positions[index]) /
          command_horizon);
    }
    if (std::isfinite(maximum_positions_[index]))
    {
      upper = std::min(upper,
          (maximum_positions_[index] - position_limit_margin_ - positions[index]) /
          command_horizon);
    }

    const double acceleration_step =
        acceleration_limit_scale_ * maximum_accelerations_[index] * control_dt;
    lower = std::max(lower, previous_velocity_command_[index] - acceleration_step);
    upper = std::min(upper, previous_velocity_command_[index] + acceleration_step);

    lower_bound[index] = lower;
    upper_bound[index] = upper;
  }
  return (lower_bound.array() <= upper_bound.array()).all();
}

Eigen::Isometry3d QPCartesianController::poseToIsometry(
    const geometry_msgs::Pose& pose) const
{
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

void QPCartesianController::publishTrajectoryCommand(
    const Eigen::VectorXd& positions,
    const Eigen::VectorXd& velocities,
    const double horizon)
{
  // 每个周期只发送一个短时域轨迹点，现有 Gazebo 轨迹控制器负责平滑跟踪。
  trajectory_msgs::JointTrajectory command;
  command.header.stamp = ros::Time::now();
  command.joint_names = joint_names_;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(static_cast<std::size_t>(positions.size()));
  point.velocities.resize(static_cast<std::size_t>(velocities.size()));
  for (Eigen::Index index = 0; index < positions.size(); ++index)
  {
    point.positions[static_cast<std::size_t>(index)] = positions[index];
    point.velocities[static_cast<std::size_t>(index)] = velocities[index];
  }
  point.time_from_start = ros::Duration(horizon);
  command.points.push_back(point);
  command_publisher_.publish(command);
}

void QPCartesianController::publishZeroVelocityHold(const Eigen::VectorXd& positions)
{
  publishTrajectoryCommand(positions, Eigen::VectorXd::Zero(positions.size()),
                           command_horizon_);
  previous_velocity_command_.setZero();
}

std::string QPCartesianController::stateToString(const ControllerState state) const
{
  switch (state)
  {
    case ControllerState::IDLE: return "IDLE";
    case ControllerState::WAITING_FOR_INPUT: return "WAITING_FOR_INPUT";
    case ControllerState::RUNNING: return "RUNNING";
    case ControllerState::REACHED: return "REACHED";
    case ControllerState::SINGULARITY_RECOVERY: return "SINGULARITY_RECOVERY";
    case ControllerState::SINGULARITY_STOP: return "SINGULARITY_STOP";
    case ControllerState::QP_INFEASIBLE: return "QP_INFEASIBLE";
    case ControllerState::QP_DEGRADED: return "QP_DEGRADED";
    case ControllerState::INVALID_INPUT: return "INVALID_INPUT";
    case ControllerState::STALE_INPUT: return "STALE_INPUT";
  }
  return "UNKNOWN";
}

void QPCartesianController::publishDiagnostics(
    const ControllerState state,
    const std::string& detail,
    const double position_error,
    const double minimum_singular_value,
    const ros::Time& stamp,
    const int qp_iterations,
    const double qp_residual)
{
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "elfin5_qp_controller";
  status.hardware_id = "elfin5_gazebo";
  status.message = stateToString(state) + ": " + detail;
  status.level = diagnostic_msgs::DiagnosticStatus::OK;
  if (state == ControllerState::SINGULARITY_STOP ||
      state == ControllerState::QP_INFEASIBLE ||
      state == ControllerState::INVALID_INPUT ||
      state == ControllerState::STALE_INPUT)
  {
    status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }
  else if (state == ControllerState::WAITING_FOR_INPUT ||
           state == ControllerState::SINGULARITY_RECOVERY ||
           state == ControllerState::QP_DEGRADED)
  {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  status.values.push_back(makeKeyValue("position_error_m", position_error));
  status.values.push_back(makeKeyValue("minimum_singular_value", minimum_singular_value));
  status.values.push_back(makeKeyValue("qp_iterations", qp_iterations));
  status.values.push_back(makeKeyValue("qp_projected_gradient_norm", qp_residual));
  array.status.push_back(status);
  diagnostics_publisher_.publish(array);
}

void QPCartesianController::controlTimerCallback(const ros::TimerEvent& event)
{
  // 如果上一个周期仍未结束，本周期直接跳过，避免两个线程同时修改RobotState。
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  if (!control_lock.owns_lock())
  {
    ROS_WARN_THROTTLE(1.0, "跳过与上一周期重叠的控制周期");
    return;
  }

  // 在data_mutex_内只复制共享消息，运动学和QP计算均在释放该锁后执行。
  sensor_msgs::JointState joint_state;
  geometry_msgs::PoseStamped base_pose;
  geometry_msgs::PointStamped target_point;
  ros::Time joint_state_receive_time;
  ros::Time base_pose_receive_time;
  ros::Time target_point_receive_time;
  bool enabled = false;
  bool have_joint_state = false;
  bool have_base_pose = false;
  bool have_target_point = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    enabled = enabled_;
    have_joint_state = have_joint_state_;
    have_base_pose = have_base_pose_;
    have_target_point = have_target_point_;
    joint_state = latest_joint_state_;
    base_pose = latest_base_pose_;
    target_point = latest_target_point_;
    joint_state_receive_time = latest_joint_state_receive_time_;
    base_pose_receive_time = latest_base_pose_receive_time_;
    target_point_receive_time = latest_target_point_receive_time_;
  }

  const ros::Time now = ros::Time::now();
  if (!have_joint_state)
  {
    publishDiagnostics(ControllerState::WAITING_FOR_INPUT,
                       "等待关节反馈 joint_states", 0.0, 0.0, now);
    return;
  }

  std::vector<double> joint_positions;
  std::string joint_state_error;
  if (!extractJointPositions(joint_state, joint_positions, joint_state_error))
  {
    publishDiagnostics(ControllerState::INVALID_INPUT, joint_state_error,
                       0.0, 0.0, now);
    return;
  }
  const Eigen::VectorXd positions = Eigen::Map<Eigen::VectorXd>(
      joint_positions.data(), static_cast<Eigen::Index>(joint_positions.size()));

  // 即使控制器尚未enable，也先完成base和关节状态检查。这样实验人员可以在
  // 机械臂保持不动时读取current_point和min_singular_value，再选择安全目标点。
  if (!have_base_pose)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::WAITING_FOR_INPUT,
                       "等待基座位姿 base_pose", 0.0, 0.0, now);
    return;
  }

  const bool joint_state_stale = isInputStale(
      now, joint_state.header.stamp, joint_state_receive_time, input_timeout_);
  const bool base_stale = isInputStale(
      now, base_pose.header.stamp, base_pose_receive_time, input_timeout_);
  if (joint_state_stale || base_stale)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::STALE_INPUT,
                       "关节反馈或基座位姿超过 input_timeout",
                       0.0, 0.0, now);
    return;
  }

  if (base_pose.header.frame_id != absolute_frame_)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::INVALID_INPUT,
                       "基座位姿必须使用绝对坐标系 absolute_frame=" + absolute_frame_,
                       0.0, 0.0, now);
    return;
  }

  Eigen::Vector3d current_point_in_base;
  Eigen::MatrixXd translation_jacobian;
  if (!computeKinematics(joint_positions, current_point_in_base, translation_jacobian))
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::QP_INFEASIBLE,
                       "运动学计算失败", 0.0, 0.0, now);
    return;
  }

  const double minimum_singular_value =
      computeMinimumSingularValue(translation_jacobian);
  if (!current_point_in_base.allFinite() || !translation_jacobian.allFinite() ||
      !std::isfinite(minimum_singular_value))
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::INVALID_INPUT,
                       "运动学计算产生非有限数值", 0.0, 0.0, now);
    return;
  }

  // current_point和奇异值属于只读监控量，在enable之前也持续发布。首次实验应先
  // 记录这两个值，再把目标设置到current_point附近，最后才允许运动。
  // world_from_base 表示基座到配置绝对系的刚体变换：真机为 absolute_world，仿真为 world。
  // 保留完整厂家模型后，相对基座的 FK 和 Jacobian 计算不变，不重复叠加模型根变换。
  const Eigen::Isometry3d world_from_base = poseToIsometry(base_pose.pose);
  geometry_msgs::PointStamped current_point_message;
  current_point_message.header.stamp = now;
  current_point_message.header.frame_id = absolute_frame_;
  const Eigen::Vector3d current_point_in_world = world_from_base * current_point_in_base;
  current_point_message.point.x = current_point_in_world.x();
  current_point_message.point.y = current_point_in_world.y();
  current_point_message.point.z = current_point_in_world.z();
  current_point_publisher_.publish(current_point_message);

  std_msgs::Float64 scalar_message;
  scalar_message.data = minimum_singular_value;
  minimum_singular_value_publisher_.publish(scalar_message);

  // disable时先下发一次当前位置保持命令，用来覆盖上一个尚未完成的
  // QP轨迹点；之后不再发布QP命令，便于实验人员用MoveIt设置初始姿态。
  if (!enabled)
  {
    if (hold_command_pending_)
    {
      publishZeroVelocityHold(positions);
      hold_command_pending_ = false;
    }
    publishDiagnostics(ControllerState::IDLE, "计算开关处于关闭状态",
                       0.0, minimum_singular_value, now);
    return;
  }

  if (!have_target_point)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::WAITING_FOR_INPUT,
                       "等待目标点 target_point", 0.0,
                       minimum_singular_value, now);
    return;
  }

  const bool target_stale = isInputStale(
      now, target_point.header.stamp, target_point_receive_time, input_timeout_);
  if (target_stale)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::STALE_INPUT,
                       "目标点超过 input_timeout", 0.0,
                       minimum_singular_value, now);
    return;
  }
  if (target_point.header.frame_id != absolute_frame_)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::INVALID_INPUT,
                       "目标点必须使用绝对坐标系 absolute_frame=" + absolute_frame_,
                       0.0, minimum_singular_value, now);
    return;
  }

  // 主控台给出 absolute<-base 变换；目标点先转换到 base 坐标再进入 QP。
  const Eigen::Vector3d target_point_in_world(
      target_point.point.x, target_point.point.y, target_point.point.z);
  const Eigen::Vector3d target_point_in_base =
      world_from_base.inverse() * target_point_in_world;
  const Eigen::Vector3d position_error_vector = target_point_in_base - current_point_in_base;
  const double position_error = position_error_vector.norm();
  if (!target_point_in_base.allFinite() || !std::isfinite(position_error))
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::INVALID_INPUT,
                       "目标坐标变换产生非有限数值", 0.0,
                       minimum_singular_value, now);
    return;
  }
  scalar_message.data = position_error;
  position_error_publisher_.publish(scalar_message);

  if (position_error <= goal_tolerance_)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::REACHED, "末端位置误差已进入目标容差",
                       position_error, minimum_singular_value, now);
    return;
  }
  double dt = (event.current_real - event.last_real).toSec();
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
  {
    dt = 1.0 / std::max(control_rate_, 1.0);
  }
  // 时域覆盖厂家接收间隔；dt仍单独用于相邻QP速度变化约束。
  const double command_horizon = command_horizon_;

  Eigen::Vector3d desired_cartesian_velocity = position_gain_ * position_error_vector;
  // 限制笛卡尔速度模长，避免远距离目标产生过大的瞬时速度请求。
  if (desired_cartesian_velocity.norm() > maximum_cartesian_speed_)
  {
    desired_cartesian_velocity *= maximum_cartesian_speed_ /
                                  desired_cartesian_velocity.norm();
  }

  const double warning_span = std::max(
      singularity_warning_threshold_ - singularity_stop_threshold_, 1e-6);
  const double singularity_ratio = clampValue(
      (singularity_warning_threshold_ - minimum_singular_value) / warning_span,
      0.0, 1.0);
  const double damping = nominal_damping_ +
      singularity_damping_ * singularity_ratio * singularity_ratio;

  const Eigen::VectorXd singularity_gradient =
      computeSingularValueGradient(joint_positions);
  if (minimum_singular_value <= singularity_stop_threshold_ &&
      singularity_gradient.norm() <= 1e-8)
  {
    // Gazebo 默认零位可能正好处于奇异位形，不能仅因 sigma 较小就立即停机；
    // 只有局部不存在提高最小奇异值的方向时才停止。
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::SINGULARITY_STOP,
                       "当前局部方向无法提高最小奇异值",
                       position_error, minimum_singular_value, now);
    return;
  }
  const Eigen::VectorXd preferred_velocity =
      computeJointCenteringVelocity(positions) +
      singularity_avoidance_gain_ * singularity_ratio * singularity_gradient;

  // QP 目标函数：0.5*qdot'*(J'J + damping*I)*qdot
  //             + (-J'v - damping*qdot_preferred)'*qdot。
  const Eigen::MatrixXd hessian = translation_jacobian.transpose() *
      translation_jacobian + damping * Eigen::MatrixXd::Identity(
          positions.size(), positions.size());
  const Eigen::VectorXd gradient =
      -translation_jacobian.transpose() * desired_cartesian_velocity -
      damping * preferred_velocity;

  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  if (!buildVelocityBounds(positions, dt, command_horizon,
                           lower_bound, upper_bound))
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::QP_INFEASIBLE,
                       "关节位置、速度或加速度约束相互冲突",
                       position_error, minimum_singular_value, now);
    return;
  }

  // 进入警戒区后加入线性化约束，要求下一时刻的 sigma_min 朝安全值增加。
  const Eigen::VectorXd* halfspace_normal = nullptr;
  double halfspace_offset = 0.0;
  if (minimum_singular_value < singularity_warning_threshold_ &&
      singularity_gradient.norm() > 1e-8)
  {
    halfspace_normal = &singularity_gradient;
    halfspace_offset = singularity_avoidance_gain_ *
        (singularity_safe_threshold_ - minimum_singular_value);
  }

  const QPSolution solution = qp_solver_.solve(
      hessian, gradient, lower_bound, upper_bound,
      previous_velocity_command_, halfspace_normal, halfspace_offset);
  if (!solution.success)
  {
    publishZeroVelocityHold(positions);
    publishDiagnostics(ControllerState::QP_INFEASIBLE, solution.message,
                       position_error, minimum_singular_value, now,
                       solution.iterations, solution.projected_gradient_norm);
    return;
  }

  // 位置增量和time_from_start使用同一个command_horizon，保证发送给
  // JointTrajectoryController的位置、速度、时间三者表达同一段运动。
  previous_velocity_command_ = solution.x;
  Eigen::VectorXd commanded_positions = positions + command_horizon * solution.x;
  commanded_positions = commanded_positions.cwiseMax(minimum_positions_)
                                         .cwiseMin(maximum_positions_);
  publishTrajectoryCommand(commanded_positions, solution.x, command_horizon);

  ControllerState running_state = ControllerState::RUNNING;
  if (!solution.converged)
  {
    running_state = ControllerState::QP_DEGRADED;
  }
  else if (minimum_singular_value < singularity_warning_threshold_)
  {
    running_state = ControllerState::SINGULARITY_RECOVERY;
  }
  publishDiagnostics(running_state, solution.message,
                     position_error, minimum_singular_value, now,
                     solution.iterations, solution.projected_gradient_norm);
}

}  // namespace elfin5_qp_controller
