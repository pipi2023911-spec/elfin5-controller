/*
 * Elfin5 笛卡尔 QP 控制器 V1。
 *
 * 程序用途：
 *   在 Gazebo 中控制 Elfin5 末端到达给定的绝对坐标目标点。机械臂基座位姿
 *   和目标点均通过 ROS 接口输入，后续视觉与主控台可以直接替换测试发布器。
 *
 * 控制模型：
 *   - 只控制末端位置，不约束末端姿态；
 *   - 使用 MoveIt RobotModel 计算正运动学和雅可比矩阵；
 *   - 使用凸 QP 同时满足关节位置、速度和加速度限制；
 *   - 使用平移雅可比最小奇异值检测奇异性；
 *   - 使用自适应阻尼、奇异值梯度和半空间约束主动远离奇异位形。
 */

#ifndef ELFIN5_QP_CONTROLLER_QP_CARTESIAN_CONTROLLER_H
#define ELFIN5_QP_CONTROLLER_QP_CARTESIAN_CONTROLLER_H

#include "elfin5_qp_controller/box_qp_solver.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <Eigen/Geometry>

#include <mutex>
#include <string>
#include <vector>

namespace elfin5_qp_controller
{

class QPCartesianController
{
public:
  QPCartesianController(const ros::NodeHandle& node_handle,
                        const ros::NodeHandle& private_node_handle);

private:
  // 控制状态通过 diagnostic_msgs 发布，供主控台显示和自动化验收。
  enum class ControllerState
  {
    IDLE,
    WAITING_FOR_INPUT,
    RUNNING,
    REACHED,
    SINGULARITY_RECOVERY,
    SINGULARITY_STOP,
    QP_INFEASIBLE,
    QP_DEGRADED,
    INVALID_INPUT,
    STALE_INPUT
  };

  // ROS 输入回调只缓存数据，QP 计算统一在定时器线程内完成。
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& message);
  void basePoseCallback(const geometry_msgs::PoseStampedConstPtr& message);
  void targetPointCallback(const geometry_msgs::PointStampedConstPtr& message);
  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response);
  void controlTimerCallback(const ros::TimerEvent& event);

  // 从 robot_description 和 MoveIt 语义参数中加载 Elfin5 模型及关节限制。
  bool loadRobotModel();
  bool updateRobotState(const std::vector<double>& joint_positions);
  bool computeKinematics(const std::vector<double>& joint_positions,
                         Eigen::Vector3d& end_point_in_base,
                         Eigen::MatrixXd& translation_jacobian);
  bool extractJointPositions(const sensor_msgs::JointState& joint_state,
                             std::vector<double>& joint_positions,
                             std::string& error_message) const;
  double computeMinimumSingularValue(const Eigen::MatrixXd& translation_jacobian) const;
  Eigen::VectorXd computeSingularValueGradient(const std::vector<double>& joint_positions);
  Eigen::VectorXd computeJointCenteringVelocity(const Eigen::VectorXd& positions) const;
  bool buildVelocityBounds(const Eigen::VectorXd& positions,
                           double control_dt,
                           double command_horizon,
                           Eigen::VectorXd& lower_bound,
                           Eigen::VectorXd& upper_bound) const;

  // 将主控台给出的基座位姿转换为 Eigen 齐次变换。
  Eigen::Isometry3d poseToIsometry(const geometry_msgs::Pose& pose) const;
  void publishTrajectoryCommand(const Eigen::VectorXd& positions,
                                const Eigen::VectorXd& velocities,
                                double horizon);
  void publishZeroVelocityHold(const Eigen::VectorXd& positions);
  void publishDiagnostics(ControllerState state,
                          const std::string& detail,
                          double position_error,
                          double minimum_singular_value,
                          const ros::Time& stamp,
                          int qp_iterations = 0,
                          double qp_residual = 0.0);
  std::string stateToString(ControllerState state) const;

  // ROS 通信对象。
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber base_pose_subscriber_;
  ros::Subscriber target_point_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher current_point_publisher_;
  ros::Publisher position_error_publisher_;
  ros::Publisher minimum_singular_value_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::ServiceServer enable_server_;
  ros::Timer control_timer_;

  // MoveIt 运动学模型对象；不调用 OMPL，只使用 FK、雅可比和关节边界。
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};
  const moveit::core::LinkModel* base_link_model_{nullptr};
  const moveit::core::LinkModel* tip_link_model_{nullptr};
  // MoveIt getJacobian() 默认把Jacobian表达在规划组首关节的父链接坐标系。
  // 该链接不一定等于对外接口使用的base_link，因此需要显式保存并转换坐标轴。
  const moveit::core::LinkModel* jacobian_reference_link_model_{nullptr};

  // 关节顺序始终采用 MoveIt 规划组中的 J1～J6 顺序。
  std::vector<std::string> joint_names_;
  Eigen::VectorXd minimum_positions_;
  Eigen::VectorXd maximum_positions_;
  Eigen::VectorXd maximum_velocities_;
  Eigen::VectorXd maximum_accelerations_;
  Eigen::VectorXd previous_velocity_command_;
  // disable时只发送一次保持命令，之后不持续抢占command话题，便于实验人员在
  // QP关闭状态下使用其他工具设置初始关节姿态。
  bool hold_command_pending_{true};

  // data_mutex_ 只保护ROS输入缓存；耗时的运动学和QP计算不持有该锁。
  std::mutex data_mutex_;
  // AsyncSpinner 有两个线程。该锁保证同一时刻最多执行一个控制周期，并保护
  // RobotState 与 previous_velocity_command_ 这两个有内部状态的对象。
  std::mutex control_mutex_;
  sensor_msgs::JointState latest_joint_state_;
  geometry_msgs::PoseStamped latest_base_pose_;
  geometry_msgs::PointStamped latest_target_point_;
  ros::Time latest_joint_state_receive_time_;
  ros::Time latest_base_pose_receive_time_;
  ros::Time latest_target_point_receive_time_;
  bool have_joint_state_{false};
  bool have_base_pose_{false};
  bool have_target_point_{false};
  bool enabled_{false};

  // 控制器参数均可通过 YAML 覆盖，避免将调参值写死在算法中。
  BoxQPSolver qp_solver_;
  std::string planning_group_;
  std::string base_link_;
  std::string tip_link_;
  std::string absolute_frame_;
  double control_rate_;
  double position_gain_;
  double maximum_cartesian_speed_;
  double goal_tolerance_;
  double position_limit_margin_;
  double velocity_limit_scale_;
  double acceleration_limit_scale_;
  double nominal_damping_;
  double singularity_damping_;
  double singularity_warning_threshold_;
  double singularity_stop_threshold_;
  double singularity_safe_threshold_;
  double singularity_avoidance_gain_;
  double joint_centering_gain_;
  double finite_difference_step_;
  double input_timeout_;
  double command_horizon_factor_;
  int qp_max_iterations_;
  double qp_tolerance_;
};

}  // namespace elfin5_qp_controller

#endif  // ELFIN5_QP_CONTROLLER_QP_CARTESIAN_CONTROLLER_H
