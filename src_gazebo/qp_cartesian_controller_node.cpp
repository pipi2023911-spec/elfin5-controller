/*
 * Elfin5 笛卡尔 QP 控制器的 ROS 程序入口。
 * 所有算法逻辑均封装在 QPCartesianController 中，使入口文件保持简洁，
 * 同时便于后续测试、阅读和维护。
 */

#include "elfin5_qp_controller/qp_cartesian_controller.h"

#include <ros/ros.h>

#include <exception>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elfin5_qp_controller");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  try
  {
    elfin5_qp_controller::QPCartesianController controller(
        node_handle, private_node_handle);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("Elfin5 QP 控制器初始化失败：" << exception.what());
    return 1;
  }
  return 0;
}
