# 构建布局说明：原V2实现仍在src_gazebo；src仅放本次新增的硬件接入代码。
# 原求解器/控制器目标继续复用，硬件网关和监视节点独立编译。
# 纯命令检查库不依赖ROS消息，可做独立单元测试；两个节点通过catkin链接ROS依赖。
# 脚本统一由catkin_install_python安装，兼容Melodic现场使用方式。

cmake_minimum_required(VERSION 3.0.2)
project(elfin5_qp_controller)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  control_msgs
  controller_manager_msgs
  tf2_ros
  tf2_msgs
  visualization_msgs
  nav_msgs
  urdf
  moveit_core
  moveit_ros_planning
)
find_package(Eigen3 REQUIRED)
# 网关含控制器查询/停止服务线程，显式链接系统线程支持。
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elfin5_box_qp_solver
  CATKIN_DEPENDS
    roscpp sensor_msgs geometry_msgs trajectory_msgs std_msgs std_srvs
    diagnostic_msgs moveit_core moveit_ros_planning rospy
    control_msgs controller_manager_msgs tf2_ros tf2_msgs visualization_msgs nav_msgs urdf
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 引用现有源码路径，不把求解器复制到硬件src目录。
add_library(elfin5_box_qp_solver
  src_gazebo/box_qp_solver.cpp
)
target_link_libraries(elfin5_box_qp_solver ${catkin_LIBRARIES})

add_library(elfin5_qp_cartesian_controller
  src_gazebo/qp_cartesian_controller.cpp
)
target_link_libraries(elfin5_qp_cartesian_controller
  elfin5_box_qp_solver
  ${catkin_LIBRARIES}
)

add_executable(elfin5_qp_controller_node
  src_gazebo/qp_cartesian_controller_node.cpp
)
target_link_libraries(elfin5_qp_controller_node
  elfin5_qp_cartesian_controller
  ${catkin_LIBRARIES}
)


# 真机接入复用 V2 求解算法，通过配置与重映射适配输入输出。
# 纯检查库与ROS节点分离，避免为了测试限值逻辑而启动硬件通信。
add_library(elfin5_real_command_gate src/real_robot_command_gate.cpp)
add_executable(real_robot_bridge_node src/real_robot_bridge_node.cpp)
target_link_libraries(real_robot_bridge_node elfin5_real_command_gate ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_executable(real_robot_monitor_node src/real_robot_monitor_node.cpp)
add_executable(stop_hold_adapter_node src/stop_hold_adapter_node.cpp)
target_link_libraries(stop_hold_adapter_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(stop_hold_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(real_robot_monitor_node elfin5_real_command_gate ${catkin_LIBRARIES})
# 确保ROS生成消息/服务目标先完成，再编译硬件节点。
add_dependencies(real_robot_bridge_node ${catkin_EXPORTED_TARGETS})
add_dependencies(real_robot_monitor_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  elfin5_real_command_gate
  real_robot_bridge_node
  real_robot_monitor_node
  stop_hold_adapter_node
  elfin5_box_qp_solver
  elfin5_qp_cartesian_controller
  elfin5_qp_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装业务脚本并生成可执行入口；测试夹具另在测试开关内安装。
catkin_install_python(PROGRAMS
  scripts/run_qp_test_suite.py
  scripts/prepare_real_robot_description.py
  scripts/plot_real_robot_error.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 测试分工：纯C++规则、ROS隔离状态机、Python模型/误差检查。
# 没有开启或运行这些测试时，不能仅凭构建成功声称功能验证通过。
if(CATKIN_ENABLE_TESTING)
  add_executable(test_box_qp_standalone test/test_box_qp_standalone.cpp)
  target_link_libraries(test_box_qp_standalone elfin5_box_qp_solver)
  add_test(NAME box_qp_regression COMMAND test_box_qp_standalone
    ${CMAKE_CURRENT_SOURCE_DIR}/test/qp_reference_cases.txt 2000)
  find_program(QP_TEST_PYTHON3 NAMES python3)
  if(QP_TEST_PYTHON3)
    add_test(NAME box_qp_file_contract COMMAND ${QP_TEST_PYTHON3}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_qp_input_files.py
      $<TARGET_FILE:test_box_qp_standalone>)
    add_test(NAME stop_adapter_offline COMMAND ${QP_TEST_PYTHON3}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stop_hold_adapter.py)
    add_test(NAME command_horizon_offline COMMAND ${QP_TEST_PYTHON3}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_command_horizon.py)
    add_test(NAME qp_command_horizon_offline COMMAND ${QP_TEST_PYTHON3}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_qp_command_horizon.py --eigen ${EIGEN3_INCLUDE_DIRS})
  endif()
  add_executable(test_gate_standalone test/test_gate_standalone.cpp)
  target_link_libraries(test_gate_standalone elfin5_real_command_gate)
  add_test(NAME gateway_core COMMAND test_gate_standalone)
  find_package(rostest REQUIRED)
  find_package(PythonInterp REQUIRED)
  catkin_install_python(PROGRAMS test/test_real_robot_wiring.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/test_real_robot_wiring.test)
  add_test(NAME real_robot_offline_checks COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_robot_tf_and_error.py)
  catkin_add_gtest(test_real_robot_command_gate test/test_real_robot_command_gate.cpp)
  if(TARGET test_real_robot_command_gate)
    target_link_libraries(test_real_robot_command_gate elfin5_real_command_gate)
  endif()
  catkin_add_gtest(test_box_qp_solver test/test_box_qp_solver.cpp)
  if(TARGET test_box_qp_solver)
    target_link_libraries(test_box_qp_solver elfin5_box_qp_solver ${catkin_LIBRARIES})
  endif()
endif()
