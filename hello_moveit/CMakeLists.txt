cmake_minimum_required(VERSION 3.8)
project(hello_moveit)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(srdfdom REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_example_controllers REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)


target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  hello_moveit
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_servo
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_perception
  moveit_visual_tools
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  franka_semantic_components
  srdfdom
)

add_library(moveit_franka_controller SHARED src/moveit_franka.cpp src/moveit_franka_gazebo.cpp)

target_include_directories(moveit_franka_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${controller_interface_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${franka_semantic_components_INCLUDE_DIRS}>
)

target_compile_features(moveit_franka_controller PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  moveit_franka_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_servo
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_perception
  moveit_visual_tools
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  srdfdom
  franka_semantic_components
  franka_example_controllers
)

# 插件导出
pluginlib_export_plugin_description_file(controller_interface plugins.xml)

install(TARGETS moveit_franka_controller
  DESTINATION lib)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()