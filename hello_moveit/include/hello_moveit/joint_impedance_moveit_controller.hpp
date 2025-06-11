#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace franka_example_controllers {

class JointImpedanceMoveItController : public controller_interface::ControllerInterface {
public:
  JointImpedanceMoveItController();
  ~JointImpedanceMoveItController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  void gripper_control_tick();
  double calculate_fingers_distance();
  void open_gripper();
  void update_joint_states();
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::string ee_pose_topic_;
  double ik_timeout_;
  std::vector<std::string> joint_names_;
  std::string arm_id_;

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;

  const std::vector<double> q_max{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
  const std::vector<double> q_min{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  const std::vector<double> dq_max{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
  const std::vector<double> ddq_max{15, 7.5, 10, 12.5, 15, 20, 20};
  const std::vector<double> tau_max{87, 87, 87, 12, 87, 12, 12};
  const std::vector<double> dtau_max{1000, 1000, 1000, 1000, 1000, 1000, 1000};

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_;
  std::vector<double> joint_velocities_current_;
  std::vector<double> joint_efforts_current_;
  Eigen::VectorXd dq_filtered_{Eigen::VectorXd::Zero(7)};
  double dq_filter_alpha_{0.99};

  std::vector<double> joint_positions_target_{7, 0.0};
  double max_joint_step_ = 0.002;
  Eigen::Vector3d filtered_force_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_highpass_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_last_{Eigen::Vector3d::Zero()};
  double hp_alpha = 0.95;

  std::vector<double> q_init_{0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4};
  bool move_to_init_{true};
  double move_init_speed_{0.2};

  std::string index_finger_frame_{"fd_virtual_clutch_link"};
  geometry_msgs::msg::PoseStamped thumb_pose_;
  geometry_msgs::msg::PoseStamped index_pose_;
  bool index_pose_valid_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr index_pose_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr gripper_move_action_client_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_grasp_action_client_;

  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_goal_options_;
  bool gripper_move_busy_ = false;
  double last_sent_width_ = -1.0;
  rclcpp::Time last_gripper_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr gripper_timer_{nullptr};
  const double min_gripper_cmd_interval_ = 1.0;
};

} // namespace franka_example_controllers