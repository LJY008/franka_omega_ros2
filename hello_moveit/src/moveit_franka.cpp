#include "hello_moveit/joint_impedance_moveit_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace franka_example_controllers {
  
JointImpedanceMoveItController::JointImpedanceMoveItController() = default;
JointImpedanceMoveItController::~JointImpedanceMoveItController() = default;

controller_interface::CallbackReturn JointImpedanceMoveItController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "初始化JointImpedanceMoveItController...");
  try {
    auto_declare<std::string>("ee_pose_topic", "/fd/ee_pose");
    auto_declare<double>("ik_timeout", 0.2);
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", std::vector<double>(7, 100.0));
    auto_declare<std::vector<double>>("d_gains", std::vector<double>(7, 10.0));
    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "初始化时出错: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn JointImpedanceMoveItController::on_configure(const rclcpp_lifecycle::State&) {
  try {
    ee_pose_topic_ = get_node()->get_parameter("ee_pose_topic").as_string();
    ik_timeout_ = get_node()->get_parameter("ik_timeout").as_double();
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    k_gains_ = get_node()->get_parameter("k_gains").as_double_array();
    d_gains_ = get_node()->get_parameter("d_gains").as_double_array();

    if (joint_names_.size() != 7 || k_gains_.size() != 7 || d_gains_.size() != 7) {
      RCLCPP_ERROR(get_node()->get_logger(), "关节名、k_gains、d_gains长度必须为7");
      return controller_interface::CallbackReturn::ERROR;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    if (!ik_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_node()->get_logger(), "等待IK服务/compute_ik超时");
      return controller_interface::CallbackReturn::ERROR;
    }
    
    force_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/fd/fd_controller/commands", 10);

    joint_positions_desired_.resize(7, 0.0);
    joint_positions_current_.resize(7, 0.0);
    joint_velocities_current_.resize(7, 0.0);
    joint_efforts_current_.resize(7, 0.0);
    dq_filtered_ = Eigen::VectorXd::Zero(7);      
    joint_positions_target_.assign(7, 0.0);
    
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state"
    );
    
    index_finger_frame_ = get_node()->declare_parameter<std::string>("index_finger_frame", "fd_virtual_clutch_link");
    index_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/fd/index_pose", 10);

    gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      get_node(), "/" + arm_id_ + "_gripper/move");
    gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      get_node(), "/" + arm_id_ + "_gripper/grasp");

    move_goal_options_.goal_response_callback = [this](auto goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_node()->get_logger(), "Gripper Move Goal NOT accepted.");
      } else {
        RCLCPP_INFO(get_node()->get_logger(), "Gripper Move Goal accepted.");
      }
    };
    move_goal_options_.feedback_callback = [this](auto, auto feedback) {
      RCLCPP_DEBUG(get_node()->get_logger(), "Gripper Move Feedback: current_width=%f", feedback->current_width);
    };
    move_goal_options_.result_callback = [this](auto result) {
      gripper_move_busy_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_node()->get_logger(), "Gripper Move Succeeded.");
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Gripper Move Failed.");
      }
    };

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "配置控制器时发生错误: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn JointImpedanceMoveItController::on_activate(const rclcpp_lifecycle::State&) {
  try {
    if (command_interfaces_.size() != 7 || state_interfaces_.size() < 21) {
      RCLCPP_ERROR(get_node()->get_logger(), "接口数量不匹配");
      return controller_interface::CallbackReturn::ERROR;
    }
    update_joint_states();
    joint_positions_desired_ = joint_positions_current_;
    joint_positions_target_ = joint_positions_current_;
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&JointImpedanceMoveItController::pose_callback, this, std::placeholders::_1));

    gripper_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(&JointImpedanceMoveItController::gripper_control_tick, this)
    );
    // ====== 新增：发布 world->camera_link 静态TF ======
    if (!static_broadcaster_) {
      static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node());
      geometry_msgs::msg::TransformStamped static_transform;
      static_transform.header.stamp = get_node()->now();
      static_transform.header.frame_id = "world";
      static_transform.child_frame_id = "camera_link";
      static_transform.transform.translation.x = 0.0; // 根据实际相机位置修改
      static_transform.transform.translation.y = 0.0;
      static_transform.transform.translation.z = 1.0;
      static_transform.transform.rotation.x = 0.0;
      static_transform.transform.rotation.y = 0.0;
      static_transform.transform.rotation.z = 0.0;
      static_transform.transform.rotation.w = 1.0;
      static_broadcaster_->sendTransform(static_transform);
      RCLCPP_INFO(get_node()->get_logger(), "已在控制器内发布 world->camera_link 静态TF");
    }
    // ====== 新增结束 ======
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "激活控制器时出错：%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // if (!index_pose_valid_) {
  //   open_gripper();
  // }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceMoveItController::on_deactivate(const rclcpp_lifecycle::State&) {
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointImpedanceMoveItController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointImpedanceMoveItController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= 7; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= 7; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  config.names.push_back(arm_id_ + "/robot_model");
  config.names.push_back(arm_id_ + "/robot_state");
  return config;
}

controller_interface::return_type JointImpedanceMoveItController::update(const rclcpp::Time&, const rclcpp::Duration& period) {
  update_joint_states();
  double dt = period.seconds();

  for (size_t i = 0; i < 7; ++i) {
    joint_positions_target_[i] = std::clamp(joint_positions_target_[i], q_min[i], q_max[i]);
  }
  for (size_t i = 0; i < 7; ++i) {
    double diff = joint_positions_target_[i] - joint_positions_desired_[i];
    if (std::abs(diff) > max_joint_step_) {
      joint_positions_desired_[i] += max_joint_step_ * (diff > 0 ? 1 : -1);
    } else {
      joint_positions_desired_[i] = joint_positions_target_[i];
    }
  }
  Eigen::Map<Eigen::VectorXd> qd(joint_positions_desired_.data(), 7);
  Eigen::Map<Eigen::VectorXd> q(joint_positions_current_.data(), 7);
  Eigen::Map<Eigen::VectorXd> dq(joint_velocities_current_.data(), 7);
  Eigen::Map<Eigen::VectorXd> K(k_gains_.data(), 7);
  Eigen::Map<Eigen::VectorXd> D(d_gains_.data(), 7);

  dq_filtered_ = (1.0 - dq_filter_alpha_) * dq_filtered_ + dq_filter_alpha_ * dq;
  Eigen::VectorXd tau = K.cwiseProduct(qd - q) - D.cwiseProduct(dq_filtered_);
  
  static Eigen::VectorXd tau_last = Eigen::VectorXd::Zero(7);
  for (int i = 0; i < 7; ++i) {
    tau(i) = std::clamp(tau(i), -tau_max[i], tau_max[i]);
    double dtau = tau(i) - tau_last(i);
    double max_dtau = dtau_max[i] * dt;
    if (std::abs(dtau) > max_dtau) {
      tau(i) = tau_last(i) + max_dtau * (dtau > 0 ? 1 : -1);
    }
  }
  tau_last = tau;

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau(i));
  }

  try {
    Eigen::VectorXd tau_measured(7);
    for (size_t i = 0; i < 7; ++i) {
      tau_measured(i) = joint_efforts_current_[i];
    }
    std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobian_array.data());
    std::array<double, 7> gravity_array = franka_robot_model_->getGravityForceVector();
    Eigen::Map<const Eigen::VectorXd> gravity(gravity_array.data(), 7);
    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    Eigen::Map<const Eigen::VectorXd> coriolis(coriolis_array.data(), 7);
    std::array<double, 49> mass_array = franka_robot_model_->getMassMatrix();
    Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> M(mass_array.data());

    static std::vector<double> last_joint_velocities(7, 0.0);
    static std::vector<double> last_joint_acc(7, 0.0);
    static rclcpp::Time last_time = get_node()->now();
    double dt_acc = (get_node()->now() - last_time).seconds();
    std::vector<double> joint_acc(7, 0.0);
    double acc_alpha = 0.8;

    if (dt_acc > 1e-4 && dt_acc < 0.1) {
      for (size_t i = 0; i < 7; ++i) {
        double raw_acc = (joint_velocities_current_[i] - last_joint_velocities[i]) / dt_acc;
        joint_acc[i] = acc_alpha * last_joint_acc[i] + (1 - acc_alpha) * raw_acc;
        last_joint_velocities[i] = joint_velocities_current_[i];
        last_joint_acc[i] = joint_acc[i];
      }
      last_time = get_node()->now();
    } else {
      joint_acc = last_joint_acc;
      last_time = get_node()->now();
    }
    Eigen::VectorXd ddq = Eigen::Map<Eigen::VectorXd>(joint_acc.data(), 7);
    Eigen::VectorXd inertia = M * ddq;
    Eigen::VectorXd tau_ext = tau_measured - gravity - coriolis;
    Eigen::VectorXd wrench_tcp = J.transpose().completeOrthogonalDecomposition().solve(tau_ext);

    double max_force = 30.0;
    double max_torque = 5.0;
    for (int i = 0; i < 3; ++i) {
      wrench_tcp(i) = std::clamp(wrench_tcp(i), -max_force, max_force);
    }
    for (int i = 3; i < 6; ++i) {
      wrench_tcp(i) = std::clamp(wrench_tcp(i), -max_torque, max_torque);
    }
    Eigen::Vector3d force = wrench_tcp.head(3);
    double force_magnitude = force.norm();
    Eigen::Vector3d force_dir = Eigen::Vector3d::Zero();
    if (force_magnitude > 0.5) {  
      force_dir = force.normalized();
    }

    force_highpass_ = hp_alpha * (force_highpass_ + force - force_last_);
    force_last_ = force;
    double lp_alpha = 0.8;
    filtered_force_ = lp_alpha * filtered_force_ + (1 - lp_alpha) * force_highpass_;
    double filtered_force_magnitude = filtered_force_.norm();

    double min_force_threshold = 1.5;
    double feedback_gain = 1.0;
    if (filtered_force_magnitude > min_force_threshold) {
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "TCP力(带通后): [%.3f, %.3f, %.3f] N (|F|=%.3f)",
        filtered_force_(0), filtered_force_(1), filtered_force_(2), filtered_force_magnitude);
      Eigen::Vector3d feedback_force = filtered_force_ * feedback_gain;

      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(7);
      msg.data[0] = feedback_force(0);
      msg.data[1] = feedback_force(1);
      msg.data[2] = feedback_force(2);
      msg.data[3] = wrench_tcp(3);
      msg.data[4] = wrench_tcp(4);
      msg.data[5] = wrench_tcp(5);
      msg.data[6] = 0.0;
      // force_pub_->publish(msg);
    } else {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(7, 0.0);
      force_pub_->publish(msg);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), "力反馈估算失败: %s", e.what());
  }

  return controller_interface::return_type::OK;
}

void JointImpedanceMoveItController::gripper_control_tick() {
  if (!index_pose_valid_) return;
  if (gripper_move_busy_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "gripper busy, skip tick");
    return;
  }
  rclcpp::Time now = get_node()->now();
  if ((now - last_gripper_cmd_time_).seconds() < min_gripper_cmd_interval_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "gripper interval not reached, skip tick");
    return;
  }
  double distance = calculate_fingers_distance() * 0.1;
  if (distance > 0) {
    double min_width = 0.01, max_width = 0.08;
    double target_width = std::clamp(distance, min_width, max_width);
    double width_delta = std::abs(target_width - last_sent_width_);
    if (width_delta > 0.002 && gripper_move_action_client_->action_server_is_ready()) {
      franka_msgs::action::Move::Goal goal;
      goal.width = target_width;
      goal.speed = 1.0;
      gripper_move_busy_ = true;
      last_gripper_cmd_time_ = now;
      gripper_move_action_client_->async_send_goal(goal, move_goal_options_);
      last_sent_width_ = target_width;
      RCLCPP_INFO(get_node()->get_logger(), "Send gripper move goal: width=%.3f", target_width);
    }
  }
}

double JointImpedanceMoveItController::calculate_fingers_distance() {
  if (!index_pose_valid_) {
    return -1.0;
  }
  if (thumb_pose_.header.frame_id != index_pose_.header.frame_id) {
    RCLCPP_WARN(get_node()->get_logger(), 
      "两指位姿参考系不同! 拇指=%s, 食指=%s",
      thumb_pose_.header.frame_id.c_str(), 
      index_pose_.header.frame_id.c_str());
    return -1.0;
  }
  double dx = index_pose_.pose.position.x - thumb_pose_.pose.position.x;
  double dy = index_pose_.pose.position.y - thumb_pose_.pose.position.y;
  double dz = index_pose_.pose.position.z - thumb_pose_.pose.position.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void JointImpedanceMoveItController::open_gripper() {
  if (!gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper move action server not available.");
    return;
  }
  franka_msgs::action::Move::Goal goal;
  goal.width = 0.08;
  goal.speed = 0.2;
  gripper_move_busy_ = true;
  gripper_move_action_client_->async_send_goal(goal, move_goal_options_);
  last_sent_width_ = 0.08;
  last_gripper_cmd_time_ = get_node()->now();
  RCLCPP_INFO(get_node()->get_logger(), "已发送夹爪打开指令");
}

void JointImpedanceMoveItController::update_joint_states() {
  for (size_t i = 0; i < 7; ++i) {
    joint_positions_current_[i] = state_interfaces_.at(i).get_value();
    joint_velocities_current_[i] = state_interfaces_.at(7 + i).get_value();
    joint_efforts_current_[i] = state_interfaces_.at(14 + i).get_value();
  }
}

void JointImpedanceMoveItController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  try {
    std::string model_frame = "base";
    if (!msg->header.frame_id.empty() && msg->header.frame_id != model_frame) {
      geometry_msgs::msg::PoseStamped pose_out;
      try {
        tf_buffer_->transform(*msg, pose_out, model_frame, tf2::durationFromSec(0.2));
        thumb_pose_ = pose_out;
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_node()->get_logger(), "拇指位姿转换失败: %s", ex.what());
      }
    } else {
      thumb_pose_ = *msg;
    }
    thumb_pose_.header.frame_id = model_frame;
    try {
      geometry_msgs::msg::TransformStamped index_transform;
      index_transform = tf_buffer_->lookupTransform(
        model_frame, index_finger_frame_, tf2::TimePointZero);
      index_pose_.header.frame_id = model_frame;
      index_pose_.header.stamp = get_node()->now();
      index_pose_.pose.position.x = index_transform.transform.translation.x;
      index_pose_.pose.position.y = index_transform.transform.translation.y;
      index_pose_.pose.position.z = index_transform.transform.translation.z;
      index_pose_.pose.orientation = index_transform.transform.rotation;
      index_pose_valid_ = true;
      index_pose_pub_->publish(index_pose_);
    } catch (const tf2::TransformException& ex) {
      index_pose_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "获取食指位姿失败: %s", ex.what());
    }
    geometry_msgs::msg::PoseStamped target_pose_msg = thumb_pose_;
    Eigen::Isometry3d eigen_pose;
    tf2::fromMsg(target_pose_msg.pose, eigen_pose);
    Eigen::AngleAxisd compensate_x(M_PI, Eigen::Vector3d::UnitX());
    eigen_pose.rotate(compensate_x);
    target_pose_msg.pose = tf2::toMsg(eigen_pose);      

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = arm_id_ + "_arm";
    request->ik_request.pose_stamped = target_pose_msg;
    request->ik_request.pose_stamped.header.frame_id = model_frame;
    request->ik_request.timeout = rclcpp::Duration::from_seconds(ik_timeout_);
    request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
    request->ik_request.robot_state.joint_state.name = joint_names_;
    std::vector<double> current_joint_positions(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      current_joint_positions[i] = state_interfaces_[i].get_value();
    }
    request->ik_request.robot_state.joint_state.position = current_joint_positions;

    auto response_callback = [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
      auto response = future.get();
      if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        const size_t elbow_idx = 3;
        double elbow_min = -2.5;
        double elbow_max =  2.5;
        double new_elbow = response->solution.joint_state.position[elbow_idx];
        if (new_elbow < elbow_min || new_elbow > elbow_max) {
          return;
        }
        joint_positions_target_ = response->solution.joint_state.position;
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "IK服务求解失败，错误码: %d", response->error_code.val);
      }
    };
    ik_client_->async_send_request(request, response_callback);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "处理位姿消息时出错: %s", e.what());
  }
}

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceMoveItController, controller_interface::ControllerInterface)