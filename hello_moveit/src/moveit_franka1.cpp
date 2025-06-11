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

namespace franka_example_controllers {

class JointImpedanceMoveItController : public controller_interface::ControllerInterface {
public:
  JointImpedanceMoveItController() = default;
  ~JointImpedanceMoveItController() override = default;

  controller_interface::CallbackReturn on_init() override {
    RCLCPP_INFO(get_node()->get_logger(), "初始化JointImpedanceMoveItController...");
    try {
      auto_declare<std::string>("ee_pose_topic", "/fd/ee_pose");
      auto_declare<double>("ik_timeout", 0.2);
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      auto_declare<std::string>("arm_id", "fr3");
      auto_declare<std::vector<double>>("k_gains", std::vector<double>(7, 100.0));
      auto_declare<std::vector<double>>("d_gains", std::vector<double>(7, 10.0));
      RCLCPP_INFO(get_node()->get_logger(), "参数声明完成");
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "初始化时出错: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_node()->get_logger(), "配置控制器...");
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

      joint_positions_desired_.resize(7, 0.0);
      joint_positions_current_.resize(7, 0.0);
      joint_velocities_current_.resize(7, 0.0);
      joint_efforts_current_.resize(7, 0.0);
      dq_filtered_ = Eigen::VectorXd::Zero(7);

      // 启动时目标等于当前，避免大跳变
      joint_positions_target_.assign(7, 0.0);

      RCLCPP_INFO(get_node()->get_logger(), "配置完成");
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "配置控制器时发生错误: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_node()->get_logger(), "激活控制器...");
    try {
      if (command_interfaces_.size() != 7 || state_interfaces_.size() < 21) {
        RCLCPP_ERROR(get_node()->get_logger(), "接口数量不匹配");
        return controller_interface::CallbackReturn::ERROR;
      }
      // 初始化当前关节状态
      update_joint_states();
      joint_positions_desired_ = joint_positions_current_;
      joint_positions_target_ = joint_positions_current_;

      pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        ee_pose_topic_, 10,
        std::bind(&JointImpedanceMoveItController::pose_callback, this, std::placeholders::_1));
      RCLCPP_INFO(get_node()->get_logger(), "已创建位姿订阅器，话题: %s", ee_pose_topic_.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "激活控制器时出错：%s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_node()->get_logger(), "停用控制器...");
    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= 7; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
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
    return config;
  }

  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration& period) override {
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
    // 阻抗控制律 tau = K(qd-q) - D(dq)
    Eigen::Map<Eigen::VectorXd> qd(joint_positions_desired_.data(), 7);
    Eigen::Map<Eigen::VectorXd> q(joint_positions_current_.data(), 7);
    Eigen::Map<Eigen::VectorXd> dq(joint_velocities_current_.data(), 7);
    Eigen::Map<Eigen::VectorXd> K(k_gains_.data(), 7);
    Eigen::Map<Eigen::VectorXd> D(d_gains_.data(), 7);

    dq_filtered_ = (1.0 - dq_filter_alpha_) * dq_filtered_ + dq_filter_alpha_ * dq;
    Eigen::VectorXd tau = K.cwiseProduct(qd - q) - D.cwiseProduct(dq_filtered_);

    // 限制力矩和力矩变化率
    static Eigen::VectorXd tau_last = Eigen::VectorXd::Zero(7);
    for (int i = 0; i < 7; ++i) {
      // 限制力矩
      tau(i) = std::clamp(tau(i), -tau_max[i], tau_max[i]);
      // 限制力矩变化率
      double dtau = tau(i) - tau_last(i);
      double max_dtau = dtau_max[i] * dt;
      if (std::abs(dtau) > max_dtau) {
        tau(i) = tau_last(i) + max_dtau * (dtau > 0 ? 1 : -1);
      }
    }
    tau_last = tau;

    // // 打印tau
    // std::ostringstream oss;
    // oss << "tau: ";
    // for (int i = 0; i < tau.size(); ++i) {
    //   oss << tau(i) << " ";
    // }
    // RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());

    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(tau(i));
    }
    return controller_interface::return_type::OK;
  }

private:
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

  std::vector<double> joint_positions_target_{7, 0.0}; // 目标关节角度
  double max_joint_step_ = 0.002; // 每周期最大关节变化(rad)，可调
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;

  // 读取当前关节状态
  void update_joint_states() {
    for (size_t i = 0; i < 7; ++i) {
      joint_positions_current_[i] = state_interfaces_.at(i).get_value();
      joint_velocities_current_[i] = state_interfaces_.at(7 + i).get_value();
      joint_efforts_current_[i] = state_interfaces_.at(14 + i).get_value();
    }
  }

  // 目标位姿回调，调用IK服务
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try {
      geometry_msgs::msg::PoseStamped target_pose_msg = *msg;
      std::string model_frame = "base";
      if (!msg->header.frame_id.empty() && msg->header.frame_id != model_frame) {
        geometry_msgs::msg::PoseStamped pose_out;
        try {
          tf_buffer_->transform(*msg, pose_out, model_frame, tf2::durationFromSec(0.2));
          target_pose_msg = pose_out;
        } catch (const tf2::TransformException& ex) {
          RCLCPP_ERROR(get_node()->get_logger(), "位姿转换失败: %s", ex.what());
          return;
        }
      }
      Eigen::Isometry3d eigen_pose;
      tf2::fromMsg(target_pose_msg.pose, eigen_pose);
      Eigen::AngleAxisd compensate_x(-M_PI, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd compensate_z(M_PI/2, Eigen::Vector3d::UnitZ());
      eigen_pose.rotate(compensate_x);
      eigen_pose.rotate(compensate_z);
      target_pose_msg.pose = tf2::toMsg(eigen_pose);
      
      // target_pose_msg.pose.position.z *= -1;

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

      // 单线程executor下可直接赋值，无需加锁
      auto response_callback = [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
        auto response = future.get();
        if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          // 限制肘部关节变化幅度
          // const size_t elbow_idx = 3; // Franka第4关节
          // double max_elbow_delta = 0.15; // 最大允许变化幅度（单位：rad，可根据实际调整）
          // double new_elbow = response->solution.joint_state.position[elbow_idx];
          // double old_elbow = joint_positions_target_[elbow_idx];
          // if (std::abs(new_elbow - old_elbow) > max_elbow_delta) {
          //   RCLCPP_WARN(get_node()->get_logger(),
          //     "IK解肘部关节变化过大(%.3f->%.3f)，丢弃本次解。", old_elbow, new_elbow);
          //   return;
          // }
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
};

} // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceMoveItController, controller_interface::ControllerInterface)