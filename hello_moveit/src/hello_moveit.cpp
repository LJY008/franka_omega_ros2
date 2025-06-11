/*********************************************************************
 * Software License Agreement (BSD License)
 * ...版权声明略...
 *********************************************************************/

/* Author: Your Name */

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");// 日志记录器

// 全局变量
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
robot_model_loader::RobotModelLoaderPtr robot_model_loader_;// 机器人模型加载器
moveit::core::RobotModelPtr kinematic_model_;// 机器人模型
moveit::core::RobotStatePtr kinematic_state_;// 机器人状态
std::string planning_group = "panda_arm";// 规划组名称
std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_ptr;// 规划场景监视器
std::shared_ptr<tf2_ros::Buffer> tf_buffer;// 指向 TF2 Buffer 对象的共享指针
std::atomic<bool> initial_motion_completed{false};// 初始运动完成标志
rclcpp::TimerBase::SharedPtr initial_motion_timer;
rclcpp::TimerBase::SharedPtr test_motion_timer;

// 全局定时器管理
std::mutex timers_mutex;// 互斥锁，用于保护定时器列表
std::vector<rclcpp::TimerBase::SharedPtr> active_timers;// 活动定时器动态列表

// 清理所有活动定时器
void clear_active_timers() {
    std::lock_guard<std::mutex> lock(timers_mutex);
    for (auto& timer : active_timers) {
        if (timer) timer->cancel();
    }
    active_timers.clear();
    RCLCPP_INFO(LOGGER, "已清除所有活动定时器");
}

// 分步发送JointJog（速度模式，持续高频发消息，直到接近目标）
void send_joint_jog_velocity(const std::vector<std::string>& joint_names,
                             std::vector<double> current,
                             const std::vector<double>& target,
                             double max_velocity = 0.5)
{
  RCLCPP_INFO(LOGGER, "开始发送关节速度命令...");
  std::vector<double> velocity(current.size());
  bool finished = true;
  
  // 输出详细日志
  RCLCPP_INFO(LOGGER, "当前vs目标关节值:");
  for (size_t i = 0; i < current.size(); ++i)
  {
    double diff = target[i] - current[i];
    RCLCPP_INFO(LOGGER, "  %s: 当前=%.6f, 目标=%.6f, 差值=%.6f", 
                joint_names[i].c_str(), current[i], target[i], diff);
                
    // 使用更小的阈值
    if (std::abs(diff) > 0.0001)  // 大大降低阈值
    {
      velocity[i] = std::clamp(diff, -max_velocity, max_velocity);
      // 确保速度不太小
      if (std::abs(velocity[i]) < 0.01) {
          velocity[i] = (diff > 0) ? 0.01 : -0.01;  // 设置最小速度，如果差值大于0则正向，否则反向
      }
      finished = false;
    }
    else
    {
      velocity[i] = 0.0;
    }
  }

  auto jog_msg = std::make_unique<control_msgs::msg::JointJog>();// 创建JointJog消息
  jog_msg->header.stamp = node_->now();// 设置时间戳
  jog_msg->joint_names = joint_names;
  jog_msg->velocities = velocity;
  jog_msg->duration = 0.2;  // 增加持续时间
  joint_cmd_pub_->publish(std::move(jog_msg));// 发布消息

  RCLCPP_INFO(LOGGER, "已发送速度命令:");
  for (size_t i = 0; i < joint_names.size(); ++i)
    RCLCPP_INFO(LOGGER, "  %s: velocity=%.4f", joint_names[i].c_str(), velocity[i]);

  // 如果未到目标，10ms后继续发下一步（100Hz）
  if (!finished)
  {
    RCLCPP_INFO(LOGGER, "目标未达到，将在10ms后继续发送下一步命令");
    std::vector<double> next(current.size());
    for (size_t i = 0; i < current.size(); ++i)
      next[i] = current[i] + velocity[i] * 0.01;  // 0.01s步长的近似积分

    std::function<void()> func;
    func = [joint_names, next, target, max_velocity]() mutable {
      send_joint_jog_velocity(joint_names, next, target, max_velocity);
    };
    
    // 创建定时器，10ms后调用func
    auto timer = node_->create_wall_timer(10ms, func);
    
    // 保存定时器引用
    {
      std::lock_guard<std::mutex> lock(timers_mutex);
      active_timers.push_back(timer);// 将定时器添加到活动列表
      RCLCPP_INFO(LOGGER, "已创建新的定时器，当前活动定时器数量: %zu", active_timers.size());
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "目标已达到，不再发送更多速度命令");
  }
}

// 末端位姿控制回调（逆解实现）
void ee_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "收到位姿命令: position=[%.3f, %.3f, %.3f], orientation=[%.3f, %.3f, %.3f, %.3f]",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
              
  // // 如果初始运动未完成，忽略位姿命令
  // if (!initial_motion_completed)
  // {
  //   RCLCPP_WARN(LOGGER, "初始运动尚未完成，忽略位姿命令");
  //   return;
  // }
  
  // 停止周期测试定时器
  if (test_motion_timer) {
    test_motion_timer->cancel();
    RCLCPP_INFO(LOGGER, "已停止周期测试");
  }

  // 清除所有现有的定时器
  clear_active_timers();

  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_ptr);
  auto current_state = scene->getCurrentState();
  *kinematic_state_ = current_state;

  // 转换位姿到正确的坐标系
  Eigen::Isometry3d target_pose;
  std::string planning_frame = scene->getPlanningFrame();
  
  RCLCPP_INFO(LOGGER, "规划坐标系: %s, 输入坐标系: %s", 
             planning_frame.c_str(), 
             msg->header.frame_id.empty() ? "未指定(默认为global)" : msg->header.frame_id.c_str());
             
  if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame) {
    try {
      geometry_msgs::msg::PoseStamped pose_in = *msg;
      geometry_msgs::msg::PoseStamped pose_out;
      pose_in.header.stamp = node_->now();
      tf_buffer->transform(pose_in, pose_out, planning_frame);
      tf2::fromMsg(pose_out.pose, target_pose);
      // 在这里加上补偿旋转（如绕X轴旋转180°）
      Eigen::AngleAxisd compensatex(M_PI, Eigen::Vector3d::UnitX());
      target_pose.rotate(compensatex);
      Eigen::AngleAxisd compensatez(M_PI/4, Eigen::Vector3d::UnitZ());
      target_pose.rotate(compensatez);
      
      RCLCPP_INFO(LOGGER, "已转换位姿从 %s 到 %s", 
                 msg->header.frame_id.c_str(), planning_frame.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(LOGGER, "位姿转换失败: %s", ex.what());
      return;
    }
  } else {
    tf2::fromMsg(msg->pose, target_pose);
  }

  const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(planning_group);
  
  RCLCPP_INFO(LOGGER, "开始IK求解...");
  bool found_ik = kinematic_state_->setFromIK(joint_model_group, target_pose, 0.2);  // 增加超时

  if (found_ik)
  {
    RCLCPP_INFO(LOGGER, "IK求解成功!");
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
    
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> current_joint_values;
    current_state.copyJointGroupPositions(joint_model_group, current_joint_values);
    
    // 检查关节限制 - 修复的代码
    const moveit::core::JointBoundsVector& bounds = joint_model_group->getActiveJointModelsBounds();
    bool all_valid = true;
    
    RCLCPP_INFO(LOGGER, "检查关节限制:");
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (i < bounds.size() && bounds[i] != nullptr && (*bounds[i]).size() > 0) {
        const moveit::core::VariableBounds& bound = (*bounds[i])[0];
        if (bound.min_position_ > joint_values[i] || bound.max_position_ < joint_values[i]) {
          RCLCPP_WARN(LOGGER, "  %s: 值=%.4f 超出限制 [%.4f, %.4f]",
                     joint_names[i].c_str(), joint_values[i], 
                     bound.min_position_, bound.max_position_);
          all_valid = false;
          
          // 修正到限制范围内
          joint_values[i] = std::clamp(joint_values[i], 
                                      bound.min_position_, 
                                      bound.max_position_);
        } else {
          RCLCPP_INFO(LOGGER, "  %s: 值=%.4f 在限制范围内 [%.4f, %.4f]",
                    joint_names[i].c_str(), joint_values[i], 
                    bound.min_position_, bound.max_position_);
        }
      }
    }

    if (!all_valid) {
      RCLCPP_WARN(LOGGER, "有关节值超出限制，已自动修正");
    }
    
    RCLCPP_INFO(LOGGER, "开始控制机器人到IK解位置");
    send_joint_jog_velocity(joint_names, current_joint_values, joint_values, 0.5);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "IK求解失败！尝试不同的目标位姿");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  RCLCPP_INFO(LOGGER, "启动 servo_demo_node...");

  // 创建TF缓冲区和监听器
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  planning_scene_monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
  kinematic_model_ = robot_model_loader_->getModel();
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  RCLCPP_INFO(LOGGER, "等待系统准备中 (4秒)...");
  rclcpp::sleep_for(4s);

  if (planning_scene_monitor_ptr->getPlanningScene())
  {
    planning_scene_monitor_ptr->startStateMonitor("/joint_states");
    planning_scene_monitor_ptr->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor_ptr->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
      "/moveit_servo/publish_planning_scene");
    planning_scene_monitor_ptr->startSceneMonitor();
    planning_scene_monitor_ptr->providePlanningSceneService();
    RCLCPP_INFO(LOGGER, "规划场景监视器已启动");
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "规划场景配置失败");
    return EXIT_FAILURE;
  }

  // 创建JointJog命令发布器
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("/servo_demo_node/delta_joint_cmds", 10);
  RCLCPP_INFO(LOGGER, "已创建JointJog命令发布器，话题: /servo_demo_node/delta_joint_cmds");

  // 创建位姿订阅器
  auto ee_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/fd/ee_pose", 10, ee_pose_callback);
  RCLCPP_INFO(LOGGER, "已创建位姿订阅器，话题: /fd/ee_pose");

  // 加载Servo参数
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "无法加载Servo参数");
    return EXIT_FAILURE;
  }
  
  // 输出关键Servo参数
  RCLCPP_INFO(LOGGER, "Servo参数:");
  RCLCPP_INFO(LOGGER, "  规划坐标系: %s", servo_parameters->planning_frame.c_str());
  RCLCPP_INFO(LOGGER, "  命令输入类型: %s", servo_parameters->command_in_type.c_str());
  RCLCPP_INFO(LOGGER, "  关节命令话题: %s", servo_parameters->joint_topic.c_str());
  RCLCPP_INFO(LOGGER, "  发布关节位置: %s", servo_parameters->publish_joint_positions ? "true" : "false");
  RCLCPP_INFO(LOGGER, "  发布关节速度: %s", servo_parameters->publish_joint_velocities ? "true" : "false");
  
  // 启动Servo
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor_ptr);
  servo->start();
  RCLCPP_INFO(LOGGER, "Servo已启动");

  // 启动初始运动定时器
  // RCLCPP_INFO(LOGGER, "开始执行初始测试运动");
  // initial_motion_timer = node_->create_wall_timer(50ms, send_initial_motion_commands);

  // 定期清理定时器(每5秒一次)
  node_->create_wall_timer(5s, []() {
    std::lock_guard<std::mutex> lock(timers_mutex);
    size_t before = active_timers.size();
    active_timers.erase(
        std::remove_if(active_timers.begin(), active_timers.end(),
                    [](const auto& timer) { return !timer || timer->is_canceled(); }),
        active_timers.end());
    size_t after = active_timers.size();
    if (before != after) {
        RCLCPP_INFO(LOGGER, "已清理 %zu 个已完成的定时器，剩余 %zu 个",
                  before - after, after);
    }
  });

  // 启动节点
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  RCLCPP_INFO(LOGGER, "节点进入执行循环");
  std::atomic<bool> exit_flag{false};
  std::thread key_thread([&executor, &exit_flag]() {
    char c;
    while (!exit_flag) {
      std::cin >> c;
      if (c == 'q' || c == 'Q') {
        RCLCPP_INFO(LOGGER, "检测到按键退出信号，准备退出...");
        executor.cancel();
        exit_flag = true;
        break;
      }
    }
  });
  executor.spin();
  // 节点退出前，主动释放资源
  RCLCPP_INFO(LOGGER, "节点即将退出，清理资源...");
  
  servo.reset();
  clear_active_timers();
  tf_buffer.reset();
  planning_scene_monitor_ptr.reset();
  node_.reset();
  rclcpp::shutdown();
  return 0;
}