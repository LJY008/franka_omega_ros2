import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
import math
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_param_builder import ParameterBuilder
from launch.actions import ExecuteProcess, DeclareLaunchArgument
def generate_launch_description():
    use_real_robot_arg = DeclareLaunchArgument(
        'use_real_robot',
        default_value='false',
        description='使用实体机械臂'
    )
    
    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value='panda',
        description='机械臂ID'
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/panda_simulated_config.yaml",
        )
        .to_dict()
    )
    servo_node = Node(
        package="hello_moveit",
        executable="hello_moveit",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, 
           
        ],
    )
    # 机器人状态发布
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("hello_moveit")
        + "/config/demo_rviz_config.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, 
        ],
    )

    # ros2_control主节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic, 
            moveit_config.robot_description_kinematics, 
            os.path.join(
                get_package_share_directory("moveit_resources_panda_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
            # 你的 moveit_franka_controller.yaml 路径
            os.path.join(
                get_package_share_directory("hello_moveit"),
                "config",
                "moveit_franka_controller.yaml",
            ),
        ],
        output="both",
    )

    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            os.path.join(
                get_package_share_directory("moveit_resources_panda_moveit_config"),
                "config",
                "controllers.yaml",
            ),
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},           
            {"planning_pipeline": "ompl"},
        ]
    )

    # 静态TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # 其他静态TF（如有需要可继续添加）
    panda_hand_to_fd_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='panda_hand_to_fd_base_tf',
        arguments=['0.3', '0.0', '0.5', '0.0', '0.0', '0.0', 'world', 'fd_base'],
        output='screen'
    )

    fd_y_yaw_to_fd_ee = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fd_y_yaw_to_fd_ee',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'fd_yaw_link', 'fd_ee'],
        output='screen'
    )

    # 控制器spawner
    load_controllers = []
    for controller in ["joint_state_broadcaster", "moveit_franka_controller"]:
        load_controllers.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
            output="screen",
        )
    )

    return LaunchDescription(
        [   use_real_robot_arg,
            arm_id_arg,    
            servo_node,  
            ros2_control_node,
            static_tf,
            panda_hand_to_fd_base_tf,            
            robot_state_publisher,
            fd_y_yaw_to_fd_ee,
            move_group_node,
            rviz_node,
            TimerAction(period=2.0, actions=load_controllers),
            
        ]
    )