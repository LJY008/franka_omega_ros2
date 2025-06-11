import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
import math

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    
    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/panda_simulated_config.yaml",
        )
        .to_dict()
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )
    
    panda_hand_to_fd_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='panda_hand_to_fd_base_tf',
        arguments=['0.3', '0.0', '0.3', '0.0', '0.0', '0.0', 'world', 'fd_base'],
        output='screen'
    )

    fd_y_yaw_to_fd_ee = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fd_y_yaw_to_fd_ee',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'fd_yaw_link', 'fd_ee'],
        output='screen'
    )
    
    # fd_ee_to_virtual_clutch_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='fd_ee_to_virtual_clutch_link',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'panda_leftfinger', 'fd_ee'],
    #     output='screen'
    # )
    

#     panda_hand_to_fd_base_tf = Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     name='panda_hand_to_fd_base_tf',
#     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'panda_hand', 'fd_base'],
#     output='screen'
# )
    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
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

    # Publishes tf's for the robot
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

    # ros2_control using FakeSystem as hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("moveit_resources_panda_moveit_config"),
                "config",
                "ros2_controllers.yaml",
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

    
    # Load controllers
    load_controllers = []
    for controller in ["joint_state_broadcaster","panda_arm_controller"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [servo_node,static_tf, panda_hand_to_fd_base_tf,fd_y_yaw_to_fd_ee,robot_state_publisher,move_group_node,rviz_node,ros2_control_node ]
        + load_controllers
    )