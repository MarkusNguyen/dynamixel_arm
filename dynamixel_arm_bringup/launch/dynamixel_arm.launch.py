import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    bringup_pkg_share = get_package_share_directory('dynamixel_arm_bringup')
    description_pkg_share = get_package_share_directory('dynamixel_arm_description')

    rviz_config_path = os.path.join(description_pkg_share, 'config', 'display.rviz')

    # Path to controllers config YAML file
    controllers_config_path = os.path.join(
        bringup_pkg_share, 'config', 'dynamixel_arm_controllers.yaml'
    )

    robot_description_path = os.path.join(
        description_pkg_share, 'urdf', 'dynamixel_arm.urdf'
    )

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    use_sim_time = {"use_sim_time": False}

    moveit_config = (
        MoveItConfigsBuilder("dynamixel_arm", package_name="dynamixel_arm_moveit_config")
        .robot_description(file_path=robot_description_path)
        .robot_description_semantic(file_path="config/dynamixel_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .sensors_3d(file_path="")
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            use_sim_time
        ]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_config_path,
            use_sim_time
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
            {"trajectory_execution.allowed_start_tolerance": 0.05},
        ],
    )

    move_trajectory_node = Node(
        package="dynamixel_arm_bringup",
        executable="MoveTrajectory.py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.to_dict(),
            use_sim_time
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
        move_trajectory_node
    ])