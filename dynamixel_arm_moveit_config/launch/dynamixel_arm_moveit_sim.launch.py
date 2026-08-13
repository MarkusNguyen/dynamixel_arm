import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("dynamixel_arm_moveit_config")

    # -------------------------------------------------------------
    # 0. Declare Launch Arguments (Switch between Sim and Hardware)
    # -------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",  # Set to 'true' for MuJoCo/Gazebo, 'false' for real hardware
        description="Use simulation clock if true, system clock if false",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -------------------------------------------------------------
    # 1. Path to .xacro file
    # -------------------------------------------------------------
    xacro_path = os.path.join(
        get_package_share_directory("dynamixel_arm_description"),
        "urdf",
        "dynamixel_arm.xacro",
    )

    # -------------------------------------------------------------
    # 2. Build MoveIt Configuration
    # -------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            "dynamixel_arm", package_name="dynamixel_arm_moveit_config"
        )
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path="config/dynamixel_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # -------------------------------------------------------------
    # 3. Define Nodes with Synchronized Clock Parameters
    # -------------------------------------------------------------

    # Node 1: Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Node 2: MoveGroup (Core Planner)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},  # Fixes timestamp mismatch errors
        ],
    )

    # Node 3: RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},  # Fixes timestamp mismatch errors
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            rsp_node,
            move_group_node,
            rviz_node,
        ]
    )