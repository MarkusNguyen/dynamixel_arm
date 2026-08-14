import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Adjust this package name if your URDF xacro is in a different package
    xacro_path = os.path.join(
        get_package_share_directory("dynamixel_arm_description"),
        "urdf",
        "dynamixel_arm.urdf",
    )

    use_sim_time = {"use_sim_time": True}

    moveit_config = (
        MoveItConfigsBuilder("dynamixel_arm", package_name="dynamixel_arm_moveit_config")
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path="config/dynamixel_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            use_sim_time,
            {
                "zeros": {
                    "joint1": 0.0,
                    "joint2": 0.4,   # Bent shoulder slightly
                    "joint3": -0.5,  # Bent elbow
                    "joint4": 0.0,
                    "joint5": 0.0,
                }
            },
        ],
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    # MoveGroup Node (Headless IK and Motion Planner)
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

    return LaunchDescription([jsp_node,rsp_node, move_group_node])