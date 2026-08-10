import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get package directories
    bringup_pkg_share = get_package_share_directory('dynamixel_arm_bringup')
    description_pkg_share = get_package_share_directory('dynamixel_arm_description')

    # Path to controllers config YAML file
    controllers_config_path = os.path.join(
        bringup_pkg_share, 'config', 'dynamixel_arm_controllers.yaml'
    )

    robot_description_path = os.path.join(
        description_pkg_share, 'urdf', 'dynamixel_arm_sim.urdf'
    )

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )

    mujoco_simulator_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controllers_config_path,
            {'use_sim_time': True}
        ]
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Arm Trajectory Controller spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        mujoco_simulator_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])