import os
import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='dynamixel_arm_description').find('dynamixel_arm_description')
    
    default_model_path = os.path.join(pkg_share, 'urdf/dynamixel_arm_sim.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')

    # 1. Robot State Publisher (Loads URDF/Xacro and publishes TF)
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)
        }]
    )

    # 2. Joint State Publisher GUI (Provides joint sliders window to move the robot in RViz)
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. RViz2 Visualizer
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf/xacro file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),

        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])