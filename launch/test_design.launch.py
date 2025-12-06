import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Paths
    pkg_path = get_package_share_directory('forklift_one')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'test_design.rviz')

    # # Convert xacro to URDF on the fly
    robot_description_config = Command([
        'xacro ', xacro_file,
        # ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config,
                     'use_sim_time': use_sim_time}]
    )

    # Joint state publisher GUI
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2 visualization
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # DeclareLaunchArgument(
        #     'use_ros2_control',
        #     default_value='true',
        #     description='Enable ros2_control if true'),

        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
