import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'forklift_one'

    # -----------------------
    # Launch arguments
    # -----------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # -----------------------
    # Paths
    # -----------------------
    pkg_path = get_package_share_directory(package_name)

    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'test_design.rviz')

    # Explicit world path (recommended by docs)
    world_file = os.path.join(
        get_package_share_directory('forklift_one'),
        'config',
        'empty.sdf'
    )

    # -----------------------
    # Robot description
    # -----------------------
    robot_description = Command([
        'xacro ', xacro_file,
        ' sim_mode:=', use_sim_time
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # -----------------------
    # Gazebo (doc-style)
    # -----------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            # Docs recommend passing the world explicitly
            'gz_args': f'-r {world_file}',
            'use_sim_time': use_sim_time
        }.items()
    )

    # -----------------------
    # Spawn robot (URDF)
    # -----------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', 'forklift',
            '-allow_renaming', 'false'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -----------------------
    # Controllers (after spawn)
    # -----------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )

    delayed_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster,
                ackermann_controller
            ],
        )
    )

    # -----------------------
    # RViz
    # -----------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -----------------------
    # ROS <-> Gazebo bridge
    # -----------------------
    bridge_config = os.path.join(
        pkg_path,
        'config',
        'gz_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[            
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}'
        ],
        output='screen'
    )

    # -----------------------
    # Launch description
    # -----------------------
    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_controller_spawner,
        ros_gz_bridge,
        rviz,
    ])
