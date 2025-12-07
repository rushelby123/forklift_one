import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    package_name = 'forklift_one'

    # Launch argument: use simulation time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'test_design.rviz')

    # Convert XACRO → URDF on the fly
    robot_description = Command([
        'xacro ', xacro_file,
        ' sim_mode:=', use_sim_time
    ])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Launch Gazebo (Ignition) via ros_gz_sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', 'forklift',
            '-allow_renaming', 'true'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn joint_state_broadcaster AFTER the robot is spawned
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
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

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_rviz,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        delayed_joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
