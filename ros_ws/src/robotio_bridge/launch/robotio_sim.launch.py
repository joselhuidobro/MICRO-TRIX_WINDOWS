from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        get_package_share_directory('robotio_description'),
        'urdf', 'robotio.urdf.xacro'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']
        ),
        launch_arguments={'gui': 'false', 'verbose': 'true'}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str),
        }],
    )

    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robotio'],
            output='screen'
        )]
    )

    bridge = Node(
        package='robotio_bridge',
        executable='run_servo_to_controller',
        name='servo_to_controller',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([gazebo, rsp, spawn, bridge])
