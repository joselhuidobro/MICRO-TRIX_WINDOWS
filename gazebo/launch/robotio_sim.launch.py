from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']
        ),
        # args opcionales: ['verbose:=true']
    )

    # Publisher/bridge que siempre se reintenta
    robotio_pub = Node(
        package='robotio_bridge',
        executable='robotio_pub',   # tu script python instalado como entry-point o ejecutable
        name='robotio_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': True}],
    )

    # Espera 3 s para spawnear robot (da tiempo a que Gazebo levante /clock)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robotio',
                output='screen',
                arguments=['-topic', 'robot_description', '-entity', 'robotio']
            )
        ]
    )

    return LaunchDescription([
        # útil si tienes recursos personalizados
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value='/workdir/gazebo/worlds:/workdir/gazebo/models'),
        gazebo_launch,
        robotio_pub,
        spawn_robot,
    ])

