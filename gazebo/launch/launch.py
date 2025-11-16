from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robotio_bridge",  # nombre del paquete
            executable="robotio_pub",  # tu script Python compilado como ejecutable
            name="robotio_node",
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "robotio"],
            output="screen",
        ),
    ])

