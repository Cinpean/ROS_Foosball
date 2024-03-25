from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    serial_publisher = Node(
        package="picamera2_for_ros2_pkg",
        executable="Serial_send_publisher"
    )

    serial_subscriber = Node(
        package="picamera2_for_ros2_pkg",
        executable="Serial_send_subscribe"
    )

    camera_v4 = Node(
        package="picamera2_for_ros2_pkg",
        executable="cameratopic_node_v4"
    )



    ld.add_action(serial_publisher)
    ld.add_action(serial_subscriber)
    ld.add_action(camera_v4)

    return ld

