from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    mode = LaunchConfiguration("mode")
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")
    publisher_topic = LaunchConfiguration("publisher_topic")
    subscription_topic = LaunchConfiguration("subscription_topic")

    declare_mode_launch_argument = DeclareLaunchArgument(
        "mode",
        default_value="server",
        description="TODO: Description of the mode argument.",
    )

    declare_ip_launch_argument = DeclareLaunchArgument(
        "ip",
        default_value="0.0.0.0",
        description="TODO: Description of the ip argument.",
    )

    declare_port_launch_argument = DeclareLaunchArgument(
        "port",
        default_value="5000",
        description="TODO: Description of the port argument.",
    )

    declare_publisher_topic_launch_argument = DeclareLaunchArgument(
        "publisher_topic",
        default_value="",
        description="TODO: Description of the publisher_topic argument.",
    )

    declare_subscription_topic_launch_argument = DeclareLaunchArgument(
        "subscription_topic",
        default_value="",
        description="TODO: Description of the subscription_topic argument.",
    )

    launch_sensor_msgs_msg_laser_scan_tcp_client_node = Node(
        condition=IfCondition(PythonExpression(['"', mode, '"', "==", '"client"'])),
        package="tcp_ip_bridge",
        executable="sensor_msgs_msg_laser_scan_tcp_client",
        name="sensor_msgs_msg_laser_scan_tcp_client",
        parameters=[
            {
                "ip": ip,
                "port": port,
                "publisher_topic": publisher_topic,
                "subscription_topic": subscription_topic,
            }
        ],
    )

    launch_sensor_msgs_msg_laser_scan_tcp_server_node = Node(
        condition=IfCondition(PythonExpression(['"', mode, '"', "==", '"server"'])),
        package="tcp_ip_bridge",
        executable="sensor_msgs_msg_laser_scan_tcp_server",
        name="sensor_msgs_msg_laser_scan_tcp_server",
        parameters=[
            {
                "port": port,
                "publisher_topic": publisher_topic,
                "subscription_topic": subscription_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_mode_launch_argument,
            declare_ip_launch_argument,
            declare_port_launch_argument,
            declare_publisher_topic_launch_argument,
            declare_subscription_topic_launch_argument,
            launch_sensor_msgs_msg_laser_scan_tcp_client_node,
            launch_sensor_msgs_msg_laser_scan_tcp_server_node,
        ]
    )
