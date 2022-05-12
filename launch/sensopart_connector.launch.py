from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the sensopart_node.

    To run this launch file, call:
      ros2 launch sensopart_connector sensopart_connector.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch sensopart_connector sensopart_connector.launch.py --show-args

    :return:
    """
    ip = LaunchConfiguration('ip')
    auto_connect = LaunchConfiguration('auto_connect')

    ip_argument = DeclareLaunchArgument(
        'ip',
        default_value='172.31.1.198',
        description="Default ip for the Sensopart Camera"
    )
    auto_connect_argument = DeclareLaunchArgument(
        'auto_connect',
        default_value='True',
        description="If set to true, the node will automatically try to connect on startup."
    )
    sensopart_connector_node = Node(
        package='sensopart_connector',
        node_executable='sensopart_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ip': ip,
            'auto_connect': auto_connect
        }]
    )

    return LaunchDescription([
        ip_argument,
        auto_connect_argument,
        sensopart_connector_node
    ])
