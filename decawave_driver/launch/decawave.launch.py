from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")

    serial_port = LaunchConfiguration("serial_port")
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0")

    enable_tf = LaunchConfiguration("enable_tf")
    enable_tf_arg = DeclareLaunchArgument("enable_tf", default_value="False")

    decawave = Node(
        package="decawave_driver",
        namespace=namespace,
        executable="decawave_driver",
        parameters=[{"enable_tf": enable_tf}, {"serial_port": serial_port}],
    )

    return LaunchDescription([namespace_arg, serial_port_arg, enable_tf_arg, decawave])
