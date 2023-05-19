import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
import launch_ros

def generate_launch_description():
    urg_node_dir = get_package_share_directory('urg_node')
    # launch_description = LaunchDescription([
    #     DeclareLaunchArgument(
    #         'sensor_interface',
    #         default_value='serial',
    #         description='sensor_interface: supported: serial, ethernet')])

    # def expand_param_file_name(context):
    #     param_file = os.path.join(
    #             urg_node_dir, 'launch',
    #             'urg_node_' + context.launch_configurations['sensor_interface'] + '.yaml')
    #     if os.path.exists(param_file):
    #         return [SetLaunchConfiguration('param', param_file)]

    # param_file_path = OpaqueFunction(function=expand_param_file_name)
    # launch_description.add_action(param_file_path)

    # hokuyo_node = Node(
    #     package='urg_node', node_executable='urg_node', output='screen',
    #     parameters=[LaunchConfiguration('param')]
    #     )
    launch_description = LaunchDescription([])
    node = Node(
        package='urg_node', executable='urg_node_driver',output='screen',parameters=[{"ip_adress":"192.168.0.10"},{"ip_port":10940},{"serial_baud":115200}]
    )
    ga= GroupAction(
        actions=[
            PushRosNamespace(namespace='tb6'),

            Node(
                package='urg_node', executable='urg_node_driver',output='screen',parameters=[{"ip_adress":"192.168.0.10"},{"ip_port":10940},{"serial_baud":115200}]
            )
        ]
    )
    
    launch_description.add_action(ga)
    return launch_description