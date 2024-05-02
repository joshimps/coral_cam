import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
                name='my_container',
                namespace='coral_camera',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Button',
                        name='button_node',
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::RealsenseCamera',
                        name='realsense_camera_node',
                        parameters = [{"point_cloud_path":"/home/josh/git/coral_cam/clouds"}],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Gpio',
                        name='gpio_node',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"gpio_number": 5},{"button_pin": 27}],
                    )
                ],
                output='screen',
        ),IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('coral_cam'), 'launch'),
         '/realsense_camera_launch.py']),
        launch_arguments={'camera_name': 'D405',
                          'pointcloud.enable': 'true',
                          'pointcloud.ordered_pc':'true',
                          'enable_sync':'true',
                          'clip_distance' : '0.8',
                          'hole_filling_filter.enable':'false',
                         }.items()
        )
        
    ])

