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
                name='coral_cam_container',
                namespace='coral_cam',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Button',
                        name='button_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"debounce_time_us":20000}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::RealsenseCamera',
                        name='realsense_camera_node',
                        namespace='coral_cam',
                        parameters = [{"point_cloud_path":"/home/pam/Documents/PointClouds",
                                       "number_of_captures":10}],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Gpio',
                        name='gpio_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"gpio_number": 5},{"button_pin": 27}],
                    ),
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::ResizeNode',
                        name='resize_node',
                        remappings=[
                            ('image/image_raw', '/camera/color/image_rect_raw'),
                            ('/image/camera_info','/camera/color/camera_info'),
                            ('/resize/image_raw','/camera/color/image_rect_resize')
                        ],
                        parameters=[{
                            'use_scale':False,
                            'width': 960,
                            'height': 540,
                        }]
            )
                ],
                output='screen',
        ),
        Node(
            package='coral_cam',
            namespace='coral_cam',
            executable='gui',
            name='gui_node'
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('coral_cam'), 'launch'),
         '/realsense_camera_launch.py']),
        launch_arguments={'camera_name': 'D405',
                          'pointcloud.enable': 'true',
                          'pointcloud.ordered_pc':'true',
                          'enable_sync':'true',
                          'clip_distance' : '0.8',
                          'decimation_filter.enable':'true',
                          'spatial_filter.enable':'true',
                          'hole_filling_filter.enable':'false',
                         }.items()
        )
        
    ])

