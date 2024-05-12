import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from rclpy.qos import QoSDurabilityPolicy



def generate_launch_description():
    
    #GPIO Variables
    
    gpio_number = 5
    capture_button_pin_ = 27
    debounce_time_us = 10000
    
    #System Settings
    number_of_captures = 10
    point_cloud_path = "/home/josh/git/coral_cam/clouds"
    
    #Realsense Camera Settings
    camera_name = 'D405'
    camera_namespace = 'real_sense'
    pointcloud_enable = 'true'
    pointcloud_ordered_pc = 'true'
    enable_sync = 'true'
    clip_distance = '0.8'
    decimation_filter_enable = 'true'
    spatial_filter_enable = 'true'
    hole_filling_filter_enable ='false'
    
    #Image Processing Settings
    
    desired_width =960
    desired_height = 540
    input_image = '/' + camera_namespace + '/color/image_rect_raw'
    input_info = '/' + camera_namespace + '/color/camera_info'
    output_image = '/' + camera_namespace + '/color/image_rect_resized'
    output_info = '/' + camera_namespace + '/color/camera_info_resized'
    
    return LaunchDescription([
        ComposableNodeContainer(
                name='coral_cam_container',
                namespace='coral_cam',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::CaptureButton',
                        name='capture_button_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"debounce_time_us":debounce_time_us}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::RealSenseCamera',
                        name='real_sense_camera_node',
                        namespace='coral_cam',
                        parameters = [{"point_cloud_path":point_cloud_path,
                                       "number_of_captures":number_of_captures}],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Gpio',
                        name='gpio_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"gpio_number": gpio_number},{"button_pin": capture_button_pin_}],
                    ),
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::ResizeNode',
                        name='resize_node',
                        remappings=[
                            ('/image/image_raw', input_image),
                            ('/image/camera_info', input_info),
                            ('/resize/image_raw',output_image),
                            ('/resize/camera_info',output_info)
                        ],
                        parameters=[{
                            'use_scale':False,
                            'width': desired_width,
                            'height': desired_height,
                            'qos_overrides./parameter_events.publisher.durability':'volatile',

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
        launch_arguments={'camera_name': camera_name,
                          'camera_namespace':camera_namespace,
                          'pointcloud.enable': pointcloud_enable,
                          'pointcloud.ordered_pc':pointcloud_ordered_pc,
                          'enable_sync':enable_sync,
                          'clip_distance' : clip_distance,
                          'decimation_filter.enable':decimation_filter_enable,
                          'spatial_filter.enable':spatial_filter_enable,
                          'hole_filling_filter.enable':hole_filling_filter_enable,
                         }.items()
        )
        
    ])

