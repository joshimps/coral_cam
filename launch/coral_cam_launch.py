import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


#GPIO Variables
gpio_number = 5
capture_button_pin = 27
debounce_time_us = 10000

#System Settings
number_of_captures = 1
point_cloud_path = "/home/pam/git/coral_cam/clouds"

#Image Processing Settings
desired_width = 800
desired_height = 360
input_image = '/real_sense/color/image_rect_raw'
input_info = '/real_sense/color/camera_info'
output_image = '/real_sense/color/image_rect_resized'
output_info = '/real_sense/color/camera_info_resized'

real_sense_node_params = [ {'name': 'camera_name',                  'default': 'D405', 'description': 'camera unique name'},
                           {'name': 'camera_namespace',             'default': 'real_sense', 'description': 'namespace for camera'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'log_level',                    'default': 'debug', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile',     'default': '0,0,0', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
                           {'name': 'enable_infra1',                'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'true', 'description': 'enable infra2 stream'},
                           {'name': 'depth_module.color_profile',   'default': '0,0,0', 'description': 'Depth module color stream profile'},
                           {'name': 'depth_module.depth_profile',   'default': '0,0,0', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_profile',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           
                           
                           #These are commented out as they disable the use of auto exposure on the D405 camera, this is a well known reported bug
                           
                           #{'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},
                           #{'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},
                           #{'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},
                           #{'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},'''
                           
                           
                           
                           {'name': 'enable_sync',                  'default': 'true', 'description': "'enable sync mode'"},
                           {'name': 'enable_rgbd',                  'default': 'false', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel',                 'default': 'false', 'description': "'enable accel stream'"},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance',                'default': '0.8', 'description': "''"},
                           {'name': 'angular_velocity_cov',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''},
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc',        'default': 'true', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'spatial_filter.enable',        'default': 'true', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable',       'default': 'false', 'description': 'enable_temporal_filter'},
                           {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
        return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]
    
def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    
    return LaunchDescription(
        declare_configurable_parameters(real_sense_node_params)+
        [
        Node(
            package='coral_cam',
            namespace='coral_cam',
            executable='gui',
            name='gui_node'
        ),
        ComposableNodeContainer(
                name='coral_cam_container',
                namespace='coral_cam',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                     ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Battery',
                        name='battery_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::CaptureButton',
                        name='capture_button_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"debounce_time_us":debounce_time_us,
                                       "capture_button_pin": capture_button_pin}],
                    ),
                    ComposableNode(
                        package='coral_cam',
                        plugin='coral_cam::Gpio',
                        name='gpio_node',
                        namespace='coral_cam',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [{"gpio_number": gpio_number},],
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
                    ),
                    ComposableNode(
                        package='realsense2_camera',
                        plugin='realsense2_camera::RealSenseNodeFactory',
                        name='D405',
                        namespace='real_sense',
                        extra_arguments=[{'use_intra_process_comms': True}],
                        parameters = [set_configurable_parameters(real_sense_node_params)],
                    ),
                ],
                emulate_tty=True,
                output='screen',
        ),
        
        #This is really bad, using a timer to delay a node startup is shoddy design. However...
        #using this was the only way I found to get around a bug where image_proc remppings wouldn't
        #apply if the realsense topics existed before image_proc was done loading.
        
        #TimerAction(
        #    period = 5.0,
        #    actions = [
        #         IncludeLaunchDescription(
        #            PythonLaunchDescriptionSource([os.path.join(
        #            get_package_share_directory('coral_cam'), 'launch'),
        #            '/realsense_camera_launch.py']),
        #            launch_arguments={'camera_name': camera_name,
        #                            'camera_namespace':camera_namespace,
        #                            'pointcloud.enable': pointcloud_enable,
        #                            'pointcloud.ordered_pc':pointcloud_ordered_pc,
        #                            'enable_sync':enable_sync,
        #                            'clip_distance' : clip_distance,
        #                            'decimation_filter.enable':decimation_filter_enable,
        #                            'spatial_filter.enable':spatial_filter_enable,
        #                            'hole_filling_filter.enable':hole_filling_filter_enable,
        #                            'enable_infra1':'true',
        #                            'enable_infra2':'true',
        #                            
        #                            }.items()
        #            ),
        #        ]
        #)
    ])

