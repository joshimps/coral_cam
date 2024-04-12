import launch
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
                name='my_container',
                namespace='',
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
        ),
    ])

