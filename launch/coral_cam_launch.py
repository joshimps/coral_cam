import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='coral_cam',
                    plugin='coral_cam::Button',
                    name='button_node',
                    parameters=[{'button_pin_number' : 27},{'gpio_number' : 5}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='coral_cam',
                    plugin='coral_cam::RealsenseCamera',
                    name='realsense_camera_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])