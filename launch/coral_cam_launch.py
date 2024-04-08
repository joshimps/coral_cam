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
                    parameters=[{'button_pin_number' : 13}],
                ),
                ComposableNode(
                    package='coral_cam',
                    plugin='coral_cam::RealsenseCamera',
                    name='realsense_camera_node'
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])