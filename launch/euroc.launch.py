from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():

    dataset_arg = DeclareLaunchArgument(
        "dataset_path", description="Path to the dateset to use")
    dataset_period_arg = DeclareLaunchArgument(
        "dataset_period", description="Publishing period in milliseconds", default_value='100')

    container_name_arg = DeclareLaunchArgument(
        name='target_container', default_value='', description='Target container where to load components'
    )

    namespace_arg = DeclareLaunchArgument(
        name='namespace', description="namespace", default_value="/")

    composable_nodes = [
        ComposableNode(package="data_publisher", plugin="simulation::EurocPublisher", parameters=[{
            "dataset_path": LaunchConfiguration(dataset_arg.name),
            "period_ms": LaunchConfiguration(dataset_period_arg.name),
            "frame_id": "camera_frame"}], namespace=LaunchConfiguration(namespace_arg.name))]

    return LaunchDescription([
        dataset_arg,
        dataset_period_arg,
        container_name_arg,
        namespace_arg,
        # Load components in an newely created container when the container name is not provided
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals(container_name_arg.name, ''),
            package='rclcpp_components',
            executable='component_container',
            name='data_euroc',
            namespace='',
            composable_node_descriptions=composable_nodes,
        ),
        # Otherwise load directly in provided container
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals(
                container_name_arg.name, ''),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration(container_name_arg.name),
        ),
        # If a container name is not provided set the name of the container
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals(container_name_arg.name, ''),
            name=container_name_arg.name,
            value='data_euroc_container'
        )
    ])
