import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # use config file if provided
    launch_config_yaml = DeclareLaunchArgument('config_yaml', 
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("bob_transformers"),
                "config", "tti.yaml")))

    # launch with terminal
    launch_terminal = DeclareLaunchArgument('terminal', 
        default_value="false")

    tti_node = Node(
        package='bob_transformers',
        executable='tti',
        name='tti',
        parameters=[
            LaunchConfiguration('config_yaml')]
    )

    terminal = Node(
        condition=IfCondition(LaunchConfiguration("terminal")),
        package='bob_topic_tools',
        executable='terminal',
        name='terminal',
        remappings=[
            ('topic_in_cr', 'tti_in'),
            ('topic_out', 'tti_in')],
        parameters=[
            {'title': 'Transformers TTI Terminal'},
            LaunchConfiguration('config_yaml')
        ]
    )

    return LaunchDescription([
        launch_config_yaml,
        launch_terminal,
        tti_node, 
        terminal
    ])
