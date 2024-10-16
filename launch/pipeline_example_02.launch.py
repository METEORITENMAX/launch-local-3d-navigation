from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the topics
    topic_octomap = '/octomap_binary'
    topic_atarStartPoint = '/interactive_marker_node_start/out/pose3d/start'
    topic_atarGoalPoint = '/interactive_marker_node_goal/out/pose3d/goal'
    package_name = 'launch-local-3d-navigation'
    # Declare the RViz config argument
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'rviz2', 'pipeline_example_02.rviz'),
        description='Path to the RViz config file'
    )

    rviz_config_file = LaunchConfiguration('rvizconfig')

    # Define the paths to the launch files
    launch_file_1_path = os.path.join(get_package_share_directory('imarker_pose_mockups'), 'launch', 'example_01.launch.py')
    launch_file_1 = PythonLaunchDescriptionSource(launch_file_1_path)

    # Include the first launch file
    include_launch_file_1 = IncludeLaunchDescription(launch_file_1)

    # Include the second launch file with a delay and pass parameters
    delayed_launch_node = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            Node(
                package='octomap_astar_ros',
                executable='AstarPlannerNode',
                name='astar_planner_node',
                output='screen',
                remappings=[
                    ('/astar_planner_node/in/pose3d/goal', topic_atarGoalPoint),
                    ('/astar_planner_node/in/pose3d/start', topic_atarStartPoint),
                    ('/astar_planner/in/octomap', '/octomap_heightmod/out/octomap_heightMod')
                ]
            )
        ]
    )

    # Nodes
    mockup_node = Node(
        package='octomap_utils',
        executable='OctomapMockupPublisherNode',
    )

    height_mod_node = Node(
        package='octomap_utils',
        executable='OctoMapHeightModNode',
        remappings=[
            ('/octomap_heightmod/in/octomap_init', topic_octomap)
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    pose_converter_node = Node(
        package='conversion_poses_to_tf',
        executable='path_to_tf_converter_node',
        name='path_to_tf_converter_node',
        output='screen',
        remappings=[
            ('/path_topic', '/astar_planner_node/out/localPath'),
        ]
    )

    return LaunchDescription([
        rviz_config_arg,
        #rviz_node,
        include_launch_file_1,
        # mockup_node,
        height_mod_node,
        delayed_launch_node,
        pose_converter_node
    ])

if __name__ == '__main__':
    generate_launch_description()