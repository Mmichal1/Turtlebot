from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for Robot 1 Navigator
    robot1_navigate = Node(
        package='turtlebot_mmrs',
        executable='robot1_navigate',
        name='robot1_navigate',
        output='screen'
    )

    # Node for Robot 2 Navigator
    robot2_navigate = Node(
        package='turtlebot_mmrs',
        executable='robot2_navigate',
        name='robot2_navigate',
        output='screen'
    )

    # Node for Controller
    controller = Node(
        package='turtlebot_mmrs',
        executable='controller',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        robot1_navigate,
        robot2_navigate,
        controller
    ])
