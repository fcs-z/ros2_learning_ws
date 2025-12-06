from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    
    server = Node(
        package="cpp07_exercise",
        executable="exe04_action_server"
    )
    
    
    return LaunchDescription([
        turtle,
        server
    ])