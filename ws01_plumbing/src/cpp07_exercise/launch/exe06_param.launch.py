from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    
    param = Node(
        package="cpp07_exercise",
        executable="exe06_param"
    )
    
    
    return LaunchDescription([
        turtle,
        param
    ])