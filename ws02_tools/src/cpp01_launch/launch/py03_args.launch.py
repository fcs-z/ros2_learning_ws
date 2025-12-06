from launch import LaunchDescription
from launch_ros.actions import Node

from pkg_resources import declare_namespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    ros2 launch cpp01_launch py03_args.launch.py
    ros2 launch cpp01_launch py03_args.launch.py background_r:=200 background_g:=80 background_b:=30
    """
    
    decl_bg_r = DeclareLaunchArgument(
        name="background_r",        # 参数名
        default_value="255"         # 参数默认值
    )
    
    decl_bg_g = DeclareLaunchArgument(
        name="background_g",
        default_value="255"
    )
    
    decl_bg_b = DeclareLaunchArgument(
        name="background_b",
        default_value="255"
    )
    
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[{
            "background_r": LaunchConfiguration("background_r"),
            "background_g": LaunchConfiguration("background_g"),
            "background_b": LaunchConfiguration("background_b"),
        }]
    )
    
    return LaunchDescription([
        decl_bg_r,
        decl_bg_g,
        decl_bg_b,
        turtle
    ])