from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="ns_1",           # 命名空间
        name="t1",                  # 节点名称
        exec_name="turtle_label",   # 流程标签
        respawn=True                # 设置为True时，关闭节点后，可以自动重启。节点崩溃后自动重启的守护机制
    )
    
    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="t2",
        # 参数设置方式一
        # parameters=[{"background_r": 0,"background_g": 0,"background_b": 0}],
        # 参数设置方式二
        parameters=[os.path.join(get_package_share_directory("cpp01_launch"),"config","t2.yaml")]
    )
    
    turtle3 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="t3",
        # 话题重映射
        remappings=[("/turtle1/cmd_vel","cmd_vel")]
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        # 节点启动时传参
        arguments=["-d",os.path.join(get_package_share_directory("cpp01_launch"),"config","my.rviz")]
    )
    
    turtle4 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        # 节点启动时传参，相当于arguments传参时添加前缀 --ros-args
        ros_arguments=["--remap","__ns:=/t4_ns",
                       "--remap","__node:=t4"]
    )
    
    return LaunchDescription([
        turtle1,
        # turtle2,
        # turtle3,
        # rviz,
        # turtle4
    ])