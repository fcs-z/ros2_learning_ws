from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    
    spawn = ExecuteProcess(
        # 方式一
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{x: 8.0, y: 9.0, theta: 0.0, name: 'turtle2'}\""],
        # 方式二
        # cmd=[
        #     FindExecutable(name="ros2"),
        #     "service call",
        #     "/spawn turtlesim/srv/Spawn",
        #     "\"{x: 8.0,y: 9.0, theta: 1.0, name: 'turtle2'}\""
        # ],
        
        output="both",      # 同时把命令的输出显示到终端 和 写入日志
        shell=True          # 允许命令带空格
    )
    
    return LaunchDescription([
        turtle,
        spawn
    ])