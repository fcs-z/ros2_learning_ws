from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess,RegisterEventHandler,LogInfo
from launch.substitutions import FindExecutable

from launch.event_handlers import OnProcessStart,OnProcessExit

def generate_launch_description():
    
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    
    spawn = ExecuteProcess(
        # 方式一
        # cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{x: 8.0, y: 9.0, theta: 0.0, name: 'turtle2'}\""],
        # 方式二
        cmd=[
            FindExecutable(name="ros2"),
            "service call",
            "/spawn turtlesim/srv/Spawn",
            "\"{x: 8.0,y: 9.0, theta: 1.0, name: 'turtle2'}\""
        ],
        
        output="both",      # 同时把命令的输出显示到终端 和 写入日志
        shell=True          # 允许命令带空格
    )
    
    start_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtle,
            on_start=spawn
        )
    )
    
    exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtle,
            on_exit=[LogInfo(msg="turtlesim_node 退出!")]
        )
    )
    
    return LaunchDescription([
        turtle,
        start_event,
        exit_event
    ])