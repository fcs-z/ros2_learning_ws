from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    t1 = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    
    t2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t2"
    )
    
    rotate = ExecuteProcess(
        cmd=["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        output="both",
        shell=True
    )
    
    pub_sub = Node(
        package="cpp07_exercise",
        executable="exe01_pub_sub"
    )
    
    # 当 rotate 这个动作（ExecuteProcess）执行完毕退出后，再启动 pub_sub 这个节点。
    rotate_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rotate,
            on_exit=pub_sub
        )
    )
    
    return LaunchDescription([
        t1,
        t2,
        rotate,
        rotate_exit_event
    ])