from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    '''
    1、ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo01_helloworld.urdf 
    2、ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo02_link.urdf
    3、ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo03_joint.urdf
        ros2 run joint_state_publisher_gui joint_state_publisher_gui
    4、ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo04_basefootprint.urdf
        rviz2中Fixed Frame设置为base_footprint
    5、 ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo05_exercise.urdf
    6、check_urdf demo05_exercise.urdf
    7、urdf_to_graphviz demo05_exercise.urdf
    8、ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/xacro/car.urdf.xacro
    '''
    
    cpp06_urdf_dir = get_package_share_directory("cpp06_urdf")
    default_model_path = os.path.join(cpp06_urdf_dir,"urdf/urdf","demo01_helloworld.urdf")
    default_rviz_path = os.path.join(cpp06_urdf_dir,"rviz","display.rviz")

    model = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path
    )
    
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("model")])
        # Command(["cat ",LaunchConfiguration("model")])
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d",default_rviz_path]
    )
    
    
    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])