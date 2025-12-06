import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

'''
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
rviz2
ros2 run py03_tf_broadcaster demo02_dynamic_tf_broadcaster_py
'''

class MinimalDynamicFrameBroadcasterPy(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 创建动态坐标变换发布方
        self.br = TransformBroadcaster(self)
        # 创建乌龟位姿订阅方
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            1
        )
        self.subscription
    
    # 根据订阅到的乌龟位姿生成坐标帧,并广播
    def handle_turtle_pose(self,msg):
        # 组织消息
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        
        quat = tf_transformations.quaternion_from_euler(0,0,msg.theta)
        t.transform.rotation.x=quat[0]
        t.transform.rotation.y=quat[1]
        t.transform.rotation.z=quat[2]
        t.transform.rotation.w=quat[3]
        
        # 发布消息
        self.br.sendTransform(t)
 
def main():
    rclpy.init()
    node = MinimalDynamicFrameBroadcasterPy("minimal_dynamic_frame_broadcaster_py")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()