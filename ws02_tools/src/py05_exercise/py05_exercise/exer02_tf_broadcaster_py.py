import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

import tf_transformations
from tf2_ros import TransformBroadcaster
 
class TurtleFrameBroadCaster(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 声明并解析乌龟名称参数
        self.declare_parameter('turtle_name','turtle1')
        self.turtlename = self.get_parameter('turtle_name').get_parameter_value().string_value
        
        # 创建动态坐标变换发布方
        self.br = TransformBroadcaster(self)
        
        # 创建乌龟位姿订阅方
        self.subscription = self.create_subscription(
            Pose,
            self.turtlename+'/pose',
            self.handle_turtle_pose,
            1
        )
        self.subscription
    
    def handle_turtle_pose(self,msg):
        # 根据订阅到的乌龟位姿生成坐标帧,并广播
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0,0,msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.br.sendTransform(t)
        
    
def main():
    rclpy.init()
    node = TurtleFrameBroadCaster("turtle_frame_broadCaster_py")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()