import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped

'''
ros2 run py03_tf_broadcaster demo01_static_tf_broadcaster_py 0.4 0 0.2 0 0 0 base_link laser
ros2 run py03_tf_broadcaster demo03_point_publisher_py 
rviz2
'''

class MinimalPointPublisherPy(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 创建坐标点发布方
        self.pub = self.create_publisher(PointStamped,"point",10)
        # 创建定时器
        self.timer = self.create_timer(1,self.on_timer)
        self.x=0.1
    
    def on_timer(self):
        # 组织并发布坐标点消息
        ps = PointStamped()
        
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'laser'
        self.x += 0.02
        ps.point.x = self.x
        ps.point.y = 0.0
        ps.point.z = 0.2
        
        self.pub.publish(ps)
 
def main():
    rclpy.init()
    node = MinimalPointPublisherPy("minimal_point_publisher_py")
    rclpy.spin(node)
    rclpy.shutdown()