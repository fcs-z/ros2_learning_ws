import rclpy
from rclpy.node import Node

import sys
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

'''
ros2 run py03_tf_broadcaster demo01_static_tf_broadcaster_py 0.4 0 0.2 0 0 0 base_link laser
ros2 run py03_tf_broadcaster demo01_static_tf_broadcaster_py -0.5 0 0.4 0 0 0 base_link camera
rviz2
'''

class MinimalStaticFrameBroadcasterPy(Node):
    def __init__(self,node_name,transformation):
        super().__init__(node_name)
        # 创建静态坐标变换发布方
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.make_transforms(transformation)
        
    def make_transforms(self,transformation):
        # 组织消息
        # static_transformStamped = TransformStamped()
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = sys.argv[7]
        t.child_frame_id = sys.argv[8]
        t.transform.translation.x = float(sys.argv[1])
        t.transform.translation.y = float(sys.argv[2])
        t.transform.translation.z = float(sys.argv[3])
        
        quat = tf_transformations.quaternion_from_euler(
            float(sys.argv[4]),
            float(sys.argv[5]),
            float(sys.argv[6])
        )
        t.transform.rotation.x=quat[0]
        t.transform.rotation.y=quat[1]
        t.transform.rotation.z=quat[2]
        t.transform.rotation.w=quat[3]
        
        # 发布消息
        self._tf_publisher.sendTransform(t)
 
def main():
    if len(sys.argv)<9:
        rclpy.logging('logger').info('运行程序时请按照: x y z roll pitch yaw frame_id child_id 的格式传入参数')
        sys.exit(0)
    rclpy.init()
    node = MinimalStaticFrameBroadcasterPy("minimal_static_frame_broadcaster_py",sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()