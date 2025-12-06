import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

class TurtleFrameListener(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 声明并解析参数
        self.declare_parameter('target_frame','turtle2')
        self.declare_parameter('source_frame','turtle1')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        
        # 创建tf缓存对象指针
        self.tf_buffer = Buffer()
        
        # 创建tf监听器
        self.tf_listener = TransformListener(self.tf_buffer,self)

        self.publisher = self.create_publisher(Twist,self.target_frame+'/cmd_vel',1)
        self.timer = self.create_timer(1.0,self.on_timer)
        
    def on_timer(self):
        # 按照条件查找符合条件的坐标系,并生成坐标转换后的坐标帧
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now
            )
        except TransformException as e:
            self.get_logger().info(f'{self.source_frame} 到 {self.target_frame} 坐标变换异常!')
            return
    
        # 生成turtle2的速度指令,并发布
        msg = Twist()
        scale_rotation_rate = 1.0
        msg.angular.z = scale_rotation_rate * math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x
        )
        
        scale_forward_speed = 0.5
        msg.linear.x = scale_forward_speed * math.sqrt(
            trans.transform.translation.x **2 + 
            trans.transform.translation.y **2
        )
        
        self.publisher.publish(msg)
 
def main():
    rclpy.init()
    node = TurtleFrameListener("py_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()