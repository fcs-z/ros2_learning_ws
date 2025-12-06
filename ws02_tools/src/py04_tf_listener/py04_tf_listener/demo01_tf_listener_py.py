import rclpy
from rclpy.node import Node

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from rclpy.time import Time

'''
ros2 run py03_tf_broadcaster demo01_static_tf_broadcaster_py 0.4 0 0.2 0 0 0 base_link laser
ros2 run py03_tf_broadcaster demo01_static_tf_broadcaster_py -0.5 0 0.4 0 0 0 base_link camera
ros2 run py04_tf_listener demo01_tf_listener_py
'''

class TFListenerPy(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 创建一个缓存对象,融合多个坐标系相对关系为一颗坐标树
        self.buffer = Buffer()
        # 创建一个监听器,绑定缓存对象,会将所有广播器广播的数据写入缓存
        self.listener = TransformListener(self.buffer,self)
        # 编写一个定时器,循环实现转换
        self.timer = self.create_timer(1,self.on_timer)
    
    def on_timer(self):
        # 判断是否可以实现转换
        if self.buffer.can_transform("camera","laser",Time()):
            ts = self.buffer.lookup_transform("camera","laser",Time())
            self.get_logger().info("-----转换后的数据-----")
            self.get_logger().info(f'转换的结果,父坐标系:{ts.header.frame_id},子坐标系:{ts.child_frame_id},偏移量:{ts.transform.translation.x,ts.transform.translation.y,ts.transform.translation.z}')
        else:
            self.get_logger().info("转换失败......")

def main():
    rclpy.init()
    node = TFListenerPy("tf_listener_py")
    rclpy.spin(node)
    rclpy.shutdown()