import rclpy
from rclpy.node import Node

import rosbag2_py
from rclpy.logging import get_logger

class SimpleBagPlayer(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 创建读取对象
        self.reader = rosbag2_py.SequentialReader()
        # 设置读取的目标对象,话题等参数
        storage_options = rosbag2_py._storage.StorageOptions(
            uri="my_bag_py",
            storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions('','')
        self.reader.open(storage_options,converter_options)
        
    def read(self):
        # 读消息
        while self.reader.has_next():
            msg = self.reader.read_next()
            # self.get_logger().warn(f'topic={msg[0]},time={msg[2]},value={msg[1]}')
            self.get_logger().warning(f'topic={msg[0]},time={msg[2]},value={msg[1]}')
            get_logger("rclpy").info("topic=%s,time=%d,value=%s"%(msg[0],msg[2],msg[1]))
 
def main():
    rclpy.init()
    node = SimpleBagPlayer("simple_bag_player")
    node.read()
    rclpy.spin(node)
    rclpy.shutdown()