import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import rosbag2_py
from rclpy.serialization import serialize_message

class SimpleBagRecorder(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 创建写出对象
        self.writer = rosbag2_py.SequentialWriter()
        # 设置写出的目标文件,话题等参数
        storage_options = rosbag2_py._storage.StorageOptions(
            uri="my_bag_py",
            storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions('','')
        self.writer.open(storage_options,converter_options)
        
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/turtle1/cmd_vel',
            type='geometry_msgs/msg/Twist',
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_info)
        
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.topic_callback,
            10
        )
        
        # 这行代码其实没有功能，只是让编辑器避免提示变量“未使用”的警告。本质作用依然是：保持引用不被回收。
        self.subscription
    
    def topic_callback(self,msg):
        # 写出消息
        self.writer.write(
            '/turtle1/cmd_vel',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
 
def main():
    rclpy.init()
    node = SimpleBagRecorder("simple_bag_recorder")
    rclpy.spin(node)
    rclpy.shutdown()