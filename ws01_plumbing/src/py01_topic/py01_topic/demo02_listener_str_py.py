import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.subscription_ = self.create_subscription(String,"topic",self.listener_callback,10)
        
    
    def listener_callback(self,msg):
        self.get_logger().info("订阅消息：'%s'" % msg.data)

def main():
    rclpy.init()
    miniml_subscriber = MinimalSubscriber("miniml_subscriber")
    rclpy.spin(miniml_subscriber)
    rclpy.shutdown()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    