import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Minimalpublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(String,"topic",10)
        self.timer_ = self.create_timer(0.5,self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'hello py :%d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info("发布消息：'%s'" % msg.data)
        self.i += 1
    
def main():
    rclpy.init()
    minimal_publisher = Minimalpublisher("minimal_publisher")
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    