import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

class MinimalSubscriber(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.subscription_ = self.create_subscription(Student,"topic_stu",self.listener_callback,10)
        
    
    def listener_callback(self,stu):
        self.get_logger().info(f"订阅的学生信息: name={stu.name},age={stu.age},height={stu.height}")

def main():
    rclpy.init()
    miniml_subscriber = MinimalSubscriber("student_subscriber")
    rclpy.spin(miniml_subscriber)
    rclpy.shutdown()
    