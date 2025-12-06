import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

class Minimalpublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Student,"topic_stu",10)
        self.timer_ = self.create_timer(0.5,self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        stu = Student()
        stu.name = "李四"
        stu.age = self.i
        stu.height = 1.70
        self.publisher_.publish(stu)
        self.get_logger().info(f"学生信息: name={stu.name},age={stu.age},height={stu.height}")
        self.i += 1
    
def main():
    rclpy.init()
    minimal_publisher = Minimalpublisher("student_publisher")
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()
 