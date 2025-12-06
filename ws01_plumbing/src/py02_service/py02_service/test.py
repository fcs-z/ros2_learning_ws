import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts
 
class S(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.server = self.create_service(AddInts,"add",self.add_callback)
        self.get_logger().info('服务启动')
    
    def add_callback(self,request,response):
        response.sum = request.num1 + request.num2
        return response
        
    
def main():
    rclpy.init()
    node = S("py_node")
    rclpy.spin(node)
    rclpy.shutdown()







import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts
import sys

class C(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.client_ = self.create_client(AddInts,"add")
    
    def is_connect(self):
        while not self.client_.wait_for_service(1):
            if not rclpy.ok():
                self.get_logger().info("服务中断")
                return False
            self.get_logger().info("服务连接中")
        return True
    
    def send(self,num1,num2):
        request = AddInts.Request()
        request.num1 = num1
        request.num2 = num2
        future = self.client_.call_async(request)
        return future
        
 
def main():
    rclpy.init()
    client = C("py_node")
    
    flag = client.is_connect()
    if flag:
        client.get_logger().info("连接成功")
    
    num1 = int(sys.argv[1])
    num2 = int(sys.argv[2])
    future = client.send(num1,num2)
    rclpy.spin_until_future_complete(client,future)
    if future.done:
        try:
            client.get_logger().info(f'{num1}+{num2}={future.result().sum}')
        except Exception as e:
            client.get_logger().warn(f'服务失败')
    else:
        client.get_logger().error("请求失败")
    
    
    rclpy.shutdown()