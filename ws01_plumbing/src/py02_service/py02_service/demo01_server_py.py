# import rclpy
# from rclpy.node import Node
# from base_interfaces_demo.srv import AddInts


# class MinimalService(Node):
#     def __init__(self,node_name):
#         super().__init__(node_name)
#         self.server_ = self.create_service(AddInts,"add_ints",self.add_two_ints_callback)
#         self.get_logger().info("服务端启动")
    
#     def add_two_ints_callback(self,request,response):
#         response.sum = request.num1 + request.num2
#         self.get_logger().info(f'请求数据:({request.num1},{request.num2}),相应结果:{response.sum}')
#         return response
        
# def main():
#     rclpy.init()
#     minimal_service = MinimalService("minimal_service_py")
#     rclpy.spin(minimal_service)
#     rclpy.shutdown()
    
    
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
        self.get_logger().info(f'{request.num1} + {request.num2} = {response.sum}')
        return response
        
    
def main():
    rclpy.init()
    node = S("py_node")
    rclpy.spin(node)
    rclpy.shutdown()