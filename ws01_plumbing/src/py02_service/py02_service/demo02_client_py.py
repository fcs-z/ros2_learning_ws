# import rclpy
# from rclpy.node import Node
# from base_interfaces_demo.srv import AddInts
# import sys

# class MinimalClient(Node):
#     def __init__(self,node_name):
#         super().__init__(node_name)
#         self.client_ = self.create_client(AddInts,"add_ints")
#         while not self.client_.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('服务连接中,请稍候')
#         self.request = AddInts.Request()
    
#     def send_request(self):
#         self.request.num1 = int(sys.argv[1])
#         self.request.num2 = int(sys.argv[2])
#         self.future = self.client_.call_async(self.request)


# def main():
#     rclpy.init()
#     minimal_client = MinimalClient("minimal_client_py")
#     minimal_client.send_request()

#     rclpy.spin_until_future_complete(minimal_client,minimal_client.future)
#     try:
#         response = minimal_client.future.result()
#     except Exception as e:
#         minimal_client.get_logger().info('服务请求失败:%r'%(e,))
#     else:
#         minimal_client.get_logger().info(f'相应结果:{minimal_client.request.num1} + {minimal_client.request.num2} = {response.sum}')
    
    
#     rclpy.shutdown()
    
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
                self.get_logger().info("连接中断")
                return False
            self.get_logger().info("连接中")
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
    if not flag:
        client.get_logger().info("连接错误")
        # rclpy.shutdown()
        return 1
    
    num1 = int(sys.argv[1])
    num2 = int(sys.argv[2])
    future = client.send(num1,num2)
    rclpy.spin_until_future_complete(client,future)
    if future.done():
        try:
            client.get_logger().info(f"{num1} + {num2} = {future.result().sum}")
        except Exception as e:
            client.get_logger().info("服务失败")
    else:
        client.get_logger().info("请求失败")
    
    
    rclpy.shutdown()
