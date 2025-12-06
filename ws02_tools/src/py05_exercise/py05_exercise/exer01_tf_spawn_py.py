import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
 
class TurtleSpawnClient(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        # 声明并获取参数
        self.x = self.declare_parameter("x",2.0)
        self.y = self.declare_parameter("y",2.0)
        self.theta = self.declare_parameter("theta",0.0)
        self.turtle_name = self.declare_parameter("turtle_name","turtle2")

        # 创建客户端
        self.cli = self.create_client(Spawn,'/spawn')
        
        # 等待服务连接
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务连接中,请稍候...')
        
        self.req = Spawn.Request()
    
    def send_request(self):
        self.req.x = self.get_parameter("x").get_parameter_value().double_value
        self.req.y = self.get_parameter("y").get_parameter_value().double_value
        self.req.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.req.name = self.get_parameter("turtle_name").get_parameter_value().string_value
        
        self.future = self.cli.call_async(self.req)
        
def main():
    rclpy.init()
    client = TurtleSpawnClient("turtle_spawn_client")
    
    client.send_request()
    
    # 处理响应
    rclpy.spin_until_future_complete(client,client.future)
    try:
        response = client.future.result()
    except Exception as e:
        client.get_logger().info(f'服务请求失败:{e}')
    else:
        if len(response.name) == 0:
            client.get_logger().info('乌龟重名了,创建失败!')
        else:
            client.get_logger().info(f'乌龟 {response.name} 被创建')
    
    rclpy.shutdown()