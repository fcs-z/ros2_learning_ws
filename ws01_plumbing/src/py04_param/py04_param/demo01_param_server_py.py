import rclpy
from rclpy.node import Node
 
class MinimalParamServer(Node):
    def __init__(self,node_name):
        super().__init__(node_name,allow_undeclared_parameters=True)
        self.get_logger().info("参数演示")
    # 声明参数 
    def declare_param(self):
        self.declare_parameter("car_type","Tiger")
        self.declare_parameter("height",1.50)
        self.declare_parameter("wheels",4)
        self.p1 = rclpy.Parameter("car_type",value="Mouse")
        self.p2 = rclpy.Parameter("undcl_test",value=100)
        self.set_parameters([self.p1,self.p2])
    # 查询参数
    def get_param(self):
        self.get_logger().info("--------查--------")
        # 判断包含
        self.get_logger().info("包含 car_type? %d" % self.has_parameter("car_type"))
        self.get_logger().info("包含 width? %d" % self.has_parameter("width"))
        # 获取指定
        car_type = self.get_parameter("car_type")
        self.get_logger().info("%s=%s" % (car_type.name,car_type.value))
        # 获取所有
        params = self.get_parameters(["car_type","height","wheels"])
        self.get_logger().info("解析所有参数:")
        for param in params:
            self.get_logger().info("%s---> %s" % (param.name,param.value))
    # 修改参数
    def update_param(self):
        self.get_logger().info("--------改--------")
        self.set_parameters([rclpy.Parameter("car_type",value="horse")])
        param = self.get_parameter("car_type")
        self.get_logger().info("修改后: car_type=%s" % param.value)
    # 删除参数
    def del_param(self):
        self.get_logger().info("--------删--------")
        self.get_logger().info("删除操作前包含: car_type吗? %d" % self.has_parameter("car_type"))
        self.undeclare_parameter("car_type")
        self.get_logger().info("删除操作后包含: car_type吗? %d" % self.has_parameter("car_type"))
        
def main():
    rclpy.init()
    param_server = MinimalParamServer("minimal_param_server")
    
    param_server.declare_param()
    param_server.get_param()
    param_server.update_param()
    param_server.del_param()
    
    rclpy.spin(param_server)
    rclpy.shutdown()