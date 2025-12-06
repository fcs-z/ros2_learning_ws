# def main():
#     print('Hi from pkg02_helloworld_py.')


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("hello world")

def main():
    rclpy.init()
    node = MyNode("py_node")
    rclpy.spin(node)
    rclpy.shutdown()



    

        































