import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from base_interfaces_demo.action import Progress
import sys

class ProgressActionClient(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.action_client_ = ActionClient(self,Progress,"get_sum")
    
    # 发送请求
    def send_goal(self,num):
        self.action_client_.wait_for_server()
        
        goal_msg = Progress.Goal()
        goal_msg.num = num
        self.send_goal_future_ = self.action_client_.send_goal_async(goal_msg,self.feedback_callback)
        self.send_goal_future_.add_done_callback(self.goal_response_callback)
    
    # 处理目标发送后的反馈
    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("请求被拒绝")
            return
        
        self.get_logger().info("请求被接受,开始执行任务")
        
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.get_result_callback)
        
    # 处理最终响应
    def get_result_callback(self,future):
        result = future.result().result
        self.get_logger().info("最终计算结果:sum = %d" % result.sum)
        # 释放资源
        # rclpy.shutdown()
    
    # 处理连续反馈
    def feedback_callback(self,feedback_msg):
        progress = (int)(feedback_msg.feedback.progress * 100)
        self.get_logger().info("当前进度: %d%%" % progress)
        
 
def main():
    if len(sys.argv) !=2:
        rclpy.logging.get_logger("rclpy").error("请提交一个整数")
        return
    
    rclpy.init()
    action_client = ProgressActionClient("progress_action_client")
    action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(action_client)
    rclpy.shutdown()