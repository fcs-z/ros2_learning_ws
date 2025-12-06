import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
from base_interfaces_demo.action import Progress

class ProgressActionServer(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.action_server_ = ActionServer(self,Progress,"get_sum",self.execute_callback)
        self.get_logger().info("动作服务端已启动")
    
    def execute_callback(self,goal_handle):
        self.get_logger().info("开始执行任务")

        # 生成连续反馈
        goal = goal_handle.request
        feedback = Progress.Feedback()
        
        sum = 0
        for i in range(1, goal.num + 1):
            sum += i
            feedback.progress = i/goal.num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("连续反馈:%.2f" % feedback.progress)
            time.sleep(1)
            
        # 生成最终响应
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum
        self.get_logger().info("任务完成！")
        return result

def main():
    rclpy.init()
    action_server = ProgressActionServer("progress_action_server")
    rclpy.spin(action_server)
    rclpy.shutdown()