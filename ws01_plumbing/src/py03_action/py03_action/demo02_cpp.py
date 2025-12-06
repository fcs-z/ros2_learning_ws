import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from base_interfaces_demo.action import Progress
import sys

from rclpy.executors import SingleThreadedExecutor

class ProgressActionClient(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.action_client_ = ActionClient(self,Progress,"get_sum")
    
    def send_goal(self,num):
        if not self.action_client_.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("服务连接失败")
            return
        
        goal_msg = Progress.Goal()
        goal_msg.num = num
        self.get_logger().info(f"发送请求数据: {num}")
        
        self.send_goal_future_ = self.action_client_.send_goal_async(goal_msg,self.feedback_callback)
        self.send_goal_future_.add_done_callback(self.goal_response_callback)
    
    # 目标响应
    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("请求被拒绝")
            return
        
        self.get_logger().info("目标被接受,等待结果中")
        
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.result_callback)
        
    # 连续反馈
    def feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        progress = int(feedback.progress * 100)
        self.get_logger().info(f"当前进度: {progress}%")
        
    # 最终响应
    def result_callback(self,future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"任务执行完毕,最终结果: {result.sum}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("任务被中止")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("任务被取消")
        else:
            self.get_logger().error("未知异常")
            
        # rclpy.shutdown()
        
def main():
    if len(sys.argv) !=2:
        rclpy.logging.get_logger("rclpy").error("请提交一个整数")
        return 1
    
    rclpy.init()
    action_client = ProgressActionClient("progress_action_client")
    
    action_client.send_goal(int(sys.argv[1]))
    
    rclpy.spin(action_client)
    rclpy.shutdown()