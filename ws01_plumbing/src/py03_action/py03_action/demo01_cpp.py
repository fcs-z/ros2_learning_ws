import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces_demo.action import Progress

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ProgressActionServer(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.action_server_ = ActionServer(
            self,
            Progress,
            "get_sum",
            execute_callback=self.execute_callback,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=ReentrantCallbackGroup()
            )
        self.get_logger().info("动作服务端已启动")
    
    def handle_goal(self,goal_request):
        self.get_logger().info(f"接受到动作客户端请求,请求数字为 {goal_request.num}")
        if goal_request.num < 1:
            return rclpy.action.GoalResponse.REJECT
        return rclpy.action.GoalResponse.ACCEPT_AND_EXECUTE
    
    def handle_cancel(self,goal_handle):
        self.get_logger().info("接受到任务取消请求")
        return rclpy.action.CancelResponse.ACCEPT
    
    def execute_callback(self,goal_handle):
        self.get_logger().info("开始执行任务")
        loop_rate = self.create_rate(1.0)

        goal = goal_handle.request
        feedback = Progress.Feedback()
        result = Progress.Result()
        
        sum = 0
        for i in range(1, goal.num + 1):
            sum += i
            feedback.progress = i/goal.num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"连续反馈中,进度: {feedback.progress:.2f}")
            
            if goal_handle.is_cancel_requested:
                result.sum = sum
                goal_handle.canceled(result)
                self.get_logger().info("任务取消")
                return result
            
            loop_rate.sleep()
        
        if rclpy.ok():
            result.sum = sum
            goal_handle.succeed()
            self.get_logger().info("任务完成")
        
        return result

        
def main():
    rclpy.init()
    action_server = ProgressActionServer("progress_action_server")
    
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server,executor=executor)
    # rclpy.spin(action_server)
    
    rclpy.shutdown()