#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_interfaces_demo/action/progress.hpp>

using base_interfaces_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ServerGoalHandle<Progress>;
using namespace std::placeholders;

class MinimalActionServer: public rclcpp::Node{
    public:
        // MinimalActionServer(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):Node(node_name,options){
        MinimalActionServer(const std::string &node_name):Node(node_name){
            // this->action_server_ = rclcpp_action::create_server<Progress>(this,"get_sum",
            action_server_ = rclcpp_action::create_server<Progress>(this,"get_sum",
                std::bind(&MinimalActionServer::handle_goal,this,_1,_2),
                std::bind(&MinimalActionServer::handle_cancel,this,_1),
                std::bind(&MinimalActionServer::handle_accepted,this,_1));
            RCLCPP_INFO(this->get_logger(),"动作服务端创建,等待请求");
        }

    private:
        // 处理请求数据
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const Progress::Goal> goal_request){
            (void) uuid;
            RCLCPP_INFO(this->get_logger(),"接受到动作客户端请求,请求数字为%ld", goal_request->num);
            if(goal_request->num < 1){
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        // 处理取消任务请求
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleProgress> goal_handle){
            (void) goal_handle;
            RCLCPP_INFO(this->get_logger(),"接受到任务取消请求");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void execute(const std::shared_ptr<GoalHandleProgress> goal_handle){
            RCLCPP_INFO(this->get_logger(),"开始执行任务");
            rclcpp::Rate loop_rate(10.0);

            // 连续反馈
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Progress::Feedback>();
            auto result = std::make_shared<Progress::Result>();

            int64_t sum = 0;
            for (int i=0; (i<=goal->num) && rclcpp::ok(); i++){
                sum += i;
                feedback->progress = (double_t)i / goal->num;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(),"连续反馈中,进度:%.2f",feedback->progress);

                // 中断
                if(goal_handle->is_canceling()){
                    result->sum = sum;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(),"任务取消");
                    return;
                }

                loop_rate.sleep();
            }

            // 生成最终结果
            if(rclcpp::ok()){
                result->sum = sum;
                goal_handle -> succeed(result);
                RCLCPP_INFO(this->get_logger(),"任务完成");
            }

        }
        // 生成连续反馈
        void handle_accepted(const std::shared_ptr<GoalHandleProgress> goal_handle){
            std::thread{std::bind(&MinimalActionServer::execute,this,_1),goal_handle}.detach();
        }
        rclcpp_action::Server<Progress>::SharedPtr action_server_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<MinimalActionServer>("minimal_action_server");
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}