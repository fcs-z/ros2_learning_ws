#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_interfaces_demo/action/progress.hpp>

using base_interfaces_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ClientGoalHandle<Progress>;
using namespace std::placeholders;
using namespace std::chrono_literals;
 
class MinimalActionClient: public rclcpp::Node{
    public:
        // MinimalActionClient(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):Node(node_name,options){
        MinimalActionClient(const std::string &node_name):Node(node_name){ 
            // this->action_client = rclcpp_action::create_client<Progress>(this,"get_sum");
            action_client = rclcpp_action::create_client<Progress>(this,"get_sum");
        }

        // 发送请求
        void send_goal(int64_t num){
            // if(!this->action_client){
            if(!action_client){
                RCLCPP_INFO(this->get_logger(),"动作客户端未被初始化");
            }
            if(!this->action_client->wait_for_action_server(10s)){
                RCLCPP_ERROR(this->get_logger(),"服务连接失败");
                return;
            }

            auto goal_msg = Progress::Goal();
            goal_msg.num = num;
            RCLCPP_INFO(this->get_logger(),"发送请求数据");

            auto send_goal_option = rclcpp_action::Client<Progress>::SendGoalOptions();
            send_goal_option.goal_response_callback = std::bind(&MinimalActionClient::goal_response_callback,this,_1);
            send_goal_option.feedback_callback = std::bind(&MinimalActionClient::feedback_callback,this,_1,_2);
            send_goal_option.result_callback = std::bind(&MinimalActionClient::result_callback,this,_1);

            auto goal_handle_future = this->action_client->async_send_goal(goal_msg, send_goal_option);
        }
    private:
        // 处理目标发送后的反馈
        void goal_response_callback(GoalHandleProgress::SharedPtr goal_handle){
            if(!goal_handle){
                RCLCPP_ERROR(this->get_logger(),"目标请求被服务器拒绝!");
            }else{
                RCLCPP_INFO(this->get_logger(),"目标被接受,等待结果中");
            }
        }
        // 处理连续反馈
        void feedback_callback(GoalHandleProgress::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback){
            (void) goal_handle;
            int32_t progress = (int32_t)(feedback->progress * 100);
            RCLCPP_INFO(this->get_logger(),"当前进度:%d%%",progress);
        }
        // 处理最终响应
        void result_callback(const GoalHandleProgress::WrappedResult &result){
            // switch(result.code){
            //     case rclcpp_action::ResultCode::SUCCEEDED:
            //         break;
            //     case rclcpp_action::ResultCode::ABORTED:
            //         RCLCPP_ERROR(this->get_logger(),"任务被中止");
            //         return;
            //     case rclcpp_action::ResultCode::CANCELED:
            //         RCLCPP_ERROR(this->get_logger(),"任务被取消");
            //         return;
            //     default:
            //     RCLCPP_ERROR(this->get_logger(),"未知异常");
            //         return;
            // }
            // RCLCPP_INFO(this->get_logger(),"任务执行完毕,最终结果:%ld",result.result->sum);
                   
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(),"任务执行完毕,最终结果:%ld",result.result->sum);
            }else if(result.code == rclcpp_action::ResultCode::ABORTED){
                RCLCPP_ERROR(this->get_logger(),"任务被中止");
            }else if(result.code == rclcpp_action::ResultCode::CANCELED){
                RCLCPP_ERROR(this->get_logger(),"任务被取消");
            }else{
                RCLCPP_ERROR(this->get_logger(),"未知异常");
            }                                                                                        
        }
        rclcpp_action::Client<Progress>::SharedPtr action_client;
};
 
int main(int argc, char* argv[]){
    if(argc != 2){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整数");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MinimalActionClient>("minimal_action_client");
    action_client->send_goal(atoi(argv[1]));
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}