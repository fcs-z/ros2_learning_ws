#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_interfaces_demo/action/nav.hpp>
#include <turtlesim/srv/spawn.hpp>

using base_interfaces_demo::action::Nav;
using namespace std::placeholders;
using namespace std::chrono_literals;

class ExeNavActionClient: public rclcpp::Node{
public:
    ExeNavActionClient(const std::string &node_name):Node(node_name){
        nav_action_client = rclcpp_action::create_client<Nav>(this,"nav");
    }

    // 发送请求数据，并处理服务端响应
    void send_goal(float x, float y, float theta){
        // 连接动作服务端，如果超时（5s），直接退出
        if(!nav_action_client->wait_for_action_server(5s)){
            RCLCPP_INFO(this->get_logger(),"服务连接失败!");
            return;
        }
        // 组织请求数据
        auto goal_msg = Nav::Goal(); // 为什么有Goal()
        goal_msg.goal_x = x;
        goal_msg.goal_y = y;
        goal_msg.goal_theta = theta;
        rclcpp_action::Client<base_interfaces_demo::action::Nav>:: SendGoalOptions options;
        options.goal_response_callback = std::bind(&ExeNavActionClient::goal_response_callback,this,_1);
        options.feedback_callback = std::bind(&ExeNavActionClient::feedback_callback,this,_1,_2);
        options.result_callback = std::bind(&ExeNavActionClient::result_callback,this,_1);
        // 发送
        nav_action_client->async_send_goal(goal_msg,options);
    }
private:
    // 处理目标响应
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle){
        if(!goal_handle){
            RCLCPP_INFO(this->get_logger(),"目标请求被服务器拒绝");
        }else{
            RCLCPP_INFO(this->get_logger(),"目标请求被接收!");
        }
    }

    // 处理响应的连续反馈
    void feedback_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle,const std::shared_ptr<const Nav::Feedback> feedback){
        (void) goal_handle;
        RCLCPP_INFO(this->get_logger(),"距离目标点还有%.2f米。", feedback->distance);
    }

    // 处理最终响应
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult &result){
        switch(result.code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(),"乌龟最终目标: (%.2f,%.2f),航向: %.2f", result.result->turtle_x,result.result->turtle_y,result.result->turtle_theta);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(),"任务被取消");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(),"任务被中止");
                break;
            default:
                RCLCPP_INFO(this->get_logger(),"未知异常");
                break;
        }
    }

    rclcpp_action::Client<Nav>::SharedPtr nav_action_client;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ExeNavActionClient>("exe_nav_action_client");

    if(argc != 5){
        RCLCPP_INFO(client->get_logger(),"请传入目标的位姿参数: {x,y,theta}");
        return 1;
    }

    // 发送目标点
    client->send_goal(atof(argv[1]),atof(argv[2]),atof(argv[3]));

    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}