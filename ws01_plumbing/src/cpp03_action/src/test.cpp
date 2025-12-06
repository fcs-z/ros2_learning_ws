#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_interfaces_demo/action/progress.hpp>

using base_interfaces_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ServerGoalHandle<Progress>;
using namespace std::placeholders;

class MinimalActionServer: public rclcpp::Node{
    public:
        MinimalActionServer(const std::string &node_name):Node(node_name){
            RCLCPP_INFO(this->get_logger(),"动作服务端创建,等待请求");

        }

    private:
        
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<MinimalActionServer>("minimal_action_server");
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}




#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_interfaces_demo/action/progress.hpp>

using base_interfaces_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ClientGoalHandle<Progress>;
using namespace std::placeholders;
using namespace std::chrono_literals;
 
class MinimalActionClient: public rclcpp::Node{
    public:
        MinimalActionClient(const std::string &node_name):Node(node_name){ 
            
        }

        
        
    private:
        
};
 
int main(int argc, char* argv[]){
    if(argc != 2){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整形数据");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MinimalActionClient>("minimal_action_client");
    
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}