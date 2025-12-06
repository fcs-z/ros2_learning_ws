#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
 
class ExeParamClient: public rclcpp::Node{
public:
    ExeParamClient(const std::string &node_name):Node(node_name),red(0){
        param_client = std::make_shared<rclcpp::SyncParametersClient>(this,"/turtlesim");
    }

    // 连接参数服务端
    bool connect_server(){
        while(!param_client->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_INFO(this->get_logger(),"终端退出!");
                return false;
             }
             RCLCPP_INFO(this->get_logger(),"参数服务连接中，请稍候...");
        }
        return true;
    }

    // 更新参数
    void update_param(){
        red = param_client->get_parameter<int32_t>("background_r");
        rclcpp::Rate rate(30.0);
        int i = red;
        while(rclcpp::ok()){
            i < 255 ? red += 5 : red -= 5;
            i+=5;
            if(i>=510)
                i=0;
            RCLCPP_INFO(this->get_logger(),"red = %d",red);
            param_client->set_parameters({rclcpp::Parameter("background_r",red)});
            rate.sleep();
        }
    }
private:
    rclcpp::SyncParametersClient::SharedPtr param_client;
    int32_t red;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto param_client = std::make_shared<ExeParamClient>("exe_param_client");

    if(!param_client->connect_server())
        return 1;
    
    param_client->update_param(); 

    // rclcpp::spin(param_client);  // update_param() 是一个死循环
    rclcpp::shutdown();
    return 0;
}


