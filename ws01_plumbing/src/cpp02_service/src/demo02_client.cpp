// #include <rclcpp/rclcpp.hpp>
// #include <base_interfaces_demo/srv/add_ints.hpp>
// using namespace std::chrono_literals;
// using base_interfaces_demo::srv::AddInts;

// class MinimalClient: public rclcpp::Node{
//     public:
//         MinimalClient(const std::string &node_name):Node(node_name){
//             client_ = this->create_client<AddInts>("add_ints");
//             RCLCPP_INFO(this->get_logger(),"客户端创建,等待连接服务端!");
//         }
//         bool connect_server(){
//             while(!client_->wait_for_service(1s)){
//                 if(!rclcpp::ok()){
//                     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强制退出");
//                     // RCLCPP_INFO(this->get_logger(),"强制退出");
//                     return false;
//                 }
//                 RCLCPP_INFO(this->get_logger(),"服务连接中,请稍候...");
//             }
//             return true;
//         }
//         rclcpp::Client<AddInts>::FutureAndRequestId send_request(int32_t num1, int32_t num2){
//             auto request = std::make_shared<AddInts::Request>();
//             request->num1 = num1;
//             request->num2 = num2;
//             // return client_->async_send_request(request);
//             auto future = client_->async_send_request(request);
//             return future;
//         }

//     private:
//         rclcpp::Client<AddInts>::SharedPtr client_;
// };
 
// int main(int argc, char* argv[]){
//     if(argc != 3){
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交两个整型数据!");
//         return 1;
//     }

//     rclcpp::init(argc, argv);
//     auto client = std::make_shared<MinimalClient>("minimal_client");

//     bool flag = client->connect_server();
//     if(!flag){
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接失败!");
//         // RCLCPP_INFO(client->get_logger(),"服务连接失败");
//         return 0;
//     }

//     int32_t num1 = atoi(argv[1]);
//     int32_t num2 = atoi(argv[2]);
//     auto future = client->send_request(num1,num2);
//     if(rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS){
//         RCLCPP_INFO(client->get_logger(),"请求正常处理");
//         RCLCPP_INFO(client->get_logger(),"相应结果:%d + %d = %d",num1,num2,future.get()->sum);
//     }else{
//         RCLCPP_INFO(client->get_logger(),"请求异常");
//     }

//     // rclcpp::spin(client);
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <base_interfaces_demo/srv/add_ints.hpp>
using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

class C: public rclcpp::Node{
public:
    C(const std::string &node_name):Node(node_name){
        client_ = this->create_client<AddInts>("add");
    }

    bool connect(){
        while (!client_->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务中止");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"服务连接中");
        }
        return true;
    }

    rclcpp::Client<AddInts>::FutureAndRequestId send(int num1, int num2){
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        auto future = client_->async_send_request(request);
        return future;
    }
    

private:
    rclcpp::Client<AddInts>::SharedPtr client_;

};

int main(int argc, char const *argv[])
{   
    if(argc != 3){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"输入两个整数");
        return 1;
    }
    rclcpp::init(argc,argv);
    auto client = std::make_shared<C>("asdf");

    bool flag = client->connect();
    if(!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"连接错误");
        return 0;
    }

    int num1 = atoi(argv[1]);
    int num2 = atoi(argv[2]);
    auto future = client->send(num1,num2);
    if(rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger()," %d + %d = %d ",num1,num2,future.get()->sum);
    }
    
    rclcpp::shutdown();
    return 0;
}

