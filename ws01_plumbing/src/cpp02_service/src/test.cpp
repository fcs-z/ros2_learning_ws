#include <rclcpp/rclcpp.hpp>
#include <base_interfaces_demo/srv/add_ints.hpp>
using base_interfaces_demo::srv::AddInts;

class S: public rclcpp::Node{
public:
    S(const std::string &node_name):Node(node_name){
        server_ = this->create_service<AddInts>("add",
            std::bind(&S::add_callback,this,std::placeholders::_1,std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(),"服务启动");
    }

private:
    void add_callback(AddInts::Request::SharedPtr request,AddInts::Response::SharedPtr response){
        response->sum = request->num1 + request->num2;
    }
    rclcpp::Service<AddInts>::SharedPtr server_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto server = std::make_shared<S>("asd");
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}


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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中");
            if(!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务中止");

            }
        }
        return true;
    }

    rclcpp::Client<AddInts>::FutureAndRequestId send(int num1,int num2){
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
    }
    rclcpp::init(argc,argv);
    auto client = std::make_shared<C>("asdf");

    bool flag = client->connect();
    if(!flag){
        RCLCPP_INFO(client->get_logger(),"连接错误");
    }

    int num1 = atoi(argv[1]);
    int num2 = atoi(argv[2]);
    auto future = client->send(num1,num2);
    if(rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(),"%d + %d =%d", num1,num2,future.get()->sum);
    } 


    rclcpp::shutdown();
    return 0;
}

