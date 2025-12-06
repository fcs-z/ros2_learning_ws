// #include <rclcpp/rclcpp.hpp>
// #include <base_interfaces_demo/srv/add_ints.hpp>
// // using base_interfaces_demo::srv::AddInts;

// class MinimalService: public rclcpp::Node{
//     public:
//         MinimalService(const std::string &node_name):Node(node_name){
//             server_ = this->create_service<base_interfaces_demo::srv::AddInts>("add_ints",
//                 std::bind(&MinimalService::add,this,std::placeholders::_1,std::placeholders::_2));
//                 RCLCPP_INFO(this->get_logger(),"add_ints服务端启动完毕,等待请求提交...");
//         }

//     private:
//         void add(const base_interfaces_demo::srv::AddInts::Request::SharedPtr request, 
//             const base_interfaces_demo::srv::AddInts::Response::SharedPtr response){
//                 response->sum = request->num1 + request->num2;
//                 RCLCPP_INFO(this->get_logger(),"请求数据:(%d,%d),相应结果:%d",request->num1,request->num2,response->sum);
//         }
//         rclcpp::Service<base_interfaces_demo::srv::AddInts>::SharedPtr server_;
// };
 
// int main(int argc, char* argv[]){
//     rclcpp::init(argc, argv);
//     auto server = std::make_shared<MinimalService>("minimal_service");
//     rclcpp::spin(server);
//     rclcpp::shutdown();
//     return 0;
// }

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
        response->sum = request-> num1 + request->num2; 
        RCLCPP_INFO(this->get_logger(),"%d + %d = %d", request->num1, request->num2, response->sum);
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