#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using namespace std::chrono_literals;

class Minimalpublisher:public rclcpp::Node{
    public:
        Minimalpublisher(const std::string &node_name):Node(node_name), count_(0){
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
            timer_ = this->create_wall_timer(500ms, std::bind(&Minimalpublisher::timer_callback,this));
        }
    private:
        void timer_callback(){
            auto msg = std_msgs::msg::String();
            msg.data = "hello c++ :" + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(),"发布消息：'%s'", msg.data.c_str());
            publisher_->publish(msg);
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Minimalpublisher>("Minimalpublisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// using namespace std::chrono_literals;

// class Minimalpublisher:public rclcpp::Node{
//     public:
//         Minimalpublisher(const std::string &node_name):Node(node_name){
//             publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
//             timer_ = this->create_wall_timer(500ms, std::bind(&Minimalpublisher::timer_callback,this));
//         }
//     private:
//         void timer_callback(){
//             auto message = std_msgs::msg::String();
//             message.data = "hello c++";
//             RCLCPP_INFO(this->get_logger(),"发送'%s'",message.data.c_str());
//             publisher_ -> publish(message);
//         }
//         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//         rclcpp::TimerBase::SharedPtr timer_;
       
// };

// int main(int argc, char* argv[]){
//     rclcpp::init(argc,argv);
//     auto node = std::make_shared<Minimalpublisher>("minimal_publisher");
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }





























