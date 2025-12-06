#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node{
    public:
        MinimalSubscriber(const std::string &node_name):Node(node_name){
            subscription_ = this->create_subscription<std_msgs::msg::String>("topic",10,
                std::bind(&MinimalSubscriber::topic_callback,this,std::placeholders::_1)); 
        }
    
    private:
        void topic_callback(const std_msgs::msg::String &msg){
            RCLCPP_INFO(this->get_logger(),"订阅消息：'%s'",msg.data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>("miniml_subscriber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// // using std::placeholders::_1;

// class MinimalSubscriber : public rclcpp::Node{
//     public:
//         MinimalSubscriber(const std::string &node_name):Node(node_name){
//             subscription_ = this->create_subscription<std_msgs::msg::String>("topic",10,
//                 std::bind(&MinimalSubscriber::topic_callback,this,std::placeholders::_1));
//         }
    
//     private:
//         void topic_callback(const std_msgs::msg::String msg){
//             RCLCPP_INFO(this->get_logger(),"收到：'%s'",msg.data.c_str());
//         }
//         rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

// int main(int argc, char* argv[]){
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MinimalSubscriber>("miniml_sublisher");
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }