#include <rclcpp/rclcpp.hpp>
#include <base_interfaces_demo/msg/student.hpp>
using base_interfaces_demo::msg::Student;

class MinimalSubscriber: public rclcpp::Node{
    public:
        MinimalSubscriber(const std::string &node_name):Node(node_name){
            subscripter_ = this->create_subscription<Student>("topic_stu",10,
                std::bind(&MinimalSubscriber::topic_callback,this,std::placeholders::_1));        
        }
        
    private:
        void topic_callback(const Student &stu){

            RCLCPP_INFO(this->get_logger(),"订阅的学生信息: name=%s, age=%d, height=%.2f", stu.name.c_str(), stu.age, stu.height);
        }
        rclcpp::Subscription<Student>::SharedPtr subscripter_;
       
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>("student_subscriber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
