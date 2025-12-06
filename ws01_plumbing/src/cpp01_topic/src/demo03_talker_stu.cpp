#include <rclcpp/rclcpp.hpp>
#include <base_interfaces_demo/msg/student.hpp>
using namespace std::chrono_literals;
using base_interfaces_demo::msg::Student;

class MinimalPublisher: public rclcpp::Node{
    public:
        MinimalPublisher(const std::string &node_name):Node(node_name),count_(0){
            publisher_ = this->create_publisher<base_interfaces_demo::msg::Student>("topic_stu",10);
            timer_ = this->create_wall_timer(500ms,std::bind(&MinimalPublisher::timer_callback,this));
        }
        

    private:
        void timer_callback(){
            auto stu = base_interfaces_demo::msg::Student();
            stu.name = "张三";
            stu.age = count_++;
            stu.height = 1.65;
            RCLCPP_INFO(this->get_logger(),"学生信息: name=%s,age=%d,hejght=%.2f", stu.name.c_str(), stu.age, stu.height);
            publisher_->publish(stu);
        }
        rclcpp::Publisher<base_interfaces_demo::msg::Student>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>("student_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
