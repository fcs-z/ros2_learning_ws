#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rosbag2_cpp/writer.hpp>
 
class SimpleBagRecorder: public rclcpp::Node{
public:
    SimpleBagRecorder(const std::string &node_name):Node(node_name){
        // 创建写出对象指针
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        // 设置写出的目标文件
        writer_ -> open("my_bag");
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel",10,std::bind(&SimpleBagRecorder::topic_callback,this,std::placeholders::_1));
    }
private:
    void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)const{
        rclcpp::Time time_stamp = this->now();
        // 写出消息
        writer_->write(msg,"/turt;e1/cmd_vel","geometry_msgs/msg/Twist",time_stamp);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleBagRecorder>("simple_bag_recorder");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}