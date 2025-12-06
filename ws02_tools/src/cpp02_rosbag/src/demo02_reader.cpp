#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rosbag2_cpp/reader.hpp>

class SimpleBagPlayer: public rclcpp::Node{
public:
    SimpleBagPlayer(const std::string &node_name):Node(node_name){
        // 创建读取对象指针
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        // 设置读取的目标文件
        reader_->open("my_bag");
        // 读取消息
        while ((reader_->has_next()))
        {
        geometry_msgs::msg::Twist twist = reader_->read_next<geometry_msgs::msg::Twist>();
        RCLCPP_INFO(this->get_logger(),"%.2f----$.2f",twist.linear.x,twist.angular.z);
        }
        // 关闭文件
        reader_->close(); 
    }
private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleBagPlayer>("simple_bag_player");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}