#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ExePubSub: public rclcpp::Node{
public:
    ExePubSub(const std::string &node_name):Node(node_name){
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel",1);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",1,
            std::bind(&ExePubSub::poseCallback,this,std::placeholders::_1));
    }
private:
    void poseCallback(const turtlesim::msg::Pose::ConstSharedPtr pose){
        geometry_msgs::msg::Twist twist;
        twist.angular.z=-(pose->angular_velocity);
        twist.linear.x=pose->linear_velocity;
        twist_pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
        
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<ExePubSub>("demo01_pub_sub");
    // rclcpp::spin(node);
    rclcpp::spin(std::make_shared<ExePubSub>("demo01_pub_sub"));
    rclcpp::shutdown();
    return 0;
}