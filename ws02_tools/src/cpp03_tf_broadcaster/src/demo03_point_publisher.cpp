#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

/*
ros2 run cpp03_tf_broadcaster demo01_static_tf_broadcaster 0.4 0 0.2 0 0 0 base_link laser
ros2 run cpp03_tf_broadcaster demo03_point_publisher
rviz2
*/

class MinimalPointPublisher: public rclcpp::Node{
public:
    MinimalPointPublisher(const std::string &node_name):Node(node_name),x(0.1){
       // 创建坐标点发布方
       point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point",10);
       // 创建定时器
       timer_ = this->create_wall_timer(0.1s,std::bind(&MinimalPointPublisher::on_timer,this));
    }
private:
    void on_timer(){
        // 组织并发布坐标点信息
        geometry_msgs::msg::PointStamped point;

        point.header.frame_id = "laser";
        point.header.stamp = this->now();
        x += 0.004;
        point.point.x=x;
        point.point.y=0.0;
        point.point.z=0.1;

        point_pub_->publish(point);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double_t x;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPointPublisher>("minimal_point_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}