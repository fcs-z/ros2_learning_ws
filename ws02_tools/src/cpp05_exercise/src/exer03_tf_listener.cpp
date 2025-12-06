#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;
 
class TurtleFrameListener: public rclcpp::Node{
public:
    TurtleFrameListener(const std::string &node_name):Node(node_name){
       // 声明并解析参数
       this->declare_parameter("target_frame","turtle2");
       this->declare_parameter("source_frame","turtle1");
       target_frame = this->get_parameter("target_frame").as_string();
       source_frame = this->get_parameter("source_frame").as_string();

       // 创建tf缓存对象指针
       tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

       // 创建tf监听器
       transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

       twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(target_frame+"/cmd_vel",10);
       timer_ = this->create_wall_timer(1s,std::bind(&TurtleFrameListener::on_timer,this));
    }
private:
    void on_timer(){
        // 按照条件查找符合条件的坐标系,并生成坐标转换后的坐标帧
        geometry_msgs::msg::TransformStamped t;
        try{
            t = tf_buffer_->lookupTransform(target_frame,source_frame,tf2::TimePointZero);
        }catch(const tf2::LookupException &e){
            RCLCPP_INFO(this->get_logger(),"坐标变换异常:%s",e.what());
            return;
        }
        
        // 生成turtle2的速度指令,并发布
        geometry_msgs::msg::Twist msg;
        static const double scaleRotationRate = 1.0;
        msg.angular.z = scaleRotationRate * atan2(
            t.transform.translation.y,
            t.transform.translation.x
        );

        static const double scaleForwardSpeed = 0.5;
        msg.linear.x = scaleForwardSpeed * sqrt(
            pow(t.transform.translation.x,2)+
            pow(t.transform.translation.y,2)
        );

        twist_pub_->publish(msg);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame;
    std::string source_frame;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleFrameListener>("cpp_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}