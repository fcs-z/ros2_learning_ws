#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*
ros2 run cpp03_tf_broadcaster demo01_static_tf_broadcaster 0.4 0 0.2 0 0 0 base_link laser
ros2 run cpp03_tf_broadcaster demo03_point_publisher
ros2 run cpp04_tf_listener demo02_message_filter
*/
 
class MessageFilterPointListener: public rclcpp::Node{
public:
    MessageFilterPointListener(const std::string &node_name):Node(node_name){
       target_frame_ = "base_link";

       typedef std::chrono::duration<int> seconds_type;
       seconds_type buffer_timeout(1);

       // 创建tf缓存对象指针
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        auto time_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()
        );
        tf2_buffer_->setCreateTimerInterface(time_interface);
       // 创建tf监听器
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

       // 创建坐标点订阅方,并订阅指定话题
        point_sub_.subscribe(this,"point");
       // 创建消息过滤器,过滤被处理的数据
        tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
            point_sub_,*tf2_buffer_,target_frame_,100,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            buffer_timeout
        );
       // 为消息过滤器注册转换坐标点数据的回调函数
       tf2_filter_ -> registerCallback(&MessageFilterPointListener::msgCallbacl,this);
    }
private:
    void msgCallbacl(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr){
        geometry_msgs::msg::PointStamped point_out_;
        try{
            tf2_buffer_->transform(*point_ptr,point_out_,target_frame_);
            RCLCPP_INFO(this->get_logger(),"坐标点相对于baselink的坐标:(%.2f,%.2f,%.2f)",
                point_out_.point.x,
                point_out_.point.y,
                point_out_.point.z
            );
        }catch(tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(),"Failure%s\n",ex.what());
        }
    }

    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessageFilterPointListener>("message_filter_point_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}