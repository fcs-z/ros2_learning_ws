#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/QuadWord.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

/*
ros2 run cpp03_tf_broadcaster demo01_static_tf_broadcaster 0.4 0 0.2 0 0 0 base_link laser
ros2 run cpp03_tf_broadcaster demo01_static_tf_broadcaster -0.5 0 0.4 0 0 0 base_link camera
ros2 run cpp04_tf_listener demo01_tf_listener
*/

class MinimalFrameListener: public rclcpp::Node{
public:
    MinimalFrameListener(const std::string &node_name):Node(node_name){
       tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
       transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
       timer_ = this->create_wall_timer(1s,std::bind(&MinimalFrameListener::on_timer,this));
    }
private:
    void on_timer(){
        try{
            auto transformStamped = tf_buffer_->lookupTransform("camera","laser",tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(),"-----转换结果-----");
            RCLCPP_INFO(this->get_logger(),"frame_id: %s",transformStamped.header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(),"child_frame_id: %s",transformStamped.child_frame_id.c_str());
            RCLCPP_INFO(this->get_logger(),"坐标:(%.2f,%.2f.%.2f)",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z
             );
        }catch(const tf2::LookupException &e){
            RCLCPP_INFO(this->get_logger(),"坐标变换异常: %s",e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalFrameListener>("minimal_frame_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}