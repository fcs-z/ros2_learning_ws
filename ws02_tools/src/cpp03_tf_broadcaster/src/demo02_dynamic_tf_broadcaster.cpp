#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_broadcaster.h>

/*
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
rviz2
ros2 run cpp03_tf_broadcaster demo02_dynamic_tf_broadcaster
*/

class MinimalDynamicFrameBroadcaster: public rclcpp::Node{
public:
    MinimalDynamicFrameBroadcaster(const std::string &node_name):Node(node_name){
        // 创建动态坐标变换发布方
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建乌龟位姿订阅方
       subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",10,
            std::bind(&MinimalDynamicFrameBroadcaster::handle_turtle_pose,this,std::placeholders::_1)
       );
    }
private:
    // 根据订阅到的乌龟位姿生成坐标帧,并广播
    void handle_turtle_pose(const turtlesim::msg::Pose &msg){
        // 组织消息
        geometry_msgs::msg::TransformStamped t;

        rclcpp::Time now = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "turtle1";
        t.transform.translation.x=msg.x;
        t.transform.translation.y=msg.y;
        t.transform.translation.z=0.0;

        tf2::Quaternion q;
        q.setRPY(0,0,msg.theta);
        t.transform.rotation.x=q.x();
        t.transform.rotation.y=q.y();
        t.transform.rotation.z=q.z();
        t.transform.rotation.w=q.w();

        // 发布消息
        tf_broadcaster_ -> sendTransform(t);
    }
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalDynamicFrameBroadcaster>("minimal_dynamic_frame_broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}