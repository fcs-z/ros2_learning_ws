#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/static_transform_broadcaster.h>

/*
ros2 run tf2_ros static_transform_publisher 
    --x -0.5 --y 0 --z 0.4 
    --yaw 0 --roll 0 --pitch 0 
    --frame-id base_link --child-frame-id camera
*/

/*
ros2 run cpp03_tf_broadcaster demo01_static_tf_broadcaster 0.4 0 0.2 0 0 0 base_link laser
ros2 run cpp03_tf_broadcaster demo01_static_tf_broadcaster -0.5 0 0.4 0 0 0 base_link camera
rviz2
*/

class MinimalStaticFrameBroadcaster: public rclcpp::Node{
public:
    MinimalStaticFrameBroadcaster(
        const std::string &node_name,
        char * transformation[]):Node(node_name){
       // 创建静态坐标变换发布方
       tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
       this->make_transforms(transformation);
    }
private:
    // 组织并发布消息
    void make_transforms(char* transformation[]){
        // 组织消息
        geometry_msgs::msg::TransformStamped t;

        rclcpp::Time now = this->get_clock()->now();
        t.header.stamp = now;
        t.header.frame_id = transformation[7];
        t.child_frame_id = transformation[8];
        t.transform.translation.x=atof(transformation[1]);
        t.transform.translation.y=atof(transformation[2]);
        t.transform.translation.z=atof(transformation[3]);

        tf2::Quaternion q;
        q.setRPY(
            atof(transformation[4]),
            atof(transformation[5]),
            atof(transformation[6])
        );
        t.transform.rotation.x=q.x();
        t.transform.rotation.y=q.y();
        t.transform.rotation.z=q.z();
        t.transform.rotation.w=q.w();

        // 发布消息
        tf_publisher_ -> sendTransform(t);
    }
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};
 
int main(int argc, char* argv[]){
    // 判断终端传入的参数是否合法
    if(argc!=9){
        RCLCPP_INFO(rclcpp::get_logger("logger"),
            "运行程序时请按照: x y z roll pitch yaw frame_id child_id 的格式传入参数");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalStaticFrameBroadcaster>("minimal_static_frame_broadcaster",argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}