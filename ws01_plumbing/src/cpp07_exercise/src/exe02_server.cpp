#include <rclcpp/rclcpp.hpp>
#include <base_interfaces_demo/srv/distance.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>

using namespace std::chrono_literals;
 
class ExeDistanceServer: public rclcpp::Node{
public:
    ExeDistanceServer(const std::string &node_name):Node(node_name){
        pose_sub = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
            std::bind(&ExeDistanceServer::poseCallBack,this,std::placeholders::_1));
        distance_server = this->create_service<base_interfaces_demo::srv::Distance>("distance",
            std::bind(&ExeDistanceServer::distanceCallBack,this,std::placeholders::_1,std::placeholders::_2));
    }
private:
    void poseCallBack(const turtlesim::msg::Pose::SharedPtr pose){
        turtle1_x = pose->x;
        turtle1_y = pose->y;
    }
    void distanceCallBack(const base_interfaces_demo::srv::Distance_Request::SharedPtr request,
        base_interfaces_demo::srv::Distance_Response::SharedPtr response){
        float goal_x = request->x;
        float goal_y = request->y;

        float x = goal_x - turtle1_x;
        float y = goal_y - turtle1_y;

        response->distance = std::sqrt(x*x + y*y);
        RCLCPP_INFO(this->get_logger(),"目标坐标：（%.2f,%.2f),距离:%.2f",goal_x,goal_y,response->distance);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
    rclcpp::Service<base_interfaces_demo::srv::Distance>::SharedPtr distance_server;
    float turtle1_x;
    float turtle1_y;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExeDistanceServer>("exe_distance_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}