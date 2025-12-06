// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world pkg01_helloworld_cpp package\n");
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>

class MyNode: public rclcpp::Node{
  public: 
    MyNode(const std::string &node_name):Node(node_name){
      RCLCPP_INFO(this->get_logger(),"hello world");
    }

};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>("cpp_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
















