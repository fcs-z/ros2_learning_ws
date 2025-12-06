#include <rclcpp/rclcpp.hpp>

class MinimalParamServer: public rclcpp::Node{
    public:
        MinimalParamServer(const std::string &node_name):Node(node_name,
            rclcpp::NodeOptions().allow_undeclared_parameters(true)){

        }
        // 声明参数
        void declare_param(){
            this->declare_parameter("car_type","Tiger");
            this->declare_parameter("height",1.50);
            this->declare_parameter("wheels",4);

            //rclcpp::NodeOptions().allow_undeclared_parameters(true),
            this->set_parameter(rclcpp::Parameter("undcl_test",100));
        }
        // 查询参数
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"--------查--------");
            // 获取指定
            rclcpp::Parameter car_type = this->get_parameter("car_type");
            RCLCPP_INFO(this->get_logger(),"car_type: %s", car_type.as_string().c_str());
            RCLCPP_INFO(this->get_logger(),"height: %.2f", this->get_parameter("height").as_double());
            RCLCPP_INFO(this->get_logger(),"wheels: %ld", this->get_parameter("wheels").as_int());
            RCLCPP_INFO(this->get_logger(),"undcl_test: %ld", this->get_parameter("undcl_test").as_int());
            // 判断包含
            RCLCPP_INFO(this->get_logger(),"包含car_type? %d", this->has_parameter("car_type"));
            RCLCPP_INFO(this->get_logger(),"包含car_typesxxxx? %d", this->has_parameter("car_typesxxxx"));
            // 获取所有
            auto params = this->get_parameters({"car_type","height","wheels"});
            for(auto &param:params){
                RCLCPP_INFO(this->get_logger(),"name=%s, value=%s", param.get_name().c_str(),param.value_to_string().c_str());
            }
        }
        // 修改参数
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"--------改--------");
            this->set_parameter(rclcpp::Parameter("height",1.75));
            RCLCPP_INFO(this->get_logger(),"height:%.2f", this->get_parameter("height").as_double());
        }
        // 删除参数
        void del_param(){
            RCLCPP_INFO(this->get_logger(),"--------删--------");
            // this->undeclare_parameter("car_type");
            // RCLCPP_INFO(this->get_logger(),"删除操作后, car_type还存在吗?%d", this->has_parameter("car_type"));
            RCLCPP_INFO(this->get_logger(),"删除操作前, undcl_test存在吗? %d", this->has_parameter("undcl_test"));
            this->undeclare_parameter("undcl_test");
            RCLCPP_INFO(this->get_logger(),"删除操作前, undcl_test存在吗? %d", this->has_parameter("undcl_test"));
        }

    private:

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);

    auto paramServer = std::make_shared<MinimalParamServer>("minimal_param_server");

    paramServer->declare_param();
    paramServer->get_param();
    paramServer->update_param();
    paramServer->del_param();

    rclcpp::spin(paramServer);
    rclcpp::shutdown();
    return 0;
}
