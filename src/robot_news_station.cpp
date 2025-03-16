#include <rclcpp/rclcpp.hpp>
#include "example_interfaces/msg/string.hpp"


using namespace std::chrono_literals;

class RobotNewsStationNode: public rclcpp::Node
{
    public:
        RobotNewsStationNode():Node("robot_news_station_node"), robot_name_("R202")
        {
            str_publisher_ = this->create_publisher<example_interfaces::msg::String>("example_interfaces_str", 10);
            timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this));
        }
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr str_publisher_;
        std::string robot_name_;
        
        void publishNews()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the robot news station.");
            str_publisher_->publish(msg);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}