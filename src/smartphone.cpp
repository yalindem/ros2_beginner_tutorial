#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>


class SmartPhoneNode : public rclcpp::Node
{
    public:
        SmartPhoneNode() : Node("smartphone_node")
        {
            subscriber_ = this->create_subscription<example_interfaces::msg::String>(
                "example_interfaces_str", 
                10, 
                std::bind(&SmartPhoneNode::callbackRobotNews, this, std::placeholders::_1)
            );
        }
    
    private:
        
        void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Message: %s ", msg->data.c_str());
        }

        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartPhoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}