#include <rclcpp/rclcpp.hpp>


class MyNode : public rclcpp::Node
{
    public:
        MyNode():Node("my_first_node")
        {
            RCLCPP_INFO(this->get_logger(), "Hello world");
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));
        }

    private:
        //rclcpp::TimerBase::SharedPtr
        std::shared_ptr<rclcpp::TimerBase> timer_;
        
        int counter_{0};

        void timerCallback()
        {
            RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
            counter_++;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}