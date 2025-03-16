#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServerNode : public rclcpp::Node
{
    public:
        AddTwoIntsServerNode() : Node("add_two_ints_server")
        {
            server_ = this->create_service<AddTwoInts>("add_two_ints", std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Add Two Ints Service has been started ...");
        }

    private:
        
        void callbackAddTwoInts(const AddTwoInts::Request::SharedPtr request, const AddTwoInts::Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
        } 

        rclcpp::Service<AddTwoInts>::SharedPtr server_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
