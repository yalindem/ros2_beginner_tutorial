#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>


using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop");
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    while(!client->wait_for_service(1s))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server ...");
    }

    auto request  = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 6;
    request->b = 2;
    auto future = client->async_send_request(request);

    rclcpp::spin_until_future_complete(node, future);

    auto response = std::make_shared<example_interfaces::srv::AddTwoInts::Response>();
    RCLCPP_INFO(node->get_logger(), "Response: %d", (int)response->sum);


    rclcpp::shutdown();

    return 0;
}
