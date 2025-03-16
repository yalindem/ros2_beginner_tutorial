#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

using namespace std::chrono_literals;

class AddTwwoIntsClient : public rclcpp::Node
{
    public:
        AddTwwoIntsClient() : Node("add_two_ints_client_node")
        {
            this->client_ = this->create_client<AddTwoInts>("add_two_ints");
        }

        void callAddTwoInts(int a, int b)
        {
            while(!client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server ...");
            }
            auto request = std::make_shared<AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            client_->async_send_request(request, std::bind(&AddTwwoIntsClient::callbackCallAddTwoInts, this, std::placeholders::_1));

        }

    private:
        rclcpp::Client<AddTwoInts>::SharedPtr client_;

        void callbackCallAddTwoInts(rclcpp::Client<AddTwoInts>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
        }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwwoIntsClient>();
    node->callAddTwoInts(1,2);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
