#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_beginner_tutorial/action/count_until.hpp"

using CountUntil = ros2_beginner_tutorial::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;

class CountUntilServerNode : public rclcpp::Node 
{
    private:
        
        rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID& uuid, 
            std::shared_ptr<const CountUntil::Goal> goal)
        {
            (void) uuid;
            (void) goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            execute_goal(goal_handle);
        }

        void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            int target_number = goal_handle->get_goal()->target_number;
            double period = goal_handle->get_goal()->period;
            
            int counter = 0;
            rclcpp::Rate loop_rate(1.0/period);

            for(int i = 0; i< target_number; ++i)
            {
                RCLCPP_INFO(this->get_logger(), "%d", counter);

                counter++;
                loop_rate.sleep();
            }
            
            auto result = std::make_shared<CountUntil::Result>();
            result->reached_number = counter;
            goal_handle->succeed(result);
        }

    public:
        CountUntilServerNode():Node("count_until_server_node")
        {
            count_until_server_ = rclcpp_action::create_server<CountUntil>(
                this,
                "count_until",
                std::bind(&CountUntilServerNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&CountUntilServerNode::cancel_callback, this, std::placeholders::_1),
                std::bind(&CountUntilServerNode::handle_accepted_goal, this, std::placeholders::_1)
            );
        }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




