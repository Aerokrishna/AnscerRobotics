#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/nav_goal.hpp"

using NavGoal = my_robot_interfaces::action::NavGoal;
using NavGoalGoalHandle = rclcpp_action::ClientGoalHandle<NavGoal>;
using namespace std::placeholders;

// Client node sends action request in the form of goal pose and the target map name
class NavGoalClientNode : public rclcpp::Node
{
public:
    NavGoalClientNode() : Node("nav_goal_client")
    {
        nav_goal_client_ = 
            rclcpp_action::create_client<NavGoal>(this, "nav_goal");
    }

    // function to send the goal, takes pose and map name
    void send_goal(float robot_x, float robot_y, float robot_theta, const std::string &map_name)
    {
        nav_goal_client_->wait_for_action_server();

        auto goal = NavGoal::Goal();
        goal.map_name = map_name;
        goal.robot_x = robot_x;
        goal.robot_y = robot_y;
        goal.robot_theta = robot_theta;

        // add callbacks
        auto options = rclcpp_action::Client<NavGoal>::SendGoalOptions();
        options.result_callback = 
            std::bind(&NavGoalClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = 
            std::bind(&NavGoalClientNode::goal_response_callback, this, _1);
        options.feedback_callback = 
            std::bind(&NavGoalClientNode::goal_feedback_callback, this, _1, _2);

        // send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        nav_goal_client_->async_send_goal(goal, options);
    }

private:

    // callback to know if the goal was accepted or rejected
    void goal_response_callback(const NavGoalGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    // callback to receive the result once the goal is done
    void goal_result_callback(const NavGoalGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
      
    }

    // callback to receive feedback during goal execution
    void goal_feedback_callback(const NavGoalGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const NavGoal::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Got feedback");
    }

    rclcpp_action::Client<NavGoal>::SharedPtr nav_goal_client_;
    NavGoalGoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavGoalClientNode>();

    // call the send goal function
    node->send_goal(0.0, 0.0, 1.57, "mapD");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}