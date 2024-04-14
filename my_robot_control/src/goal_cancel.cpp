#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class GoalCancelNode : public rclcpp::Node //GoalCancelNode subclass inherited from Node class
{
public:
    GoalCancelNode() : Node("goal_cancel") 
    {
        // Client created for the navigation action server
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Subscribe to the feedback topic to get active goal feedback data
        feedback_subscriber_ = this->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
        "/navigate_to_pose/_action/feedback", 10, std::bind(&GoalCancelNode::feedbackCallback, this, std::placeholders::_1));

        // Initialize the timer to periodically check for active goal
        check_goal_timer_ = create_wall_timer(1s, std::bind(&GoalCancelNode::checkActiveGoal, this));
    }

private:
    void feedbackCallback(const typename nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr feedback) {
        if (feedback->feedback.navigation_time.sec == 0 && feedback->feedback.navigation_time.nanosec == 0) //If the data received is 0, then there is no active goal
        {
            active_goal_ = false;
        } 
        else 
        {
            active_goal_ = true; //Assign value as true if data received is not zero
        }
        //RCLCPP_INFO(this->get_logger(), "Received feedback: [%d,%d]", feedback->feedback.navigation_time.sec, feedback->feedback.navigation_time.nanosec);
        active_goal_ = true;
    }

    void checkActiveGoal() 
    {
        if (active_goal_) 
        {
            RCLCPP_INFO(get_logger(), "Found an active goal. Cancelling it...");
            cancelGoal(); //Call cancelGoal function, if there is an active goal
        } 
        else 
        {
            RCLCPP_INFO(get_logger(), "No active goal found.");
        }
    }

    void cancelGoal() {
        auto cancel_future = action_client_->async_cancel_all_goals();
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future);
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr feedback_subscriber_;
    rclcpp::TimerBase::SharedPtr check_goal_timer_;
    bool active_goal_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); //Initialize ROS2 node
    auto node = std::make_shared<GoalCancelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown(); //Shutdown when spinning is finished
    return 0; //Returning 0 means the program has successfully executed without any errors
}
