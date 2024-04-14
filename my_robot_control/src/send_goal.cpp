#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalSenderNode : public rclcpp::Node //GoalSenderNode subclass inherited from Node class
{
public:
    GoalSenderNode() : Node("goal_sender") //Construction initialized node name 
    {
        // Client created for the navigation action server
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        // Wait for 5 seconds for action server to become available
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(get_logger(), "Action server not available, waiting...");
        }

        // Calling the sendGoal function
        sendGoal();
    }

private:
    void sendGoal()
    {
        //Creating Goal message and assigning the goal pose values
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = 1.8127097332523006; // Example x coordinate
        goal_msg.pose.pose.position.y = 0.5556312219998707; // Example y coordinate
        goal_msg.pose.pose.orientation.z = -0.903072007772838; // Example orientation
        goal_msg.pose.pose.orientation.w = 0.4294891718974242; // Example orientation
        goal_msg.pose.header.frame_id = "map"; // Assuming the goal is specified in the map frame

        //Asynchronously goal is send, if it succesfully sends a goal then it will log
        //a sucessfull message otherwise it will print the other two log messages
        auto goal_handle_future = action_client_->async_send_goal(goal_msg);
        
        auto result_future = rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future);
        if (result_future != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to send goal");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server");
            return;
    }

        RCLCPP_INFO(get_logger(), "Goal sent successfully");
    }
    
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); //Initialize ROS2 node
    auto node = std::make_shared<GoalSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown(); //Shutdown when spinning is finished
    return 0; //Returning 0 means the program has successfully executed without any errors
}
