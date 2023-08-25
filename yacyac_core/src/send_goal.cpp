#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/header.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class SendGoalClient {
public:
    SendGoalClient(double waypoint_x, double waypoint_y, double waypoint_theta) : waypoint_x_(waypoint_x), waypoint_y_(waypoint_y), waypoint_theta_(waypoint_theta)
    {
        // Initialize the ROS node
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("send_goal_client");
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

        // Wait for the action server to become available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Initialize the goal message
        goal_msg_.pose.header.frame_id = "map";
        goal_msg_.pose.pose.position.x = waypoint_x_;
        goal_msg_.pose.pose.position.y = waypoint_y_;
        goal_msg_.pose.pose.position.z = 0.0;
        goal_msg_.pose.pose.orientation.x = 0;
        goal_msg_.pose.pose.orientation.y = 0;
        goal_msg_.pose.pose.orientation.z = 0;
        goal_msg_.pose.pose.orientation.w = waypoint_theta_;
    }
    void sendGoal()
    {
        RCLCPP_INFO(node_->get_logger(), "Sending goal");

        auto goal_handle_future = action_client_->async_send_goal(goal_msg_);

        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
            rclcpp::shutdown();
            return;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            rclcpp::shutdown();
            return;
        }

        auto result_future = action_client_->async_get_result(goal_handle);

        RCLCPP_INFO(node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to receive result");
            rclcpp::shutdown();
            return;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: RCLCPP_INFO(node_->get_logger(), "Goal reached successfully!"); break;
        case rclcpp_action::ResultCode::ABORTED: RCLCPP_ERROR(node_->get_logger(), "Goal was aborted"); break;
        case rclcpp_action::ResultCode::CANCELED: RCLCPP_ERROR(node_->get_logger(), "Goal was canceled"); break;
        default: RCLCPP_ERROR(node_->get_logger(), "Unknown result code"); break;
        }

        rclcpp::shutdown();
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    nav2_msgs::action::NavigateToPose::Goal goal_msg_;
    double waypoint_x_;
    double waypoint_y_;
    double waypoint_theta_;
};

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <waypoint_x> <waypoint_y> <waypoint_theta>" << std::endl;
        return 1;
    }

    double waypoint_x = std::stod(argv[1]);
    double waypoint_y = std::stod(argv[2]);
    double waypoint_theta = std::stod(argv[3]);
    std::cout << "start" << std::endl;
    SendGoalClient client(waypoint_x, waypoint_y, waypoint_theta);
    client.sendGoal();
    return 0;
}
