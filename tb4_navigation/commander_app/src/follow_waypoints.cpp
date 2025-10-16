#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_util/lifecycle_service_client.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class WaypointFollower : public rclcpp::Node
{
public:
    WaypointFollower()
    : Node("waypoint_follower")
    {
        // Initialize the action client
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Wait for the action server to be available
        this->client_ptr_->wait_for_action_server();

        // Set initial pose
        geometry_msgs::msg::PoseStamped initial_pose;
        initial_pose.header.frame_id = "map";
        initial_pose.pose.orientation.z = 1.0;
        initial_pose.pose.orientation.w = 0.0;
        // Assuming you have a method to set the initial pose in your navigation stack
        // setInitialPose(initial_pose);

        // Send your route
        std::vector<geometry_msgs::msg::PoseStamped> inspection_points;
        geometry_msgs::msg::PoseStamped inspection_pose;
        inspection_pose.header.frame_id = "map";
        inspection_pose.header.stamp = this->now();
        inspection_pose.pose.orientation.z = 1.0;
        inspection_pose.pose.orientation.w = 0.0;
        for (const auto &pt : inspection_route_)
        {
            inspection_pose.pose.position.x = pt[0];
            inspection_pose.pose.position.y = pt[1];
            inspection_points.push_back(inspection_pose);
        }

        // Navigate through the waypoints
        for (const auto &pose : inspection_points)
        {
            if (!navigateToPose(pose))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to navigate to waypoint");
                return;
            }
        }
    }

private:
    bool navigateToPose(const geometry_msgs::msg::PoseStamped &pose)
    {
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = pose;

        RCLCPP_INFO(this->get_logger(), "Navigating to: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);

        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        auto result_future = this->client_ptr_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Get result call failed");
            return false;
        }

        auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
        return true;
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    std::vector<std::array<double, 2>> inspection_route_ = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}}; // Example route
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}