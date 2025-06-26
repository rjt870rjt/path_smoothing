#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <random>

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher()
    : Node("waypoint_publisher")
    {
        // Declare and get parameters
        this->declare_parameter("num_waypoints", 6);
        this->declare_parameter("publish_rate", 1.0);
        this->declare_parameter("frame_id", "map");

        num_waypoints_ = this->get_parameter("num_waypoints").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&WaypointPublisher::publishPath, this)
        );

        RCLCPP_INFO(this->get_logger(), "Waypoint Publisher initialized.");
    }

private:
    void publishPath()
    {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = frame_id_;

        // Generate random waypoints within a bounded box (e.g., 5m x 5m)
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(-2.5, 2.5);

        for (int i = 0; i < num_waypoints_; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = dist(gen);
            pose.pose.position.y = dist(gen);
            pose.pose.orientation.w = 1.0;
            poses.push_back(pose);
        }

        path_msg.poses = poses;
        path_pub_->publish(path_msg);

        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing %ld random waypoints", poses.size());
    }

    // Parameters
    int num_waypoints_;
    double publish_rate_;
    std::string frame_id_;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
