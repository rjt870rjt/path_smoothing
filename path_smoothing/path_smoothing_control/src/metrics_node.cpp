#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <vector>
#include <chrono>

class MetricsNode : public rclcpp::Node
{
public:
    MetricsNode() : Node("metrics_node")
    {
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("trajectory_topic", "/adjusted_trajectory");

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 50,
            std::bind(&MetricsNode::odomCallback, this, std::placeholders::_1)
        );

        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            trajectory_topic_, 10,
            std::bind(&MetricsNode::trajectoryCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Metrics Node initialized.");
    }

private:
    std::string odom_topic_;
    std::string trajectory_topic_;

    std::vector<geometry_msgs::msg::PoseStamped> reference_trajectory_;
    std::vector<geometry_msgs::msg::PoseStamped> actual_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;

    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        reference_trajectory_ = msg->poses;
        actual_path_.clear();  // reset previous run
        RCLCPP_INFO(this->get_logger(), "Reference trajectory received with %ld points.", reference_trajectory_.size());
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (reference_trajectory_.empty()) return;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        actual_path_.push_back(pose);

        if (actual_path_.size() % 20 == 0) { // evaluate every ~2 sec
            double error = computeTrackingError();
            double path_len = computePathLength(reference_trajectory_);
            double complete = 100.0 * static_cast<double>(actual_path_.size()) / reference_trajectory_.size();

            RCLCPP_INFO(this->get_logger(), 
                "📏 Avg Error: %.3f m | 📈 Path Length: %.2f m | ✅ Completion: %.1f%%",
                error, path_len, std::min(complete, 100.0));
        }
    }

    double computeTrackingError()
    {
        if (actual_path_.empty() || reference_trajectory_.empty()) return 0.0;

        double total_error = 0.0;
        int count = 0;

        for (const auto& ap : actual_path_) {
            double min_dist = std::numeric_limits<double>::max();
            for (const auto& rp : reference_trajectory_) {
                double dx = ap.pose.position.x - rp.pose.position.x;
                double dy = ap.pose.position.y - rp.pose.position.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist) min_dist = dist;
            }
            total_error += min_dist;
            ++count;
        }

        return count > 0 ? total_error / count : 0.0;
    }

    double computePathLength(const std::vector<geometry_msgs::msg::PoseStamped>& path)
    {
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].pose.position.x - path[i - 1].pose.position.x;
            double dy = path[i].pose.position.y - path[i - 1].pose.position.y;
            length += std::sqrt(dx * dx + dy * dy);
        }
        return length;
    }
};
  
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetricsNode>());
    rclcpp::shutdown();
    return 0;
}
