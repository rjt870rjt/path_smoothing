#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>
#include <cmath>

class TrajectoryGenerator : public rclcpp::Node
{
public:
    TrajectoryGenerator() : Node("trajectory_generator")
    {
        // Declare parameters
        this->declare_parameter("velocity_profile", "trapezoidal");
        this->declare_parameter("max_velocity", 0.5);
        this->declare_parameter("frame_id", "map");

        velocity_profile_ = this->get_parameter("velocity_profile").as_string();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Subscriber
        smoothed_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "smoothed_path", 10,
            std::bind(&TrajectoryGenerator::pathCallback, this, std::placeholders::_1)
        );

        // Publisher
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "Trajectory Generator initialized with %s profile", velocity_profile_.c_str());
    }

private:
    std::string velocity_profile_;
    double max_velocity_;
    std::string frame_id_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smoothed_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        const auto& path = msg->poses;
        if (path.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough path points to generate trajectory.");
            return;
        }

        nav_msgs::msg::Path trajectory;
        trajectory.header.stamp = this->now();
        trajectory.header.frame_id = frame_id_;

        double current_time = 0.0;

        for (size_t i = 0; i < path.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose = path[i];
            pose.header.stamp = this->now() + rclcpp::Duration::from_seconds(current_time);

            // Velocity at this point (optional to use later)
            double velocity = getVelocity(i, path.size());

            if (i > 0) {
                const auto& p1 = path[i-1].pose.position;
                const auto& p2 = path[i].pose.position;
                double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
                current_time += dist / velocity;
            }

            trajectory.poses.push_back(pose);
        }

        trajectory_pub_->publish(trajectory);
        RCLCPP_INFO(this->get_logger(), "Published trajectory with %ld poses", trajectory.poses.size());
    }

    double getVelocity(int index, int total)
    {
        if (velocity_profile_ == "constant") {
            return max_velocity_;
        } else if (velocity_profile_ == "smooth") {
            double progress = static_cast<double>(index) / total;
            return max_velocity_ * (1 - std::cos(progress * M_PI)) / 2.0;
        } else if (velocity_profile_ == "trapezoidal") {
            double progress = static_cast<double>(index) / total;
            if (progress < 0.3) {
                return max_velocity_ * (progress / 0.3);
            } else if (progress > 0.7) {
                return max_velocity_ * ((1.0 - progress) / 0.3);
            }
            return max_velocity_;
        }
        return max_velocity_;  // fallback
    }
};
  
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}
