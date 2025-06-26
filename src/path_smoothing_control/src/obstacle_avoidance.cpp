#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>

struct Obstacle
{
    double x;
    double y;
    double radius;
};

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance() : Node("obstacle_avoidance")
    {
        this->declare_parameter("avoidance_radius", 1.0);
        this->declare_parameter("avoidance_strength", 2.0);
        this->declare_parameter("frame_id", "map");

        avoidance_radius_ = this->get_parameter("avoidance_radius").as_double();
        avoidance_strength_ = this->get_parameter("avoidance_strength").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "trajectory", 10,
            std::bind(&ObstacleAvoidance::trajectoryCallback, this, std::placeholders::_1)
        );

        // (Optional: subscribe to obstacle topic — here using static list)
        // If needed, add a subscriber to `/scan` or `/obstacles`

        adjusted_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("adjusted_trajectory", 10);

        // Fake some obstacles for now
        createDummyObstacles();

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance node initialized.");
    }

private:
    double avoidance_radius_;
    double avoidance_strength_;
    std::string frame_id_;
    std::vector<Obstacle> obstacles_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr adjusted_traj_pub_;

    void createDummyObstacles()
    {
        // You can replace this with actual detection later
        obstacles_.push_back({0.0, 0.0, 0.5});
        obstacles_.push_back({1.5, 1.5, 0.3});
        obstacles_.push_back({-1.0, 1.0, 0.4});
    }

    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        nav_msgs::msg::Path adjusted;
        adjusted.header.stamp = this->now();
        adjusted.header.frame_id = frame_id_;

        for (const auto& pose_stamped : msg->poses)
        {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;

            double ax = 0.0;
            double ay = 0.0;

            for (const auto& obs : obstacles_)
            {
                double dx = x - obs.x;
                double dy = y - obs.y;
                double dist = std::hypot(dx, dy);

                if (dist < avoidance_radius_ + obs.radius && dist > 0.01)
                {
                    double force = avoidance_strength_ / std::max((dist - obs.radius), 0.01);
                    ax += (dx / dist) * force;
                    ay += (dy / dist) * force;
                }
            }

            geometry_msgs::msg::PoseStamped new_pose = pose_stamped;
            new_pose.pose.position.x += ax * 0.1; // Adjust tuning multiplier
            new_pose.pose.position.y += ay * 0.1;
            adjusted.poses.push_back(new_pose);
        }

        adjusted_traj_pub_->publish(adjusted);
        RCLCPP_INFO(this->get_logger(), "Published adjusted trajectory with obstacle avoidance.");
    }
};
  
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}
