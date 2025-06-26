#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <string>
#include <vector>

class PathSmoother : public rclcpp::Node
{
public:
    PathSmoother() : Node("path_smoother")
    {
        // Parameters
        this->declare_parameter("smoothing_type", "bezier");
        this->declare_parameter("resolution", 30);
        this->declare_parameter("frame_id", "map");

        smoothing_type_ = this->get_parameter("smoothing_type").as_string();
        resolution_ = this->get_parameter("resolution").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Subscriber
        waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints", 10,
            std::bind(&PathSmoother::waypointCallback, this, std::placeholders::_1)
        );

        // Publisher
        smoothed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("smoothed_path", 10);

        RCLCPP_INFO(this->get_logger(), "Path Smoother initialized. Type: %s", smoothing_type_.c_str());
    }

private:
    std::string smoothing_type_;
    int resolution_;
    std::string frame_id_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;

    void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        const auto& waypoints = msg->poses;

        if (waypoints.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough waypoints for smoothing.");
            return;
        }

        nav_msgs::msg::Path smoothed_path;
        smoothed_path.header.stamp = this->now();
        smoothed_path.header.frame_id = frame_id_;

        if (smoothing_type_ == "bezier") {
            smoothed_path.poses = smoothWithBezier(waypoints);
        } else {
            smoothed_path.poses = smoothWithLinear(waypoints);
        }

        smoothed_path_pub_->publish(smoothed_path);
        RCLCPP_INFO(this->get_logger(), "Published smoothed path with %ld points", smoothed_path.poses.size());
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothWithLinear(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
    {
        std::vector<geometry_msgs::msg::PoseStamped> smoothed;
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            const auto& p1 = waypoints[i].pose.position;
            const auto& p2 = waypoints[i + 1].pose.position;

            for (int j = 0; j <= resolution_; ++j) {
                double t = static_cast<double>(j) / resolution_;
                geometry_msgs::msg::PoseStamped interp;
                interp.header.frame_id = frame_id_;
                interp.pose.position.x = p1.x + (p2.x - p1.x) * t;
                interp.pose.position.y = p1.y + (p2.y - p1.y) * t;
                interp.pose.orientation.w = 1.0;
                smoothed.push_back(interp);
            }
        }
        return smoothed;
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothWithBezier(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
    {
        std::vector<geometry_msgs::msg::PoseStamped> smoothed;

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            const auto& p0 = waypoints[i].pose.position;
            const auto& p1 = waypoints[i + 1].pose.position;

            geometry_msgs::msg::Point control1;
            control1.x = (2.0 * p0.x + p1.x) / 3.0;
            control1.y = (2.0 * p0.y + p1.y) / 3.0;
            control1.z = 0.0;

            geometry_msgs::msg::Point control2;
            control2.x = (p0.x + 2.0 * p1.x) / 3.0;
            control2.y = (p0.y + 2.0 * p1.y) / 3.0;
            control2.z = 0.0;


            for (int j = 0; j <= resolution_; ++j) {
                double t = static_cast<double>(j) / resolution_;
                double u = 1.0 - t;
                double tt = t * t;
                double uu = u * u;

                double x = uu * p0.x + 2 * u * t * control1.x + tt * p1.x;
                double y = uu * p0.y + 2 * u * t * control1.y + tt * p1.y;

                geometry_msgs::msg::PoseStamped point;
                point.header.frame_id = frame_id_;
                point.pose.position.x = x;
                point.pose.position.y = y;
                point.pose.orientation.w = 1.0;

                smoothed.push_back(point);
            }
        }

        return smoothed;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSmoother>());
    rclcpp::shutdown();
    return 0;
}
