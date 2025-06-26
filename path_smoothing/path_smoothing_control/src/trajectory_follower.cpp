#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Needed for tf2::fromMsg
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <vector>
#include <cmath>

class TrajectoryFollower : public rclcpp::Node
{
public:
    TrajectoryFollower() : Node("trajectory_follower")
    {
        this->declare_parameter("controller_gain", 1.0);
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("robot_base_frame", "base_footprint");

        controller_gain_ = this->get_parameter("controller_gain").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        robot_frame_ = this->get_parameter("robot_base_frame").as_string();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "trajectory", 10,
            std::bind(&TrajectoryFollower::trajectoryCallback, this, std::placeholders::_1)
        );

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryFollower::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Trajectory Follower initialized.");
    }

private:
    double controller_gain_;
    std::string frame_id_, robot_frame_;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        trajectory_ = msg->poses;
        RCLCPP_INFO(this->get_logger(), "Received new trajectory with %ld points", trajectory_.size());
    }

    geometry_msgs::msg::Pose getCurrentPose()
    {
        geometry_msgs::msg::Pose pose;
        try {
            auto tf = tf_buffer_->lookupTransform(frame_id_, robot_frame_, tf2::TimePointZero);
            pose.position.x = tf.transform.translation.x;
            pose.position.y = tf.transform.translation.y;

            // Set orientation from TF
            pose.orientation = tf.transform.rotation;

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Lookup failed: %s", ex.what());
        }
        return pose;
    }

    void controlLoop()
    {
        if (trajectory_.empty()) return;

        geometry_msgs::msg::Pose robot_pose = getCurrentPose();
        double rx = robot_pose.position.x;
        double ry = robot_pose.position.y;

        // Find closest point on trajectory
        size_t closest_index = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < trajectory_.size(); ++i) {
            double dx = trajectory_[i].pose.position.x - rx;
            double dy = trajectory_[i].pose.position.y - ry;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }

        if (closest_index >= trajectory_.size()) return;

        auto target = trajectory_[closest_index].pose.position;
        double dx = target.x - rx;
        double dy = target.y - ry;
        double angle_to_target = std::atan2(dy, dx);

        // Convert geometry_msgs::Quaternion to tf2::Quaternion using fromMsg()
        tf2::Quaternion tf_q;
        tf2::fromMsg(robot_pose.orientation, tf_q);
        double robot_yaw = tf2::getYaw(tf_q);

        double angle_error = angle_to_target - robot_yaw;
        if (angle_error > M_PI) angle_error -= 2 * M_PI;
        if (angle_error < -M_PI) angle_error += 2 * M_PI;

        double distance = std::hypot(dx, dy);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = controller_gain_ * distance;
        cmd.angular.z = controller_gain_ * angle_error;

        cmd_pub_->publish(cmd);

        if (distance < 0.05 && closest_index == trajectory_.size() - 1) {
            RCLCPP_INFO(this->get_logger(), "Trajectory completed!");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            trajectory_.clear();
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryFollower>());
    rclcpp::shutdown();
    return 0;
}
