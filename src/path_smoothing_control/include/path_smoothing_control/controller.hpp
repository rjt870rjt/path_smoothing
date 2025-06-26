#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

namespace path_smoothing_control
{
    class Controller
    {
    public:
        static double computeDistance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
        {
            double dx = a.position.x - b.position.x;
            double dy = a.position.y - b.position.y;
            return std::hypot(dx, dy);
        }

        static double computeHeadingToTarget(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)
        {
            double dx = to.position.x - from.position.x;
            double dy = to.position.y - from.position.y;
            return std::atan2(dy, dx);
        }

        static double computeYawFromQuaternion(const geometry_msgs::msg::Pose& pose)
        {
            double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
            double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
            return std::atan2(siny_cosp, cosy_cosp);
        }

        static double angleDiff(double target, double current)
        {
            double diff = target - current;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            return diff;
        }
    };
}
