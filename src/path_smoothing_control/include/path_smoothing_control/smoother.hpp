#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

namespace path_smoothing_control
{
    class Smoother
    {
    public:
        static std::vector<geometry_msgs::msg::PoseStamped> linearInterpolate(
            const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
            int resolution)
        {
            std::vector<geometry_msgs::msg::PoseStamped> result;
            for (size_t i = 0; i < waypoints.size() - 1; ++i)
            {
                const auto& p1 = waypoints[i].pose.position;
                const auto& p2 = waypoints[i + 1].pose.position;

                for (int j = 0; j <= resolution; ++j)
                {
                    double t = static_cast<double>(j) / resolution;
                    geometry_msgs::msg::PoseStamped point;
                    point.header = waypoints[i].header;
                    point.pose.position.x = p1.x + (p2.x - p1.x) * t;
                    point.pose.position.y = p1.y + (p2.y - p1.y) * t;
                    point.pose.orientation.w = 1.0;
                    result.push_back(point);
                }
            }
            return result;
        }

        static std::vector<geometry_msgs::msg::PoseStamped> bezierSmooth(
            const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
            int resolution)
        {
            std::vector<geometry_msgs::msg::PoseStamped> result;

            for (size_t i = 0; i < waypoints.size() - 1; ++i)
            {
                const auto& p0 = waypoints[i].pose.position;
                const auto& p1 = waypoints[i + 1].pose.position;

                geometry_msgs::msg::Point c1 = {
                    (2.0 * p0.x + p1.x) / 3.0,
                    (2.0 * p0.y + p1.y) / 3.0,
                    0.0
                };

                for (int j = 0; j <= resolution; ++j)
                {
                    double t = static_cast<double>(j) / resolution;
                    double u = 1.0 - t;
                    double x = u * u * p0.x + 2 * u * t * c1.x + t * t * p1.x;
                    double y = u * u * p0.y + 2 * u * t * c1.y + t * t * p1.y;

                    geometry_msgs::msg::PoseStamped pt;
                    pt.header = waypoints[i].header;
                    pt.pose.position.x = x;
                    pt.pose.position.y = y;
                    pt.pose.orientation.w = 1.0;
                    result.push_back(pt);
                }
            }

            return result;
        }
    };
}
