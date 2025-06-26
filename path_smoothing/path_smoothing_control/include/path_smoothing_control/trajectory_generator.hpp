#pragma once

namespace path_smoothing_control
{
    class VelocityProfiles
    {
    public:
        static double trapezoidal(int index, int total, double max_vel)
        {
            double progress = static_cast<double>(index) / total;
            if (progress < 0.3)
                return max_vel * (progress / 0.3);
            else if (progress > 0.7)
                return max_vel * ((1.0 - progress) / 0.3);
            return max_vel;
        }

        static double smooth(int index, int total, double max_vel)
        {
            double progress = static_cast<double>(index) / total;
            return max_vel * (1.0 - std::cos(progress * M_PI)) / 2.0;
        }

        static double constant(double max_vel)
        {
            return max_vel;
        }
    };
}
