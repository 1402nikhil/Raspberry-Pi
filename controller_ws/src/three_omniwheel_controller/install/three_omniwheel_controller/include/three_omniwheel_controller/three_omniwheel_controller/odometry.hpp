/*
use the perpendicular encoder for feedback instead of motor encoders
added imu data for feedback

*/
#ifndef THREE_OMNIWHEEL_CONTROLLER__ODOMETRY_HPP_
#define THREE_OMNIWHEEL_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace three_omniwheel_controller
{
    class Odometry
    {
        public:
          explicit Odometry(size_t velocity_rolling_window_size = 10);
        
          void init(const rclcpp::Time & time);
          bool update(double x_pos, double y_pos, double ang_pos, const rclcpp::Time & time);
          bool updateFromVelocity(double x_vel, double y_vel, double ang_vel, const rclcpp::Time & time);
          void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
          void resetOdometry();
        
          double getX() const { return x_; }
          double getY() const { return y_; }
          double getHeading() const { return heading_; }
          double getLinear() const { return linear_; }
          double getAngular() const { return angular_; }
        
          void setWheelParams(double x_wheel_radius, double y_wheel_radius);
          void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
        
        private:
          using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
        
          void integrateRungeKutta2(double linear, double angular);
          void integrateExact(double linear, double angular);
          void resetAccumulators();
        
          // Current timestamp:
          rclcpp::Time timestamp_;
        
          // Current pose:
          double x_;        //   [m]
          double y_;        //   [m]
          double heading_;  // [rad]
        
          // Current velocity:
          double linear_;   //   [m/s]
          double angular_;  // [rad/s]
        
          // Wheel kinematic parameters [m]:
          double x_wheel_radius_;
          double y_wheel_radius_;
        
          // Previous wheel position/state [rad]:
          double x_wheel_old_pos_;
          double y_wheel_old_pos_;
          double ang_old_pos_;
        
          // Rolling mean accumulators for the linear and angular velocities:
          size_t velocity_rolling_window_size_;
          RollingMeanAccumulator linear_accumulator_;
          RollingMeanAccumulator angular_accumulator_;

    };
}  // namespace three_omniwheel_controller

#endif  // THREE_OMNIWHEEL_CONTROLLER__ODOMETRY_HPP_