#include "three_omniwheel_controller/odometry.hpp"

namespace three_omniwheel_controller
{
    Odometry::Odometry(size_t velocity_rolling_window_size)
        : timestamp_(0.0),
          x_(0.0),
          y_(0.0),
          heading_(0.0),
          linear_(0.0),
          linear_x_(0.0),
          linear_y_(0.0),
          angular_(0.0),
          x_wheel_radius_(0.0),
          y_wheel_radius_(0.0),
          x_wheel_old_pos_(0.0),
          y_wheel_old_pos_(0.0),
          velocity_rolling_window_size_(velocity_rolling_window_size),
          linear_accumulator_x_(velocity_rolling_window_size),
          linear_accumulator_y_(velocity_rolling_window_size),
          angular_accumulator_(velocity_rolling_window_size)
    {
    }

    void Odometry::init(const rclcpp::Time &time)
    {
        // Reset accumulators and timestamp:
        resetAccumulators();
        timestamp_ = time;
    }

    bool Odometry::update(double x_pos, double y_pos, double ang_pos, const rclcpp::Time &time)
    {
        // x_pos, y_pos are angular displacement in radians
        //  We cannot estimate the speed with very small time intervals:
        const double dt = time.seconds() - timestamp_.seconds();
        if (dt < 0.0001)
        {
            return false; // Interval too small to integrate with
        }

        // Get current wheel joint positions:
        const double x_wheel_cur_pos = x_pos * x_wheel_radius_;
        const double y_wheel_cur_pos = y_pos * y_wheel_radius_;
        const double ang_cur_pos = ang_pos;

        // Estimate velocity of wheels using old and current position:
        const double x_wheel_est_vel = (x_wheel_cur_pos - x_wheel_old_pos_) / dt;
        const double y_wheel_est_vel = (y_wheel_cur_pos - y_wheel_old_pos_) / dt;
        const double ang_est_vel = (ang_cur_pos - ang_old_pos_) / dt;

        // Update old position with current:
        x_wheel_old_pos_ = x_wheel_cur_pos;
        y_wheel_old_pos_ = y_wheel_cur_pos;
        ang_old_pos_ = ang_cur_pos;

        updateFromVelocity(x_wheel_est_vel, y_wheel_est_vel, ang_est_vel, time);

        return true;
    }

    bool Odometry::updateFromVelocity(double x_vel, double y_vel, double ang_vel, const rclcpp::Time &time)
    {
        const double dt = time.seconds() - timestamp_.seconds();
        if (dt < 0.0001)
        {
            return false; // Interval too small to integrate with
        }
        // Compute linear and angular diff:
        const double linear = sqrt((x_vel * x_vel) + (y_vel * y_vel));
        // Now there is a bug about scout angular velocity
        const double angular = ang_vel;

        // Integrate odometry:
        integrateExact(x_vel, y_vel, angular);

        timestamp_ = time;

        // Estimate speeds using a rolling mean to filter them out:
        linear_accumulator_x_.accumulate(x_vel / dt);
        linear_accumulator_y_.accumulate(y_vel / dt);
        angular_accumulator_.accumulate(angular / dt);

        linear_x_ = linear_accumulator_x_.getRollingMean();
        linear_y_ = linear_accumulator_y_.getRollingMean();
        angular_ = angular_accumulator_.getRollingMean();

        return true;
    }

    void Odometry::updateOpenLoop(double linear, double linear_x, double linear_y, double angular, const rclcpp::Time &time)
    {
        /// Save last linear and angular velocity:
        linear_x_ = linear_x;
        linear_y_ = linear_y;

        // linear_ = sqrt((linear_x * linear_x) + (linear_y * linear_y));
        angular_ = angular;

        /// Integrate odometry:
        const double dt = time.seconds() - timestamp_.seconds();
        timestamp_ = time;
        integrateExact(linear_x * dt, linear_y * dt, angular * dt);
    }

    void Odometry::resetOdometry()
    {
        x_ = 0.0;
        y_ = 0.0;
        heading_ = 0.0;
    }

    void Odometry::setWheelParams( // wheel parameter of dead wheel
        double x_wheel_radius, double y_wheel_radius)
    {
        x_wheel_radius_ = x_wheel_radius;
        y_wheel_radius_ = y_wheel_radius;
    }

    void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
    {
        velocity_rolling_window_size_ = velocity_rolling_window_size;

        resetAccumulators();
    }

    void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
    {
        // const double direction = heading_ + angular * 0.5;

        // /// Runge-Kutta 2nd order integration:
        // x_ += linear_x * cos(direction);
        // y_ += linear_y * sin(direction);

        heading_ += angular;

        const double cos_heading = cos(heading_);
        const double sin_heading = sin(heading_);

        x_ += linear_x * cos_heading - linear_y * sin_heading;
        y_ += linear_x * sin_heading + linear_y * cos_heading;
    }

    // void Odometry::integrateExact(double linear_x, double linear_y, double angular)
    // {
    //     if (fabs(angular) < 1e-6)
    //     {
    //         integrateRungeKutta2(linear_x, linear_y, angular);
    //     }
    //     else
    //     {
    //         /// Exact integration (should solve problems when angular is zero):
    //         const double heading_old = heading_;
    //         const double r_x_ = linear_x / angular;
    //         const double r_y_ = linear_y / angular;
    //         heading_ += angular;
    //         x_ += r_x_ * (sin(heading_) - sin(heading_old));
    //         y_ += -r_y_ * (cos(heading_) - cos(heading_old));
    //     }
    // }
    
    void Odometry::integrateExact(double linear_x, double linear_y, double angular)
    {
        const double heading_old = heading_;
        heading_ += angular;
    
        const double cos_heading = cos(heading_old);
        const double sin_heading = sin(heading_old);
    
        if (fabs(angular) < 1e-6)
        {
            // Straight motion (no rotation)
            x_ += linear_x * cos_heading - linear_y * sin_heading;
            y_ += linear_x * sin_heading + linear_y * cos_heading;
        }
        else
        {
            // Circular arc motion
            const double r_x = linear_x / angular;
            const double r_y = linear_y / angular;
    
            x_ += r_x * (sin(heading_) - sin(heading_old)) - r_y * (cos(heading_) - cos(heading_old));
            y_ += r_x * (cos(heading_) - cos(heading_old)) + r_y * (sin(heading_) - sin(heading_old));
        }
    }
    
    void Odometry::resetAccumulators()
    {
        linear_accumulator_x_ = RollingMeanAccumulator(velocity_rolling_window_size_);
        linear_accumulator_y_ = RollingMeanAccumulator(velocity_rolling_window_size_);
        angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    }
}