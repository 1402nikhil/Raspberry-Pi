
#include <utility>
#include <cstring>

#include "three_omniwheel_controller/three_omniwheel_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rcutils/error_handling.h>
#include "tf2/LinearMath/Quaternion.h"

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace three_omniwheel_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    ThreeOmniwheelController::ThreeOmniwheelController() : controller_interface::ControllerInterface() {}

    const char *ThreeOmniwheelController::feedback_type() const
    {
        return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
    }

    const char *ThreeOmniwheelController::imu_feedback_type() const
    {
        return "orientation";
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration ThreeOmniwheelController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.front_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + std::string(HW_IF_VELOCITY));
        }
        for (const auto &joint_name : params_.back_right_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + std::string(HW_IF_VELOCITY));
        }
        for (const auto &joint_name : params_.back_left_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + std::string(HW_IF_VELOCITY));
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration ThreeOmniwheelController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.front_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        for (const auto &joint_name : params_.back_right_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        for (const auto &joint_name : params_.back_left_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        for (const auto &joint_name : params_.x_dead_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + std::string(feedback_type()));
        }

        for (const auto &joint_name : params_.y_dead_wheel_name)
        {
            conf_names.push_back(joint_name + "/" + std::string(feedback_type()));
        }

        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type ThreeOmniwheelController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // auto logger = get_node()->get_logger();
        // auto logger = rclcpp::get_logger("ThreeOmniwheelController");
        if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
        {
            if (!is_halted)
            {
                halt();
                is_halted = true;
            }
            return controller_interface::return_type::OK;
        }

        std::shared_ptr<Twist> last_command_msg;
        received_velocity_msg_ptr_.get(last_command_msg);

        if (last_command_msg == nullptr)
        {
            // RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
            // RCLCPP_WARN(logger, "jbn");
            std::string msg = "nikhil";
            RCLCPP_WARN(rclcpp::get_logger("ThreeOmniwheelController"), msg.c_str());
            return controller_interface::return_type::ERROR;
        }

        const auto age_of_last_command = time - last_command_msg->header.stamp;
        // Brake if cmd_vel has timeout, override the stored command
        if (age_of_last_command > cmd_vel_timeout_)
        {
            last_command_msg->twist.linear.x = 0.0;
            last_command_msg->twist.linear.y = 0.0;
            last_command_msg->twist.angular.z = 40.0;
            last_command_msg->twist.angular.x = 40.0;
        }

        // command may be limited further by SpeedLimit,
        // without affecting the stored twist command
        Twist command = *last_command_msg;
        // double ang_z = command.twist.angular.z - 1.0;
        // double ang_x = - command.twist.angular.x + 1.0;

        double angular_value = -(command.twist.angular.z - command.twist.angular.x) / 2.0;
        // float angular_value = 0.0;
        // double dummy = 0.0;
        
        double linear_command_x_ = command.twist.linear.x;
        double linear_command_y_ = command.twist.linear.y;
        double angular_command_z_ = command.twist.angular.z;
        double angular_command_x_ = command.twist.angular.x;

        double linear_command = sqrt((command.twist.linear.x * command.twist.linear.x) + (command.twist.linear.y * command.twist.linear.y));
        double angular_command = angular_value;

        previous_update_timestamp_ = time;

        // Apply (possibly new) multipliers:
        const double x_dead_wheel_radius = params_.x_wheel_radius_multiplier * params_.dead_wheel_radius;
        const double y_dead_wheel_radius = params_.y_wheel_radius_multiplier * params_.dead_wheel_radius;

        RCLCPP_INFO(rclcpp::get_logger("ThreeOmniwheelController")," before uponloop %.2f", angular_command);

        if (params_.open_loop)
        {
            odometry_.updateOpenLoop(linear_command, linear_command_x_, linear_command_y_, angular_command, time);
        }
        else
        {
            double x_feedback_mean = 0.0;
            double y_feedback_mean = 0.0;
            double ang_feedback_mean = 0.0;

            const double x_feedback = registered_x_wheel_handle_[0].feedback.get().get_value(); // angular diaplacement in radians
            const double y_feedback = registered_y_wheel_handle_[0].feedback.get().get_value();
            const double ang_feedback = registered_imu_handle_[0].orientation.get().get_value();

            if (std::isnan(x_feedback) || std::isnan(y_feedback) || std::isnan(ang_feedback))
            {
                RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Either the x or y dead wheel or imu %s is invalid", feedback_type());
                return controller_interface::return_type::ERROR;
            }

            x_feedback_mean += x_feedback / 1;
            y_feedback_mean += y_feedback / 1;
            ang_feedback_mean += ang_feedback / 1;

            if (params_.position_feedback)
            {
                odometry_.update(x_feedback_mean, y_feedback_mean, ang_feedback_mean, time);
            }
            else
            {
                odometry_.updateFromVelocity(
                    x_feedback_mean * x_dead_wheel_radius * period.seconds(),
                    y_feedback_mean * y_dead_wheel_radius * period.seconds(), ang_feedback_mean * period.seconds(), time);
            }
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, odometry_.getHeading());

        bool should_publish = false;
        try
        {
            if (previous_publish_timestamp_ + publish_period_ < time)
            {
                previous_publish_timestamp_ += publish_period_;
                should_publish = true;
            }
        }
        catch (const std::runtime_error &)
        {
            // Handle exceptions when the time source changes and initialize publish timestamp
            previous_publish_timestamp_ = time;
            should_publish = true;
        }

        if (should_publish)
        {
            if (realtime_odometry_publisher_->trylock())
            {
                auto &odometry_message = realtime_odometry_publisher_->msg_;
                odometry_message.header.stamp = time;
                odometry_message.pose.pose.position.x = odometry_.getX();
                odometry_message.pose.pose.position.y = odometry_.getY();
                odometry_message.pose.pose.orientation.x = orientation.x();
                odometry_message.pose.pose.orientation.y = orientation.y();
                odometry_message.pose.pose.orientation.z = orientation.z();
                odometry_message.pose.pose.orientation.w = orientation.w();
                odometry_message.twist.twist.linear.x = odometry_.getLinearX();
                odometry_message.twist.twist.linear.y = odometry_.getLinearY();
                odometry_message.twist.twist.angular.z = odometry_.getAngular();
                realtime_odometry_publisher_->unlockAndPublish();
            }

            if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
            {
                auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
                transform.header.stamp = time;
                transform.transform.translation.x = odometry_.getX();
                transform.transform.translation.y = odometry_.getY();
                transform.transform.rotation.x = orientation.x();
                transform.transform.rotation.y = orientation.y();
                transform.transform.rotation.z = orientation.z();
                transform.transform.rotation.w = orientation.w();
                realtime_odometry_transform_publisher_->unlockAndPublish();
            }
        }

  RCLCPP_INFO(rclcpp::get_logger("ThreeOmniwheelController")," before limit %.2f", angular_command);

        auto &last_command = previous_commands_.back().twist;
        auto &second_to_last_command = previous_commands_.front().twist;
        limiter_linear_x_.limit(linear_command_x_, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
        limiter_linear_y_.limit(linear_command_y_, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
        limiter_angular_z_.limit(angular_command_z_, last_command.angular.z, second_to_last_command.angular.z, period.seconds());
        limiter_angular_x_.limit(angular_command_x_, last_command.angular.x, second_to_last_command.angular.x, period.seconds());


  RCLCPP_INFO(rclcpp::get_logger("ThreeOmniwheelController")," last %.2f", angular_command);

        previous_commands_.pop();
        previous_commands_.emplace(command);

        //    Publish limited velocity
        if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
        {
            auto &limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
            limited_velocity_command.header.stamp = time;
            limited_velocity_command.twist = command.twist;
            realtime_limited_velocity_publisher_->unlockAndPublish();
        }

        angular_value = (angular_command_z_ - angular_command_x_) / 2.0;

        angular_command = angular_value;
        // double &angular_command = command.twist.angular.x;
        double theta = atan2(linear_command_y_, linear_command_x_);
        // double theta = atan2(10,10);

        // Compute wheels velocities:
        const double vx = linear_command_x_;
        const double vy = linear_command_y_;

        // const double vx = command.twist.linear.x;
        // const double vy = command.twist.linear.y;

        const double front_velocity = vx + angular_command * params_.drive_wheel_radius;
        const double back_right_velocity = -vx - 0.577 * vy + angular_command * params_.drive_wheel_radius;
        const double back_left_velocity = -vx + 0.577 * vy + angular_command * params_.drive_wheel_radius;


        // const double front_velocity = 0;
        // const double back_right_velocity = 0;
        // const double back_left_velocity = 0;

        // RCLCPP_INFO(rclcpp::get_logger("ThreeOmniwheelControllerrr"),
        //             "Wheel Velocities -> Front: %.2f, Back Right: %.2f, Back Left: %.2f, Angular cmd: %.2f , %.2f , %.2f , %.4f ",
        //             front_velocity, back_right_velocity, back_left_velocity, angular_command, linear_command_x_, linear_command_y_, theta);

        // Set wheels velocities:
        registered_front_wheel_handle_[0].velocity.get().set_value(front_velocity);
        registered_back_right_wheel_handle_[0].velocity.get().set_value(back_right_velocity);
        registered_back_left_wheel_handle_[0].velocity.get().set_value(back_left_velocity);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::on_configure(
        const rclcpp_lifecycle::State &)
    {
        auto logger = get_node()->get_logger();

        // update parameters if they have changed
        if (param_listener_->is_old(params_))
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(rclcpp::get_logger("ThreeOmniwheelController"), "Parameters were updated");
        }

        if (params_.front_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Front wheel name parameter is empty!");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (params_.back_right_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Back right wheel name parameter is empty!");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (params_.back_left_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Back right wheel name parameter is empty!");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (params_.x_dead_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "x dead wheel name parameter is empty!");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (params_.y_dead_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "y dead wheel name parameter is empty!");
            return controller_interface::CallbackReturn::ERROR;
        }

        const double x_dead_wheel_radius = params_.x_wheel_radius_multiplier * params_.dead_wheel_radius;
        const double y_dead_wheel_radius = params_.y_wheel_radius_multiplier * params_.dead_wheel_radius;

        odometry_.setWheelParams(x_dead_wheel_radius, y_dead_wheel_radius);
        odometry_.setVelocityRollingWindowSize(static_cast<size_t>(params_.velocity_rolling_window_size));

        cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
        publish_limited_velocity_ = params_.publish_limited_velocity;
        use_stamped_vel_ = params_.use_stamped_vel;

        limiter_linear_x_ = SpeedLimiter(
            params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
            params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
            params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
            params_.linear.x.max_jerk);

        limiter_linear_y_ = SpeedLimiter(
            params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
            params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity, params_.linear.y.max_velocity,
            params_.linear.y.min_acceleration, params_.linear.y.max_acceleration, params_.linear.y.min_jerk,
            params_.linear.y.max_jerk);

        limiter_angular_z_ = SpeedLimiter(
            params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
            params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
            params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
            params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

        limiter_angular_x_ = SpeedLimiter(
            params_.angular.x.has_velocity_limits, params_.angular.x.has_acceleration_limits,
            params_.angular.x.has_jerk_limits, params_.angular.x.min_velocity,
            params_.angular.x.max_velocity, params_.angular.x.min_acceleration,
            params_.angular.x.max_acceleration, params_.angular.x.min_jerk, params_.angular.x.max_jerk);

        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (publish_limited_velocity_)
        {
            limited_velocity_publisher_ =
                get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
            realtime_limited_velocity_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
        }

        const Twist empty_twist;
        received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

        // Fill last two commands with default constructed commands
        previous_commands_.emplace(empty_twist);
        previous_commands_.emplace(empty_twist);

        // initialize command subscriber
        if (use_stamped_vel_)
        {
            velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
                DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<Twist> msg) -> void
                {
                    if (!subscriber_is_active_)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("ThreeOmniwheelController"), "Can't accept new commands. subscriber is inactive");
                        return;
                    }
                    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                    {
                        RCLCPP_WARN_ONCE(rclcpp::get_logger("ThreeOmniwheelController"),
                                         "Received TwistStamped with zero timestamp, setting it to current "
                                         "time, this message will only be shown once");
                        msg->header.stamp = get_node()->get_clock()->now();
                    }
                    received_velocity_msg_ptr_.set(std::move(msg));
                });
        }
        else
        {
            velocity_command_unstamped_subscriber_ =
                get_node()->create_subscription<geometry_msgs::msg::Twist>(
                    DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
                    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
                    {
                        if (!subscriber_is_active_)
                        {
                            RCLCPP_WARN(rclcpp::get_logger("ThreeOmniwheelController"), "Can't accept new commands. subscriber is inactive");
                            return;
                        }

                        // Write fake header in the stored stamped command
                        std::shared_ptr<Twist> twist_stamped;
                        received_velocity_msg_ptr_.get(twist_stamped);
                        twist_stamped->twist = *msg;
                        twist_stamped->header.stamp = get_node()->get_clock()->now();
                    });
        }

        // initialize odometry publisher and message
        odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
            DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

        // Append the tf prefix if there is one
        std::string tf_prefix = "";
        if (params_.tf_frame_prefix_enable)
        {
            if (params_.tf_frame_prefix != "")
            {
                tf_prefix = params_.tf_frame_prefix;
            }
            else
            {
                tf_prefix = std::string(get_node()->get_namespace());
            }

            // Make sure prefix does not start with '/' and always ends with '/'
            if (tf_prefix.back() != '/')
            {
                tf_prefix = tf_prefix + "/";
            }
            if (tf_prefix.front() == '/')
            {
                tf_prefix.erase(0, 1);
            }
        }

        const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
        const auto base_frame_id = tf_prefix + params_.base_frame_id;

        auto &odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.frame_id = odom_frame_id;
        odometry_message.child_frame_id = base_frame_id;

        // limit the publication on the topics /odom and /tf
        publish_rate_ = params_.publish_rate;
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

        // initialize odom values zeros
        odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

        constexpr size_t NUM_DIMENSIONS = 6;
        for (size_t index = 0; index < 6; ++index)
        {
            // 0, 7, 14, 21, 28, 35
            const size_t diagonal_index = NUM_DIMENSIONS * index + index;
            odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
            odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
        }

        // initialize transform publisher and message
        odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
            DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_odometry_transform_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                odometry_transform_publisher_);

        // keeping track of odom and base_link transforms only
        auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
        odometry_transform_message.transforms.resize(1);
        odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
        odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

        previous_update_timestamp_ = get_node()->get_clock()->now();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::on_activate(
        const rclcpp_lifecycle::State &)
    {
        const auto front_result =
            configure_drive_wheel("front", params_.front_wheel_name, registered_front_wheel_handle_);
        const auto back_right_result =
            configure_drive_wheel("right", params_.back_right_wheel_name, registered_back_right_wheel_handle_);
        const auto back_left_result =
            configure_drive_wheel("left", params_.back_left_wheel_name, registered_back_left_wheel_handle_);
        const auto x_result =
            configure_dead_wheel("x_wheel", params_.x_dead_wheel_name, registered_x_wheel_handle_);
        const auto y_result =
            configure_dead_wheel("y_wheel", params_.y_dead_wheel_name, registered_y_wheel_handle_);

        if (
            front_result == controller_interface::CallbackReturn::ERROR ||
            back_right_result == controller_interface::CallbackReturn::ERROR ||
            back_left_result == controller_interface::CallbackReturn::ERROR ||
            x_result == controller_interface::CallbackReturn::ERROR ||
            y_result == controller_interface::CallbackReturn::ERROR)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (registered_front_wheel_handle_.empty() || registered_back_right_wheel_handle_.empty() || registered_back_left_wheel_handle_.empty() || registered_x_wheel_handle_.empty() || registered_y_wheel_handle_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"),
                         "Either left wheel interfaces, right wheel interfaces are non existent");
            return controller_interface::CallbackReturn::ERROR;
        }

        is_halted = false;
        subscriber_is_active_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger("ThreeOmniwheelController"), "Subscriber and publisher are now active.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }
        registered_front_wheel_handle_.clear();
        registered_back_right_wheel_handle_.clear();
        registered_back_left_wheel_handle_.clear();
        registered_x_wheel_handle_.clear();
        registered_y_wheel_handle_.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        received_velocity_msg_ptr_.set(std::make_shared<Twist>());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::on_error(const rclcpp_lifecycle::State &)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool ThreeOmniwheelController::reset()
    {
        odometry_.resetOdometry();

        // release the old queue
        std::queue<Twist> empty;
        std::swap(previous_commands_, empty);

        registered_front_wheel_handle_.clear();
        registered_back_right_wheel_handle_.clear();
        registered_back_left_wheel_handle_.clear();
        registered_x_wheel_handle_.clear();
        registered_y_wheel_handle_.clear();

        subscriber_is_active_ = false;
        velocity_command_subscriber_.reset();
        velocity_command_unstamped_subscriber_.reset();

        received_velocity_msg_ptr_.set(nullptr);
        is_halted = false;
        return true;
    }

    void ThreeOmniwheelController::halt()
    {
        const auto halt_wheels = [](auto &wheel_handles)
        {
            for (const auto &wheel_handle : wheel_handles)
            {
                wheel_handle.velocity.get().set_value(0.0);
            }
        };

        halt_wheels(registered_front_wheel_handle_);
        halt_wheels(registered_back_right_wheel_handle_);
        halt_wheels(registered_back_left_wheel_handle_);
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::configure_dead_wheel(
        const std::string &which, const std::vector<std::string> &dead_wheel_name,
        std::vector<DeadWheelHandle> &registered_dead_wheel_handles)
    {
        auto logger = get_node()->get_logger();

        if (dead_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "No '%s' dead wheel name specified", which.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // register handles
        registered_dead_wheel_handles.reserve(dead_wheel_name.size());
        for (const auto &wheel_name : dead_wheel_name)
        {
            const auto interface_name = feedback_type();
            const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                                                   [&wheel_name, &interface_name](const auto &interface)
                                                   {
                                                       return interface.get_prefix_name() == wheel_name &&
                                                              interface.get_interface_name() == interface_name;
                                                   });

            if (state_handle == state_interfaces_.cend())
            {
                // std::vector<std::string> name = wheel_name;
                RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Unable to obtain joint state handle for %s", wheel_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            registered_dead_wheel_handles.emplace_back(
                DeadWheelHandle{std::ref(*state_handle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::configure_drive_wheel(
        const std::string &side, const std::vector<std::string> &drive_wheel_name,
        std::vector<DriveWheelHandle> &registered_drive_wheel_handles)
    {
        auto logger = get_node()->get_logger();

        if (drive_wheel_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "No '%s' dead wheel name specified", side.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // register handles
        registered_drive_wheel_handles.reserve(drive_wheel_name.size());
        for (const auto &wheel_name : drive_wheel_name)
        {
            const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                                                   [&wheel_name](const auto &interface)
                                                   {
                                                       return interface.get_prefix_name() == wheel_name &&
                                                              interface.get_interface_name() == HW_IF_VELOCITY;
                                                   });

            // if (state_handle == state_interfaces_.cend())
            // {
            //     RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Unable to obtain joint state handle for %s", wheel_name.c_str());
            //     return controller_interface::CallbackReturn::ERROR;
            // }
            if (state_handle == state_interfaces_.cend())
            {
                RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"),
                             "Unable to obtain joint state handle for %s. Available interfaces:", wheel_name.c_str());

                for (const auto &interface : state_interfaces_)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"),
                                 "  Found: %s/%s", interface.get_prefix_name().c_str(),
                                 interface.get_interface_name().c_str());
                }

                return controller_interface::CallbackReturn::ERROR;
            }

            const auto command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&wheel_name](const auto &interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == HW_IF_VELOCITY;
                });

            if (command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Unable to obtain joint command handle for %s", wheel_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            registered_drive_wheel_handles.emplace_back(
                DriveWheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeOmniwheelController::configure_imu(
        const std::string &rcl_send_response, const std::vector<std::string> &sensor_name,
        std::vector<IMUHandle> &registered_sensor_handles)
    {
        auto logger = get_node()->get_logger();

        if (sensor_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "No '%s' dead wheel name specified", rcl_send_response.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // register handles
        registered_sensor_handles.reserve(sensor_name.size());
        for (const auto &wheel_name : sensor_name)
        {
            const auto interface_name = imu_feedback_type();
            const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                                                   [&wheel_name, &interface_name](const auto &interface)
                                                   {
                                                       return interface.get_prefix_name() == wheel_name &&
                                                              interface.get_interface_name() == interface_name;
                                                   });

            if (state_handle == state_interfaces_.cend())
            {
                RCLCPP_ERROR(rclcpp::get_logger("ThreeOmniwheelController"), "Unable to obtain joint state handle for %s", wheel_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            registered_sensor_handles.emplace_back(
                IMUHandle{std::ref(*state_handle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }
} // namespace three_omniwheel_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    three_omniwheel_controller::ThreeOmniwheelController, controller_interface::ControllerInterface)