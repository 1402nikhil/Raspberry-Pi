#include "three_wheel/threewheel_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace three_wheel
{
    hardware_interface::CallbackReturn ThreeWheelHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {

        // Create a ROS node for subscribing to encoder data
        hardware_node_ = std::make_shared<rclcpp::Node>("three_wheel_hardware");
        // node = std::make_shared<rclcpp::Node>("three_wheel_hardware_checker");     //change the node name

        encoder_readings_sub_ = hardware_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_encoders", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                encoder_readings_ = msg;
                micro_ros_active_ = true;
            });

        wheel_vel_pub_ = hardware_node_->create_publisher<std_msgs::msg::Float32MultiArray>("/cmd_vel", 10);

        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        //loks for these hardware parameters in the ros2 xacro file
        drive_cfg_.front_wheel_name = info_.hardware_parameters["front_wheel_name"];
        drive_cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
        drive_cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];

        dead_cfg_.x_dead_wheel_name = info_.hardware_parameters["x_dead_wheel_name"];
        dead_cfg_.y_dead_wheel_name = info_.hardware_parameters["y_dead_wheel_name"];
        dead_cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            dead_cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            dead_cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            dead_cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            dead_cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "PID values not supplied, using defaults.");
        }

        wheel_f_.setup(drive_cfg_.front_wheel_name);
        wheel_b_r_.setup(drive_cfg_.back_right_wheel_name);
        wheel_b_l_.setup(drive_cfg_.back_left_wheel_name);

        wheel_x_.setup(dead_cfg_.x_dead_wheel_name, dead_cfg_.enc_counts_per_rev);
        wheel_y_.setup(dead_cfg_.y_dead_wheel_name, dead_cfg_.enc_counts_per_rev);

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // Identify if this is a drive wheel or a dead wheel
            bool is_drive_wheel = (joint.name == drive_cfg_.front_wheel_name ||
                                   joint.name == drive_cfg_.back_right_wheel_name ||
                                   joint.name == drive_cfg_.back_left_wheel_name);

            bool is_dead_wheel = (joint.name == dead_cfg_.x_dead_wheel_name ||
                                  joint.name == dead_cfg_.y_dead_wheel_name);

            // Drive wheels should have only velocity command interface
            if (is_drive_wheel)
            {
                if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("ThreeWheelHardware"),
                                 "Drive Joint '%s' must have exactly 1 command interface of type 'velocity'.",
                                 joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                if (joint.command_interfaces.size() == 1)
                {
                    RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"),
                                "is drive wheel true");
                }
                if (joint.state_interfaces.size() != 2)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("ThreeWheelHardware"),
                        "Drive Wheel Joint '%s' must have 2 state interfaces (position, velocity). Found %zu.",
                        joint.name.c_str(), joint.state_interfaces.size());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("ThreeWheelHardware"),
                        "Drive Wheel Joint '%s' must have position as the first state interface.",
                        joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("ThreeWheelHardware"),
                        "Drive Wheel Joint '%s' must have velocity as the second state interface.",
                        joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                if (joint.state_interfaces.size() == 2)
                {
                    RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"),
                                "is Drive wheel true");
                }
            }

            // Dead wheels (encoders) should have position + velocity feedback
            if (is_dead_wheel)
            {
                if (joint.state_interfaces.size() != 2)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("ThreeWheelHardware"),
                        "Dead Wheel Joint '%s' must have 2 state interfaces (position, velocity). Found %zu.",
                        joint.name.c_str(), joint.state_interfaces.size());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("ThreeWheelHardware"),
                        "Dead Wheel Joint '%s' must have position as the first state interface.",
                        joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("ThreeWheelHardware"),
                        "Dead Wheel Joint '%s' must have velocity as the second state interface.",
                        joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                if (joint.state_interfaces.size() == 2)
                {
                    RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"),
                                "is dead wheel true");
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"),
                    "on init successful");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ThreeWheelHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        //state interface of dead wheel
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_x_.name, hardware_interface::HW_IF_POSITION, &wheel_x_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_x_.name, hardware_interface::HW_IF_VELOCITY, &wheel_x_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_y_.name, hardware_interface::HW_IF_POSITION, &wheel_y_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_y_.name, hardware_interface::HW_IF_VELOCITY, &wheel_y_.vel));

        //state interface of drive wheel
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_f_.name, hardware_interface::HW_IF_POSITION, &wheel_f_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_b_r_.name, hardware_interface::HW_IF_POSITION, &wheel_b_r_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_b_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_r_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_b_l_.name, hardware_interface::HW_IF_POSITION, &wheel_b_l_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_b_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_l_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ThreeWheelHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        //command interface of drive wheel
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_b_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_r_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_b_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_l_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ThreeWheelHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Configuring ...please wait...");

        // Check if the micro-ROS node is alive by listening to a topic or service

        auto topic_name = "/communication_check"; // Change to an actual topic published by the microcontroller
        check_sub_ = hardware_node_->create_subscription<std_msgs::msg::Bool>(
            topic_name, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                check_msg_ = msg->data; // Update check_msg_ when a message is received
            });

        // Start spinning the node in a separate thread
        stop_spin_thread_ = false;
        spin_thread_ = std::thread([this]()
                                   {
                rclcpp::Rate rate(100);  // Adjust the loop rate as needed
                while (rclcpp::ok() && !stop_spin_thread_) {
                    rclcpp::spin_some(hardware_node_);
                    rate.sleep();
                } });

        // // Wait up to 2 seconds for the first message
        // auto start_time = std::chrono::steady_clock::now();
        // while (!check_msg_ && (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))) {
        //     rclcpp::sleep_for(std::chrono::milliseconds(100));  // Allow time for message reception
        // }

        // Give some time for checking the status
        rclcpp::sleep_for(std::chrono::seconds(2));

        // if (check_msg_)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node is active.");
        // }
        // else
        // {
        //     RCLCPP_WARN(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node not responding.");
        // }
        // while(!check_msg_){
        //     RCLCPP_WARN(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node not responding.");
        //     rclcpp::sleep_for(std::chrono::seconds(2));
        // }

        if (check_msg_)
        {
            RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node is active.");
        }

        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeWheelHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Cleaning up ...please wait...");

        // Reset ROS communication handles (example: publisher/subscriber)
        encoder_readings_sub_.reset();
        wheel_vel_pub_.reset(); // change according to the name of publisher
        check_sub_.reset();
        pid_values_pub_.reset();
        // sensor_feedback_subscription_.reset();  //change according to the name of subscriber     //not defined

        // Stop the spin thread safely
        stop_spin_thread_ = true;
        if (spin_thread_.joinable())
        {
            spin_thread_.join(); // Wait for the thread to finish
        }

        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeWheelHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Activating ...please wait...");

        rclcpp::sleep_for(std::chrono::seconds(2));

        // if (!check_msg_) {          //uncomment
        //     RCLCPP_ERROR(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node not responding.");
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // while (!check_msg_) {
        //     RCLCPP_ERROR(rclcpp::get_logger("ThreeWheelHardware"), "on_activate: Micro-ROS node not responding.");
        //     rclcpp::sleep_for(std::chrono::seconds(2));
        // }

        // RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node is active.");

        // Send PID values if configured
        if (dead_cfg_.pid_p > 0)
        {
            pid_values_pub_ = hardware_node_->create_publisher<std_msgs::msg::Float32MultiArray>("/set_pid", 10);
            auto pid_msg = std_msgs::msg::Float32MultiArray();
            pid_msg.data = {dead_cfg_.pid_p, dead_cfg_.pid_d, dead_cfg_.pid_i, dead_cfg_.pid_o};
            pid_values_pub_->publish(pid_msg);
            RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "PID values sent.");
        }

        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeWheelHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        stop_spin_thread_ = true;
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }

        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ThreeWheelHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // // Ensure the micro-ROS node is active before reading
        // if (!check_msg_) {       //uncomment
        //     RCLCPP_ERROR(rclcpp::get_logger("ThreeWheelHardware"), "Micro-ROS node not responding.");
        //     return hardware_interface::return_type::ERROR;
        // }

        if (!encoder_readings_)
        {
            // RCLCPP_WARN(rclcpp::get_logger(""), "encoder_readings_ is not initialized yet!");
            // return;
        }
        else if (encoder_readings_->data.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger(""), "Received empty Float32MultiArray!");
            // return;
        }
        else
        {
            wheel_x_.enc = encoder_readings_->data[0]; // x encoder data
            wheel_y_.enc = encoder_readings_->data[1]; // y encoder data
        }

        double delta_seconds = period.seconds();

        double pos_prev = wheel_x_.pos;
        wheel_x_.pos = wheel_x_.calc_enc_angle();
        wheel_x_.vel = (wheel_x_.pos - pos_prev) / delta_seconds;

        pos_prev = wheel_y_.pos;
        wheel_y_.pos = wheel_y_.calc_enc_angle();
        wheel_y_.vel = (wheel_y_.pos - pos_prev) / delta_seconds;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ThreeWheelHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // if (!check_msg_)    //uncomment
        // {
        //     return hardware_interface::return_type::ERROR;
        // }

        auto wheel_vel_ = std::make_unique<std_msgs::msg::Float32MultiArray>();

        // double left_wheel_vel = wheel_l_.cmd / wheel_l_.rads_per_count;
        // double right_wheel_vel = wheel_r_.cmd / wheel_r_.rads_per_count;
        float front_wheel_vel = wheel_f_.cmd;
        float back_right_wheel_vel = wheel_b_r_.cmd;
        float back_left_wheel_vel = wheel_b_l_.cmd;

        // Convert wheel velocities to linear and angular velocity
        wheel_vel_->data = {front_wheel_vel, back_right_wheel_vel, back_left_wheel_vel};

        wheel_vel_pub_->publish(std::move(wheel_vel_));

        RCLCPP_INFO(rclcpp::get_logger("ThreeWheelHardware"), "%f , %f, %f", front_wheel_vel, back_right_wheel_vel, back_left_wheel_vel);

        return hardware_interface::return_type::OK;
    }
} // namespace three_wheel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(three_wheel::ThreeWheelHardware, hardware_interface::SystemInterface) // registers ThreeWheelHardware as a plugin under the hardware_interface::SystemInterface category,
                                                                                             //  it makes it discoverable to three_wheel.xml