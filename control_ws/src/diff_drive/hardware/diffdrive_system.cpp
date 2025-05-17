#include "/home/piros/Work/ROS2/control_ws/src/diff_drive/hardware/include/diff_drive/diffdrive_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diff_drive
{
    hardware_interface::CallbackReturn DiffDriveHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {

        // Create a ROS node for subscribing to encoder data
        hardware_node_ = std::make_shared<rclcpp::Node>("Diff_drive_hardware");
        // node = std::make_shared<rclcpp::Node>("diff_drive_hardware_checker");     //change the node name

        encoder_readings_sub_ = hardware_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_encoders", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                encoder_readings_ = msg;
                micro_ros_active_ = true;
            });

        wheel_vel_pub_ = hardware_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        cfg_.wheel_separation = std::stof(info_.hardware_parameters["wheel_separation"]);

        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            cfg_.pid_d = std::stod(info_.hardware_parameters["pid_d"]);
            cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "PID values not supplied, using defaults.");
        }

        wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // diff_drive has exactly 2 states and 1 commands interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("DiffDriveHardware"),
                             "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                             joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(rclcpp::get_logger("DiiDriveHardware"),
                             "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriveHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriveHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriveHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),
                    "on init successful");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffDriveHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffDriveHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Configuring ...please wait...");

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

        if (check_msg_)
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Micro-ROS node is active.");
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), "Micro-ROS node not responding.");
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Cleaning up ...please wait...");

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

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Activating ...please wait...");

        rclcpp::sleep_for(std::chrono::seconds(2));

        // if (!check_msg_) {          //uncomment
        //     RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Micro-ROS node not responding.");
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Micro-ROS node is active.");

        // Send PID values if configured
        if (cfg_.pid_p > 0)
        {
            pid_values_pub_ = hardware_node_->create_publisher<std_msgs::msg::Float32MultiArray>("/set_pid", 10);
            auto pid_msg = std_msgs::msg::Float32MultiArray();
            pid_msg.data = {cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o};
            pid_values_pub_->publish(pid_msg);
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "PID values sent.");
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        stop_spin_thread_ = true;
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffDriveHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // // Ensure the micro-ROS node is active before reading
        // if (!check_msg_) {       //uncomment
        //     RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Micro-ROS node not responding.");
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
            wheel_l_.enc = encoder_readings_->data[0];
            wheel_r_.enc = encoder_readings_->data[1];
        }

        double delta_seconds = period.seconds();

        double pos_prev = wheel_l_.pos;
        wheel_l_.pos = wheel_l_.calc_enc_angle();
        wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

        pos_prev = wheel_r_.pos;
        wheel_r_.pos = wheel_r_.calc_enc_angle();
        wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffDriveHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // if (!check_msg_)    //uncomment
        // {
        //     return hardware_interface::return_type::ERROR;
        // }

        auto wheel_vel_ = std::make_unique<geometry_msgs::msg::Twist>();

        // double left_wheel_vel = wheel_l_.cmd / wheel_l_.rads_per_count;
        // double right_wheel_vel = wheel_r_.cmd / wheel_r_.rads_per_count;
        double left_wheel_vel = wheel_l_.cmd;
        double right_wheel_vel = wheel_r_.cmd;

        // Convert wheel velocities to linear and angular velocity
        wheel_vel_->linear.x = (left_wheel_vel + right_wheel_vel) / 2.0;
        wheel_vel_->angular.z = (right_wheel_vel - left_wheel_vel) / cfg_.wheel_separation;

        wheel_vel_pub_->publish(std::move(wheel_vel_));

        // RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "%f , %f, %f", left_wheel_vel, wheel_l_.cmd, wheel_l_.rads_per_count);

        return hardware_interface::return_type::OK;
    }
} // namespace diff_drive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_drive::DiffDriveHardware, hardware_interface::SystemInterface) // registers DiffDriveHardware as a plugin under the hardware_interface::SystemInterface category,
                                                                                           //  it makes it discoverable to diif_drive.xml
