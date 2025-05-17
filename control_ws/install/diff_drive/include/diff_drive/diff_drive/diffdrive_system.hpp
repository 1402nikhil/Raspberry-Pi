#ifndef DIFF_DRIVE__DIFFDRIVE_SYSTEM_HPP_
#define DIFF_DRIVE__DIFFDRIVE_SYSTEM_HPP_

// #include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "visibility_control.h"
#include "wheel.hpp"

namespace diff_drive
{
    class DiffDriveHardware : public hardware_interface::SystemInterface
    {
        struct Config
        {
            std::string left_wheel_name = "";
            std::string right_wheel_name = "";
            int enc_counts_per_rev = 0;
            float wheel_separation = 0;
            float pid_p = 0;
            float pid_d = 0;
            float pid_i = 0;
            float pid_o = 0;
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

        DIFF_DRIVE_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        DIFF_DRIVE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        DIFF_DRIVE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        DIFF_DRIVE_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        DIFF_DRIVE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        DIFF_DRIVE_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        DIFF_DRIVE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        DIFF_DRIVE_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        DIFF_DRIVE_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        Config cfg_;
        Wheel wheel_l_;
        Wheel wheel_r_;

        rclcpp::Node::SharedPtr hardware_node_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr encoder_readings_sub_; // convert it to a custom msg for encoder data
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr check_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_values_pub_;

        std_msgs::msg::Float32MultiArray::SharedPtr encoder_readings_;
        std::thread spin_thread_;                   // Thread for spinning the ROS node
        std::atomic<bool> stop_spin_thread_{false}; // Flag to stop spinning safely
        bool micro_ros_active_ = false;
        bool check_msg_ = false;
    };
} // namespace diff_drive

#endif