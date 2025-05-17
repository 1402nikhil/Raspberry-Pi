#ifndef THREE_WHEEL__THREEWHEEL_SYSTEM_HPP_
#define THREE_WHEEL__THREEWHEEL_SYSTEM_HPP_

// #include <vector>

// #include "/home/nikhil/Work/ROS/controller_ws/src/three_omniwheel_controller/include/three_omniwheel_controller/three_omniwheel_controller.hpp"
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
#include "drive_wheel.hpp"
#include "dead_wheel.hpp"

namespace three_wheel
{
    class ThreeWheelHardware : public hardware_interface::SystemInterface
    {
        struct DriveWheelConfig
        {
            std::string front_wheel_name = "";
            std::string back_right_wheel_name = "";
            std::string back_left_wheel_name = "";
        };
        struct DeadWheelConfig
        {
            std::string x_dead_wheel_name = "";
            std::string y_dead_wheel_name = "";
            int enc_counts_per_rev = 0;
            float pid_p = 0;
            float pid_d = 0;
            float pid_i = 0;
            float pid_o = 0;
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ThreeWheelHardware)

        THREE_WHEEL_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        THREE_WHEEL_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        THREE_WHEEL_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        THREE_WHEEL_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        THREE_WHEEL_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        THREE_WHEEL_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        THREE_WHEEL_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        THREE_WHEEL_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        THREE_WHEEL_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        DriveWheelConfig drive_cfg_;
        DeadWheelConfig dead_cfg_;
        DriveWheel wheel_f_;
        DriveWheel wheel_b_r_;
        DriveWheel wheel_b_l_;
        DeadWheel wheel_x_;
        DeadWheel wheel_y_;

        rclcpp::Node::SharedPtr hardware_node_;  //node that communicates with the micrcontroller

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr encoder_readings_sub_; // convert it to a custom msg for encoder data
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr check_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_values_pub_;

        std_msgs::msg::Float32MultiArray::SharedPtr encoder_readings_;
        std::thread spin_thread_;                   // Thread for spinning the ROS node
        std::atomic<bool> stop_spin_thread_{false}; // Flag to stop spinning safely
        bool micro_ros_active_ = false;
        bool check_msg_ = false;
    };
} // namespace three_wheel

#endif