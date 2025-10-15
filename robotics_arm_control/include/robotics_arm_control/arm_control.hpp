#ifndef ARM_CONTROL_HPP
#define ARM_CONTROL_HPP

#include <iostream>
#include <fcntl.h>      // open()
#include <termios.h>    // termios, tcgetattr(), tcsetattr()
#include <unistd.h>     // read(), write(), close()
#include <cstring>      // memset()
#include <boost/asio.hpp>

#include "pluginlib/class_list_macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace robotics_arm
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RoboticsArm : public hardware_interface::SystemInterface
    {
        public:
            RoboticsArm();
            ~ RoboticsArm();

            // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            // Implementing hardware_interface::SystemInterface
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::string port_;
            int baudrate;
            std::vector<double> position_commands_;
            std::vector<double> prev_position_commands_;
            std::vector<double> position_states_;
            boost::asio::io_service io_service_;
            boost::asio::serial_port serial_port_;
    };

}

#endif