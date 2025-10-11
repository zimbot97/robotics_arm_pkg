#include <robotics_arm_control/arm_control.hpp>

namespace robotics_arm
{

RoboticsArm::RoboticsArm()
{

}


RoboticsArm::~RoboticsArm()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}

CallbackReturn RoboticsArm::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboticsArm::export_state_interfaces()
{
    return {};
}

std::vector<hardware_interface::CommandInterface> RoboticsArm::export_command_interfaces()
{
    return {};
}

//read from arduino
hardware_interface::return_type RoboticsArm::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

//write to arduino
hardware_interface::return_type RoboticsArm::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

CallbackReturn RoboticsArm::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn RoboticsArm::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

}// namespace robotics_arm



PLUGINLIB_EXPORT_CLASS(robotics_arm::RoboticsArm, hardware_interface::SystemInterface)
