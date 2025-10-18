#include <robotics_arm_control/arm_control.hpp>

namespace robotics_arm
{

RoboticsArm::RoboticsArm() : baudrate(115200), serial_port_(io_service_), start_ros_(false)
{
}


RoboticsArm::~RoboticsArm()
{
    if (serial_port_.is_open())
    {
        serial_port_.close();
    }
}

CallbackReturn RoboticsArm::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Serial Port %s", port_.c_str());
        baudrate = std::stoi(info_.hardware_parameters.at("baudrate"));
    }
    catch (const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "No Serial Port provided! Aborting");
        return CallbackReturn::FAILURE;
    }
    

    position_commands_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size());
    prev_position_commands_.resize(info_.joints.size());

    for (size_t i = 0; i < position_states_.size(); i++) {
        position_states_[i] = 1.57;
        position_commands_[i] = 1.57;
        prev_position_commands_[i] = 1.57;
        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "position_states_[%zu] = %f", i, position_states_[i]) ;
    }
    position_states_[5] = 0.8;
    position_commands_[5] = 0.8;
    prev_position_commands_[5] = 0.8;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboticsArm::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Provide only a position Interafce
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Joint %s : %f", info_.joints[i].name.c_str(), position_states_[i]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboticsArm::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Provide only a position Interafce
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        RCLCPP_INFO(rclcpp::get_logger("export_command_interfaces"), "Joint %s : %f", info_.joints[i].name.c_str(), position_states_[i]);
    }

    return command_interfaces;
}

//read from arduino
hardware_interface::return_type RoboticsArm::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    //Open Loop, current setup no encoder feedback
    position_states_ = position_commands_;
    return hardware_interface::return_type::OK;
}

//write to arduino
hardware_interface::return_type RoboticsArm::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (position_commands_ == prev_position_commands_)
    {
        // Nothing changed, do not send any command
        return hardware_interface::return_type::OK;
    }
    RCLCPP_INFO(rclcpp::get_logger("RoboticsArm::write"), "position :{%f, %f, %f, %f, %f, %f}", 
        position_commands_[0], position_commands_[1], position_commands_[2], 
        position_commands_[3], position_commands_[4], position_commands_[5] );

    std::string msg;
    int joint1 = static_cast<int>(position_commands_[0]  * 180 / M_PI);
    msg.append(std::to_string(joint1));
    msg.append(",");

    int joint2 = static_cast<int>(position_commands_[1]  * 180 / M_PI);
    msg.append(std::to_string(joint2));
    msg.append(",");


    int joint3 = static_cast<int>(position_commands_[2]  * 180 / M_PI);
    msg.append(std::to_string(joint3));
    msg.append(",");

    int joint4 = static_cast<int>(position_commands_[3]  * 180 / M_PI);
    msg.append(std::to_string(joint4));
    msg.append(",");

    int joint5 = static_cast<int>(position_commands_[4]  * 180 / M_PI);
    msg.append(std::to_string(joint5));
    msg.append(",");

    int gripper = static_cast<int>(position_commands_[5]  * 180 / M_PI);
    msg.append(std::to_string(gripper));
    msg.append("\n");

    boost::system::error_code ec;
    boost::asio::write(serial_port_, boost::asio::buffer(msg), ec);
    if (ec) {
        RCLCPP_WARN(rclcpp::get_logger("RoboticsArm"), "Failed to send");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Sent : %s", msg.c_str());
    }
    prev_position_commands_ = position_commands_;
    return hardware_interface::return_type::OK;
}

CallbackReturn RoboticsArm::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Starting robot hardware ...");

    boost::system::error_code ec;
    serial_port_.open(port_, ec);
    
    if (ec) {
        RCLCPP_WARN(rclcpp::get_logger("RoboticsArm"), "Failed to connect to serial port ");
        return hardware_interface::CallbackReturn::FAILURE;
    } else {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
            static_cast<uint32_t>(baudrate)));
        serial_port_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Successfully connected to serial port ");
        start_ros_ = true;
        serial_thread_ = std::thread(&RoboticsArm::serial_read_loop, this);
        return CallbackReturn::SUCCESS;
    }
}

CallbackReturn RoboticsArm::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    if (serial_port_.is_open()) 
    {
        try
        {
            serial_port_.close();
            RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Successfully connected to serial port %s",
                    port_.c_str());
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoboticsArm"),
                          "Something went wrong while closing connection with port " << port_);

        }
    } 
    else 
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("RoboticsArm"),
                          "Something went wrong while closing connection with port " << port_);
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");

    if (serial_thread_.joinable())
    {
        start_ros_ = false;
        serial_thread_.join();
        RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Serial thread stopped.");
    }
    return CallbackReturn::SUCCESS;
}

void RoboticsArm::serial_read_loop()
{
    RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Serial read loop started.");
    boost::asio::streambuf buf;
    boost::system::error_code ec;

    while (start_ros_)
    {
        try
        {
            boost::asio::streambuf buf;
            boost::system::error_code ec;

            // Block until newline is received
            size_t n = boost::asio::read_until(serial_port_, buf, '\n', ec);

            if (ec)
            {
                RCLCPP_WARN(rclcpp::get_logger("RoboticsArm"), "Serial read error: %s", ec.message().c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            std::istream is(&buf);
            std::string line;
            std::getline(is, line);

            if (!line.empty())
            {
                RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Serial received: %s", line.c_str());
            }

            buf.consume(n); // clear processed data
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RoboticsArm"), "Exception in serial thread: %s", e.what());
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("RoboticsArm"), "Serial read loop exiting.");
}

}// namespace robotics_arm



PLUGINLIB_EXPORT_CLASS(robotics_arm::RoboticsArm, hardware_interface::SystemInterface)
