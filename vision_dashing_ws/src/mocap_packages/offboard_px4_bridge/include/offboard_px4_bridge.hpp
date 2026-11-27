#pragma once
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include "ros2_muavp_interface/msg/load_cells_tension.hpp"
#include "ros2_muavp_interface/msg/bluetooth_ready.hpp"
#include "ros2_muavp_interface/msg/input_setpoint.hpp"
#include "ros2_muavp_interface/msg/failure_action.hpp"
#include "ros2_muavp_interface/msg/ready_signals.hpp"
#include "ros2_muavp_interface/msg/states_info.hpp"
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
// #include "udp_utils.hpp"

#include <chrono>
#include <Eigen/Dense>

using CommandParameters = Eigen::Matrix<double, 7, 1>;

class OffboardPX4Bridge : public rclcpp::Node
{
  public:
    OffboardPX4Bridge(std::string agent_name, double force_limit);

    void arm();
    void disarm();

  private:

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _start;
    rclcpp::Clock _clocksis;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr _vehicle_attitude_ctrl_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _setpoint_publisher;

    rclcpp::Subscription<ros2_muavp_interface::msg::BluetoothReady>::SharedPtr _bt_ready_sub; /* BLUETOOTH */
    rclcpp::Subscription<ros2_muavp_interface::msg::LoadCellsTension>::SharedPtr _load_cells_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _vcm_sub;
    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rc_input_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::FailureAction>::SharedPtr _failure_action_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_pre_takeoff_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_sub;


    px4_msgs::msg::RcChannels _rc_input_data;
    ros2_muavp_interface::msg::InputSetpoint _setpoint_msg;
    ros2_muavp_interface::msg::FailureAction _failure_action_data;
    ros2_muavp_interface::msg::ReadySignals _ready_signal_pre_takeoff_data;
    ros2_muavp_interface::msg::StatesInfo _states_info_data;
    ros2_muavp_interface::msg::BluetoothReady _btr_data; /* BLUETOOTH */
    ros2_muavp_interface::msg::LoadCellsTension _load_cells_tension_data;


    std::string _agent_name = "";
    double _force_limit = 0.0;

    bool _armed;
    bool _offboard;
    bool _failure_occured;

    //counter for the number of setpoints sent
    uint64_t _offboard_setpoint_counter;

    void setpointCallback(const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp);

    void publishOffboardControlMode();
    void publishSetpointRTPS();
    void publishVehicleCommand(uint16_t command, const CommandParameters &params);

};
