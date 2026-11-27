#include <cstdio>
#include "offboard_px4_bridge.hpp"
#include <cmath>

// Constructor, a node
OffboardPX4Bridge::OffboardPX4Bridge(std::string agent_name, double force_limit) : Node("offboard_px4_bridge")
{
  // saving the parameters
  _agent_name = agent_name;
  _force_limit = force_limit;

  // initializing some vars
  _offboard_setpoint_counter = 0;
  _armed = false;
  _offboard = false;
  _failure_occured = false;

  // initializing the value as false.
  _btr_data.connection_ok = false;

  // creating the topics
  _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/" + agent_name + "/fmu/in/offboard_control_mode", 10);
  _setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/" + agent_name + "/fmu/in/trajectory_setpoint", 10);
  _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/" + agent_name + "/fmu/in/vehicle_command", 10);
  _vehicle_attitude_ctrl_pub = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/" + agent_name + "/fmu/in/vehicle_attitude_setpoint", 10);

  // getting armed flag and control mode
  _vcm_sub = this->create_subscription<px4_msgs::msg::VehicleControlMode>("/" + agent_name + "/fmu/out/vehicle_control_mode",
                  rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                  [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg_vcm)
                  {
                    _armed = msg_vcm->flag_armed;
                    _offboard = msg_vcm->flag_control_offboard_enabled;
                  });

  // subbing to the transmitter topic, so to capture an external input (change mode)
  // the sub is done on the first vehicle, for each UAV, so that one transmitter
  // is used to change the flying mode of all the UAVs.
  _rc_input_sub = this->create_subscription<px4_msgs::msg::RcChannels>("/x500_1/fmu/out/rc_channels",
                  rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                  [this](const px4_msgs::msg::RcChannels::UniquePtr msg_rc)
                  {
                    _rc_input_data = *msg_rc;
                  });



  // and the setpoint stream
  _setpoint_sub = this->create_subscription<ros2_muavp_interface::msg::InputSetpoint>("/" + agent_name + "/input_setpoint",
                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                std::bind(&OffboardPX4Bridge::setpointCallback, this, std::placeholders::_1));

  // listening to the possible failure directives
  _failure_action_sub = this->create_subscription<ros2_muavp_interface::msg::FailureAction>("/" + agent_name + "/failure_action", 1,
                        [this](const ros2_muavp_interface::msg::FailureAction::SharedPtr msg_fa)
                        {
                          _failure_action_data = *msg_fa;
                          _failure_occured = true;
                        });

  _ready_pre_takeoff_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_pre_takeoff",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            // getting the data from the topic and  saving it
            _ready_signal_pre_takeoff_data = *msg_ready;
          });

  // listening to the current task information
  _states_info_sub = this->create_subscription<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si)
          {
            // getting the data from the topic and saving it
            _states_info_data = *msg_si;
          });

  _bt_ready_sub = this->create_subscription<ros2_muavp_interface::msg::BluetoothReady>("/" + _agent_name + "/bluetooth_ready",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                  [this](const ros2_muavp_interface::msg::BluetoothReady::UniquePtr msg_btr)
                  {
                    _btr_data = *msg_btr;
                  });

  _load_cells_sub = this->create_subscription<ros2_muavp_interface::msg::LoadCellsTension>("/" + _agent_name + "/load_cells/tension",
  			  rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
  			  [this](const ros2_muavp_interface::msg::LoadCellsTension::SharedPtr msg_lc)
  			  {
  			    _load_cells_tension_data = *msg_lc;
          });


  // lambda that executes the main task: arm -> offboard
	auto _timercallback = [this]() -> void {

    // std::cout << "---------------------------------------" << std::endl;
    // std::cout << "input_channels4: " << _rc_input_data.channels[4] << std::endl;
    // std::cout << "input_channels6: " << _rc_input_data.channels[6] << std::endl; // -1.0 = Manual - 1.0 = Offboard. Could Be useful?

    CommandParameters params = CommandParameters::Zero();

    // Existing RC mapping: 0 requests Hold, +1 requests Land; -1 permits the
    // offboard branch below. Readiness still controls the original arm sequence.
    if(round(_rc_input_data.channels[4]) != -1.0)
    {
      if(round(_rc_input_data.channels[4]) == 0.0)
        params << 1, 4, 3, 0.0, 0.0, 0.0, 0.0; // Hold
      else if(round(_rc_input_data.channels[4]) == 1.0)
        params << 1, 4, 6, 0.0, 0.0, 0.0, 0.0; // Land

      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params);
    }
    else if(_setpoint_msg.out_of_mocap || _setpoint_msg.safety_kill)
    {
      params << 1, 4, 6, 0.0, 0.0, 0.0, 0.0;
      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params); // land
    }
    else if(_setpoint_msg.setpoint_type == 4)
    {
      // params << 1, 4, 3, 0.0, 0.0, 0.0, 0.0; // Hold
      params << 1, 4, 6, 0.0, 0.0, 0.0, 0.0;
      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params); // land
    }
    else if(_offboard_setpoint_counter >= 10 && !_armed)     // if the minimum number of setpoints is reached AND the vehicle is not armed yet
    {

      params << 1, 6, 0.0, 0.0, 0.0, 0.0, 0.0;
      // Change to Offboard mode after 10 setpoints
      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params);
      // Arm the vehicle
      this->arm();
    }
    // this allows each iteration to keep sending the arm command if the drone doesn't arm itself


    if(!_setpoint_msg.out_of_mocap && round(_rc_input_data.channels[4]) == -1.0 && !_setpoint_msg.safety_kill)
    {
      // offboard_control_mode needs to be paired with trajectory setpoint
      publishOffboardControlMode();
      publishSetpointRTPS();
    }

    // incrementing the counter 'til it reaches the minimum amount of iterations before going offboard
    // considering also a triggering condition, based on wether the parameters phase has been successfully completed
    int curr_state = (int)_states_info_data.curr_state;
    // bool offboard_cond = _ready_signal_pre_takeoff_data.uav_1_ready && _ready_signal_pre_takeoff_data.uav_2_ready && _ready_signal_pre_takeoff_data.uav_3_ready && curr_state > 0;

    //bool offboard_cond = _ready_signal_pre_takeoff_data.uav_2_ready && _ready_signal_pre_takeoff_data.uav_3_ready && curr_state > 0;

    bool offboard_cond = _ready_signal_pre_takeoff_data.uav_1_ready && curr_state > 0;
    // bluetooth ready and tension ready conditions.
    // allowing Offboard iff they are both verified.
    bool bt_tension_cond = (_btr_data.connection_ok); //  && _load_cells_tension_data.tension[2] != 0.0

    if(_offboard_setpoint_counter < 11 && offboard_cond) // && bt_tension_cond
      _offboard_setpoint_counter++;
	};
  // executing the callback every tot ms, when the timer expires
	_timer = this->create_wall_timer(std::chrono::milliseconds(10), _timercallback);
}



// function to send arm command to drone
void OffboardPX4Bridge::arm()
{
  CommandParameters params = CommandParameters::Zero();
  params << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, params);

	// RCLCPP_INFO(this->get_logger(), "Arm command sent");
}
// and the one to disarm it
void OffboardPX4Bridge::disarm()
{
  CommandParameters params = CommandParameters::Zero();
  params << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, params);

  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

// this function enables offboard control mode
void OffboardPX4Bridge::publishOffboardControlMode()
{
  // the allowed modalities are specified below
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;


  if(_setpoint_msg.setpoint_type == 0)
  {
    msg.position = true;
  	msg.acceleration = true;
  	msg.attitude = false;
  	msg.acceleration = false;
    msg.thrust_and_torque = false;
  }
  else if(_setpoint_msg.setpoint_type == 1)
  {
    msg.position = true;
  	msg.velocity = false;
  	msg.attitude = false;
  	msg.acceleration = false;
    msg.thrust_and_torque = false;
  }
  else if(_setpoint_msg.setpoint_type == 2)
  {
    msg.position = false;
  	msg.velocity = false;
  	msg.attitude = false;
  	msg.acceleration = false;
    msg.thrust_and_torque = true;
  }
  else if(_setpoint_msg.setpoint_type == 3)
  {
    msg.position = false;
  	msg.velocity = true;
  	msg.attitude = false;
  	msg.acceleration = true;
    msg.thrust_and_torque = false;
  }
  else if(_setpoint_msg.setpoint_type == 5)
  {
    msg.position = false;
  	msg.velocity = false;
    msg.attitude = true;
  	msg.acceleration = false;
    msg.thrust_and_torque = false;
  }

  msg.body_rate = false;
  msg.direct_actuator = false;


  // and the message is sent on the topic
  _offboard_control_mode_pub->publish(msg);
}

// this function pubishes the setpoints coming from the control algorithm
// in the RTPS topic
void OffboardPX4Bridge::publishSetpointRTPS()
{

  // LOCAL X = EAST
  // LOCAL Y = NORTH
  // the check on msg type is needed also to avoid the "failed agent" to give
  // actuator setpoints as the other agents, but keep giving position ones instead
  if(_setpoint_msg.setpoint_type == 0)
  {

    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.position[0] = _setpoint_msg.setpoint[0];
    msg.position[1] = _setpoint_msg.setpoint[1];
    msg.position[2] = 0.0;
    msg.acceleration[0] = 0.0;
    msg.acceleration[1] = 0.0;
    msg.acceleration[2] = _setpoint_msg.setpoint[2];
    msg.yaw = _setpoint_msg.setpoint[3];

    _setpoint_publisher->publish(msg);
  }
  else if(_setpoint_msg.setpoint_type == 1)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.position[0] = _setpoint_msg.setpoint[0];
    msg.position[1] = _setpoint_msg.setpoint[1];
    msg.position[2] = _setpoint_msg.setpoint[2];
    msg.acceleration[0] = 0.0;
    msg.acceleration[1] = 0.0;
    msg.acceleration[2] = 0.0;

    msg.yaw = _setpoint_msg.setpoint[3];

    _setpoint_publisher->publish(msg);
  }
  else if(_setpoint_msg.setpoint_type == 3)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position[0] = NAN;
    msg.position[1] = NAN;
    msg.position[2] = NAN;
    msg.velocity[0] = _setpoint_msg.setpoint[0];
    msg.velocity[1] = _setpoint_msg.setpoint[1];
    msg.velocity[2] = 0.0;
    msg.acceleration[0] = 0.0;
    msg.acceleration[1] = 0.0;
    msg.acceleration[2] = _setpoint_msg.setpoint[2];
    // std::cout << "input acc "<< _setpoint_msg.setpoint[2] << std::endl;

    msg.yaw = _setpoint_msg.setpoint[3];

    _setpoint_publisher->publish(msg);
  }
  else if(_setpoint_msg.setpoint_type == 5)
  {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // msg.roll_body = _setpoint_msg.setpoint[0];
    // msg.pitch_body = _setpoint_msg.setpoint[1];
    // msg.yaw_body = _setpoint_msg.setpoint[3];

    msg.thrust_body[0] = 0.0;
    msg.thrust_body[1] = 0.0;
    msg.thrust_body[2] = std::clamp((double)-_setpoint_msg.thrust_setpoint, -_force_limit, 0.0)/_force_limit;

    msg.q_d[0] = _setpoint_msg.q_setpoint[0];
    msg.q_d[1] = _setpoint_msg.q_setpoint[1];
    msg.q_d[2] = _setpoint_msg.q_setpoint[2];
    msg.q_d[3] = _setpoint_msg.q_setpoint[3];

    // std::cout << "input thrust "<< msg.thrust_body[2] << std::endl;

    // std::cout << "input q_0 "<< msg.q_d[0] << std::endl;
    // std::cout << "input q_1 "<< msg.q_d[1] << std::endl;
    // std::cout << "input q_2 "<< msg.q_d[2] << std::endl;
    // std::cout << "input q_3 "<< msg.q_d[3] << std::endl;

    _vehicle_attitude_ctrl_pub->publish(msg);
  }

}


// this function is used to arm/disarm the vehicle, depending on the input
void OffboardPX4Bridge::publishVehicleCommand(uint16_t command, const CommandParameters &params)
{
  // filling the message accordingly
	px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  if(params(0) != 0.0)
	  msg.param1 = params(0);
  if(params(1) != 0.0)
 	  msg.param2 = params(1);
  if(params(2) != 0.0)
 	  msg.param3 = params(2);
  if(params(3) != 0.0)
    msg.param4 = params(3);
  if(params(4) != 0.0)
 	  msg.param5 = params(4);
  if(params(5) != 0.0)
 	  msg.param6 = params(5);
  if(params(6) != 0.0)
    msg.param7 = params(6);

	msg.command = command;
  if(_agent_name == "x500_1")
  {
  	msg.target_system = 1;
  }
  else if(_agent_name == "x500_2")
  {
  	msg.target_system = 2;
  }
  else if(_agent_name == "x500_3")
  {
  	msg.target_system = 3;
  }
  msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
  // and publishing it
	_vehicle_command_pub->publish(msg);
}


void OffboardPX4Bridge::setpointCallback(const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp)
{
  const bool safety_kill = _setpoint_msg.safety_kill || msg_sp->safety_kill;
  const bool out_of_mocap = _setpoint_msg.out_of_mocap || msg_sp->out_of_mocap;
  _setpoint_msg = *msg_sp;
  // Regular control messages must not erase an earlier safety LAND request.
  _setpoint_msg.safety_kill = safety_kill;
  _setpoint_msg.out_of_mocap = out_of_mocap;
  _setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
}
