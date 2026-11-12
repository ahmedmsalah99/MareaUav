#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <cstdlib>
#include <ctime>


bool synchronized = false;
bool global_position_set = false;
bool vehicle_cmd_ack_received = false;
std::string agent_name = "";
px4_msgs::msg::SensorGps sensor_gps_data;
px4_msgs::msg::VehicleCommandAck last_vehicle_cmd_ack;


void synchronizeSystemData()
{
    const uint64_t gps_time_usec = sensor_gps_data.time_utc_usec;

    if(gps_time_usec == 0)
    {
        std::cout << "Invalid GPS UTC time." << std::endl;
        return;
    }

    timespec ts;
    ts.tv_sec = static_cast<time_t>(gps_time_usec / 1000000ULL);
    ts.tv_nsec = static_cast<long>((gps_time_usec % 1000000ULL) * 1000ULL);

    if(clock_settime(CLOCK_REALTIME, &ts) != 0)
    {
        std::cout << "clock_settime failed: " << std::strerror(errno) << std::endl;
        return;
    }

    synchronized = true;

    std::cout << "System time updated from GPS: " << gps_time_usec << " usec" << std::endl;
}


void set_global_position_origin(uint64_t timestamp, rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub)
{
  // filling the message accordingly, setting the global position origin
	px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = timestamp;
  msg.param5 = 44.40353187618028;
  msg.param6 = 8.958392568821875;
  msg.param7 = 0.0;

	msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
  if(agent_name == "x500_1")
  {
  	msg.target_system = 1;
  }
  else if(agent_name == "x500_2")
  {
  	msg.target_system = 2;
  }
  else if(agent_name == "x500_3")
  {
  	msg.target_system = 3;
  }
  msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
  msg.confirmation = true;
	msg.from_external = true;
  // and publishing it
	vehicle_command_pub->publish(msg);
}

void checkLastCommandAck(rclcpp::Node::SharedPtr node)
{
  if (!vehicle_cmd_ack_received)
  {
    RCLCPP_WARN(node->get_logger(), "No VehicleCommandAck received yet");
    return;
  }

  if (last_vehicle_cmd_ack.command ==
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN)
  {
    switch (last_vehicle_cmd_ack.result)
    {
      case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
        RCLCPP_INFO(node->get_logger(), "SET_GPS_GLOBAL_ORIGIN accepted");
        global_position_set = true;
        break;

      case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
        RCLCPP_WARN(node->get_logger(), "SET_GPS_GLOBAL_ORIGIN temporarily rejected");
        break;

      case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
        RCLCPP_ERROR(node->get_logger(), "SET_GPS_GLOBAL_ORIGIN denied");
        break;

      case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
        RCLCPP_ERROR(node->get_logger(), "SET_GPS_GLOBAL_ORIGIN unsupported");
        break;

      case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
        RCLCPP_ERROR(node->get_logger(), "SET_GPS_GLOBAL_ORIGIN failed");
        break;

      default:
        RCLCPP_WARN(node->get_logger(), "SET_GPS_GLOBAL_ORIGIN unknown result=%u",
                    last_vehicle_cmd_ack.result);
        break;
    }
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments.size() < 2 || (arguments[1] != "x500_1" && arguments[1] != "x500_2" && arguments[1] != "x500_3"))
  {
    RCLCPP_ERROR(rclcpp::get_logger("time_synchro_gps_sys"), "Expected agent x500_1, x500_2 or x500_3");
    rclcpp::shutdown();
    return 1;
  }
  agent_name = arguments[1];

  std::cout << "New registered Time Synchro GPS-System Node with Agent: " << agent_name << std::endl;

  auto node = std::make_shared<rclcpp::Node>("time_synchro_gps_sys");


  auto sensor_gps_sub = node->create_subscription<px4_msgs::msg::SensorGps>("/" + agent_name + "/fmu/out/sensor_gps",
        rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                [node](const px4_msgs::msg::SensorGps::UniquePtr msg_sg)
                {
                  sensor_gps_data = *msg_sg;
                  // if the timestamp has been retrieved from the GPS,
                  // and I didn't synchronize yet, I do it.
                  if(msg_sg->time_utc_usec != 0 && !synchronized)
                    synchronizeSystemData();

                });
  auto vehicle_command_ack_sub = node->create_subscription<px4_msgs::msg::VehicleCommandAck>("/" + agent_name + "/fmu/out/vehicle_command_ack",
      rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                [node](const px4_msgs::msg::VehicleCommandAck::UniquePtr msg_ack)
                {
                  last_vehicle_cmd_ack = *msg_ack;
                  vehicle_cmd_ack_received = true;
                });

  auto vehicle_command_pub = node->create_publisher<px4_msgs::msg::VehicleCommand>("/" + agent_name + "/fmu/in/vehicle_command", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && (!synchronized || !global_position_set))
  {
      if(!global_position_set)
      {
        // spamming the command so it is set for sure.
        const uint64_t timestamp = node->get_clock()->now().nanoseconds() / 1000;
        set_global_position_origin(timestamp, vehicle_command_pub);
        checkLastCommandAck(node);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      executor.spin_some(); // Process incoming messages without recreating an executor
  }



  rclcpp::shutdown();
  return 0;
}
