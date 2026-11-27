#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <cstdio>
#include "offboard_px4_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  // (void) argc;
  // (void) argv;
  std::string agent_name = "";
  std::string file_path = "";
  double force_limit;
  if(arguments.size() > 2 && !arguments[1].empty())
  {
    agent_name = arguments[1];
    std::size_t consumed = 0;
    force_limit = std::stod(arguments[2], &consumed);
    if (consumed != arguments[2].size() || !std::isfinite(force_limit) || force_limit <= 0.0)
      throw std::runtime_error("Maximum thrust must be a finite positive value in newtons");
  }
  else
  {
    std::cout << "No Agent Name provided for Offboard-PX4 Bridge Node, stopping" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  
  rclcpp::spin(std::make_shared<OffboardPX4Bridge>(agent_name, force_limit));

  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("startup"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
