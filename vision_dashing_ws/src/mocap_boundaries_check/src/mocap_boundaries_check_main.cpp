#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "mocap_boundaries_check_handler.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::string agent_name = "";
  if(arguments.size() > 1 && !arguments[1].empty())
  {
    agent_name = arguments[1];
    // std::cout << "New registered MoCap Boundaries Check Node with Agent: " << agent_name << std::endl;
  }
  else
  {
    std::cout << "No Agent Name provided for MoCap Boundaries Check Node, stopping" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // std::cout << "New registered MoCap Boundaries Check Node" << std::endl;

  
  rclcpp::spin(std::make_shared<MoCapBoundariesHandler>(agent_name));

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
