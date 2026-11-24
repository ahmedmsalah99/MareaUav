#ifndef MOCAPBOUNDARIESHANDLER_HPP
#define MOCAPBOUNDARIESHANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"
#include "ros2_muavp_interface/msg/ready_signals.hpp"

#include "ros2_muavp_interface/msg/states_info.hpp" // SIMULATION ONLY

#include <Eigen/Dense>
#include <vector>
#include <limits>

using namespace std;
using namespace Eigen;

typedef Vector3d Point3D;
typedef Vector2d Point2D;

class MoCapBoundariesHandler : public rclcpp::Node
{
public:

    MoCapBoundariesHandler(std::string agent_name);
    ~MoCapBoundariesHandler();

private:

  std::string _agent_name = "";
  std::string _prev_agent_name = "";
  std::string _next_agent_name = "";

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_uav_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_prev_uav_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_next_uav_sub;

  rclcpp::Subscription<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_sub; // SIMULATION ONLY

  rclcpp::Publisher<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_uav_pub;


  px4_msgs::msg::VehicleOdometry _odom_uav_data;
  ros2_muavp_interface::msg::InputSetpoint _setpoint_prev_uav_msg;
  ros2_muavp_interface::msg::InputSetpoint _setpoint_next_uav_msg;

  ros2_muavp_interface::msg::StatesInfo _states_info_data; // SIMULATION ONLY


  rclcpp::TimerBase::SharedPtr _timer;

  // MoCap boundaries, vertices positions
  // TODO: configuration file
  vector<Point2D> _octagonVertices =
  {
    {-0.7911, -2.031},
    {-1.378, -0.838},
    {-1.532, 0.574},
    {-1.284, 1.881},
    {-0.184, 2.429},
    {1.021, 2.035},
    {1.470, 0.834},
    {1.379, -0.805},
    {0.952, -2.026}
  };

  vector<Point2D> _inner_vertices;

  double _z_max = 2.9;
  // double _edge_proximity_threshold = 0.15;
  double _edge_proximity_threshold = 0.05;


  Point3D checkMoCapBoundaries(const Point3D& testPoint, bool& stop_uav);
  bool isPointInPolygon(const vector<Point2D>& vertices, const Point2D& point);
  double computeDistanceToEdge(const Point2D& point, const Point2D& p1, const Point2D& p2);

  void sendChangeModeUAVs(const Point3D& closest_point_uav);
  void sendSafetyKillUAVs();
};

#endif // MOCAPBOUNDARIESHANDLER_HPP
