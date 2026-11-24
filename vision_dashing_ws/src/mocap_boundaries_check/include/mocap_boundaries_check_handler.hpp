#ifndef MOCAPBOUNDARIESHANDLER_HPP
#define MOCAPBOUNDARIESHANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "mocap_boundaries_check/msg/input_setpoint.hpp"

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
  rclcpp::Subscription<mocap_boundaries_check::msg::InputSetpoint>::SharedPtr _setpoint_prev_uav_sub;
  rclcpp::Subscription<mocap_boundaries_check::msg::InputSetpoint>::SharedPtr _setpoint_next_uav_sub;

  rclcpp::Publisher<mocap_boundaries_check::msg::InputSetpoint>::SharedPtr _setpoint_uav_pub;


  px4_msgs::msg::VehicleOdometry _odom_uav_data;
  mocap_boundaries_check::msg::InputSetpoint _setpoint_prev_uav_msg;
  mocap_boundaries_check::msg::InputSetpoint _setpoint_next_uav_msg;


  rclcpp::TimerBase::SharedPtr _timer;

  // MoCap boundaries, vertices positions (ENU, from config/mocap_boundaries.yaml)
  vector<Point2D> _octagonVertices;

  vector<Point2D> _inner_vertices;

  double _z_max;
  double _edge_proximity_threshold;


  Point3D checkMoCapBoundaries(const Point3D& testPoint, bool& stop_uav);
  bool isPointInPolygon(const vector<Point2D>& vertices, const Point2D& point);
  double computeDistanceToEdge(const Point2D& point, const Point2D& p1, const Point2D& p2);

  void sendChangeModeUAVs(const Point3D& closest_point_uav);
  void sendSafetyKillUAVs();
};

#endif // MOCAPBOUNDARIESHANDLER_HPP
