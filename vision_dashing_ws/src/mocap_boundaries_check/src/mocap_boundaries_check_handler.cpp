#include "mocap_boundaries_check_handler.hpp"

MoCapBoundariesHandler::MoCapBoundariesHandler(std::string agent_name) : Node("mocap_boundaries_check_handler")
{

  _agent_name = agent_name;
  if(_agent_name == "x500_1")
  {
    _prev_agent_name = "x500_3";
    _next_agent_name = "x500_2";
  }
  else if(_agent_name == "x500_2")
  {
    _prev_agent_name = "x500_1";
    _next_agent_name = "x500_3";
  }
  else if(_agent_name == "x500_3")
  {
    _prev_agent_name = "x500_2";
    _next_agent_name = "x500_1";
  }



  // listening to the current odometry information
  _odom_uav_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/" + _agent_name + "/fmu/out/vehicle_odometry", // /VehicleOdometry_PubSubTopic
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom)
          {
            _odom_uav_data = *msg_odom;

            /*if(_agent_name == "x500_1")
            {
              _odom_uav_data.position[0] += 0.0;
              _odom_uav_data.position[1] += -1.695;
            }
            else if(_agent_name == "x500_2")
            {
              _odom_uav_data.position[0] += -1.4684;
              _odom_uav_data.position[1] += 0.85;
            }
            else if(_agent_name == "x500_3")
            {
              _odom_uav_data.position[0] += 1.4684;
              _odom_uav_data.position[1] += 0.85;
            }

            _odom_uav_data.position[2] += -0.21;*/

          });



  _setpoint_uav_pub = this->create_publisher<ros2_muavp_interface::msg::InputSetpoint>("/" + _agent_name + "/input_setpoint", 1);

  // subbing to the other UAVs topics so to get their 'MoCap out of boudaries' flag
  _setpoint_prev_uav_sub = this->create_subscription<ros2_muavp_interface::msg::InputSetpoint>("/" + _prev_agent_name + "/input_setpoint",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp)
          {
            _setpoint_prev_uav_msg = *msg_sp;
          });

  _setpoint_next_uav_sub = this->create_subscription<ros2_muavp_interface::msg::InputSetpoint>("/" + _next_agent_name + "/input_setpoint",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp)
          {
            _setpoint_next_uav_msg = *msg_sp;
          });





  // SIMULATION ONLY
  _states_info_sub = this->create_subscription<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si)
          {
            // getting the data from the topic and saving it
            _states_info_data = *msg_si;
          });

  // init
  _states_info_data.curr_state = 0;


  // Cache the fixed inset polygon once; runtime checks use the same geometry.
  Point2D center = Point2D::Zero();
  for (const auto& vertex : _octagonVertices)
  {
      center += vertex;
  }
  center /= _octagonVertices.size();

  double scalingFactor = 1.0 - (_edge_proximity_threshold / (_octagonVertices[0] - center).norm());

  _inner_vertices.reserve(_octagonVertices.size());
  for (const auto& vertex : _octagonVertices)
  {
      _inner_vertices.push_back(center + (vertex - center) * scalingFactor);
  }


  // lambda that executes the main task
	auto _timercallback = [this]() -> void 
  {

    // THIS CHECK IS JUST IN SIMULATION
    // if((int)_states_info_data.curr_state != 0 && (int)_states_info_data.curr_state != 1)
    // {
      // checking data and taking decisions for each UAV based on its current position
      bool stop_uav = false;
      // converting from NED to XYZ(up)
      Point3D uav_position(_odom_uav_data.position[1], _odom_uav_data.position[0], -_odom_uav_data.position[2]);
      Point3D closest_point_uav = checkMoCapBoundaries(uav_position, stop_uav);

      // std::cout << _odom_uav_data.z << std::endl;
      // if anyone of the UAVs exceeds the limits imposed by the MoCap, handle it
      if(stop_uav || _setpoint_prev_uav_msg.out_of_mocap || _setpoint_next_uav_msg.out_of_mocap)
      {
        sendChangeModeUAVs(closest_point_uav);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Out of MoCap boundaries: requesting LAND");

      }

      // checking for the safety kill signal as well
      if(_setpoint_prev_uav_msg.safety_kill || _setpoint_next_uav_msg.safety_kill)
      {
        sendSafetyKillUAVs();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Safety kill: requesting LAND");

      }

    // }
	};
  // executing the callback every tot ms, when the timer expires
	_timer = this->create_wall_timer(std::chrono::milliseconds(20), _timercallback);
}


Point3D MoCapBoundariesHandler::checkMoCapBoundaries(const Point3D& testPoint, bool& stop_uav)
{
  Point2D projectedTestPoint(testPoint.x(), testPoint.y());

  // Check if the test point is inside the 2D projection of the octagon
  bool insideXY = isPointInPolygon(_octagonVertices, projectedTestPoint);
  bool insideZ = (testPoint.z() <= _z_max);
  bool isInside = insideXY && insideZ;

  // std::cout << "insideXY: " << insideXY << std::endl;
  // std::cout << "insideZ: " << insideZ << std::endl;
  // std::cout << "x: " << testPoint.x() << std::endl;
  // std::cout << "y: " << testPoint.y() << std::endl;
  // Check proximity to edges
  bool isNearEdge = false;
  if (isInside)
  {
      for (size_t i = 0; i < _octagonVertices.size(); ++i)
      {
          Point2D p1 = _octagonVertices[i];
          Point2D p2 = _octagonVertices[(i + 1) % _octagonVertices.size()];
          double distanceToEdge = computeDistanceToEdge(projectedTestPoint, p1, p2);
          if (distanceToEdge <= _edge_proximity_threshold)
          {
              isNearEdge = true;
              break;
          }
      }
  }

  bool isNearEdgeZ = (testPoint.z() >= (_z_max - _edge_proximity_threshold)) && (testPoint.z() <= _z_max);

  // std::cout << "isNearEdge: " << isNearEdge << std::endl;
  // std::cout << "isNearEdgeZ: " << isNearEdgeZ << std::endl;
  // std::cout << "-----------------------------------" << std::endl;
  // Compute inner polygon vertices
  const auto &innerVertices = _inner_vertices; // Fixed geometry, computed at construction.
  double innerZ = _z_max - _edge_proximity_threshold;
  Point3D closestPoint = testPoint;

  if (!isInside || isNearEdge)
  {
      double minDist = numeric_limits<double>::infinity();
      for (size_t i = 0; i < innerVertices.size(); ++i)
      {
          Point2D p1 = innerVertices[i];
          Point2D p2 = innerVertices[(i + 1) % innerVertices.size()];
          double distanceToEdge = computeDistanceToEdge(projectedTestPoint, p1, p2);
          if (distanceToEdge < minDist)
          {
              minDist = distanceToEdge;
              closestPoint = Point3D(p1.x(), p1.y(), innerZ);
          }
      }
  }
  else if (isNearEdgeZ)
  {
      closestPoint.z() = innerZ;
  }

  if(!isInside || isNearEdge || isNearEdgeZ)
  {
    stop_uav = true;
  }

  return closestPoint;
}


// Checks if the point is inside the 2D projection of the polygon using ray-casting
bool MoCapBoundariesHandler::isPointInPolygon(const vector<Point2D>& vertices, const Point2D& point)
{
    if (vertices.size() < 3) return false;
    int n = static_cast<int>(vertices.size());
    bool inside = false;
    Point2D p1 = vertices[0];

    for (int i = 1; i <= n; ++i) {
        Point2D p2 = vertices[i % n];
        if (point.y() > min(p1.y(), p2.y())) {
            if (point.y() <= max(p1.y(), p2.y()) && point.x() <= max(p1.x(), p2.x())) {
                double xinters = (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
                if (p1.x() == p2.x() || point.x() <= xinters) {
                    inside = !inside;
                }
            }
        }
        p1 = p2;
    }
    return inside;
}

// Function to compute closest point on an edge to the test point
double MoCapBoundariesHandler::computeDistanceToEdge(const Point2D& point, const Point2D& p1, const Point2D& p2)
{
    Vector2d edgeVec = p2 - p1;
    Vector2d pointVec = point - p1;
    if (edgeVec.squaredNorm() == 0.0) return pointVec.norm();
    double projLength = pointVec.dot(edgeVec) / edgeVec.squaredNorm();

    Point2D closestPoint;
    if (projLength < 0) {
        closestPoint = p1;
    } else if (projLength > 1) {
        closestPoint = p2;
    } else {
        closestPoint = p1 + projLength * edgeVec;
    }
    return (point - closestPoint).norm();
}


void MoCapBoundariesHandler::sendChangeModeUAVs(const Point3D& closest_point_uav)
{
  ros2_muavp_interface::msg::InputSetpoint uav_sp;

  // transforming back to NED
  uav_sp.setpoint[0] = closest_point_uav(1);
  uav_sp.setpoint[1] = closest_point_uav(0);
  uav_sp.setpoint[2] = -closest_point_uav(2);
  uav_sp.out_of_mocap = true;
  uav_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  // each UAV's offboard_rtps node will handle its own mode change
  _setpoint_uav_pub->publish(uav_sp);
}



void MoCapBoundariesHandler::sendSafetyKillUAVs()
{
  ros2_muavp_interface::msg::InputSetpoint uav_sp;

  uav_sp.safety_kill = true;
  uav_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  // each UAV's offboard_rtps node will handle its own mode change
  _setpoint_uav_pub->publish(uav_sp);
}



MoCapBoundariesHandler::~MoCapBoundariesHandler()
{
}
