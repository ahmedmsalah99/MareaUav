#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <mutex>
#include <string>

// ROS 2 (gazebo_ros) — replaces the old LCM transport.
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

namespace gazebo
{

    // Plain holders replacing the old lcm geometry::pose / exec::gps_status so
    // the physics logic below is left untouched.
    struct PoseData
    {
        double position[3]    {0, 0, 0};
        double velocity[3]    {0, 0, 0};
        double orientation[4] {1, 0, 0, 0};
        double yaw            {0};
    };

    struct GpsData
    {
        double lon {0};
        double lat {0};
        double alt {0};
        int    status {0};
    };

    class PayloadManager : public ModelPlugin
    {

    private:

        double _gravity;
        double _x_offset;
        double _y_offset;
        double _z_offset;
        double _home_alt;

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;

        std::string _cmd_pose_topic;
        std::string _local_pose_topic;

        // ROS 2 node + endpoints (gazebo_ros spins the node on its own thread).
        gazebo_ros::Node::SharedPtr _ros_node;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _rel_pay_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _drone_sub;
        rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr _gps_sub;

        // Guards the data written by ROS callbacks and read in OnUpdate.
        std::mutex _data_mutex;

        PoseData _drone_pos;
        std::string _releasePayload;
        GpsData _drone_glob_pos;

        double _startingFallZ;

        double _actual_sim_time;
        double _prev_sim_time;
        bool _got_t_offs;
        double _time_offs;
        double _dt;

        double _startingFallTime;

        bool _releasingPayload;
        bool _payloadReleased;
        bool _delivered;

        bool _first;

        ignition::math::Pose3d _pose;

        void manageTime(const common::UpdateInfo & _info)
        {

            double t = (double)_info.simTime.sec +  (double)_info.simTime.nsec / (double)_info.simTime.nsInSec;
            if(!_got_t_offs)
            {
                _time_offs = t;
                _got_t_offs = true;
            }

            _actual_sim_time = - _time_offs + t;
            _dt = _actual_sim_time - _prev_sim_time;
            _prev_sim_time = _actual_sim_time;

        }

        double yawFromQuaternion(ignition::math::Quaterniond& q)
        {
            return atan2(2.0 * (q.W() * q.Z() + q.X() * q.Y()), 1.0 - 2.0 * (q.Y() * q.Y() + q.Z() * q.Z()));
        }

    public:

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
                // Store the pointer to the model
                this->_model = _parent;
                _model.get()->SetStatic(true);
                _model.get()->SetGravityMode(false);
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&PayloadManager::OnUpdate, this, _1));

                _local_pose_topic = _model.get()->GetName() + "/pose";
                _cmd_pose_topic   = _model.get()->GetName() + "/pose_cmd";

                // Grab the gazebo_ros node tied to this plugin's SDF.
                // Pass an explicit, unique node name — otherwise gazebo_ros derives
                // it from the SDF <plugin name="plugin"> attribute, which both this
                // and the landing-platform plugin share, causing a "/plugin" clash.
                _ros_node = gazebo_ros::Node::Get(_sdf, "payload_manager");

                auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
                auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

                _rel_pay_sub = _ros_node->create_subscription<std_msgs::msg::String>(
                    "release_payload_feedback", reliable_qos,
                    std::bind(&PayloadManager::relpayloadCallback, this, std::placeholders::_1));

                _drone_sub = _ros_node->create_subscription<px4_msgs::msg::VehicleOdometry>(
                    "vision_position_estimate", px4_qos,
                    std::bind(&PayloadManager::positionDroneCallback, this, std::placeholders::_1));

                _gps_sub = _ros_node->create_subscription<px4_msgs::msg::SensorGps>(
                    "/fmu/out/vehicle_gps_position", px4_qos,
                    std::bind(&PayloadManager::globalPosDroneCallback, this, std::placeholders::_1));

                _pose = _model.get()->WorldPose();

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;

                _startingFallZ = 0.0;

                _startingFallTime = 0.0;

                _releasingPayload = false;
                _payloadReleased = false;
                _delivered = false;

                _gravity = 9.81;
                _x_offset = 0.0;
                _y_offset = -0.13;
                _z_offset = -0.07;
                _home_alt = 488.0;

                _first = true;
                RCLCPP_INFO(_ros_node->get_logger(), "New payload registered, topics:");
                RCLCPP_INFO(_ros_node->get_logger(), "  %s", _local_pose_topic.c_str());
                RCLCPP_INFO(_ros_node->get_logger(), "  %s", _cmd_pose_topic.c_str());

            }


            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & _info)
            {

                manageTime(_info);

                // Snapshot subscriber-fed data under the lock.
                std::unique_lock<std::mutex> lk(_data_mutex);

                if(_releasePayload == "W")
                  _releasingPayload = true;
                else if(_releasePayload == "R")
                  _payloadReleased = true;

                _pose = _model.get()->WorldPose();

                // computing new payload position, depending on drone position and orientation.

                double newX = 0;
                double newY = 0;
                double newZ = 0;

                // if the payload has been released, the payload needs to stay where it is
                if (_payloadReleased && _delivered)
                {
                  newX = _pose.Pos()[0];
                  newY = _pose.Pos()[1];
                  newZ = _pose.Pos()[2];
                }
                else if(_releasingPayload)
                {
                  // if the payload needs to be released, we proceed to change its position according to Newton laws.
                  if(_first)
                  {
                    _startingFallZ = (_drone_glob_pos.alt - _home_alt) + _z_offset;
                    _startingFallTime = _actual_sim_time;
                    _first = false;
                  }
                  // in this case, offsets need to be updated wrt drone orientation.
                  newX = _drone_pos.position[0] + (cosf(_drone_pos.yaw) * _x_offset - sinf(_drone_pos.yaw) * _y_offset);
                  newY = _drone_pos.position[1] + (sinf(_drone_pos.yaw) * _x_offset + cosf(_drone_pos.yaw) * _y_offset);
                  // computing time since the start of the fall
                  double time = _actual_sim_time - _startingFallTime;
                  // free fall motion
                  newZ = _startingFallZ - _gravity*(pow(time,2))/2.0;
                  // saturation
                  if(newZ < 0.05)
                  {
                    newZ = 0.05;
                    // this var is used to allow the payload to fall correctly, cause the "Released"
                    // signal arrives before the payload can reach the ground, and the first if would
                    // be true while the payload is in mid-air.
                    _delivered = true;
                  }
                }
                else
                {
                  // in this case,the payload is not yet spawned, so it stays away.
                  newX = 5000;
                  newY = 5000;
                  newZ = -5000;
                }

                // Done reading shared subscriber data.
                lk.unlock();

                _pose.Set(ignition::math::Vector3<double>(newX, newY, newZ), ignition::math::Quaternion<double>(1,0,0,0));

                _model.get()->SetAngularVel(ignition::math::Vector3<double>(0,0,0));
                _model.get()->SetWorldPose(_pose);

                // _prev_pose = _pose;

            }

            void relpayloadCallback(const std_msgs::msg::String::SharedPtr msg)
            {
              std::lock_guard<std::mutex> lk(_data_mutex);
              _releasePayload = msg->data;
            }

            void positionDroneCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(_data_mutex);

                _drone_pos.position[0] = (float)msg->position[0];
                _drone_pos.position[1] = (float)msg->position[1];
                _drone_pos.position[2] = (float)msg->position[2];

                _drone_pos.velocity[0] = (float)msg->velocity[0];
                _drone_pos.velocity[1] = (float)msg->velocity[1];
                _drone_pos.velocity[2] = (float)msg->velocity[2];

                _drone_pos.orientation[0] = (float)msg->q[0];
                _drone_pos.orientation[1] = (float)msg->q[1];
                _drone_pos.orientation[2] = (float)msg->q[2];
                _drone_pos.orientation[3] = (float)msg->q[3];

                ignition::math::Quaterniond q(
                    _drone_pos.orientation[0], _drone_pos.orientation[1],
                    _drone_pos.orientation[2], _drone_pos.orientation[3]);
                _drone_pos.yaw = yawFromQuaternion(q);
            }

            void globalPosDroneCallback(const px4_msgs::msg::SensorGps::SharedPtr msg)
            {
              std::lock_guard<std::mutex> lk(_data_mutex);
              _drone_glob_pos.lon = 1.0e-7 * msg->longitude_deg;
              _drone_glob_pos.lat = 1.0e-7 * msg->latitude_deg;
              // Same scaling the old bridge used when filling exec::gps_status.alt.
              _drone_glob_pos.alt = 1.0e-3 * msg->altitude_msl_m;
              _drone_glob_pos.status = msg->fix_type;
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PayloadManager);
}
