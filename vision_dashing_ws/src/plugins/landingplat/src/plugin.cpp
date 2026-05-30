#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <fstream>
#include <mutex>

// ROS 2 (gazebo_ros) — replaces the old LCM transport.
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "WaveGen.h"

#define VEL 0.1

#define CRUISE_VEL 0.8

#define MAX_DIST 15

#define DIST_VEL_THRE 5

namespace gazebo
{

    // Plain holder for the pose data the plugin consumes. Replaces the old
    // lcm geometry::pose so the motion logic below is left untouched.
    struct PoseData
    {
        double position[3]    {0, 0, 0};
        double velocity[3]    {0, 0, 0};
        double orientation[4] {1, 0, 0, 0};
        double yaw            {0};
    };

    class PlatformManager : public ModelPlugin
    {

    private:

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;

        std::string _cmd_pose_topic;
        std::string _local_pose_topic;

        // ROS 2 node + endpoints (gazebo_ros spins the node on its own thread).
        gazebo_ros::Node::SharedPtr _ros_node;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pose_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _cmd_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _drone_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _nav_sub;

        // Guards the data written by ROS callbacks and read in OnUpdate.
        std::mutex _data_mutex;

        PoseData _position_sp;
        PoseData _drone_pos;
        PoseData _nav_pos;

        double _actual_sim_time;
        double _prev_sim_time;
        bool _got_t_offs;
        double _time_offs;
        double _dt;
        double _platVel;
        double _dir;
        WaveGen wave;

        bool _rescueTarget;

        bool _test_rosbag;

        std::fstream myfile;

        ignition::math::Pose3d _pose, _start_pose, _prev_pose;

        bool _first_rb;
        double _nav_pos_0;

        void manageTime(const common::UpdateInfo & _info){

            double t = (double)_info.simTime.sec +  (double)_info.simTime.nsec / (double)_info.simTime.nsInSec;;
            if(!_got_t_offs){
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

        ~PlatformManager()
        {
          myfile.close();
        }

    public:

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {


                myfile.open("/home/data/final_sim_data.txt", std::ios::out);
                myfile << "x,y,z\n";
                // Store the pointer to the model
                this->_model = _parent;
                _model.get()->SetStatic(true);
                _model.get()->SetGravityMode(false);
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&PlatformManager::OnUpdate, this, _1));

                _local_pose_topic = _model.get()->GetName() + "/pose";
                _cmd_pose_topic   = _model.get()->GetName() + "/pose_cmd";

                // Grab the gazebo_ros node tied to this plugin's SDF.
                _ros_node = gazebo_ros::Node::Get(_sdf, "platform_manager");

                auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
                auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

                _pose_pub = _ros_node->create_publisher<nav_msgs::msg::Odometry>(
                    _local_pose_topic, reliable_qos);

                _cmd_sub = _ros_node->create_subscription<nav_msgs::msg::Odometry>(
                    _cmd_pose_topic, reliable_qos,
                    std::bind(&PlatformManager::positionSetpointCallback, this, std::placeholders::_1));

                _drone_sub = _ros_node->create_subscription<px4_msgs::msg::VehicleOdometry>(
                    "vision_position_estimate", px4_qos,
                    std::bind(&PlatformManager::positionDroneCallback, this, std::placeholders::_1));

                _nav_sub = _ros_node->create_subscription<nav_msgs::msg::Odometry>(
                    "nav_pose", reliable_qos,
                    std::bind(&PlatformManager::navPoseCallback, this, std::placeholders::_1));

                wave.init(10,2.0,0.5,7.0);

                _pose = _model.get()->WorldPose();
                _prev_pose  = _pose;
                _start_pose = _pose;

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;

                _rescueTarget = false;

                _test_rosbag = true;

                _nav_pos.position[0] = 0;
                _nav_pos.position[1] = 0;

                _nav_pos.orientation[0] = 1;
                _nav_pos.orientation[1] = 0;
                _nav_pos.orientation[2] = 0;
                _nav_pos.orientation[3] = 0;

                RCLCPP_INFO(_ros_node->get_logger(), "New platform registered, topics:");
                RCLCPP_INFO(_ros_node->get_logger(), "  %s", _local_pose_topic.c_str());
                RCLCPP_INFO(_ros_node->get_logger(), "  %s", _cmd_pose_topic.c_str());
                wave.info();
                _platVel = VEL;
                _dir = 1;
                _first_rb = true;
                RCLCPP_INFO(_ros_node->get_logger(), "PlatformManager LOADED");
            }
            void invertDirection(){

                _dir = -_dir;

            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & _info) {


                manageTime(_info);

                // Snapshot subscriber-fed data under the lock.
                std::unique_lock<std::mutex> lk(_data_mutex);

                _pose = _model.get()->WorldPose();
                // _pose.rot = ignition::math::Quaternion(1,0,0,0);
                // _pose.pos.z = wave.generateWaveHeightAtPoint(_pose.Pos().x,_pose.Pos().y,_actual_sim_time,0) + 2;

                double xTemp = _pose.Pos()[0];

                double yaw_des = 0.0;

                if(_test_rosbag)
                {
                  ignition::math::Quaternion<double> orientation;

                  orientation.W(_nav_pos.orientation[0]);
                  orientation.X(_nav_pos.orientation[1]);
                  orientation.Y(_nav_pos.orientation[2]);
                  orientation.Z(_nav_pos.orientation[3]);

                  if(_actual_sim_time <= 77.0)
                    _pose.Set(ignition::math::Vector3<double>(_nav_pos.position[0], _nav_pos.position[1], wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0)/2.0 + 2), orientation);
                  else
                  {
                    // if(_first_rb)
                    // {
                    //   _first_rb = false;
                    //   _nav_pos_0 = _nav_pos.position[0];
                    // }
                    // _nav_pos_0+=_dt*(0.40);
                    // _pose.Set(ignition::math::Vector3<double>(_nav_pos_0, _nav_pos.position[1], wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), orientation);
                    _pose.Set(ignition::math::Vector3<double>(_nav_pos.position[0], _nav_pos.position[1], wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), orientation);
                  }

                  myfile << std::to_string(_pose.Pos()[0]) + "," + std::to_string(_pose.Pos()[1]) + "," + std::to_string(_pose.Pos()[2]) + "\n";
                }
                else if(_rescueTarget)
                {
                  std::cout << "Target Identified!" << std::endl;
                  std::cout << "LONGITUDE: " << _position_sp.position[0] << std::endl;
                  std::cout << "LATITUDE: " << _position_sp.position[1] << std::endl;

                  double dist = sqrt(pow((_pose.Pos()[0] - _drone_pos.position[0]),2) + pow((_pose.Pos()[1] - _drone_pos.position[1]),2));

                  if(dist < DIST_VEL_THRE)
                    _platVel = 0.0;
                  else
                    _platVel = CRUISE_VEL;


                  double yTemp = _pose.Pos()[1];

                  double x_err = _position_sp.position[0] - xTemp;
                  double y_err = _position_sp.position[1] - yTemp;

                  double dirX = (x_err > 0) ? 1 : ((x_err < 0) ? -1 : 0);
                  double dirY = (y_err > 0) ? 1 : ((y_err < 0) ? -1 : 0);

                  double newX = xTemp + dirX * (_dt * _platVel);
                  double newY = yTemp + dirY * (_dt * _platVel);

                  ignition::math::Quaternion<double> orientation;

                  orientation.W(_position_sp.orientation[0]);
                  orientation.X(_position_sp.orientation[1]);
                  orientation.Y(_position_sp.orientation[2]);
                  orientation.Z(_position_sp.orientation[3]);

                  _pose.Set(ignition::math::Vector3<double>(newX, newY, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), orientation);

                  yaw_des = _position_sp.yaw;

                  // std::cout << "YAW: " << yaw_des << std::endl;
                }
                else
                {

                  if(fabs(- xTemp + _start_pose.Pos()[0]) > ((MAX_DIST - 0.01))) invertDirection();
                  double newX = xTemp + _dir * (_dt * _platVel);

                  //_pose.pos.x = newX;

                  _pose.Set(ignition::math::Vector3<double>(0, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));
                  // _pose.Set(ignition::math::Vector3<double>(0, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));

                  yaw_des = 1.57;
                }

                // Done reading shared subscriber data.
                lk.unlock();

                _model.get()->SetAngularVel(ignition::math::Vector3<double>(0,0,0));
                _model.get()->SetWorldPose(_pose);
                //Prepare pose message
                double vx,vy,vz;
                nav_msgs::msg::Odometry plat_pose;

                vx = (_pose.Pos()[0] - _prev_pose.Pos()[0])/_dt;
                vy = (_pose.Pos()[1] - _prev_pose.Pos()[1])/_dt;
                vz = (_pose.Pos()[2] - _prev_pose.Pos()[2])/_dt;

                plat_pose.header.stamp = _ros_node->now();
                plat_pose.header.frame_id = "Landing_Site";

                // giving an error to the quadrotor to trigger inspection phase
                plat_pose.pose.pose.position.x = _pose.Pos()[0]; //x - 5.7
                plat_pose.pose.pose.position.y = _pose.Pos()[1]; //y + 3.5
                plat_pose.pose.pose.position.z = _pose.Pos()[2]; //z

                plat_pose.twist.twist.linear.x = vx;
                plat_pose.twist.twist.linear.y = vy;
                plat_pose.twist.twist.linear.z = vz;

                plat_pose.pose.pose.orientation.w = _pose.Rot().W(); // w
                plat_pose.pose.pose.orientation.x = _pose.Rot().X(); // x
                plat_pose.pose.pose.orientation.y = _pose.Rot().Y(); // y
                plat_pose.pose.pose.orientation.z = _pose.Rot().Z(); // z

                _pose_pub->publish(plat_pose);

                _prev_pose = _pose;
            }

            void positionSetpointCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(_data_mutex);

                _position_sp.position[0] = (float)msg->pose.pose.position.x;
                _position_sp.position[1] = (float)msg->pose.pose.position.y;
                _position_sp.position[2] = (float)msg->pose.pose.position.z;

                _position_sp.velocity[0] = (float)msg->twist.twist.linear.x;
                _position_sp.velocity[1] = (float)msg->twist.twist.linear.y;
                _position_sp.velocity[2] = (float)msg->twist.twist.linear.z;

                _position_sp.orientation[0] = (float)msg->pose.pose.orientation.w;
                _position_sp.orientation[1] = (float)msg->pose.pose.orientation.x;
                _position_sp.orientation[2] = (float)msg->pose.pose.orientation.y;
                _position_sp.orientation[3] = (float)msg->pose.pose.orientation.z;

                ignition::math::Quaterniond q(
                    _position_sp.orientation[0], _position_sp.orientation[1],
                    _position_sp.orientation[2], _position_sp.orientation[3]);
                _position_sp.yaw = yawFromQuaternion(q);

                // A pose command means a rescue target has been identified.
                _rescueTarget = true;
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

            void navPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(_data_mutex);

                _nav_pos.position[0] = msg->pose.pose.position.x;
                _nav_pos.position[1] = msg->pose.pose.position.y;
                _nav_pos.position[2] = msg->pose.pose.position.z;

                _nav_pos.velocity[0] = msg->twist.twist.linear.x;
                _nav_pos.velocity[1] = msg->twist.twist.linear.y;
                _nav_pos.velocity[2] = msg->twist.twist.linear.z;

                _nav_pos.orientation[0] = msg->pose.pose.orientation.w;
                _nav_pos.orientation[1] = msg->pose.pose.orientation.x;
                _nav_pos.orientation[2] = msg->pose.pose.orientation.y;
                _nav_pos.orientation[3] = msg->pose.pose.orientation.z;
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager);
}
