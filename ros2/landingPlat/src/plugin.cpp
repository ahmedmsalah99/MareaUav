#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "WaveGen.h"

#define VEL 0.1

#define CRUISE_VEL 0.8

#define MAX_DIST 15

#define DIST_VEL_THRE 5

static constexpr double G = 9.81;

namespace gazebo
{

    class PlatformManager : public ModelPlugin
    {

    private:

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;

        gazebo_ros::Node::SharedPtr _ros_node;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

        double _actual_sim_time;
        double _prev_sim_time;
        bool _got_t_offs;
        double _time_offs;
        double _dt;
        double _platVel;
        double _dir;
        WaveGen wave;

        bool _rescueTarget;

        ignition::math::Pose3d _pose, _start_pose,_prev_pose;

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

    public:

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                // Store the pointer to the model
                this->_model = _parent;
                _model.get()->SetStatic(true);
                _model.get()->SetGravityMode(false);
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&PlatformManager::OnUpdate, this, _1));

                _ros_node = gazebo_ros::Node::Get(_sdf);
                _imu_pub  = _ros_node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

                wave.init(10,2.0,0.5,7.0);

                _pose = _model.get()->WorldPose();
                _prev_pose  = _pose;
                _start_pose = _pose;

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;

                _rescueTarget = false;

                std::cout << "New platform registered." << std::endl;
                std::cout << "Publishing IMU on: imu/data" << std::endl;
                wave.info();
                _platVel = VEL;
                _dir = 1;

            }
            void invertDirection(){

                _dir = -_dir;

            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & _info) {

                manageTime(_info);

                _pose = _model.get()->WorldPose();

                double xTemp = _pose.Pos()[0];

                double yaw_des = 0.0;

                if(_rescueTarget)
                {
                  double yTemp = _pose.Pos()[1];

                  double x_err = _position_sp_x - xTemp;
                  double y_err = _position_sp_y - yTemp;

                  double dist = sqrt(x_err * x_err + y_err * y_err);

                  if(dist < DIST_VEL_THRE)
                    _platVel = 0.0;
                  else
                    _platVel = CRUISE_VEL;

                  double dirX = (x_err > 0) ? 1 : ((x_err < 0) ? -1 : 0);
                  double dirY = (y_err > 0) ? 1 : ((y_err < 0) ? -1 : 0);

                  double newX = xTemp + dirX * (_dt * _platVel);
                  double newY = yTemp + dirY * (_dt * _platVel);

                  _pose.Set(ignition::math::Vector3<double>(newX, newY, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));
                }
                else
                {

                  if(fabs(- xTemp + _start_pose.Pos()[0]) > ((MAX_DIST - 0.01))) invertDirection();
                  double newX = xTemp + _dir * (_dt * _platVel);

                  //_pose.pos.x = newX;

                  _pose.Set(ignition::math::Vector3<double>(0, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));

                  yaw_des = 1.57;
                }

                _model.get()->SetAngularVel(ignition::math::Vector3<double>(0,0,0));
                _model.get()->SetWorldPose(_pose);

                // Publish IMU: compute z-acceleration via central difference on wave height,
                // then apply measurement model: az_imu = z̈ - g  (Eq. 7 of Küchler et al.)
                if (_dt > 1e-9) {
                    const double h = 1e-4;
                    double x = _pose.Pos()[0];
                    double y = _pose.Pos()[1];
                    double z_p = wave.generateWaveHeightAtPoint(x, y, _actual_sim_time + h, 0) + 2;
                    double z_0 = wave.generateWaveHeightAtPoint(x, y, _actual_sim_time,     0) + 2;
                    double z_m = wave.generateWaveHeightAtPoint(x, y, _actual_sim_time - h, 0) + 2;
                    double az  = (z_p - 2.0 * z_0 + z_m) / (h * h);

                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp    = _ros_node->get_clock()->now();
                    imu_msg.header.frame_id = "vessel_imu";
                    imu_msg.linear_acceleration.x = 0.0;
                    imu_msg.linear_acceleration.y = 0.0;
                    imu_msg.linear_acceleration.z = az - G;   // gravity bias (z-down convention)
                    imu_msg.linear_acceleration_covariance[0] = -1;  // covariance unknown (REP-145)
                    _imu_pub->publish(imu_msg);
                }

                _prev_pose = _pose;

            }

    private:
        double _position_sp_x = 0.0;
        double _position_sp_y = 0.0;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager);
}
