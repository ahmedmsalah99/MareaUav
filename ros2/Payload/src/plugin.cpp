#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include "lcm_messages/exec/gps_status.hpp"
#include "poll.h"

namespace gazebo
{

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

        lcm::LCM _handler;
        lcm::LCM _handler2;
        lcm::LCM _handler3;

        lcm::Subscription *_sub;
        lcm::Subscription *_sub2;
        lcm::Subscription *_sub3;

        geometry::pose _drone_pos;
        exec::release_payload _releasePayload;
        exec::gps_status _drone_glob_pos;

      	struct pollfd _fds[3];

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

                _sub = _handler.subscribe("release_payload_feedback", &PayloadManager::relpayloadCallback, this);
                _sub2 = _handler2.subscribe("vision_position_estimate", &PayloadManager::positionDroneCallback, this);
                _sub3 = _handler3.subscribe("gps_status", &PayloadManager::globalPosDroneCallback, this);

                _sub->setQueueCapacity(1);
                _sub2->setQueueCapacity(1);
              	_sub3->setQueueCapacity(1);

                _fds[0].fd = _handler.getFileno();
                _fds[0].events = POLLIN;

                _fds[1].fd = _handler2.getFileno();
                _fds[1].events = POLLIN;

                _fds[2].fd = _handler3.getFileno();
                _fds[2].events = POLLIN;

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
                std::cout << "New payload registered, topics:" << std::endl;
                std::cout << "  "<<_local_pose_topic << std::endl;
                std::cout << "  "<<_cmd_pose_topic   << std::endl;

            }


            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & _info)
            {

                manageTime(_info);

                int ret = poll(_fds,3,0);
                //
                if(_fds[0].revents & POLLIN)
                {
                  _handler.handle();
                  if(_releasePayload.data == "W")
                    _releasingPayload = true;
                  else if(_releasePayload.data == "R")
                    _payloadReleased = true;
                }

                if(_fds[1].revents & POLLIN)
                {
                  _handler2.handle();
                }

                if(_fds[2].revents & POLLIN)
                {
                  _handler3.handle();
                }

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

                _pose.Set(ignition::math::Vector3<double>(newX, newY, newZ), ignition::math::Quaternion<double>(1,0,0,0));

                _model.get()->SetAngularVel(ignition::math::Vector3<double>(0,0,0));
                _model.get()->SetWorldPose(_pose);

                // _prev_pose = _pose;

            }

            void relpayloadCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::release_payload* msg)
            {
              _releasePayload.data = msg->data;
            }

            void positionDroneCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::pose* msg)
            {

                _drone_pos.position[0] = (float)msg->position[0];
                _drone_pos.position[1] = (float)msg->position[1];
                _drone_pos.position[2] = (float)msg->position[2];

                _drone_pos.velocity[0] = (float)msg->velocity[0];
                _drone_pos.velocity[1] = (float)msg->velocity[1];
                _drone_pos.velocity[2] = (float)msg->velocity[2];

                _drone_pos.orientation[0] = (float)msg->orientation[0];
                _drone_pos.orientation[1] = (float)msg->orientation[1];
                _drone_pos.orientation[2] = (float)msg->orientation[2];
                _drone_pos.orientation[3] = (float)msg->orientation[3];

                _drone_pos.yaw = (float)msg->yaw;

            }

            void globalPosDroneCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::gps_status* msg)
            {
              _drone_glob_pos.lon = (float)msg->lon;
              _drone_glob_pos.lat = (float)msg->lat;
              _drone_glob_pos.alt = (float)msg->alt;
              _drone_glob_pos.status = msg->status;
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PayloadManager);
}
