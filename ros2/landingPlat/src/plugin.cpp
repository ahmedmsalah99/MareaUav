#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/pose.hpp"
#include "WaveGen.h"
#include "poll.h"

#define VEL 0.1

#define CRUISE_VEL 0.8

#define MAX_DIST 15

#define DIST_VEL_THRE 5

namespace gazebo
{

    class PlatformManager : public ModelPlugin
    {

    private:

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;

        std::string _cmd_pose_topic;
        std::string _local_pose_topic;

        lcm::LCM _handler;
        lcm::LCM _handler2;
        lcm::LCM _handler3;

        lcm::Subscription *_sub;
        lcm::Subscription *_sub2;

        geometry::pose _position_sp;
        geometry::pose _drone_pos;

      	struct pollfd _fds[2];


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

                _local_pose_topic = _model.get()->GetName() + "/pose";
                _cmd_pose_topic   = _model.get()->GetName() + "/pose_cmd";

                _sub = _handler2.subscribe(_cmd_pose_topic, &PlatformManager::positionSetpointCallback, this);
                _sub2 = _handler3.subscribe("vision_position_estimate", &PlatformManager::positionDroneCallback, this);

                _sub->setQueueCapacity(1);
              	_sub2->setQueueCapacity(1);

                _fds[0].fd = _handler2.getFileno();
                _fds[0].events = POLLIN;

                _fds[1].fd = _handler3.getFileno();
                _fds[1].events = POLLIN;

                wave.init(10,3.5,0.5,7.0);

                _pose = _model.get()->WorldPose();
                _prev_pose  = _pose;
                _start_pose = _pose;

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;

                _rescueTarget = false;

                std::cout << "New platform registered, topics:" << std::endl;
                std::cout << "  "<<_local_pose_topic << std::endl;
                std::cout << "  "<<_cmd_pose_topic   << std::endl;
                std::cout << "And some info: " << std::endl;
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

                int ret = poll(_fds,2,0);
                //
                if(_fds[0].revents & POLLIN)
                {
                  _handler2.handle();
                  _rescueTarget = true;
                }

                if(_fds[1].revents & POLLIN)
                {
                  _handler3.handle();
                }

                _pose = _model.get()->WorldPose();
                // _pose.rot = ignition::math::Quaternion(1,0,0,0);
                // _pose.pos.z = wave.generateWaveHeightAtPoint(_pose.Pos().x,_pose.Pos().y,_actual_sim_time,0) + 2;

                double xTemp = _pose.Pos()[0];

                double yaw_des = 0.0;

                if(_rescueTarget)
                {
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
                }
                else
                {

                  if(fabs(- xTemp + _start_pose.Pos()[0]) > ((MAX_DIST - 0.01))) invertDirection();
                  double newX = xTemp + _dir * (_dt * _platVel);

                  //_pose.pos.x = newX;

                  _pose.Set(ignition::math::Vector3<double>(newX, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));
                  // _pose.Set(ignition::math::Vector3<double>(0, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));

                  yaw_des = 1.57;
                }

                _model.get()->SetAngularVel(ignition::math::Vector3<double>(0,0,0));
                _model.get()->SetWorldPose(_pose);
                //Prepare LCM material
                double vx,vy,vz;
                geometry::pose plat_pose;

                vx = (_pose.Pos()[0] - _prev_pose.Pos()[0])/_dt;
                vy = (_pose.Pos()[1] - _prev_pose.Pos()[1])/_dt;
                vz = (_pose.Pos()[2] - _prev_pose.Pos()[2])/_dt;

                plat_pose.position[0] = _pose.Pos()[0]; //x
                plat_pose.position[1] = _pose.Pos()[1]; //y
                plat_pose.position[2] = _pose.Pos()[2]; //z

                plat_pose.velocity[0] = vx;
                plat_pose.velocity[1] = vy;
                plat_pose.velocity[2] = vz;

                plat_pose.yaw = yaw_des;

                plat_pose.isValid = 1;
                plat_pose.orientation[0] = _pose.Rot().W(); // w
                plat_pose.orientation[1] = _pose.Rot().X(); // x
                plat_pose.orientation[2] = _pose.Rot().Y(); // y
                plat_pose.orientation[3] = _pose.Rot().Z(); // z


                _handler.publish(_local_pose_topic,&plat_pose);

                _prev_pose = _pose;

            }

            void positionSetpointCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::pose* msg)
            {

                _position_sp.position[0] = (float)msg->position[0];
                _position_sp.position[1] = (float)msg->position[1];
                _position_sp.position[2] = (float)msg->position[2];

                _position_sp.velocity[0] = (float)msg->velocity[0];
                _position_sp.velocity[1] = (float)msg->velocity[1];
                _position_sp.velocity[2] = (float)msg->velocity[2];

                _position_sp.orientation[0] = (float)msg->orientation[0];
                _position_sp.orientation[1] = (float)msg->orientation[1];
                _position_sp.orientation[2] = (float)msg->orientation[2];
                _position_sp.orientation[3] = (float)msg->orientation[3];

                _position_sp.yaw = (float)msg->yaw;

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
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager);
}
