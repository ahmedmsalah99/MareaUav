#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/pose.hpp"
#include "WaveGen.h"

#define VEL 0.2
#define MAX_DIST 15

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

        double _actual_sim_time;
        double _prev_sim_time;
        bool _got_t_offs;
        double _time_offs;
        double _dt;
        double _platVel;
        double _dir;
        WaveGen wave;
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

                wave.init(10,4.5,0.5,7.0);

                _pose = _model.get()->WorldPose();
                _prev_pose  = _pose;
                _start_pose = _pose;

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;

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

                _pose = _model.get()->WorldPose();
                // _pose.rot = ignition::math::Quaternion(1,0,0,0);
                // _pose.pos.z = wave.generateWaveHeightAtPoint(_pose.Pos().x,_pose.Pos().y,_actual_sim_time,0) + 2;

                double xTemp = _pose.Pos()[0];
                if(fabs(- xTemp + _start_pose.Pos()[0]) > ((MAX_DIST - 0.01))) invertDirection();
                double newX = xTemp + _dir * (_dt * _platVel);

                //_pose.pos.x = newX;

                _pose.Set(ignition::math::Vector3<double>(newX, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));
                //_pose.Set(ignition::math::Vector3<double>(0, 0, wave.generateWaveHeightAtPoint(_pose.Pos()[0],_pose.Pos()[1],_actual_sim_time,0) + 2), ignition::math::Quaternion<double>(1,0,0,0));
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

                plat_pose.isValid = 1;
                plat_pose.orientation[0] = _pose.Rot().W(); // w
                plat_pose.orientation[1] = _pose.Rot().X(); // x
                plat_pose.orientation[2] = _pose.Rot().Y(); // y
                plat_pose.orientation[3] = _pose.Rot().Z(); // z


                _handler.publish(_local_pose_topic,&plat_pose);

                _prev_pose = _pose;

            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager);
}
