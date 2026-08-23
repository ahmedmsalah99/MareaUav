#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <fstream>
#include <mutex>
#include <random>

// ROS 2 (gazebo_ros) — replaces the old LCM transport.
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ulisse_msgs/msg/simulated_system.hpp>

#include "WaveGen.h"

#define VEL 0.1

#define CRUISE_VEL 0.8

#define MAX_DIST 15

#define DIST_VEL_THRE 5

namespace gazebo
{
    // Wraps an angle (radians) into [-pi, pi].
    static double wrapAngle(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // Caps how far `current` may move toward `target` this tick, given a max
    // rate of change [units/s] — a slew-rate limiter. Used to keep any
    // discontinuity in the raw /ulisse/simulated_system signal (e.g. the jump
    // between the last and first sample when a short recording is looped)
    // from being applied to the simulated platform in a single tick: the
    // limiter spreads it out over multiple ticks instead, bounding the
    // effective velocity/acceleration seen downstream.
    static double slewLimit(double target, double current, double maxRate, double dt)
    {
        double maxStep = maxRate * dt;
        double delta = target - current;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

    // Same as slewLimit(), but for an angle — the shortest-path delta is
    // wrapped into [-pi, pi] before clamping, so e.g. yaw doesn't take the
    // long way around through +-pi.
    static double slewLimitAngle(double target, double current, double maxRate, double dt)
    {
        double maxStep = maxRate * dt;
        double delta = wrapAngle(target - current);
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

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
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _cmd_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _drone_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _nav_sub;
        rclcpp::Subscription<ulisse_msgs::msg::SimulatedSystem>::SharedPtr _sim_system_sub;

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

        ignition::math::Pose3d _pose, _start_pose, _prev_pose, _prev_prev_pose;

        // IMU publishing
        bool   _publish_imu;
        double _imu_noise_std;
        bool   _imu_history_ready;   // true once we have three poses for double diff
        std::mt19937 _rng{std::random_device{}()};
        std::normal_distribution<double> _noise{0.0, 1.0};

        bool _first_rb;
        double _nav_pos_0;

        // ---- Roll / pitch / yaw wave simulation ----------------------------
        // Each angle is driven by a sum of N sinusoids with random phases,
        // configurable amplitude and frequency range from the SDF.
        struct AngleWaveGen {
            std::vector<double> freqs;
            std::vector<double> phases;
            double amplitude = 0.0;

            void init(int n, double amp, double min_freq, double max_freq,
                      std::mt19937& rng)
            {
                amplitude = amp;
                freqs.resize(n);
                phases.resize(n);
                std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);
                double step = (n > 1) ? (max_freq - min_freq) / (n - 1) : 0.0;
                for (int i = 0; i < n; ++i) {
                    freqs[i]  = min_freq + i * step;
                    phases[i] = phase_dist(rng);
                }
            }

            double generate(double t) const
            {
                if (freqs.empty()) return 0.0;
                double sum = 0.0;
                for (size_t i = 0; i < freqs.size(); ++i)
                    sum += std::sin(freqs[i] * t + phases[i]);
                return amplitude * sum / static_cast<double>(freqs.size());
            }
        };

        bool         _enable_wave_rpy = false;
        AngleWaveGen _roll_wave;
        AngleWaveGen _pitch_wave;
        AngleWaveGen _yaw_wave;

        // ---- /ulisse/simulated_system playback (recorded-bag motion) -------
        // Highest-priority motion source, once heard: the platform's motion
        // is driven by a recorded ulisse_msgs/SimulatedSystem stream (e.g. a
        // Jazzy-recorded rosbag, played in a loop since a real recording is
        // only a few minutes long). The message's absolute position field is
        // deliberately NOT used — instead position is dead-reckoned from the
        // body-frame linear velocity, which stays continuous across the
        // bag's loop point even though absolute position would jump there.

        // Latest raw sample from the topic — written by simulatedSystemCallback()
        // under _data_mutex, read (snapshotted) in OnUpdate() under the same lock.
        struct SimSystemData
        {
            double rpy[3]          {0, 0, 0};   // bodyframe_angular_position
            double lin_vel_body[3] {0, 0, 0};   // bodyframe_linear_velocity
            double ang_vel_body[3] {0, 0, 0};   // bodyframe_angular_velocity
            double gyro_bias[3]    {0, 0, 0};
        } _sim_system_raw;

        bool _use_sim_system = false;   // latched true on the first message heard

        // Slew-rate-limited values actually used to drive the simulation —
        // updated once per OnUpdate() tick (not once per message), so a
        // discontinuity in the raw signal is spread out over many ticks
        // instead of being applied in one step.
        double _sim_rpy_filt[3]     {0, 0, 0};
        double _sim_lin_vel_filt[3] {0, 0, 0};
        double _sim_ang_vel_filt[3] {0, 0, 0};
        double _sim_gyro_bias[3]    {0, 0, 0};   // snapshotted alongside the above
        bool   _sim_filt_init = false;

        // Dead-reckoned world-frame position, integrated from
        // _sim_lin_vel_filt (rotated into world frame) each tick.
        ignition::math::Vector3d _sim_position;

        // Slew-rate limits (SDF-configurable) — the resilience knobs against
        // jumps in the recorded signal (loop point, sensor dropouts, ...).
        double _sim_max_lin_accel;   // m/s^2 — caps how fast linear velocity may change
        double _sim_max_ang_accel;   // rad/s^2 — caps how fast angular velocity may change
        double _sim_max_rpy_rate;    // rad/s — caps how fast roll/pitch/yaw may change

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

                _publish_imu    = _ros_node->declare_parameter("publish_imu",    true);
                _imu_noise_std  = _ros_node->declare_parameter("imu_noise_std",  0.01);

                // ---- Roll / pitch / yaw wave motion (SDF-configurable) ----
                _enable_wave_rpy = _sdf->Get<bool>("enable_wave_rpy", false).first;

                if (_enable_wave_rpy) {
                    auto getd = [&](const std::string& name, double def) {
                        return _sdf->Get<double>(name, def).first;
                    };
                    auto geti = [&](const std::string& name, int def) {
                        return _sdf->Get<int>(name, def).first;
                    };

                    _roll_wave.init(
                        geti("roll_n_modes",        3),
                        getd("roll_amplitude",       0.15),   // rad (~8.6°)
                        getd("roll_min_frequency",   0.5),    // rad/s
                        getd("roll_max_frequency",   1.5),
                        _rng);

                    _pitch_wave.init(
                        geti("pitch_n_modes",        3),
                        getd("pitch_amplitude",      0.08),   // rad (~4.6°)
                        getd("pitch_min_frequency",  0.4),
                        getd("pitch_max_frequency",  1.2),
                        _rng);

                    _yaw_wave.init(
                        geti("yaw_n_modes",          2),
                        getd("yaw_amplitude",        0.05),   // rad (~2.9°)
                        getd("yaw_min_frequency",    0.1),
                        getd("yaw_max_frequency",    0.5),
                        _rng);

                    RCLCPP_INFO(_ros_node->get_logger(),
                        "Wave RPY enabled — roll: ±%.3f rad, pitch: ±%.3f rad, yaw: ±%.3f rad",
                        _roll_wave.amplitude, _pitch_wave.amplitude, _yaw_wave.amplitude);
                }

                // ---- /ulisse/simulated_system playback params --------------
                _sim_max_lin_accel = _sdf->Get<double>("sim_system_max_lin_accel", 3.0).first;
                _sim_max_ang_accel = _sdf->Get<double>("sim_system_max_ang_accel", 2.0).first;
                _sim_max_rpy_rate  = _sdf->Get<double>("sim_system_max_rpy_rate",  1.0).first;

                auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
                auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

                _pose_pub = _ros_node->create_publisher<nav_msgs::msg::Odometry>(
                    _local_pose_topic, reliable_qos);
                _imu_pub = _ros_node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);


                _cmd_sub = _ros_node->create_subscription<nav_msgs::msg::Odometry>(
                    _cmd_pose_topic, reliable_qos,
                    std::bind(&PlatformManager::positionSetpointCallback, this, std::placeholders::_1));

                _drone_sub = _ros_node->create_subscription<px4_msgs::msg::VehicleOdometry>(
                    "vision_position_estimate", px4_qos,
                    std::bind(&PlatformManager::positionDroneCallback, this, std::placeholders::_1));

                _nav_sub = _ros_node->create_subscription<nav_msgs::msg::Odometry>(
                    "nav_pose", reliable_qos,
                    std::bind(&PlatformManager::navPoseCallback, this, std::placeholders::_1));

                _sim_system_sub = _ros_node->create_subscription<ulisse_msgs::msg::SimulatedSystem>(
                    "/ulisse/simulated_system", reliable_qos,
                    std::bind(&PlatformManager::simulatedSystemCallback, this, std::placeholders::_1));

                wave.init(10,2.0,0.5,7.0);

                _pose = _model.get()->WorldPose();
                _prev_pose      = _pose;
                _prev_prev_pose = _pose;
                _start_pose     = _pose;
                _imu_history_ready = false;

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

                if(_use_sim_system)
                {
                  // Snapshot the raw sample while still locked.
                  SimSystemData raw = _sim_system_raw;
                  for (int i = 0; i < 3; i++) _sim_gyro_bias[i] = raw.gyro_bias[i];

                  if (!_sim_filt_init) {
                    // First tick in this mode: seed the filters/accumulator
                    // directly (nothing to slew-limit against yet) instead of
                    // starting from zero and slewing all the way up.
                    for (int i = 0; i < 3; i++) {
                      _sim_rpy_filt[i]     = raw.rpy[i];
                      _sim_lin_vel_filt[i] = raw.lin_vel_body[i];
                      _sim_ang_vel_filt[i] = raw.ang_vel_body[i];
                    }
                    _sim_position  = _pose.Pos();
                    _sim_filt_init = true;
                  } else if (_dt > 1e-9) {
                    for (int i = 0; i < 3; i++) {
                      _sim_lin_vel_filt[i] = slewLimit(raw.lin_vel_body[i], _sim_lin_vel_filt[i], _sim_max_lin_accel, _dt);
                      _sim_ang_vel_filt[i] = slewLimit(raw.ang_vel_body[i], _sim_ang_vel_filt[i], _sim_max_ang_accel, _dt);
                    }
                    _sim_rpy_filt[0] = slewLimitAngle(raw.rpy[0], _sim_rpy_filt[0], _sim_max_rpy_rate, _dt);
                    _sim_rpy_filt[1] = slewLimitAngle(raw.rpy[1], _sim_rpy_filt[1], _sim_max_rpy_rate, _dt);
                    _sim_rpy_filt[2] = slewLimitAngle(raw.rpy[2], _sim_rpy_filt[2], _sim_max_rpy_rate, _dt);
                  }

                  ignition::math::Quaterniond orientation(
                      ignition::math::Vector3d(_sim_rpy_filt[0], _sim_rpy_filt[1], _sim_rpy_filt[2]));

                  if (_dt > 1e-9) {
                    // The message's own (absolute) position is deliberately
                    // NOT used — it would jump at the recording's loop point.
                    // Instead dead-reckon from the slew-limited body-frame
                    // linear velocity, rotated into world frame, which stays
                    // continuous by construction.
                    ignition::math::Vector3d v_body(
                        _sim_lin_vel_filt[0], _sim_lin_vel_filt[1], _sim_lin_vel_filt[2]);
                    _sim_position += (orientation * v_body) * _dt;
                  }

                  _pose.Set(_sim_position, orientation);
                  yaw_des = _sim_rpy_filt[2];
                }
                else if(_test_rosbag)
                {
                  ignition::math::Quaternion<double> orientation;

                  if (_enable_wave_rpy) {
                      // Generate wave-driven roll/pitch/yaw and convert to quaternion
                      double roll_w  = _roll_wave.generate(_actual_sim_time);
                      double pitch_w = _pitch_wave.generate(_actual_sim_time);
                      double yaw_w   = _yaw_wave.generate(_actual_sim_time);
                      orientation = ignition::math::Quaterniond(
                          ignition::math::Vector3d(roll_w, pitch_w, yaw_w));
                  } else {
                      orientation.W(_nav_pos.orientation[0]);
                      orientation.X(_nav_pos.orientation[1]);
                      orientation.Y(_nav_pos.orientation[2]);
                      orientation.Z(_nav_pos.orientation[3]);
                  }

                  if(_actual_sim_time <= 30.0)
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

                plat_pose.pose.pose.orientation.w = _pose.Rot().W();
                plat_pose.pose.pose.orientation.x = _pose.Rot().X();
                plat_pose.pose.pose.orientation.y = _pose.Rot().Y();
                plat_pose.pose.pose.orientation.z = _pose.Rot().Z();

                if (_dt > 1e-9) {
                    // NOTE: Euler() with no args is the GETTER returning a Vector3d.
                    // The 3-arg Euler(r,p,y) form is a SETTER and would leave these
                    // uninitialized — which is why the rates were reading zero.
                    ignition::math::Vector3d e0 = _pose.Rot().Euler();
                    ignition::math::Vector3d e1 = _prev_pose.Rot().Euler();
                    double dy = e0.Z() - e1.Z();
                    while (dy >  M_PI) dy -= 2.0 * M_PI;
                    while (dy < -M_PI) dy += 2.0 * M_PI;
                    plat_pose.twist.twist.angular.x = (e0.X() - e1.X()) / _dt;
                    plat_pose.twist.twist.angular.y = (e0.Y() - e1.Y()) / _dt;
                    plat_pose.twist.twist.angular.z = dy                / _dt;
                }

                _pose_pub->publish(plat_pose);

                // ---- IMU publishing ----
                // Derive readings from observed platform motion (three-point history):
                //   linear acceleration = second finite difference of position / dt²
                //   angular velocity    = first finite difference of Euler angles / dt
                // Specific force: a_imu = a_world − g  (gravity subtracted on z, z-up frame).
                // Enabled by the publish_imu parameter; noise magnitude by imu_noise_std.
                if (_publish_imu && _imu_history_ready && _dt > 1e-9) {
                    constexpr double G = 9.81;
                    double n = _imu_noise_std;

                    // Linear acceleration via second finite difference of world position
                    double ax = (_pose.Pos()[0] - 2.0*_prev_pose.Pos()[0] + _prev_prev_pose.Pos()[0]) / (_dt * _dt);
                    double ay = (_pose.Pos()[1] - 2.0*_prev_pose.Pos()[1] + _prev_prev_pose.Pos()[1]) / (_dt * _dt);
                    double az = (_pose.Pos()[2] - 2.0*_prev_pose.Pos()[2] + _prev_prev_pose.Pos()[2]) / (_dt * _dt);

                    // Angular velocity via first finite difference of Euler angles.
                    // Use the no-arg Euler() getter (returns Vector3d); the 3-arg
                    // form is a setter and would leave the values uninitialized.
                    ignition::math::Vector3d e0 = _pose.Rot().Euler();
                    ignition::math::Vector3d e1 = _prev_pose.Rot().Euler();

                    // Wrap yaw difference to [-π, π]
                    double dy = e0.Z() - e1.Z();
                    while (dy >  M_PI) dy -= 2.0 * M_PI;
                    while (dy < -M_PI) dy += 2.0 * M_PI;

                    double wx = (e0.X() - e1.X()) / _dt;
                    double wy = (e0.Y() - e1.Y()) / _dt;
                    double wz =  dy               / _dt;

                    if (_use_sim_system) {
                      // Feed the recorded (slew-limited) angular velocity
                      // directly, bias-corrected, instead of the noisier
                      // Euler-angle finite difference above — this is the
                      // actual rate signal the recording provides.
                      wx = _sim_ang_vel_filt[0] - _sim_gyro_bias[0];
                      wy = _sim_ang_vel_filt[1] - _sim_gyro_bias[1];
                      wz = _sim_ang_vel_filt[2] - _sim_gyro_bias[2];
                    }

                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp    = _ros_node->now();
                    imu_msg.header.frame_id = "vessel_imu";

                    imu_msg.linear_acceleration.x = ax     + n * _noise(_rng);
                    imu_msg.linear_acceleration.y = ay     + n * _noise(_rng);
                    imu_msg.linear_acceleration.z = az - G + n * _noise(_rng);

                    imu_msg.angular_velocity.x = wx + n * _noise(_rng);
                    imu_msg.angular_velocity.y = wy + n * _noise(_rng);
                    imu_msg.angular_velocity.z = wz + n * _noise(_rng);

                    // Orientation from current pose (ground truth, no noise)
                    imu_msg.orientation.w = _pose.Rot().W();
                    imu_msg.orientation.x = _pose.Rot().X();
                    imu_msg.orientation.y = _pose.Rot().Y();
                    imu_msg.orientation.z = _pose.Rot().Z();

                    // Diagonal covariance matrices (noise_std² per axis)
                    double var = n * n;
                    imu_msg.linear_acceleration_covariance[0] = var;
                    imu_msg.linear_acceleration_covariance[4] = var;
                    imu_msg.linear_acceleration_covariance[8] = var;
                    imu_msg.angular_velocity_covariance[0]    = var;
                    imu_msg.angular_velocity_covariance[4]    = var;
                    imu_msg.angular_velocity_covariance[8]    = var;
                    imu_msg.orientation_covariance[0]         = -1; // orientation is exact

                    _imu_pub->publish(imu_msg);
                }

                _prev_prev_pose    = _prev_pose;
                _prev_pose         = _pose;
                _imu_history_ready = true;

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

            void simulatedSystemCallback(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(_data_mutex);

                // Position (inertialframe_linear_position) is intentionally
                // NOT consumed — see OnUpdate()'s _use_sim_system branch.
                _sim_system_raw.rpy[0] = msg->bodyframe_angular_position.roll;
                _sim_system_raw.rpy[1] = msg->bodyframe_angular_position.pitch;
                _sim_system_raw.rpy[2] = msg->bodyframe_angular_position.yaw;

                for (int i = 0; i < 3; i++) {
                    _sim_system_raw.lin_vel_body[i] = msg->bodyframe_linear_velocity[i];
                    _sim_system_raw.ang_vel_body[i] = msg->bodyframe_angular_velocity[i];
                    _sim_system_raw.gyro_bias[i]    = msg->gyro_bias[i];
                }

                // Latch into simulated-system mode on the first message heard
                // — from then on it takes priority over every other motion
                // source in OnUpdate(), for the lifetime of the plugin.
                if (!_use_sim_system) {
                    RCLCPP_INFO(_ros_node->get_logger(),
                        "First /ulisse/simulated_system message received — "
                        "switching platform motion to recorded playback.");
                }
                _use_sim_system = true;
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager);
}
