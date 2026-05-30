#ifndef EXECUTIONER_H
#define EXECUTIONER_H
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/task.hpp"
#include "lcm_messages/exec/safety.hpp"
#include "lcm_messages/exec/safety_ack.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include "lcm_messages/exec/plat_task_status.hpp"
#include <string>
#include "common/MavState.h"
#include "common/common.h"
#include "common/conversions.h"
#include <deque>
// PX 4 / sensor ROS 2 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

// Other ROS 2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <ulisse_msgs/msg/vehicle_status.hpp>

// Custom ROS 2 messages mirroring the old LCM exec/* structs.
#include <ulisse_exec_msgs/msg/actual_task.hpp>
#include <ulisse_exec_msgs/msg/safety.hpp>
#include <ulisse_exec_msgs/msg/safety_ack.hpp>
using namespace std;


class Executioner{

public:

	Executioner();
	void run();
	bool CheckActions(int a);
	bool readyToPublish();
	void setNextTask(const exec::task task);
	void setLastTask(const exec::task task);
	void clearList();
  	void abort();
	void rtl();
  	void loadTask();
	void setState(MavState s);
	void setPlatformState(MavState ps);
  	void init();
	void setList(std::deque<exec::task> list);
	void setStatus(bool armed, bool landed);
	void setRelPayFeedback(const std::string& feedback);
	void batteryGoneHandler(const ulisse_exec_msgs::msg::Safety& safety);
	void batteryLimitHandler(const ulisse_exec_msgs::msg::Safety& safety);
	void cameraGoneHandler(const ulisse_exec_msgs::msg::Safety& safety);
	void gpsGoneHandler();
	void sonarGoneHandler();
	void moveToSafeHarbor(const ulisse_exec_msgs::msg::Safety& safety);
	void setSkip();
	void resetLanding();

	void setNavTaskStatus(const ulisse_msgs::msg::VehicleStatus& nav_task_status);

	exec::task _actualTask;
	bool _can_run;
	int _actualNode;
	bool _idle;
	bool skip;
	bool env_sim;

	bool isBatterySafetyTaskRunning;
	bool isGPSSafetyTaskRunning;
	bool isCameraSafetyTaskRunning;
	bool isSonarSafetyTaskRunning;
	bool isGoingInSafeHarbor;
	bool isGoingToPlatform;

private:

	MavState _state;
	MavState _platState;
	std::deque<exec::task> _nodeList;
	bool _newTask;
	bool _publish_task;
	bool _payloadReleased;
	bool _nav_mission_finished;
	bool _first_hold;

	bool _armed;
	bool _landed;

   	double _PI = 3.141592653589;

	exec::release_payload _relpay_feedback;

};
#endif
