#ifndef EXECUTIONER_H
#define EXECUTIONER_H
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/task.hpp"
#include "lcm_messages/exec/safety.hpp"
#include "lcm_messages/exec/safety_ack.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include <string>
#include "common/MavState.h"
#include "common/common.h"
#include "common/conversions.h"
#include <deque>

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
	void setRelPayFeedback(exec::release_payload feedback);
	void batteryGoneHandler(exec::safety safety);
	void batteryLimitHandler(exec::safety safety);
	void cameraGoneHandler(exec::safety safety);
	void gpsGoneHandler();
	void sonarGoneHandler();
	void moveToSafeHarbor(exec::safety safety);
	void setSkip();
	void resetLanding();

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

	bool _armed;
	bool _landed;

   	double _PI = 3.141592653589;

	float _defaultYaw;

	exec::release_payload _relpay_feedback;

};
#endif
