#ifndef STATE_H
#define STATE_H

#include "FiniteStateMachine.h"
//#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <raspicam/raspicam.h>

class FiniteStateMachine;

class State {

public:
	State(FiniteStateMachine *machine);
	virtual ~State() = default;

	int32_t getID() const;
	void setMode(int32_t mode);
//	void setPrevMode();

	virtual void setParameters() = 0;

protected:
	virtual void setID() = 0;

public:
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisherRaw_;
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisherCompressed_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr      pubCameraInfo_;

	static raspicam::RaspiCam cameraComp2_;
	static raspicam::RaspiCam cameraComp_;
	//raspicam::RaspiCam cameraComp_;

	enum states { IDLE, LANDING, SEARCH, STOP };

protected:
	int32_t             id_;
	FiniteStateMachine *machine_;
	static int32_t      flightMode_;
	static int32_t      preFlightMode_;
	bool 		    Init;
};

#endif
