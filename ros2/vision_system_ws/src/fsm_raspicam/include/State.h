#ifndef STATE_H
#define STATE_H

#include "FiniteStateMachine.h"
#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <raspicam/raspicam.h>


class FiniteStateMachine;

class State{
	
public:
		
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_raw;
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_compressed;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cameraInfo;


	
	static raspicam::RaspiCam Camera_raw;
	
	static raspicam::RaspiCam Camera_comp;
	
	State(FiniteStateMachine *machine);
	~State();
	
	enum states{ IDLE,LANDING,SEARCH,STOP};
	
	int getID() const;
	void setMode(int mode);
	void setPrevMode();
	
	
	virtual void setParameters() = 0;
	
protected:

	virtual void setID() = 0;
	int ID;
	FiniteStateMachine *Machine;
	static int FlightMode;
	static int PreFlightMode;
	
};

#endif
