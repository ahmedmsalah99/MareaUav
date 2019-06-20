#include "State.h"

raspicam::RaspiCam State::cameraComp2_;
raspicam::RaspiCam State::cameraComp_;

int State::flightMode_ = -1;
int State::preFlightMode_ = -1;

State::State(FiniteStateMachine *machine) : machine_(machine)
{
	static auto node = rclcpp::Node::make_shared("Camera_publisher");

	publisherCompressed_ = node->create_publisher<sensor_msgs::msg::CompressedImage>("raspi/compressed_image", 1);
	pubCameraInfo_       = node->create_publisher<sensor_msgs::msg::CameraInfo>("raspi/camera_info", 1);
}

int State::getID() const
{ 
	return id_;
}

void State::setMode(int mode)
{
	flightMode_ = mode;
}

