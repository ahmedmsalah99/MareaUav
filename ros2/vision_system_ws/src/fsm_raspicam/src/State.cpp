#include "State.h"

raspicam::RaspiCam State::cameraRaw_;
raspicam::RaspiCam State::cameraComp_;

int State::flightMode_ = -1;
int State::preFlightMode_ = -1;

State::State(FiniteStateMachine *machine) : Machine(machine)
{
	static auto node = rclcpp::Node::make_shared("Camera_publisher");

	publisher_raw = node->create_publisher<sensor_msgs::msg::Image>("raspi/image_raw", 1);
	publisher_compressed = node->create_publisher<sensor_msgs::msg::CompressedImage>("raspi/compressed_image", 1);
	pub_cameraInfo = node->create_publisher<sensor_msgs::msg::CameraInfo>("raspi/camera_info", 1);
}

int State::getID() const
{ 
	return ID; 
}

void State::setMode(int mode)
{
	flightMode_ = mode;
}

//void State::setPrevMode()
//{
//	PreFlightMode = !FlightMode;
//}

State::~State()
{
}
