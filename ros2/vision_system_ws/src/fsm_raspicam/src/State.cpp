#include "State.h"

raspicam::RaspiCam State::Camera_raw;
raspicam::RaspiCam State::Camera_comp;

int State::FlightMode = -1;
int State::PreFlightMode = -1;

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
	FlightMode = mode;
}

//void State::setPrevMode()
//{
//	PreFlightMode = !FlightMode;
//}

State::~State()
{
}
