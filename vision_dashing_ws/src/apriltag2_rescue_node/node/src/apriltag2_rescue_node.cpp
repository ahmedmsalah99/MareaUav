#include <AprilTag2RescueNode.hpp>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AprilTag2RescueNode>());
	rclcpp::shutdown();
	return 0;
}
