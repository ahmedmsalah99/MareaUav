#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ctime>
#include <stdio.h>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <std_msgs/msg/string.hpp>


void chatterCallback(const std_msgs::msg::String::SharedPtr msg)
{
    
    std::cout<<"hey "<<msg->data<<std::endl;
    
}



int main ( int argc,char **argv ) {

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("mine_listener");
    
	 auto sub = node->create_subscription<std_msgs::msg::String>("topic",chatterCallback,1);

	rclcpp::Rate loop_rate(10);

	while (rclcpp::ok() ){
		 
	loop_rate.sleep();

        rclcpp::spin_some(node);


	}


return 0;
}
