#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ctime>
#include <stdio.h>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_still.h>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>

int cont = 0;

void chatterCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    
    //std::cout<<"hey "<<cont<<std::endl;
    cont++;
}



int main ( int argc,char **argv ) {

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("boh_listener");
    
	 auto sub = node->create_subscription<sensor_msgs::msg::Image>("raspi/image_raw",chatterCallback,1);

	rclcpp::Rate loop_rate(80);

	struct timeval timer;
    
	struct timeval end;

        gettimeofday(&timer,NULL);

	while (rclcpp::ok() ){
		 
		 
	loop_rate.sleep();
        rclcpp::spin_some(node);

	gettimeofday(&end,NULL);
        std::cout<<"rate: "<< cont*1000000.0/ (double)( (end.tv_sec - timer.tv_sec)*1000000 + (end.tv_usec -timer.tv_usec) )<<std::endl;

	}



return 0;
}
