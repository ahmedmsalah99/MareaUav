
/**
*/
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
using namespace std;

int main ( int argc,char **argv ) {
        
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Raspicam_compressed");
    
    auto image_compressed_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>("raspi/compressed_image",1);
    
    raspicam::RaspiCam_Still Camera;

    Camera.setDefaults();
    cout<<"Opening Camera..."<<endl;
    if ( !Camera.open()) {cerr<<"Error opening camera"<<endl;return -1;}
    cout<<"Sleeping for 3 secs"<<endl;
    usleep(3*1000000);     //wait a while until camera stabilizes
    
    uint32_t FullSize = 480*640;
    
    unsigned char *data_compressed = new unsigned char[ FullSize ];
        
    auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    
    compressed_image_msg->header.frame_id = "camera";
        
    struct timeval timer;

    struct timeval end;

                               
    //rclcpp::WallRate loop_rate(50);

    compressed_image_msg->format = "jpeg";

    //bool save = true;

    while (rclcpp::ok() ){
           
             
    gettimeofday(&timer,NULL);
  
    compressed_image_msg->header.stamp.sec = timer.tv_sec ;
    compressed_image_msg->header.stamp.nanosec = 1000*timer.tv_usec;
    
    
    Camera.grab_retrieve(data_compressed,FullSize);
        
    for(uint32_t i = 0; i < Camera.getImageBufferSize(); i++)
    	compressed_image_msg->data.push_back( data_compressed[i] );

    
      
    image_compressed_pub->publish(compressed_image_msg);

    compressed_image_msg->data.clear();

    rclcpp::spin_some(node);
    //loop_rate.sleep();         
      
    
    gettimeofday(&end,NULL);

    std::cout<<"rate: "<< 1000000.0/ (double)( (end.tv_sec - timer.tv_sec)*1000000 + (end.tv_usec -timer.tv_usec) )<<std::endl;

         
     }
    
    rclcpp::shutdown();
        
    //free resrources    
    delete data_compressed;
    return 0;
}


//std::ofstream outFile ( "image_trial.ppm", std::ios::binary );
    //outFile.write ( ( char* ) data_compressed, Camera.getImageBufferSize() );


