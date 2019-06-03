
/**
*/
#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ctime>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include "jpeglib.h"
using namespace std;

int main ( int argc,char **argv ) {
    
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("Raspicam_raw");
    
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("raspi/image_raw",rmw_qos_profile_default);
    
    //JPEG STUFF//

    JSAMPROW row_pointer[1];
    int row_stride;

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    
    //FILE *outputFile;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);


/*
    if( (outputFile = fopen("compressed.ppm", "wb")) == NULL ){
        std::cout<<"error"<<std::endl;
        return 1;
    }
  */
  //jpeg_stdio_dest(&cinfo, outputFile);
    
    
    
    
    unsigned char *mem = NULL;
    unsigned long mem_size = 0;
    
    jpeg_mem_dest(&cinfo, &mem, &mem_size);

    cinfo.image_width = 640;
    cinfo.image_height = 480;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);

    jpeg_set_quality(&cinfo,90,TRUE);

    jpeg_start_compress(&cinfo,TRUE);


    row_stride = cinfo.image_width*3;
    
    /////
    
    
    
    
    raspicam::RaspiCam Camera; //Camera object
    Camera.setWidth(640);
    Camera.setHeight(480);
    Camera.setFrameRate(20);

    cout<<"Opening Camera..."<<endl;
    if ( !Camera.open()) {cerr<<"Error opening camera"<<endl;return -1;}
    cout<<"Sleeping for 3 secs"<<endl;
    usleep(3*1000000);     //wait a while until camera stabilizes
    
    Camera.grab(); //capture
    
    //allocate memory
    uint32_t imageSize = Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB );
    
    unsigned char *data_raw=new unsigned char[ imageSize ];
    
    auto raw_image_msg = std::make_shared<sensor_msgs::msg::Image>();
    
    raw_image_msg->header.frame_id = "camera";
        
    struct timeval timer;
    
    struct timeval end;
    
    rclcpp::WallRate loop_rate(20);

    raw_image_msg->data.reserve(imageSize);
    
    raw_image_msg->data.resize(imageSize);

    int cont = 0;

    bool jpeg = true;

    while (rclcpp::ok()){

        gettimeofday(&timer,NULL);

        cont++;

        Camera.grab();
        //extract the image in rgb format
        Camera.retrieve ( data_raw,raspicam::RASPICAM_FORMAT_RGB );//get camera image    
    
        raw_image_msg->header.stamp.sec = timer.tv_sec ;
        raw_image_msg->header.stamp.nanosec = 1000*timer.tv_usec;
        
        raw_image_msg->height = 480;
        raw_image_msg->width = 640;
        raw_image_msg->encoding = "rbg8";
        
        for(uint32_t i = 0; i < imageSize; i++)
            raw_image_msg->data[i] = data_raw[i];
                
        image_pub->publish(raw_image_msg);
        
        /////
        
        if(jpeg){
        
        while(cinfo.next_scanline < cinfo.image_height){
        row_pointer[0] = &data_raw[cinfo.next_scanline*row_stride];
        jpeg_write_scanlines(&cinfo,row_pointer,1);

        }
        std::cout<<"size:"<<mem_size<<std::endl;

        std::ofstream outFile ( "trial.ppm", std::ios::binary );
        outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
        outFile.write ( ( char* ) mem, mem_size );


        jpeg_finish_compress(&cinfo);
        //fclose(outputFile);

        jpeg_destroy_compress(&cinfo);

        mem_size = 0;
        
        jpeg = false;
                
       }
        
        ////
        
        
        
        
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
        
        if(cont == 5){
        gettimeofday(&end,NULL);
        std::cout<<"rate: "<< 1000000.0/ (double)( (end.tv_sec - timer.tv_sec)*1000000 + (end.tv_usec -timer.tv_usec) )<<std::endl;
        cont = 0;
        }
        
        
        
}
    
    
    
    
    
    
    
        //cout<<"FPS: "<<50/( (float)(clock()-starting_time)/CLOCKS_PER_SEC )<<endl;
        //save
        //sprintf(filename,"image%d.ppm",i);
        //std::ofstream outFile ( filename, std::ios::binary );
        //outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
        //outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );




    //free resrources    
    delete data_raw;
    return 0;
}
//
