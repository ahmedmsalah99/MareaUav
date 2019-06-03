#include "CameraStates.h"
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <fstream>


void IdleState::setParameters(){
		
	if( FlightMode == 0 ){ //raw image
	
		Camera_comp.release();
		Camera_raw.release();
	
		usleep(2*1000000);
	
		//Camera_raw.setWidth(1280);
		//Camera_raw.setHeight(960);
		//Camera_raw.setFormat(raspicam::RASPICAM_FORMAT_BGR);
		//Camera_raw.setFrameRate(20);

		std::cout<<"Opening Camera for raw image..."<<std::endl;
		if ( !Camera_raw.open()) {
			std::cerr<<"Error opening camera"<<std::endl;
		}
		std::cout<<"Sleeping for 3 secs"<<std::endl;
    
		usleep(3*1000000);     //wait a while until camera stabilizes
		
		this->Machine->setState(nextLandingState);
		
	}  //land
	
	else if( FlightMode == 1 ){ //Compressed image
		
		Camera_comp.release();
		Camera_raw.release();
	
		usleep(2*1000000);

		std::cout<<"Opening Camera for compressed image..."<<std::endl;
		if ( !Camera_comp.open()) {
			std::cerr<<"Error opening camera"<<std::endl;
		}
		std::cout<<"Sleeping for 3 secs"<<std::endl;
    
		usleep(3*1000000);     //wait a while until camera stabilizes
		
		this->Machine->setState(nextSearchState);
	     
	}

//PreFlightMode = FlightMode;
}


void LandingState::setParameters(){ //raw image
     
	static bool save = true;
     
	gettimeofday(&time,NULL);

        Camera_raw.grab();
        //extract the image in rgb format
        Camera_raw.retrieve ( data_raw,raspicam::RASPICAM_FORMAT_BGR );//get camera image    
    
        raw_image_msg->header.stamp.sec = time.tv_sec ;
        raw_image_msg->header.stamp.nanosec = 1000*time.tv_usec;
        
	std::cout<<"dim_raw:"<<imageSize<<std::endl;

	
        for(uint32_t i = 0; i < imageSize; i++)
            raw_image_msg->data[i] = data_raw[i];
        
	
	if(save){
	std::ofstream outFile( "photo_raw.ppm", std::ios::binary );
        outFile<<"P6\n"<<Camera_raw.getWidth() <<" "<<Camera_raw.getHeight() <<" 255\n";
        outFile.write ( ( char* ) data_raw, Camera_raw.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR ) );
	
	save = false;
	}
	
	
	publisher_raw->publish(raw_image_msg);
	
	
	//CAMERA INFO DATA
	CamInfo_msg->header.stamp.sec = time.tv_sec ;
        CamInfo_msg->header.stamp.nanosec = 1000*time.tv_usec;
	pub_cameraInfo->publish(CamInfo_msg);
	
	
	if( PreFlightMode == 0 && FlightMode == 1 )
		this->Machine->setState(nextState);


PreFlightMode = FlightMode;

}

void SearchState::setParameters(){ //compressed image

	static bool save = true;

        Camera_comp.grab();
        //extract the image in rgb format
        Camera_comp.retrieve ( data_raw,raspicam::RASPICAM_FORMAT_BGR );//get camera image    

	unsigned char *mem = NULL;
        unsigned long mem_size = 0;
        jpeg_mem_dest(&cinfo, &mem, &mem_size);

        jpeg_start_compress(&cinfo,TRUE);
        
        while(cinfo.next_scanline < cinfo.image_height){
        row_pointer[0] = &data_raw[cinfo.next_scanline*row_stride];
        jpeg_write_scanlines(&cinfo,row_pointer,1);

        }

	jpeg_finish_compress(&cinfo);

	std::cout<<"dim_compressed: "<<mem_size<<std::endl;

	compressed_image_msg->data.resize(mem_size);
	
	for(uint32_t i = 0; i < mem_size; i++)
            compressed_image_msg->data[i] = mem[i];
	
	gettimeofday(&time,NULL);
	compressed_image_msg->header.stamp.sec = time.tv_sec ;
        compressed_image_msg->header.stamp.nanosec = 1000*time.tv_usec;

	publisher_compressed->publish(compressed_image_msg);
	
	if(save){
        
        std::ofstream outFile ( "photo_compressed.ppm", std::ios::binary );
        outFile.write ( ( char* ) mem, mem_size);

        
        save = false;
        }
	
	delete mem;

	//CAMERA INFO DATA
	CamInfo_msg->header.stamp.sec = time.tv_sec ;
        CamInfo_msg->header.stamp.nanosec = 1000*time.tv_usec;
	pub_cameraInfo->publish(CamInfo_msg);

	
	if( PreFlightMode == 1 && FlightMode == 0 )
		this->Machine->setState(nextState);

PreFlightMode = FlightMode;

}


void StopState::setParameters(){
	
	std::cout<<"hey"<<std::endl;
}
