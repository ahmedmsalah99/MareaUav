#include "CameraStates.h"
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <fstream>


void IdleState::setParameters()
{
	if (flightMode_ == 0) { //LANDING ACTION

		cameraComp_.release();

		usleep(2*1000000);

		this->machine_->setState(nextLandingState_);

	}

	else if (flightMode_ == 1) { //SEARCH&RESCUE

		cameraComp_.release();

		usleep(2 * 1000000);

		this->machine_->setState(nextSearchState_);
	}
	else if(flightMode_ == 2) //STOP CAMERA
		this->machine_->setState(nextStopState_);

}


LandingState::LandingState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate) : State(machine)
{

	Init = false;

	width_ = width;
	height_ = height;
	rate_ = rate;

        compressedImageMsg_ = std::make_shared<sensor_msgs::msg::CompressedImage>();

        imageSize_ = cameraComp_.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR );
        dataRaw_ = new uint8_t[ imageSize_ ];

        compressedImageMsg_->header.frame_id = "camera_compressed";
        compressedImageMsg_->format = "jpeg";

	//JPEG CONFIG

        cInfo_.err = jpeg_std_error(&jErr_);
        jpeg_create_compress(&cInfo_);

        cInfo_.image_width = cameraComp_.getWidth();
        cInfo_.image_height = cameraComp_.getHeight();
        cInfo_.input_components = 3;
        cInfo_.in_color_space = JCS_EXT_RGB;

        jpeg_set_defaults(&cInfo_);

        jpeg_set_quality(&cInfo_, 100, TRUE);

        rowStride_ = cInfo_.image_width * 3;

	setID();
}

void LandingState::setParameters()
{
	if(!Init){
		cameraComp_.setBrightness(45);
        	cameraComp_.setContrast(-10);
        	cameraComp_.setWidth(width_);
        	cameraComp_.setHeight(height_);
        	cameraComp_.setFormat(raspicam::RASPICAM_FORMAT_BGR);
        	cameraComp_.setFrameRate(rate_);

		std::cout<<"Opening Camera for landing action..."<<std::endl;
                if (!cameraComp_.open()) {
                        std::cerr<<"Error opening camera"<<std::endl;
                }
        	std::cout<<"Sleeping for 3 secs"<<std::endl;

        	usleep(3 * 1000000); // wait a while until camera stabilizes
		Init = true;
	}

        //static bool save = true;

        cameraComp_.grab();
        //extract the image in rgb format
        cameraComp_.retrieve(dataRaw_, raspicam::RASPICAM_FORMAT_BGR); // get camera image

        uint8_t *mem = NULL; // This is allocated with malloc() --> deallocate with free()
        long unsigned int memSize = 0;

        jpeg_mem_dest(&cInfo_, &mem, &memSize);
        jpeg_start_compress(&cInfo_, TRUE);

        while (cInfo_.next_scanline < cInfo_.image_height) {
                rowPointer_[0] = &dataRaw_[cInfo_.next_scanline * rowStride_];
                jpeg_write_scanlines(&cInfo_, rowPointer_, 1);
        }

	jpeg_finish_compress(&cInfo_);

        std::cout<<"dim compression: "<<memSize<<std::endl;

        compressedImageMsg_->data.resize(memSize);

        for(uint32_t i = 0; i < memSize; i++) {
                compressedImageMsg_->data[i] = mem[i];
        }

        gettimeofday(&time_, NULL);
        compressedImageMsg_->header.stamp.sec = time_.tv_sec ;
        compressedImageMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;

        publisherCompressed_->publish(compressedImageMsg_);

	/*
        if (save) {
                std::ofstream outFile ( "photo_compressed_land.ppm", std::ios::binary );
                outFile.write ( ( char* ) mem, memSize);
                save = false;
        }
	*/

	if (mem != NULL) {
        	free(mem);
                mem = NULL;
        }

        if (preFlightMode_ == 0 && flightMode_ == 1){
                this->machine_->setState(nextState_);
		Init = false;
	}

	else if(preFlightMode_ == 0 && flightMode_ == 2){
		this->machine_->setState(nextStopState_);
		Init = false;
	}

        preFlightMode_ = flightMode_;

}

SearchState::SearchState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate) : State(machine)
{

	Init = false;

	width_ = width;
	height_ = height;
	rate_ = rate;

	compressedImageMsg_ = std::make_shared<sensor_msgs::msg::CompressedImage>();

	camInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();

	camInfoMsg_->header.frame_id = "camera_compressed";
	camInfoMsg_->width = width_;
	camInfoMsg_->height = height_;

	//Taken from raspberry Pi camera documentation
	const double fx = (3.04 / 3.68) * width;
   	const double fy = (3.04 / 2.76) * height;
   	const double cx = width/2.0;
    	const double cy = height/2.0;

	camInfoMsg_->k = {fx,0,cx,
			   0,fy,cy,
			   0,0,1};

	imageSize_ = cameraComp_.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR );
	dataRaw_ = new uint8_t[ imageSize_ ];

	compressedImageMsg_->header.frame_id = "camera_compressed";
	compressedImageMsg_->format = "jpeg";

	//JPEG CONFIG

	cInfo_.err = jpeg_std_error(&jErr_);
	jpeg_create_compress(&cInfo_);

	cInfo_.image_width = cameraComp_.getWidth();
	cInfo_.image_height = cameraComp_.getHeight();
	cInfo_.input_components = 3;
	cInfo_.in_color_space = JCS_EXT_RGB;

	jpeg_set_defaults(&cInfo_);

	jpeg_set_quality(&cInfo_, 100, TRUE);

	rowStride_ = cInfo_.image_width * 3;
	///////

	setID();
}

void SearchState::setParameters()
{

	if(!Init){
		cameraComp_.setBrightness(45);
        	cameraComp_.setContrast(-10);
        	cameraComp_.setWidth(width_);
       	        cameraComp_.setHeight(height_);
        	cameraComp_.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	        cameraComp_.setFrameRate(rate_);

		std::cout<<"Opening Camera for search&rescue..."<<std::endl;
                if ( !cameraComp_.open()) {
                        std::cerr<<"Error opening camera"<<std::endl;
                }
                std::cout<<"Sleeping for 3 secs"<<std::endl;

                usleep(3 * 1000000); // wait a while until camera stabilizes
		Init = true;
	}

	//compressed image
	//static bool save = true;

	cameraComp_.grab();
	//extract the image in rgb format
	cameraComp_.retrieve(dataRaw_, raspicam::RASPICAM_FORMAT_BGR); // get camera image

	uint8_t *mem = NULL; // This is allocated with malloc() --> deallocate with free()
	long unsigned int memSize = 0;

	jpeg_mem_dest(&cInfo_, &mem, &memSize);
	jpeg_start_compress(&cInfo_, TRUE);

	while (cInfo_.next_scanline < cInfo_.image_height) {
		rowPointer_[0] = &dataRaw_[cInfo_.next_scanline * rowStride_];
		jpeg_write_scanlines(&cInfo_, rowPointer_, 1);
	}

	jpeg_finish_compress(&cInfo_);

	std::cout<<"dim compression "<<memSize<<std::endl;

	compressedImageMsg_->data.resize(memSize);

	for(uint32_t i = 0; i < memSize; i++) {
		compressedImageMsg_->data[i] = mem[i];
	}

	gettimeofday(&time_, NULL);
	compressedImageMsg_->header.stamp.sec = time_.tv_sec ;
	compressedImageMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;

	publisherCompressed_->publish(compressedImageMsg_);

	/*
	if (save) {
		std::ofstream outFile ( "photo_compressed_search.ppm", std::ios::binary );
		outFile.write ( ( char* ) mem, memSize);
		save = false;
	}
	*/

	if (mem != NULL) {
		free(mem);
		mem = NULL;
	}

	// CAMERA INFO DATA
	camInfoMsg_->header.stamp.sec = time_.tv_sec ;
	camInfoMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;
	pubCameraInfo_->publish(camInfoMsg_);

	if (preFlightMode_ == 1 && flightMode_ == 0){
		this->machine_->setState(nextState_);
		Init = false;
	}

	else if(preFlightMode_ == 1 && flightMode_ == 2){
                this->machine_->setState(nextStopState_);
		Init = false;
	}

	preFlightMode_ = flightMode_;
}

void StopState::setParameters()
{
	if(!Init){
		cameraComp_.release();
		Init = true;
	}
	usleep( 1000000);

	if(flightMode_ != 2){
		this->machine_->setState(nextState_);
		Init = false;
	}

}

