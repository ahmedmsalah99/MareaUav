#include "CameraStates.h"
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <fstream>


void IdleState::setParameters()
{
	if (flightMode_ == 0) {
		// raw image
		cameraComp_.release();
		cameraRaw_.release();

		usleep(2*1000000);

		//Camera_raw.setWidth(1280);
		//Camera_raw.setHeight(960);
		//Camera_raw.setFormat(raspicam::RASPICAM_FORMAT_BGR);
		//Camera_raw.setFrameRate(20);

		std::cout<<"Opening Camera for raw image..."<<std::endl;
		if (!cameraRaw_.open()) {
			std::cerr<<"Error opening camera"<<std::endl;
		}
		std::cout<<"Sleeping for 3 secs"<<std::endl;

		usleep(3 * 1000000); // wait a while until camera stabilizes

		this->Machine->setState(nextLandingState_);

	}
	// land
	else if (flightMode_ == 1) {
		// Compressed image

		cameraComp_.release();
		cameraRaw_.release();

		usleep(2 * 1000000);

		std::cout<<"Opening Camera for compressed image..."<<std::endl;
		if ( !cameraComp_.open()) {
			std::cerr<<"Error opening camera"<<std::endl;
		}
		std::cout<<"Sleeping for 3 secs"<<std::endl;

		usleep(3 * 1000000); // wait a while until camera stabilizes

		this->Machine->setState(nextSearchState_);
	}
	//PreFlightMode = FlightMode;
}

void LandingState::setParameters()
{
	//raw image

	//static bool save = true;

	gettimeofday(&time_,NULL);

	cameraRaw_.grab();
	//extract the image in rgb format
	cameraRaw_.retrieve(dataRaw_, raspicam::RASPICAM_FORMAT_BGR); // get camera image

	rawImageMsg_->header.stamp.sec = time_.tv_sec ;
	rawImageMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;

	std::cout<<"dim_raw:"<<imageSize_<<std::endl;

	for(uint32_t i = 0; i < imageSize_; i++) {
		rawImageMsg_->data[i] = dataRaw_[i];
	}
	
	/*
	if (save) {
		std::ofstream outFile( "photo_raw.ppm", std::ios::binary );
			outFile<<"P6\n"<<Camera_raw.getWidth() <<" "<<Camera_raw.getHeight() <<" 255\n";
			outFile.write ( ( char* ) data_raw, Camera_raw.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR ) );
		save = false;
	}
	*/

	publisher_raw->publish(rawImageMsg_);

	//CAMERA INFO DATA
	camInfoMsg_->header.stamp.sec = time_.tv_sec ;
	camInfoMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;
	pub_cameraInfo->publish(camInfoMsg_);
	
	if (preFlightMode_ == 0 && flightMode_ == 1) {
		// Release the camera resource

		// Set next state
		this->Machine->setState(nextState_);
	}

	preFlightMode_ = flightMode_;
}

void SearchState::setParameters()
{
	//compressed image
	//static bool save = true;

	cameraComp_.grab();
	//extract the image in rgb format
	cameraComp_.retrieve(dataRaw_, raspicam::RASPICAM_FORMAT_BGR); // get camera image

	uint8_t *mem = NULL;
	long unsigned int memSize = 0;

	jpeg_mem_dest(&cInfo_, &mem, &memSize);
	jpeg_start_compress(&cInfo_, TRUE);
	
	while (cInfo_.next_scanline < cInfo_.image_height) {
		rowPointer_[0] = &dataRaw_[cInfo_.next_scanline * rowStride_];
		jpeg_write_scanlines(&cInfo_, rowPointer_, 1);
	}

	jpeg_finish_compress(&cInfo_);

	std::cout<<"dim_compressed: "<<memSize<<std::endl;

	compressedImageMsg_->data.resize(memSize);
	
	for(uint32_t i = 0; i < memSize; i++) {
		compressedImageMsg_->data[i] = mem[i];
	}
	
	gettimeofday(&time_, NULL);
	compressedImageMsg_->header.stamp.sec = time_.tv_sec ;
	compressedImageMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;

	publisher_compressed->publish(compressedImageMsg_);
	
	/*
	if (save) {
		std::ofstream outFile ( "photo_compressed.ppm", std::ios::binary );
		outFile.write ( ( char* ) mem, mem_size);
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
	pub_cameraInfo->publish(camInfoMsg_);

	if (preFlightMode_ == 1 && flightMode_ == 0) {
		this->Machine->setState(nextState_);
	}

	preFlightMode_ = flightMode_;
}

void StopState::setParameters()
{
	std::cout<<"hey"<<std::endl;
}
