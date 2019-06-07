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

		this->machine_->setState(nextLandingState_);

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

		this->machine_->setState(nextSearchState_);
	}
	//PreFlightMode = FlightMode;
}

LandingState::LandingState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate) : State(machine)
{
	width_ = width;
	height_ = height;
	rate_ = rate;

	rawImageMsg_ = std::make_shared<sensor_msgs::msg::Image>();

	camInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();

	camInfoMsg_->header.frame_id = "camera_compressed";
	camInfoMsg_->width = width_;
	camInfoMsg_->height = height_;

	const double fx = (3.04 / 3.68) * width_;
	const double fy = (3.04 / 2.76) * height_;
	const double cx = width_ / 2.0;
	const double cy = height_ / 2.0;

	camInfoMsg_->k = {fx, 0, cx,
						0, fy, cy,
						0,  0,  1};

	setID();

	cameraRaw_.setWidth(width_);
	cameraRaw_.setHeight(height_);
	cameraRaw_.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	cameraRaw_.setFrameRate(rate_);

	imageSize_ = cameraRaw_.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR );

	dataRaw_ = new uint8_t[ imageSize_ ];

	rawImageMsg_->data.reserve(imageSize_);
	rawImageMsg_->data.resize(imageSize_);

	rawImageMsg_->header.frame_id = "camera_raw";
	rawImageMsg_->height = cameraRaw_.getHeight();
	rawImageMsg_->width = cameraRaw_.getWidth();
	rawImageMsg_->encoding = "bgr8";
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

	publisherRaw_->publish(rawImageMsg_);

	//CAMERA INFO DATA
	camInfoMsg_->header.stamp.sec = time_.tv_sec ;
	camInfoMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;
	pubCameraInfo_->publish(camInfoMsg_);
	
	if (preFlightMode_ == 0 && flightMode_ == 1) {
		// Release the camera resource

		// Set next state
		this->machine_->setState(nextState_);
	}

	preFlightMode_ = flightMode_;
}

SearchState::SearchState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate) : State(machine)
{
	width_ = width;
	height_ = height;
	rate_ = rate;

	compressedImageMsg_ = std::make_shared<sensor_msgs::msg::CompressedImage>();

	camInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();

	camInfoMsg_->header.frame_id = "camera_compressed";
	camInfoMsg_->width = width_;
	camInfoMsg_->height = height_;

	//const double fx = (3.60/3.76)*Width;
	//const double fy = (3.60/2.74)*Height;
	//const double cx = Width/2.0;
	//const double cy = Height/2.0;

	const double fx = 508.624627;
	const double cx = 323.83127;
	const double fy = 508.77675;
	const double cy = 248.651286;

	camInfoMsg_->k = {fx,0,cx,
						0,fy,cy,
						0,0,1};

	cameraComp_.setBrightness(45);
	cameraComp_.setContrast(-10);

	//std::cout<<"bright: "<<(unsigned)cameraComp_.getBrightness()<<std::endl;
	//std::cout<<"contrast: "<<cameraComp_.getContrast()<<std::endl;

	cameraComp_.setWidth(width_);
	cameraComp_.setHeight(height_);
	cameraComp_.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	cameraComp_.setFrameRate(rate_);

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

	std::cout<<"dim_compressed: "<<memSize<<std::endl;

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
	pubCameraInfo_->publish(camInfoMsg_);

	if (preFlightMode_ == 1 && flightMode_ == 0) {
		this->machine_->setState(nextState_);
	}

	preFlightMode_ = flightMode_;
}

void StopState::setParameters()
{
	std::cout<<"hey"<<std::endl;
}
