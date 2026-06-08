#include "CameraStates.h"
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>

static constexpr int USEC_PER_SEC = 1000000;
static constexpr int SLEEP_IDLE_TRANSITION_SEC  = 2;
static constexpr int SLEEP_CAMERA_STABILIZE_SEC = 3;
static constexpr int SLEEP_STOP_SEC             = 1;


void IdleState::setParameters()
{
	if (flightMode_ == FM_LAND) {

		cameraComp_.release();

		usleep(SLEEP_IDLE_TRANSITION_SEC * USEC_PER_SEC);

		this->machine_->setState(nextLandingState_);

	}

	else if (flightMode_ == FM_SEARCH) {

		cameraComp_.release();

		usleep(SLEEP_IDLE_TRANSITION_SEC * USEC_PER_SEC);

		this->machine_->setState(nextSearchState_);
	}
	else if(flightMode_ == FM_STOP)
		this->machine_->setState(nextStopState_);
	else // If we are before landing or search states
	{
		// we need to publish camera info for safety tasks on mocap2mav side,
		// sending a timestamp just to show that the camera is active, if it is oc
		if (!cameraComp_.open())
		{
			std::cerr<<"Error opening camera on IDLE state"<<std::endl;
		}
		else
		{
			camInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
	    gettimeofday(&time_, NULL);
			camInfoMsg_->header.stamp.sec = time_.tv_sec ;
	 		camInfoMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;
	 		pubCameraInfo_->publish(camInfoMsg_);

			cameraComp_.release();
		}
	}

}


LandingState::LandingState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate) : State(machine)
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

        compressedImageMsg_->header.frame_id = "camera_compressed";
        compressedImageMsg_->format = "rgb8; jpeg compressed bgr8";

	//JPEG CONFIG

        cInfo_.err = jpeg_std_error(&jErr_);
        jpeg_create_compress(&cInfo_);

        cInfo_.image_width = cameraComp_.getWidth();
        cInfo_.image_height = cameraComp_.getHeight();
        cInfo_.input_components = 3;
        cInfo_.in_color_space = JCS_EXT_BGR;

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
        	cameraComp_.setFormat(raspicam::RASPICAM_FORMAT_RGB);
        	cameraComp_.setFrameRate(rate_);

		std::cout<<"Opening Camera for landing action..."<<std::endl;
                if (!cameraComp_.open()) {
                        std::cerr<<"Error opening camera on LAND state"<<std::endl;
												_cameraOpened = false;
                }
								else
								{
										_cameraOpened = true;
								}
        	std::cout<<"Sleeping for 3 secs"<<std::endl;

        	usleep(3 * 1000000); // wait a while until camera stabilizes
		Init = true;
	}
	if(_cameraOpened)
	{
					imageSize_ = cameraComp_.getImageBufferSize();
					dataRaw_ = new uint8_t[ imageSize_ ];

					cameraComp_.grab();
					//extract the image in rgb format
					cameraComp_.retrieve(dataRaw_, raspicam::RASPICAM_FORMAT_RGB); // get camera image

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

			if (mem != NULL) {
						free(mem);
									mem = NULL;
					}
			if(dataRaw_ != NULL)
			{
						delete dataRaw_;
						dataRaw_ = NULL;
			}

			// CAMERA INFO DATA
			gettimeofday(&time_, NULL);
			camInfoMsg_->header.stamp.sec = time_.tv_sec ;
			camInfoMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;
			pubCameraInfo_->publish(camInfoMsg_);


					if (preFlightMode_ == FM_LAND && flightMode_ == FM_SEARCH){
									this->machine_->setState(nextState_);
			Init = false;
			}

			else if(preFlightMode_ == FM_LAND && flightMode_ == FM_STOP){
			this->machine_->setState(nextStopState_);
			Init = false;
			}

					preFlightMode_ = flightMode_;

	}
	else
	{
		cameraComp_.release();
		Init = false;
	}
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

	compressedImageMsg_->header.frame_id = "camera_compressed";
	compressedImageMsg_->format = "rgb8; jpeg compressed bgr8";

	//JPEG CONFIG

	cInfo_.err = jpeg_std_error(&jErr_);
	jpeg_create_compress(&cInfo_);

	cInfo_.image_width = cameraComp_.getWidth();
	cInfo_.image_height = cameraComp_.getHeight();
	cInfo_.input_components = 3;
	cInfo_.in_color_space = JCS_EXT_BGR;

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
        	cameraComp_.setFormat(raspicam::RASPICAM_FORMAT_RGB);
	        cameraComp_.setFrameRate(rate_);

		std::cout<<"Opening Camera for search&rescue..."<<std::endl;
                if ( !cameraComp_.open()) {
                        std::cerr<<"Error opening camer on SEARCH state"<<std::endl;
												_cameraOpened = false;
                }
								else
								{
										_cameraOpened = true;
								}
                std::cout<<"Sleeping for 3 secs"<<std::endl;

                usleep(SLEEP_CAMERA_STABILIZE_SEC * USEC_PER_SEC); // wait a while until camera stabilizes
		Init = true;
	}
	if(_cameraOpened)
	{
		imageSize_ = cameraComp_.getImageBufferSize();
		dataRaw_ = new uint8_t[ imageSize_ ];

		cameraComp_.grab();
		//extract the image in rgb format
		cameraComp_.retrieve(dataRaw_, raspicam::RASPICAM_FORMAT_RGB); // get camera image

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

		if (mem != NULL) {
			free(mem);
			mem = NULL;
		}
		if(dataRaw_ != NULL)
		{
					delete dataRaw_;
					dataRaw_ = NULL;
		}

		// CAMERA INFO DATA
		gettimeofday(&time_, NULL);
		camInfoMsg_->header.stamp.sec = time_.tv_sec ;
		camInfoMsg_->header.stamp.nanosec = 1000 * time_.tv_usec;
		pubCameraInfo_->publish(camInfoMsg_);

		if (preFlightMode_ == FM_SEARCH && flightMode_ == FM_LAND){
			this->machine_->setState(nextState_);
			Init = false;
		}

		else if(preFlightMode_ == FM_SEARCH && flightMode_ == FM_STOP){
									this->machine_->setState(nextStopState_);
			Init = false;
		}

		preFlightMode_ = flightMode_;
	}
	else
	{
		cameraComp_.release();
		Init = false;
	}
}

void StopState::setParameters()
{
	if(!Init){
		cameraComp_.release();
		Init = true;
	}
	usleep(SLEEP_STOP_SEC * USEC_PER_SEC);

	if(flightMode_ != FM_STOP){
		this->machine_->setState(nextState_);
		Init = false;
	}

}
