#ifndef CAMERASTATES_H
#define CAMERASTATES_H

#include "FiniteStateMachine.h"
#include <sys/time.h>
#include "jpeglib.h"

class IdleState : public State {

public:
	IdleState(FiniteStateMachine *machine) : State(machine) {
		setID();
	}

	void setID() override {
		ID = IDLE;
	}

	void setParameters();

	State *nextLandingState_;
	State *nextSearchState_;
};

class LandingState : public State {

public:
	LandingState(FiniteStateMachine *machine, int width, int height, int rate) : State(machine) {
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

		dataRaw_ = new unsigned char[ imageSize_ ];

		rawImageMsg_->data.reserve(imageSize_);
		rawImageMsg_->data.resize(imageSize_);

		rawImageMsg_->header.frame_id = "camera_raw";
		rawImageMsg_->height = cameraRaw_.getHeight();
		rawImageMsg_->width = cameraRaw_.getWidth();
		rawImageMsg_->encoding = "bgr8";
	}

	void setID() override {
		ID = LANDING;
	}

	void setParameters();

	State *nextState_;

	private:
		std::shared_ptr<sensor_msgs::msg::Image> rawImageMsg_;
		std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsg_;

		uint32_t imageSize_;
		struct timeval time_;
		unsigned char *dataRaw_;
		int width_;
		int height_;
		int rate_;
};

class SearchState : public State {
	
public:
	SearchState(FiniteStateMachine *machine, int width, int height, int rate) : State(machine) {
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
		dataRaw_ = new unsigned char[ imageSize_ ];

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

	void setID() override {
		ID = SEARCH;
	}

	void setParameters();

	State *nextState_;

	private:
		std::shared_ptr<sensor_msgs::msg::CompressedImage> compressedImageMsg_;
		std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsg_;

		int width_;
		int height_;
		int rate_;
		struct timeval time_;
		unsigned char *dataRaw_;
		uint32_t imageSize_;

		//JPEG STUFF//
		JSAMPROW rowPointer_[1];
		int rowStride_;
		struct jpeg_compress_struct cInfo_;
		struct jpeg_error_mgr jErr_;
};

class StopState : public State {

public:
	StopState(FiniteStateMachine *machine) : State(machine) {

	setID();
}

	void setID() override {
		ID = STOP;
	}

	void setParameters();

	State *nextState_;
};

#endif
