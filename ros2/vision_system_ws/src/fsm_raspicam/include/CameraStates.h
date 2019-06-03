#ifndef CAMERASTATES_H
#define CAMERASTATES_H

#include "FiniteStateMachine.h"
#include <sys/time.h>
#include "jpeglib.h"

class IdleState : public State {

public:
	IdleState(FiniteStateMachine *machine) : State(machine){
	
	setID();	
	}
	
	void setID() override {
	
	ID = IDLE;
	}

	void setParameters();

	State* nextLandingState;
	State* nextSearchState;

	
};

class LandingState : public State {

public:
	LandingState(FiniteStateMachine *machine, int width, int height, int rate) : State(machine){
	
		Width = width;
		Height = height;
		Rate = rate;
	
		raw_image_msg = std::make_shared<sensor_msgs::msg::Image>();
		
		CamInfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
		
		CamInfo_msg->header.frame_id = "camera_compressed";
		CamInfo_msg->width = Width;
		CamInfo_msg->height = Height;
		
		const double fx = (3.04/3.68)*Width;
		const double fy = (3.04/2.76)*Height;
		const double cx = Width/2.0;
		const double cy = Height/2.0;
		
		CamInfo_msg->k = {fx,0,cx,
						  0,fy,cy,
						  0,0,1};

		
		setID();
		
		Camera_raw.setWidth(Width);
		Camera_raw.setHeight(Height);
		Camera_raw.setFormat(raspicam::RASPICAM_FORMAT_BGR);
		Camera_raw.setFrameRate(Rate);
		
		imageSize = Camera_raw.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR );

		data_raw = new unsigned char[ imageSize ];
		
		raw_image_msg->data.reserve(imageSize);
		raw_image_msg->data.resize(imageSize);
		
		raw_image_msg->header.frame_id = "camera_raw";
        raw_image_msg->height = Camera_raw.getHeight();
        raw_image_msg->width = Camera_raw.getWidth();
        raw_image_msg->encoding = "bgr8";
		
	}
	
	void setID() override {
	
	ID = LANDING;
	}
	
	void setParameters();

	State* nextState;
	
	private:
		std::shared_ptr<sensor_msgs::msg::Image> raw_image_msg;
		std::shared_ptr<sensor_msgs::msg::CameraInfo> CamInfo_msg;
		
		uint32_t imageSize;
		struct timeval time;
		unsigned char *data_raw;
		int Width;
		int Height;
		int Rate;
};


class SearchState : public State {
	
public:
	SearchState(FiniteStateMachine *machine, int width, int height, int rate) : State(machine){
	
		Width = width;
		Height = height;
		Rate = rate;
	
		compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
		
		CamInfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
		
		CamInfo_msg->header.frame_id = "camera_compressed";
		CamInfo_msg->width = Width;
		CamInfo_msg->height = Height;
		
		//const double fx = (3.60/3.76)*Width;
		//const double fy = (3.60/2.74)*Height;
		//const double cx = Width/2.0;
		//const double cy = Height/2.0;
		
		const double fx = 508.624627;
		const double cx = 323.83127;
		const double fy = 508.77675;
		const double cy = 248.651286;
		
		CamInfo_msg->k = {fx,0,cx,
						  0,fy,cy,
						  0,0,1};

		Camera_comp.setBrightness(45);
		Camera_comp.setContrast(-10);
		
		//std::cout<<"bright: "<<(unsigned)Camera_comp.getBrightness()<<std::endl;
		//std::cout<<"contrast: "<<Camera_comp.getContrast()<<std::endl;
		
		Camera_comp.setWidth(Width);
		Camera_comp.setHeight(Height);
		Camera_comp.setFormat(raspicam::RASPICAM_FORMAT_BGR);
		Camera_comp.setFrameRate(Rate);
		
		imageSize = Camera_comp.getImageTypeSize ( raspicam::RASPICAM_FORMAT_BGR );
		data_raw = new unsigned char[ imageSize ];
		
		compressed_image_msg->header.frame_id = "camera_compressed";
        compressed_image_msg->format = "jpeg";
		
		//JPEG CONFIG
		
		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&cinfo);

		cinfo.image_width = Camera_comp.getWidth();
		cinfo.image_height = Camera_comp.getHeight();
		cinfo.input_components = 3;
		cinfo.in_color_space = JCS_EXT_RGB;

		jpeg_set_defaults(&cinfo);

		jpeg_set_quality(&cinfo,100,TRUE);
    
    
		row_stride = cinfo.image_width*3;
		///////
		
		setID();	
}

	void setID() override {
	
	ID = SEARCH;
	}

	void setParameters();

	State* nextState;
		
	private:
		std::shared_ptr<sensor_msgs::msg::CompressedImage> compressed_image_msg;
		std::shared_ptr<sensor_msgs::msg::CameraInfo> CamInfo_msg;
		
		int Width;
		int Height;
		int Rate;
		struct timeval time;
		unsigned char *data_raw;
		uint32_t imageSize;
		

	
		//JPEG STUFF//
		JSAMPROW row_pointer[1];
		int row_stride;
		struct jpeg_compress_struct cinfo;
		struct jpeg_error_mgr jerr;
	
};

class StopState : public State {
	
public:
	StopState(FiniteStateMachine *machine) : State(machine){
	
	setID();	
}

	void setID() override {
	
	ID = STOP;
	}

	void setParameters();

	State* nextState;
		
};

#endif

