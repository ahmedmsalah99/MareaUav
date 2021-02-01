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
		id_ = IDLE;
	}

	void setParameters();

public:
	State *nextLandingState_;
	State *nextSearchState_;
	State *nextStopState_;

private:
	
	std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsg_;
	struct timeval time_;
};

class LandingState : public State {

public:
	LandingState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate);

	void setID() override {
		id_ = LANDING;
	}

	void setParameters();

	State *nextState_;
	State *nextStopState_;
	private:
		//std::shared_ptr<sensor_msgs::msg::Image>      rawImageMsg_;
		std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsg_;
		std::shared_ptr<sensor_msgs::msg::CompressedImage> compressedImageMsg_;

		uint64_t imageSize_;
		uint8_t *dataRaw_;
		int32_t  width_;
		int32_t  height_;
		int32_t  rate_;

		struct timeval time_;

		//JPEG STUFF//
        	JSAMPROW                    rowPointer_[1];
        	int32_t                     rowStride_;
        	struct jpeg_compress_struct cInfo_;
        	struct jpeg_error_mgr       jErr_;
		bool _cameraOpened = true;

};

class SearchState : public State {
	
public:
	SearchState(FiniteStateMachine *machine, int32_t width, int32_t height, int32_t rate);

	void setID() override {
		id_ = SEARCH;
	}

	void setParameters();

public:
	State *nextState_;
	State *nextStopState_;
private:
	std::shared_ptr<sensor_msgs::msg::CompressedImage> compressedImageMsg_;
	std::shared_ptr<sensor_msgs::msg::CameraInfo>      camInfoMsg_;

	int32_t  width_;
	int32_t  height_;
	int32_t  rate_;
	uint8_t *dataRaw_;
	uint64_t imageSize_;

	struct timeval time_;

	//JPEG STUFF//
	JSAMPROW                    rowPointer_[1];
	int32_t                     rowStride_;
	struct jpeg_compress_struct cInfo_;
	struct jpeg_error_mgr       jErr_;
	bool _cameraOpened = true;
};

class StopState : public State {

public:
	StopState(FiniteStateMachine *machine) : State(machine) {
	Init = false;
	setID();
}

	void setID() override {
		id_ = STOP;
	}

	void setParameters();

	State *nextState_;
};

#endif
