#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/UltrasonicPosition.hpp"
#include "utils/TimeHelpers.hpp"
#include <queue>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ultrasonic_sensor/msg/UltrasonicMsg.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "MedianFilter.h"


/************************************************************
FILE TO COMMUNICATE SENSOR DATA WITH MOCAP2MAV
KALMAN FILTER TO ESTIMATE POSITION&VELOCITY DATA
*************************************************************/


#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.


//just to connect the ultrasonic module with the mocap2mav
lcm::LCM handler; 
geometry::UltrasonicPosition temp_plat_filtr;
geometry::UltrasonicPosition temp_plat_prev_filtr;

TimeManager tm;

bool first2 = true;

typedef std::queue<double> queue_t; //it allows to implement FIFO access.
queue_t stack_z2;
queue_t	dataBuffer;

double integral_z2 = 0;


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Two_states_kalman_filter_for_ultra_Sensor");
    
	UltrasonicSensor Sonar1(trigger, echo);
	
	//sensor data filtered with two states kalman filter	
    auto PubSensorData = node->create_publisher<ultrasonic_sensor::msg::UltrasonicMsg>("/UltrasonicSensor_full_filtr", rmw_qos_profile_default);

	//ros::Rate loop_rate(40);
    rclcpp::Rate loop_rate(30);


	//ROS MESSAGES
	//ultrasonic_sensor::UltrasonicMsg state;
	//ultrasonic_sensor::UltrasonicMsg Full_state_filtr;
    auto state = std::make_shared<ultrasonic_sensor::msg::UltrasonicMsg>();
    auto Full_state_filtr = std::make_shared<ultrasonic_sensor::msg::UltrasonicMsg>();

    
	//SETUP FULL KALMAN FILTER
	int stateSize = 2; 
	int measSize = 1;
	int contrSize = 0;
	unsigned int type = CV_32F;	

	//initialize the kalman filter
	cv::KalmanFilter kf(stateSize, measSize, contrSize );

	
	//set dt at each processing step.
	//it initialize MATRIX F
	cv::setIdentity(kf.transitionMatrix); 
	//std::cout<<"matrix F = "<< std::endl << " "<< kf.transitionMatrix<< std::endl << std::endl; 


	//Measure Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1;
	//std::cout<<"matrix H = "<< std::endl << " "<< kf.measurementMatrix << std::endl << std::endl;


	//Process Noise Covariance matrix Q
	cv::setIdentity(kf.processNoiseCov);
	kf.processNoiseCov.at<float>(0) = 1e-4;
	kf.processNoiseCov.at<float>(3) = 1e-3;
	//std::cout<<"matrix Q = "<< std::endl << " "<< kf.processNoiseCov << std::endl << std::endl;


	//Measures Noise covariance Matrix R
	kf.measurementNoiseCov = 2.92e-3;
	
	//Initialization covariance error
	//cv::Mat CovInit = 0.003*cv::Mat::eye(stateSize, stateSize, type);	

	cv::setIdentity(kf.errorCovPre);
	kf.errorCovPre.at<float>(0) = 0.003;
	kf.errorCovPre.at<float>(3) = 0.003;

	//it takes the value of the sensor
	state->z_Position = Sonar1.distance(timeout);

	tm.updateTimer();	
	
    MedianFilter<double, 5> f;

	while(rclcpp::ok()){
	
	tm.updateTimer();	

	// FULL KALMAN FILTER
	kf.transitionMatrix.at<float>(1) = tm._dt; //Updating of matrix F

	cv::Mat Prediction = kf.predict();

	cv:: Mat meas(1,1,type);
	meas = state->z_Position;
	kf.correct(meas); //correction

	Full_state_filtr->z_Position = kf.statePost.at<float>(0); //filtered position
    temp_plat_filtr.isValid = Sonar1.GetValidity();

        
	//Full_state_filtr.z_Velocity = kf.statePost.at<float>(1); TO MUCH NOISE
	///////////////////////


	//LCM STUFF TO DO THE VELOCITY

	temp_plat_filtr.Z_Position = Full_state_filtr->z_Position;

	//estimated velocity based on filtered position
	EstimatedVelocity( &temp_plat_filtr, &temp_plat_prev_filtr, &stack_z2, &first2, tm, &integral_z2 );

    f.addSample(temp_plat_filtr.Z_Velocity);
        
	//FILTERING VELOCITY

	if( f.isReady() )
		Full_state_filtr->z_Velocity = f.getMedian();

	else
		Full_state_filtr->z_Velocity = 0;
        
    temp_plat_filtr.Z_Velocity = state_filtr->z_Velocity;
    
     
	PubSensorData->publish( Full_state_filtr );
        
	handler.publish("UltrasonicSensor/platform",&temp_plat_filtr);

        
	ros::spinOnce;
	
	loop_rate.sleep();

	}

}
