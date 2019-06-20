#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/UltrasonicPosition.hpp"
//#include "utils/TimeHelpers.hpp"
#include <queue>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ultrasonic_sensor/msg/ultrasonic_msg.hpp"
#include "MedianFilter.h"
#include <sys/time.h>


/************************************************************
FILE TO COMMUNICATE SENSOR DATA WITH MOCAP2MAV
KALMAN FILTER USING ONLY POSITION DATA
*************************************************************/

#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 22 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.


//just to connect the ultrasonic module with the mocap2mav
lcm::LCM handler;
geometry::UltrasonicPosition temp_plat_filtr;
//geometry::UltrasonicPosition temp_plat_prev_filtr;

//TimeManager tm;

bool init = true;

typedef std::queue<double> queue_t; //it allows to implement FIFO access.
queue_t stack_z;

//for computing the velocity
double integral_z = 0;
double previous_distance = 0;
int window = 6;

struct timeval start;
struct timeval end;
double dt;
int RATE = 30;

//Parameters of Kalman filter
double Pred_P_k_1 = 0.003;
double Xk;
double Pk;
double Q = 0.5*1e-3;
double R = 2.92e-3;


int main(int argc, char **argv) {

        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("Basic_kalman_filter_for_ultra_Sensor");

	UltrasonicSensor Sonar1(trigger, echo); //it sets the sensor

    	auto PubSensorData = node->create_publisher<ultrasonic_sensor::msg::UltrasonicMsg>("/UltrasonicSensor_filtr", 1);

    	rclcpp::WallRate loop_rate(30);

	//ROS messages for plotting
    	auto state = std::make_shared<ultrasonic_sensor::msg::UltrasonicMsg>();
    	auto state_filtr = std::make_shared<ultrasonic_sensor::msg::UltrasonicMsg>();

	state->z_position = Sonar1.distance(timeout);

	double Pred_Xk_1 = state->z_position;

	MedianFilter<double, 5> f;

	dt = 1/RATE;

	//tm.updateTimer();
	while(rclcpp::ok()){

	gettimeofday(&start,NULL);

	//tm.updateTimer();

	/*
	F = [1]
	B = [0]
	H = [1]
	Q = [0.5*1e-3]
	R = [2,92e-3]
        */

	KF(Pred_Xk_1, Pred_P_k_1, state->z_position, &Xk, &Pk, Q, R); //kalman filter

	//update the covariance of the error and the states of both filters
	Pred_P_k_1 = Pk;
	Pred_Xk_1 = Xk;

	//taking the distance from the sensor
	state->z_position = Sonar1.distance(timeout);

	state_filtr->z_position = Xk; //filtered data

	temp_plat_filtr.isValid = Sonar1.GetValidity();

	//LCM STUFF TO DO THE VELOCITY
	temp_plat_filtr.Z_Position = state_filtr->z_position; //kalman position

	//velocity after kalman filter
	f.addSample( EstimatedVelocity( &state_filtr->z_position, &previous_distance, &stack_z, &init, dt, &integral_z, window) );

	//FOR PLOTTING THE VELOCITY
	if( f.isReady() )
		state_filtr->z_velocity = f.getMedian();

	else
		state_filtr->z_velocity = 0;

    	temp_plat_filtr.Z_Velocity = state_filtr->z_velocity;

	PubSensorData->publish( state_filtr );

	handler.publish("UltrasonicSensor/platform",&temp_plat_filtr);

        rclcpp::spin_some(node);

        loop_rate.sleep();
	gettimeofday(&end,NULL);

	dt = ((end.tv_sec - start.tv_sec)*1000000.0 + (end.tv_usec - start.tv_usec))*0.000001;

	}

}
