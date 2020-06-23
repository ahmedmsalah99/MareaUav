#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

//#include "lcm_messages/geometry/UltrasonicPosition.hpp"
//#include "lcm/lcm-cpp.hpp"
#include <queue>
#include <sensor_msgs/msg/range.hpp>
#include "utils/TimeHelpers.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node_options.hpp>
class UltrasonicSensor : public rclcpp::Node {

public:

	UltrasonicSensor();

	void setSensorData(const sensor_msgs::msg::Range::SharedPtr r); //sets data from the simulated sensor

	double getDistance(); //It returns the distance

	bool GetValidity() const; //It checks if the result is valid(based on the timeout expired )

private:

	
	bool isExpired( int timeout, long time ); //it checks if the timeout is expired.

	int trigger; //trigger pin

	int echo; //echo pin

	volatile long startTimeUsec;

	volatile long endTimeUsec;

	double distanceMeters;

	long travelTimeUsec;

	long now;

	bool isValid; //timeout is expired? if yes, the data are not valid.

	sensor_msgs::msg::Range _sensorData;

	TimeManager _tm;

	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr _ultrasonic_sensor_sub; 
};

/*It allows to estimate the velocity based on two measurements.
Parameters:
1) z position of the platform
2) previous z position of the platform
3) FIFO queue.
4) flag to control the first time the function is used.
5) time object to obtain dt for computing the temporal derivative.
*/
double EstimatedVelocity(double *distance, double *prev_distance, std::queue<double> *queue_t, bool *flag, double _dt, double *int_z, int n_window);

//mean value.
void mean(double& sum,double plus,double minus);

/*
Kalman filter with one state variable which is the distance obtained by the sen$

Pseudo-code:

Pred_Xk = F * Pred_Xk_1

Pred_Pk = F * Pred_P_k_1 * F^t + Q

Kk = Pred_Pk * H^t(H * Pred_Pk * H^t + R)^(-1)

Xk = Pred_Xk + Kk * (Zk - H * Pred_Xk)

Pk = (I - Kk * H) * Pred_Pk

--state: state variable

--Pred_P_k_1: prediction of covariance

--Zk: measurement

--Xk: update the estimate of the state

--Pk: update the covariance error

--Q: covariance of process noise

--R: covariance of the measurement noise
*/
void KF(double state, double Pred_P_k_1, double  Zk, double *Xk, double *Pk, double Q, double R);




#endif

















