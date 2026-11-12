#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
extern "C"
{
	#include <lgpio.h>
}
#include "UltrasonicSensor.h"
#define SpeedOfSound 343.00

// GPIO goes through lgpio (/dev/gpiochip0), which works on Ubuntu 22.04 on the Pi;
// wiringPi did not (the echo pin never read high). Pins are BCM numbers.

static long micros()
{
	return std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

UltrasonicSensor::UltrasonicSensor() : Node("ultrasonic_sensor", rclcpp::NodeOptions().use_intra_process_comms(true))
{
}

void UltrasonicSensor::setParameters(int trigger, int echo , int timeout)
{
	this->trigger = trigger; //set TRIG pin
	this->echo = echo;       // set ECHO pin
	this->timeout = timeout; // set Timeout (ms)

	_gpio_handle = lgGpiochipOpen(0);
	if (_gpio_handle < 0 ||
	    lgGpioClaimOutput(_gpio_handle, 0, trigger, 0) < 0 ||
	    lgGpioClaimInput(_gpio_handle, 0, echo) < 0)
	{
		RCLCPP_FATAL(get_logger(), "Cannot open/claim GPIO (trigger %d, echo %d): %s. "
			"Run with access to /dev/gpiochip0 (sudo or the gpio group).",
			trigger, echo, lguErrorText(_gpio_handle < 0 ? _gpio_handle : LG_GPIO_BUSY));
		std::exit(1);
	}

	// TRIG low initially, give the sensor some time to settle.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void UltrasonicSensor::TimeOfTravel()
{
	startTimeUsec = micros();

	// Bounded wait: a stuck-high echo must not freeze the node.
	while (lgGpioRead(_gpio_handle, echo) == 1)
	{
		if ((micros() - startTimeUsec) / 1000.0 >= timeout) { isValid = false; break; }
	}

	endTimeUsec = micros();
}

bool UltrasonicSensor::isExpired( int timeout, long time ) {
//it checks if the timeout is expired.

isValid = ( micros() - time )/1000.0 < timeout;

return isValid;
}


void UltrasonicSensor::setSensorData(const sensor_msgs::msg::Range::SharedPtr r)
{
	_sensorData.radiation_type = r->radiation_type;
	_sensorData.field_of_view = r->field_of_view;
	_sensorData.min_range = r->min_range;
	_sensorData.max_range = r->max_range;
	_sensorData.range = r->range;
}


double UltrasonicSensor::getDistance()
{
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// >=10 us high pulse on TRIG starts a measurement.
	lgGpioWrite(_gpio_handle, trigger, 1);
	std::this_thread::sleep_for(std::chrono::microseconds(20));
	lgGpioWrite(_gpio_handle, trigger, 0);

	now = micros();

	// ECHO stays high for the round-trip time of the pulse.
	while ( lgGpioRead(_gpio_handle, echo) == 0 && isExpired( timeout,now ) );
	if (!isValid)
		return distanceMeters; // no echo: keep the last distance, flagged invalid

	TimeOfTravel();

	travelTimeUsec = endTimeUsec - startTimeUsec;

	if (isValid)
		distanceMeters = ( (travelTimeUsec/1000000.0)*SpeedOfSound )/2;

	return distanceMeters;
}


bool UltrasonicSensor::GetValidity() const {

	return isValid;
}


void mean(double& sum,double plus,double minus){

    sum += plus - minus;

}

double EstimatedVelocity(double *distance, double *prev_distance, std::queue<double> *queue_t, bool *flag, double _dt, double *int_z , int n_window) {

	double Z_velocity = 0;

	if( *flag ) {
		*flag = false;
		return 0;
	}
	else {

		double dz = *distance - *prev_distance;
		double vz = dz/_dt;

		//DEBUG:
		//std::cout<<"dz: "<<dz<<std::endl;
		//std::cout<<"vz: "<<vz<<std::endl;
		//std::cout<<"time: "<<_dt<<std::endl;


		//mean for smoothing
		if (queue_t->size() == n_window) {

			mean(*int_z , vz, queue_t->front()); //return the oldest element of the FIFO queue
			queue_t->pop(); //it removes the oldest method.

		}else
            	mean(*int_z,vz,0);

		queue_t->push(vz);
		Z_velocity = *int_z/queue_t->size();
	}

	*prev_distance = *distance;
	return Z_velocity;

}


void KF(double Pred_Xk_1, double Pred_P_k_1,double  Zk, double *Xk, double *Pk,double Q, double R){
/**
Pred_Xk = F * Pred_Xk_1

Pred_Pk = F * Pred_P_k_1 * F^t + Q

Kk = Pred_Pk * H^t(H * Pred_Pk * H^t + R)^(-1)

Xk = Pred_Xk + Kk * (Zk - H * Pred_Xk)

Pk = (I - Kk * H) * Pred_Pk

In this case:

F = [1]

B = [0]

H = [1]

Q = [1e-4]

R = [2,92e-3]

**/

double F = 1;
double H = 1;
//double Q = 1e-4;
//double R = 2.92e-3;


double Pred_Xk = F * Pred_Xk_1;

double Pred_Pk = F * Pred_P_k_1 * F + Q;

double Kk = Pred_Pk * H * ( 1/(H * Pred_Pk * H + R ) );

 *Xk = Pred_Xk + Kk * (Zk - H * Pred_Xk);

 *Pk = (1 - Kk * H) * Pred_Pk;


}
