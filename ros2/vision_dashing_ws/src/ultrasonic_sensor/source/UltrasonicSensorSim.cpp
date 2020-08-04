#include <iostream>
#include "UltrasonicSensor.h"
#define SpeedOfSound 343.00

UltrasonicSensor::UltrasonicSensor() : Node("ultrasonic_sensor", rclcpp::NodeOptions().use_intra_process_comms(true))
{
	// _ultrasonic_sensor_sub = this->create_subscription<sensor_msgs::msg::Range>("/ultrasonic/sonar", 1, std::bind(&UltrasonicSensor::setSensorData, this, std::placeholders::_1));
}

void UltrasonicSensor::setParameters(int trigger, int echo)
{
		this->trigger = trigger;
		this->echo = echo;
}

bool UltrasonicSensor::isExpired( int timeout, long time ) {
//it checks if the timeout is expired.

isValid = ( _tm.getTimeMicroSecond() - time )/1000.0 < timeout;

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
	return _sensorData.range;
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
