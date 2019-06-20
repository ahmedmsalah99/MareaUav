#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#define SpeedOfSound 343.00

UltrasonicSensor::UltrasonicSensor( int trigger, int echo ) {

	wiringPiSetupGpio();
	//it allows the calling program to use broadcom GPIO pins numbers directly
	//with no re-mapping.

	this->trigger = trigger; //set TRIG pin

	this->echo = echo; // set ECHO pin

	pinMode( trigger, OUTPUT ); //set TRIG pin as output

	pinMode( echo, INPUT ); //set ECHO pin as output

	//ensure that, initially, the TRIG pin is set low, and give some time to
	// settle the sensor.
	digitalWrite( trigger, LOW ); 

	delay(500); //delay in milliseconds.

}

void UltrasonicSensor::TimeOfTravel() {

	startTimeUsec = micros();

	while ( digitalRead(echo) == HIGH );

	endTimeUsec = micros();

}

bool UltrasonicSensor::isExpired( int timeout, long time ) { 
//it checks if the timeout is expired. 	

isValid = ( micros() - time )/1000.0 < timeout;

return isValid;
}

double UltrasonicSensor::distance( int timeout) {

	//firstly, transmit at least 10us high level pulse to the trigger pin.
	//the sensor requires a short 10us pulse to trigger the module.
	//these three functions do this for us.
	delay(10);

	digitalWrite( trigger, HIGH );

	delayMicroseconds(20);

	digitalWrite( trigger, LOW);

	//taking the actual time.
	now = micros();    

	//The sensor sets ECHO to high for the amount of time it takes for the pulse to go and 		come back, so it is necessary to measure the amount of time that the ECHO pin stays high.
	while ( digitalRead(echo) == LOW && isExpired( timeout,now ) );	
	TimeOfTravel();


	travelTimeUsec = endTimeUsec - startTimeUsec;

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
















