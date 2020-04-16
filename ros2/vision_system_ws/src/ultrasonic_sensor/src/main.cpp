#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/UltrasonicPosition.hpp"
#include "utils/TimeHelpers.hpp"
#include <queue>
#include <cmath>

#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.

lcm::LCM handler; 
geometry::UltrasonicPosition temp_plat;
geometry::UltrasonicPosition temp_plat_prev;
TimeManager tm;

bool first = true;
typedef std::queue<double> queue_t; //it allows to implement FIFO access.
queue_t stack_z;
double integral_z = 0;

int main() {

	UltrasonicSensor Sonar1(trigger, echo);

	tm.updateTimer();	

	while(1){
	
	tm.updateTimer();	
	
	temp_plat.Z_Position = round( Sonar1.distance(timeout) * 1000 ) / 1000;  	    
	
	temp_plat.isValid = Sonar1.GetValidity();
	
	EstimatedVelocity( &temp_plat, &temp_plat_prev, &stack_z, &first, tm, &integral_z );

	handler.publish("UltrasonicSensor/platform",&temp_plat);


	//DEBUG:
	std::cout << "Platform z position:" << std::endl;
    std::cout << "valid:"<<temp_plat.isValid<<std::endl;
    std::cout << temp_plat.Z_Position << std::endl;
    std::cout << "Platform z velo:" << std::endl;
    std::cout << temp_plat.Z_Velocity << std::endl;	



	}
	

	/*
	long now = micros();    
	std::cout<<"distance in meters: "<<Sonar1.distance(timeout)<<std::endl;	
	std::cout<<"Time necessary: "<<(micros()-now)/1000000.0<<" s"<<std::endl;	
	std::cout<<"is Valid the measure? "<<Sonar1.GetValidity()<<std::endl;
	*/
}
