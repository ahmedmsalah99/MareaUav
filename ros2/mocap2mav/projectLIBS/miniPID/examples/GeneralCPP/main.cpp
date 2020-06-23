
#include <iostream>
#include <chrono>
#include <zconf.h>
#include "MiniPID.h"
void loop(double& setpoint, double& sensor, MiniPID& pid);

int main(){

	double setpoint=1;
	double sensor=0;

	// Create a PID system. 
	// It's not very well tuned, but some sample values will get things started. 
	MiniPID pid = MiniPID(.1,.01,0);

	while(true){
		loop(setpoint,sensor,pid);
		usleep(100000);

	}

	return 0;
	
}

void loop(double& setpoint, double& sensor, MiniPID& pid){
	 
	setpoint=setpoint*0.99;

	//Run through the PID calculations, and get the desired output values
	double out=pid.getOutput(sensor,setpoint);

	// "Do something" with the output. 
	// A real world system would do something more interesting, such as turn a motor or servo.
	// The sensor would then report back the "effect" of our output. 
	// For this example, we don't have a physical system, so we'll instead just add them 		together.
	sensor=sensor+out;

	std::cout << "*****************************"<< std::endl;
	std::cout << "sensor: "   << sensor << std::endl;
	std::cout << "setpoint: " << setpoint << std::endl;
	std::cout << "output: "   << out << std::endl;
	std::cout << "*****************************"<< std::endl;

}
