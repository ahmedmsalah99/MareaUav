# VisionSystem with Gazebo simulator and PX4 SITL simulation

## Objective:
Landing action using vision system

## Download phase:

### In the repo folder:

Firstly, load its sub-modules

Then:

	1. Firmware module: branch stable.
	Move to sitl_gazebo folder to load its the submodules, then set the branch VisionSystem.
	It allows to run the PX4 simulator.

	2. mocap2mav module: branch VisionSystem or VisionSystemBeta.
	Load the submodules.
	This module is the guidance controller.

	3. landingPlat module: branch master.
	It's allows to move the platform in Gazebo simulation.

	4. tb3_aprilTag module: branch VisionSystem.
	Then, load sub-module "lcm_bridge".
	It's a ROS workspace in which there are the vision system and the bridge for communicating
	with the guidance controller and a package to set the simulation.

## Initial setup:

Remember to export the variable M2M_INCLUDE to link the lcm_bridge and the guidance controller.

### To compile Firmware module:
```
cd ~/Firmware
make px4_sitl_default
```

The last instruction will wait to connect to the simulator

### To set the vision system:
```
cd ~/aprilTag
source environment.sh
```

### Link Gazebo plugin:
```
cd ~/landingPlat
```
Compile it and put the .so file in the build folder.

### To set the simulation with the vision system:
```
cd ~/aprilTag/catkin_ws/src/load_simulation
source setup_variable.sh
```
It links Gazebo with Firmware, the bridge with Guidance controller, it includes the plugin for the platform. Remember to set the correct paths

### To compile the guidance controller:
```
cd ~/mocap2mav
source build_package.sh
./app_start.sh
```
Then load the action list called list.txt following the form of "list_example.txt"


## How to run:

1. Firmware module:
```
no_sim=1 make px4_sitl_default gazebo
```

2. tb3_aprilTag module:
```
source devel/setup.bash
roslaunch load_simulation load_simulation.launch
```
It runs MAVROS package and aprilTag vision system.

3. mocap2mav module:
```
cd ~/mocap2mav
source build_package.sh
./app_start.sh
sim
```
The actions will be parsed

4. the bridge module:
```
source devel/setup.bash
rosrun lcm_bridge bridge_node
```
In the catkin workspace


Coming back to the first terminal and write:
```
commander mode arm
commander mode offboard
```
or do it in QGroundControl.
That's done to allow the mocap2mav module to communicate with the firmware.
 
Besides rostopic to see the vision data.




