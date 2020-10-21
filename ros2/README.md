# VisionSystem with Gazebo simulator and PX4 SITL simulation

## Objective:
Landing action using vision system

## Download phase:

### In the repo folder:

Firstly, load its sub-modules

Then:

	1. Firmware module: branch master.
	Move to sitl_gazebo folder to load its the submodules, then set the branch VisionSystem.
	It allows to run the PX4 simulator.

	2. mocap2mav module: branch master.
	Load the submodules.
	This module is the guidance controller.

	3. landingPlat module: branch master.
	It's allows to move the platform in Gazebo simulation.

	4. vision_dashing_ws module: branch master.
	Then, load sub-module "lcm_bridge".
	It's a ROS2 workspace in which there are the vision system and the bridge for communicating
	with the guidance controller and a package to set the simulation.

	5. vision_melodic_ws module: branch master.
	This module is needed for the ros1bridge between ros2 and px4.

	6. fsm_raspicam moduel: branch master.
	This module is needed when using the raspicamera in real tests; it should be put inside
	vision_dashing_ws in real tests, while kept outside in simulations.

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
cd ~/vision_dashing_ws
source environment.sh
```

### Link Gazebo plugin (Landing Platform):
```
cd ~/landingPlat
```
Compile it and put the .so file in the build folder (if not already there).
Compilation being done by these commands:

```
mkdir build
cd build
cmake ..
make
```

### Link Gazebo plugin (Payload):
```
cd ~/Payload
```
Compile it and put the .so file in the build folder (if not already there).
Compilation being done by these commands:

```
mkdir build
cd build
cmake ..
make
```

### To set the simulation with the vision system:
```
cd ~/vision_dashing_ws/src/load_nodes
source setup_variable.sh
```
It links Gazebo with Firmware, the bridge with Guidance controller, it includes the plugin for the platform. Remember to set the correct paths.
This step needs to be done for every terminal requiring the M2M_INCLUDE variable.

### To compile the guidance controller:
```
cd ~/mocap2mav
source build_package.sh
./app_start.sh
```
Then load the action list called list.txt following the form of "list_example.txt"


## How to run:

1. Firmware module (Firmware directory):
```
no_sim=1 make px4_sitl_rtps gazebo
```

2. roscore (on any directory):
```
. /opt/ros/melodic/setup.bash
roscore
```

3. ros1_bridge (vision_dashing_ws directory):
```
. /opt/ros/dashing/setup.bash
. install/local_setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

4. Mavros (vision_melodic_ws):

NEEDS M2M_INCLUDE => source environment.sh and setup_variable.sh first

```
. devel/setup.bash
roslaunch load_nodes load_simulation.launch
```

5. RTPS Agent (vision_dashing_ws):
```
. install/setup.bash
micrortps_agent -t UDP
```

6. RTPS Client (on the first terminal (Firmware NuttShell))

```
micrortps_client start -t UDP
```

7. apriltag + lcm_bridge + ultrasonic_sensor + payload_releaser modules:

NEEDS M2M_INCLUDE => source environment.sh and setup_variable.sh first

```
. install/setup.bash
ros2 launch load_nodes ros2_sim.launch.py
```

8. mocap2mav module:
```
cd ~/mocap2mav
source setting_test.sh
source build_package.sh
./app_start.sh
sim
```

on the spawned terminals, allow the takeoff if you need to perform it.

9. Coming back to the first terminal (Firmware) and write:
```
commander mode arm
commander mode offboard
```
or do it in QGroundControl.
That's done to allow the mocap2mav module to communicate with the firmware.
 
Besides ros2 topic to see the vision data.


10. On another terminal (vision_melodic_ws):
```
. devel/setup.bash
rosrun image_view image_view image:=/apriltag/image/compressed
```
this node allows to see the images coming from the camera



