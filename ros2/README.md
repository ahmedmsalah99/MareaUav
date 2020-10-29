# VisionSystem with Gazebo simulator and PX4 SITL simulation

## Objective:
Research of a shipwrecked person, releasing payload and Landing on a platform using vision system and ultrasonic sensor

## Download phase:

### In the repo folder:

Firstly, load its sub-modules

Then:

	1. Firmware module: branch v1.9.2, download it from the official PX4 github.
	inject this command: git clone --recursive https://github.com/PX4/Firmware.git -b v1.9.2
	Move to Tools and substitute sitl_gazebo with the one contained in this git folder.
	Move to msg/tools and substitute the uorb configuration file for rtps with the one in this git folder
	It allows to run the PX4 simulator with RTPS support.

	2. mocap2mav module: branch master.
	Load the submodules.
	This module is the guidance controller.

	3. landingPlat module: branch master.
	It's allows to move the platform in Gazebo simulation.

	4. Payload module: branch master.
	It's allows to spawn and move the payload in Gazebo simulation.

	4. vision_dashing_ws module: branch master.
	Then, load sub-module "lcm_bridge".
	It's a ROS2 workspace in which there are the vision system and the bridge for communicating
	with the guidance controller and a package to set the simulation.
	Before compiling every package, it's highly recommended to configure and compile some of them (such as 
	px4_msgs and px4_ros_com). References can be found at https://dev.px4.io/master/en/middleware/micrortps.html
	NOTE: it's recommended to download the px4_ros_com version on this repository, cause during compilation it was
	giving some errors that got fixed in the CMAKE file, and that may be not present in the most current versions
	online.

	5. vision_melodic_ws module: branch master.
	This module is needed for the ros1bridge between ros2 and px4.

	6. fsm_raspicam module: branch master.
	This module is needed when using the raspicamera in real tests; it should be put inside
	vision_dashing_ws in real tests, while kept outside in simulations.

NOTE: if running a real test and not a simulation (SITL/HITL), landingPlat, Payload directories are not necessary.
Firmware will be instead uploaded on the Pixhawk.

## Initial setup:

Remember to export the variable M2M_INCLUDE to link the lcm_bridge and the guidance controller.
References to set up the PX4-FastRTPS Bridge can be found at https://dev.px4.io/master/en/middleware/micrortps.html
Remember to change the paths in environment.sh and setup_variable.sh files (both in vision_dashing_ws and vision_melodic_ws) to make them fit to yours.

### To compile Firmware module:
```
cd ~/Firmware
make px4_sitl_rtps
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

### Link Gazebo plugin (Payload), same as Landing Platform one:
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

If you are running a real test, run instead setup_variable_real.sh.

It links Gazebo with Firmware, the bridge with Guidance controller, it includes the plugin for the platform.
REMEMBER TO SET THE CORRECT PATHS.
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

NEEDS M2M_INCLUDE => source environment.sh and setup_variable(_real).sh first

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

NEEDS M2M_INCLUDE => source environment.sh and setup_variable(_real).sh first

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



