# VisionSystem with Gazebo simulator and PX4 SITL simulation

## Objective:
Research of a shipwrecked person, releasing payload and Landing on a platform using vision system and ultrasonic sensor

## Download phase:

### In the repo folder:

Firstly, load its sub-modules

Then:

	1. Firmware module: branch v1.16.1, download it from the official PX4 github.
	inject this command: git clone --recursive https://github.com/PX4/Firmware.git -b v1.16.1
	Move to Tools and substitute sitl_gazebo models and world files with the one contained in this git folder.
		
	2. vision_humble_ws: the workspace with all required nodes.

	3. fsm_raspicam module: branch master.
	This module is needed when using the raspicamera in real tests; it should be put inside
	vision_humble_ws in real tests, while kept outside in simulations.
	
	4. RaspiCam module: branch master.
	Needed to handle the camera images as a suupport for fsm_raspicam. To be compiled apart. Read the instructions.

## Initial setup:

In a new folder run
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
```

Then try to run

```
MicroXRCEAgent udp4 -p 8888
```

This is important to dynamically bridge uORB messages from PX4 to ROS2.


### Compile the workspace:
```
cd vision_humble_ws
source environment.sh
colcon build
```

### Link Gazebo plugins:
 After compiling the workspace, check the "vision_humble_ws/build/payload/libpayloadPlugin.so" and "vision_humble_ws/build/landingplat/libplatformPlugin.so"

 Copy the two files into your GAZEBO_PLUGIN_PATH.
 To check your current plugin path:
 ```
 echo $GAZEBO_PLUGIN_PATH
 ```
To set a new one use
```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:your_path
```

### To set the simulation with the vision system:
If you are running a real test, run instead setup_variable_real.sh.

It links Gazebo with Firmware, the bridge with Guidance controller, it includes the plugin for the platform.


For simulation  

1. run:

```
cd vision_humble_ws
source environment.sh
ros2 launch load_nodes ros2_sim.launch.py
```

2. In another terminal run:
```
MicroXRCEAgent udp4 -p 8888
```
3. To run the simulation:

```
cd vision_humble_ws
source install/setub.bash
```
then cd to your PX4 cloned repo and run:
```
make px4_sitl gazebo-classic_iris__iris
```

4. In the last terminal write:
```
commander arm
commander mode offboard
```
or do it in QGroundControl. You can also takeoff then change the mode to offboard in QGroundControl (recommended).
 

5. ros2bag:
```
source install/setub.bash
ros2 bag play ../tarmem/folder
```
this node allows to replay catamaran's behavior on landing platform

## HITL TEST:

1. Firmware module (Firmware directory):
```
DONT_RUN=1 make px4_sitl_rtps gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_rtps
```

plug in the Pixhawk, go to vision_melodic_ws and issue:

NEEDS M2M_INCLUDE => source environment.sh and setup_variable.sh first

```
. devel/setup.bash
roslaunch load_nodes load_simulation.launch
```

The other terminals don't change wrt SITL

## REAL TEST:

2. roscore (ros_melodic_ws):
```
. install_isolated/setup.bash
roscore
```

3. ros1_bridge (vision_dashing_ws directory):
```
. ../ros_melodic_ws/install_isolated/setup.bash
. ../ros2_dashing_ws/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

4. Mavros (vision_melodic_ws):

NEEDS M2M_INCLUDE => source environment.sh and setup_variable_real.sh first

```
. devel/setup.bash
roslaunch load_nodes load_mavros.launch
```

5. RTPS Agent (vision_dashing_ws):

```
. install/setup.bash
micrortps_agent -d /dev/ttyUSB0
```

6. RTPS Client (on Firmware NuttShell)

Disable Mavlink instance 0 in QGC parameters (we don't want RTPS and Mavlink to run in the same channel)

```
micrortps_client start -d /dev/ttyS1 -b 460800
```

7. apriltag + lcm_bridge + ultrasonic_sensor + payload_releaser + fsm_raspicam modules:

NEEDS M2M_INCLUDE => source environment.sh and setup_variable_real.sh first

```
. install/setup.bash
ros2 launch load_nodes ros2_real.launch.py
```

8. mocap2mav module:
```
cd ~/mocap2mav
source setting_test.sh
source build_package.sh
./app_start.sh
real
```


The other terminals don't change wrt SITL (1. does not exist in this kind of tests)
