# Finite State Machine for Raspberry Pi Camera #
Ros2 package which allows to publish camera parameters and image compressed data via ros2 topic.
It has a dependency of `raspicam` package ( `git clone https://NicoNelli@bitbucket.org/isme_robotics/mar_utils.git` ).

It's C++ set of classes which communicate with low level Raspberry Pi camera library.

It is a State Machine composed of four states, namely : IDLE, LAND, SEARCH, STOP.

During the LAND state the camera has the following parameters:

resolution: 640x480.

frame rate: 20 Hz.

During the SEARCH state the camera has the following parameters:

resolution: 1280x960 

frame rate: 2 Hz.

During the STOP state the camera is closed and there are no publications.

The switch between the two camera configurations is handled by the state machine.

## How to Run ##

To run the finite state machine:

```
ros2 run fsm_raspicam camera_main_node
```
The package publishes camera parameters and image compressed data through ros2 topics.

## Ros2 Topics:
- `/apriltag/image/camera_info` (type: `sensor_msgs/CameraInfo`)
- `/apriltag/image/compressed_image/compressed_image` (type: `sensor_msgs/CompressedImage`)


To change the state of the machine, publish on /drone_state 0 or 1 or 2.

```
ros2 topic pub /drone_state std_msgs/Float32 "data: 1"
```

LAND state: 0

SEARCH state: 1 

STOP state: 2



