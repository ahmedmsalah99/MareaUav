# Bridge between ROS 2 and LCM #

This ROS 2 package allows the communication with the guidance controller. The guidance controller is composed of independent
modules communicating with each other through Lightweight Communications and Marshalling middleware (LCM).
The guidance controller takes as input: absolute state of the drone, data coming from the vision system and data coming from ultrasonic sensor.

To run the bridge:

```
ros2 run lcm_bridge bridge_node
```
The package subscribes to ROS 2 topics and it publishes onto LCM topics.

## Ros2 Topics Subscription:
- `/raspi/camera_info` (type: `sensor_msgs/CameraInfo`)
- `/raspi/compressed_image` (type: `sensor_msgs/CompressedImage`)

## LCM Topics Publication:





