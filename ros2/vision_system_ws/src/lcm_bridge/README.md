# Bridge between ROS 2 and LCM #

This ROS 2 package allows the communication with the guidance controller. The guidance controller is composed of independent
modules communicating with each other through Lightweight Communications and Marshalling middleware (LCM).
The guidance controller takes as input: absolute state of the drone, data coming from the vision system and data coming from ultrasonic sensor.

To run the bridge:

```
ros2 run lcm_bridge bridge_node
```
The package subscribes and publishes to ROS 2 topics and it publishes onto LCM topics.

## Ros2 Topics Subscription:
- `/mavros/local_position/odom` (type: `nav_msgs/Odometry`, local position and velocity of the drone)
- `/mavros/state` (type: `mavros_msgs/State`, state of the drone)
- `/mavros/extended_state` (type: `mavros_msgs/ExtendedState`, status of the drone)
- `/platform_center` (type: `geometry_msgs/TransformStamped`, information coming from the vision system)

These topics allows to take information coming from the PX4(via mavros) and from the vision system.

## Ros2 Topics Publication:
- `/mavros/setpoint_position/local` (type: `geometry_msgs/PoseStamped`, position setpoint to send to the PX4)
- `/mavros/setpoint_velocity/cmd_vel` (type: `geometry_msgs/TwistStamped`, velocity setpoint to send to the PX4)

These topics allows to send position/velocity setpoints to the PX4. This setpoints are the output of the guidance controller.

## LCM Topics Publication:
- `apriltag_vision_system` ( relative pose between platform and drone)
- `vision_position_estimate` (position and velocity of the drone)
- `state` (state of the drone)

These topics are the input of the guidance controller. With this information, the guidance controller is able to generate position/velocity setpoints 
based on the actual data..


