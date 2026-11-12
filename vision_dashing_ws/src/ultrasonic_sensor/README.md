# ROS 2 Ultrasonic Sensor package for Raspberry Pi
It's a ROS 2 package which computes relative position and velocity between the sensor and a surface. The employed sensor is HC-SR04
and the lgpio library (`sudo apt install liblgpio-dev`; the node needs access to `/dev/gpiochip0`) is used for the connection between sensor and board.
The relative position is obtained by the ultrasonic sensor; the relative velocity is computed using a Kalman filter.
These information are published onto ROS 2 topic and LCM topic. 

To run the sensor:

```
ros2 run ultrasonic_sensor mainMocap2mav.cpp
```
## Ros2 Topics Publication:
- `/UltrasonicSensor/platform` (type: `ultrasonic_sensor/UltrasonicMsg`)

The custom message contains relative position and velocity.

## LCM Topics Publication:
- `UltrasonicSensor/platform` (relative position and velocity)

The LCM topic allows to send data to the guidance controller.






