
# mocap2mav
It's a guidance controller architecture for controlling generic quadrotor platform.

It consists of a modular lightweight on-board guidance controller written in C++. Modules are independent programs communicating
with each other through Lightweight Communications and Marshalling middleware (LCM). LCM is used for publisher/subscriber communication
and its custom messages can be compiled via:

```
	cd include/lcm_messages/
	lcm-gen -x message.lcm
```

or, if dealing with Python messages (release_payload needs both c++ and py), inject:

```
	lcm-gen -p message.lcm
```
For other programming languages, check the references online.
To see pre-compiled messages, please check in `mocap2mav/include/lcm_messages`.

## How to Compile

To Get eigen, deps and lcm software

```
	cd mocap2mav
	source configure.sh
```

Then

```
	source setting_test.sh
	source build_package.sh
```

## How to Run

```
	cd mocap2mav
	source app_start.sh
```

Then, when it asks for the configuration file, enter

```
	sim
```

## General Overview of Automatic Module
The automatic module takes as input a finite set of tasks. Each of them represents a basic autonomous action that the quadrotor
can perform. A sequence of tasks represents an entire mission. The finite set of tasks are encoded in a text file, loaded offine,
which is parsed by the architecture obtaining, as output, the type of action and its desired references. It is possible to see
an example of template file in `cd/mocap2mav/lists`.
The described architecture implements essential navigation actions which typically involve a generic flight of a quadrotor (taking-off,
climbing, cruising, descending, landing).

For any doubt or clarification, `doxygen_config` will generate Doxygen documentation.

### LCM Topics Subscription:

- `vision_position_estimate` Estimated pose of the quadcopter.

- `actual_task` Actual task to schedule.

- `apriltag_vision_system` Visual feedback provided by apriltag system.

- `UltrasonicSensor/platform` Position and Velocity obtained by the ultrasonic sensor module.

### LCM Topics Publication:
- `local_position_sp` Position or Velocity setpoint.

Such module obtains data from and publish data to `lcm_bridge` module.
Please refer to `https://bitbucket.org/isme_robotics/mar_frankenrotor_ros/src/master/ros2/vision_system_ws/src/lcm_bridge`.
