# ROS2 mavros_msgs package #
The communication with the autopilot/flight controller is possible thanks to mavros package and its sub-packages. 
Mavros is a bridge between mavlink (PX4 side) and ROS (companion computer side).
Our whole architecture is written using ROS 2. However, mavros is only available in ROS since ROS 2 is still under development.
For that reason the community developed a bridge (ros1_bridge) to allow the communication between ROS and ROS 2 taking also into account
the mapping between messages. Automatic mapping between ROS and ROS 2 messages is performed based on the names.
If the package name, message name, and all field names and types are the same between a ROS package and a ROS 2 package,
and they satisfy the package suffix condition above, then the messages will automatically be associated with each other without
additional specification. The bridge needs to be recompiled, after the ROS and ROS 2 packages have been compiled and sourced,
each in a separate workspace, for a total of three workspaces (ROS 2, ROS 2 overlaid and ROS).

We create this package being an exact copy of a ROS package called mavros_msgs. Adding this packages mapping, we are now able to exchange all
possible information between autopilot/flight controller and the offboard computer.

## Messages ##

The messages created are:

- `ExtendedState.msg` (info: landing state and VTOL status)
- `State.msg` (info: state of the drone: armed, connected..)





