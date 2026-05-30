# TODO

- [ ] Replace geometry::vision member `_drone_pose` with ROS2-friendly storage (PoseStamped) in AprilTag2Node.hpp
- [ ] Update AprilTag2Node::getDroneOdometry to fill the new ROS2 structure
- [ ] Replace local `geometry::vision vision_pose` / `vision_temp` and the `_handler.publish(...)` path in AprilTag2Node2_0.cpp
- [ ] Add a ROS2 publisher for the new pose output (PoseStamped) and publish conditionally on validity
- [ ] Remove now-unused geometry::vision usage and includes
- [ ] Build the ROS2 workspace to confirm compilation

