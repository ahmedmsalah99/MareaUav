cmake_minimum_required(VERSION 3.5)
project(ulisse_msgs)

#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(fastrtps)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "srv/*.srv"
    "*.xml"
    "launch/*.py"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

rosidl_generate_interfaces(ulisse_msgs
  # Sensors
  #"msg/Time.msg"
  "msg/RPY.msg"
  #"msg/LatLong.msg"
  #"msg/PIDData.msg"
  #"msg/IMUData.msg"
  "msg/DVLData.msg"
  "msg/FOGData.msg"
  #"msg/AmbientSensors.msg"
  #"msg/Compass.msg"
  #"msg/Magnetometer.msg"
  "msg/ThrustersReference.msg"
  #"msg/MicroLoopCount.msg"
  "msg/GPSData.msg"
  "msg/GPSStatus.msg"
  "msg/LatLongAlt.msg"
  "msg/ReferenceVelocities.msg"
  "msg/VehicleStatus.msg"
  "msg/ThrusterMappingControl.msg"
  #"msg/SimulatedVelocitySensor.msg"
  "msg/FeedbackGui.msg"
  "msg/SimulatedSystem.msg"
  "msg/DynamicPidControl.msg"

  # Guidance app (custom, not part of upstream Ulisse — mirror the old
  # LCM exec::task / exec::safety / exec::safety_ack structs so guidance_system
  # doesn't need field-name changes; see ExecNode.cpp/SafetyNode.cpp comments).
  "msg/ActualTask.msg"
  "msg/Safety.msg"
  "msg/SafetyAck.msg"

  # Commands
  "msg/CommandHold.msg"
  "msg/CommandLatLong.msg"
  "msg/CommandSpeedHeading.msg"
  "msg/CommandPathFollow.msg"
  
  # LLC
  "msg/LLCAck.msg"
  "msg/LLCBattery.msg"
  "msg/Sw485InOut.msg"
  "msg/LLCSw485Status.msg"
  "msg/LLCVersion.msg"
  "msg/ThrusterData.msg"
  "msg/LLCThrusters.msg"
  #"msg/LLCStatus.msg"
  "msg/LLCStatusFlags.msg"
  "msg/LLCBeep.msg"
  "msg/LLCConfig.msg"
  "msg/LLCEnableRef.msg"
  "msg/LLCPowerButtons.msg"
  #"msg/LLCPumps.msg"
  
  # Control
  "msg/SurgeHeading.msg"
  "msg/SurgeYawRate.msg"
  "msg/NavFilterData.msg"
  "msg/TaskStatus.msg"
  #"msg/Curves.msg"
  "msg/PathData.msg"
#  "msg/PolygonPath.msg"
  "msg/CoordinateList.msg"
  "msg/TPIKAction.msg"
  "msg/TPIKPriorityLevel.msg"
  "msg/Obstacle.msg"
  "msg/PathFollow.msg"

  ## Services ##
  "srv/ControlCommand.srv"
  "srv/LLCCommand.srv"
  "srv/RosbagCmd.srv"
  "srv/NavFilterCommand.srv" 
  "srv/SetBoundaries.srv"
  "srv/GetBoundaries.srv"
  "srv/SetCruiseControl.srv"
  "srv/ResetConfiguration.srv"
  "srv/MinSrv.srv"

  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
