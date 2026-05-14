cmake_minimum_required(VERSION 3.5)
project(ulisse_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

rosidl_generate_interfaces(ulisse_msgs
  # Sensors
  "msg/Time.msg"
  "msg/RPY.msg"
  "msg/LatLong.msg"
  "msg/PIDData.msg"
  "msg/IMUData.msg"
  "msg/AmbientSensors.msg"
  "msg/Compass.msg"
  "msg/Magnetometer.msg"
  "msg/ThrustersReference.msg"
  "msg/MicroLoopCount.msg"
  "msg/GPSData.msg"
  "msg/GPSStatus.msg"
  "msg/LatLongAlt.msg"
  "msg/ReferenceVelocities.msg"
  "msg/VehicleStatus.msg"
  "msg/ThrusterMappingControl.msg"
  "msg/SimulatedVelocitySensor.msg"
  "msg/FeedbackGui.msg"
  "msg/SimulatedSystem.msg"
  "msg/DynamicPidControl.msg"

  # Commands
  "msg/CommandHold.msg"
  "msg/CommandLatLong.msg"
  "msg/CommandSpeedHeading.msg"
  "msg/CommandNavigate.msg"
  
  # LLC
  "msg/LLCAck.msg"
  "msg/LLCBattery.msg"
  "msg/Sw485InOut.msg"
  "msg/LLCSw485Status.msg"
  "msg/LLCVersion.msg"
  "msg/ThrusterData.msg"
  "msg/LLCThrusters.msg"
  "msg/LLCStatus.msg"
  "msg/LLCStatusFlags.msg"
  "msg/LLCBeep.msg"
  "msg/LLCConfig.msg"
  "msg/LLCEnableRef.msg"
  "msg/LLCPowerButtons.msg"
  "msg/LLCPumps.msg"
  
  # Other
  "msg/SpeedHeading.msg"
  "msg/NavFilterData.msg"
  "msg/TaskStatus.msg"
  "msg/Curves.msg"
  "msg/Path.msg"
  "msg/Boundaries.msg"

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
)

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
