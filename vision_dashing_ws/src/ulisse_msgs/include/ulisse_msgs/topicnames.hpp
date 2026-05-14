#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace ulisse_msgs {

namespace topicnames {

    // LLC
    const std::string micro_loop_count = "/ulisse/llc/micro_loop_count";
    const std::string sensor_gps_data = "/ulisse/llc/sensor/gps_data";
    const std::string sensor_gps_status = "/ulisse/llc/sensor/gps_status";
    const std::string sensor_compass = "/ulisse/llc/sensor/compass";
    const std::string sensor_imu = "/ulisse/llc/sensor/imu";
    const std::string sensor_ambient = "/ulisse/llc/sensor/ambient";
    const std::string sensor_magnetometer = "/ulisse/llc/sensor/magnetometer";
    const std::string llc_thrusters_reference_perc = "/ulisse/llc/reference_thrusters_percentage"; // thrusters_data
    const std::string llc_thrusters_applied_perc = "/ulisse/llc/applied_thrusters_percentage";     // motor_applied_ref
    const std::string llc_status = "/ulisse/llc/status";
    const std::string llc_config = "/ulisse/llc/config";
    const std::string llc_thrusters = "/ulisse/llc/thrusters"; //llc_motors
    const std::string llc_version = "/ulisse/llc/version";
    const std::string llc_ack = "/ulisse/llc/ack";
    const std::string llc_battery_left = "/ulisse/llc/battery_left";
    const std::string llc_battery_right = "/ulisse/llc/battery_right";
    const std::string llc_sw485status = "/ulisse/llc/sw485status";

    // SIM
    const std::string simulated_system = "/ulisse/simulated_system";

    // NAV FILTER
    const std::string nav_filter_data = "/ulisse/nav_filter/data";

    // CTRL
    const std::string vehicle_status = "/ulisse/ctrl/vehicle_status";
    const std::string thruster_mapping_control = "/ulisse/ctrl/thruster_mapping_info";
    const std::string classic_pid_control = "/ulisse/ctrl/classic_pid_control_info";
    const std::string computed_torque_control = "/ulisse/ctrl/computed_torque_control_info";
    const std::string sliding_mode_control = "/ulisse/ctrl/sliding_mode_info";
    const std::string reference_velocities = "/ulisse/ctrl/reference_velocities";
    const std::string simulated_velocity_sensor = "/ulisse/ctrl/water_relative_surge";
    const std::string feedback_gui = "/ulisse/ctrl/feedback_gui";
    const std::string set_cruise_control_service = "/ulisse/ctrl/set_cruise_control";
    const std::string reset_kcl_conf_service = "/ulisse/ctrl/reset_kcl_configuration";
    const std::string reset_dcl_conf_service = "/ulisse/ctrl/reset_dcl_configuration";
    const std::string speed_heading = "/ulisse/ctrl/speed_heading";

    // SERVICES
    const std::string rosbag_service = "/record_bag";
    const std::string llc_cmd_service = "/ulisse/service/llc_cmd";
    const std::string control_cmd_service = "/ulisse/service/control_cmd";
    const std::string navfilter_cmd_service = "/ulisse/service/navfilter_cmd";
    const std::string set_boundaries_service = "/ulisse/ctrl/set_boundaries";
    const std::string get_boundaries_service = "/ulisse/ctrl/get_boundaries";
}
}

#endif // TOPICNAMES_HPP
