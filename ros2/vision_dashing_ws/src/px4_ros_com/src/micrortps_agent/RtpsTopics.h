/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <fastcdr/Cdr.h>
#include <condition_variable>
#include <queue>

#include "microRTPS_timesync.h"

#include "ActuatorArmed_Publisher.h"
#include "AdcReport_Publisher.h"
#include "Airspeed_Publisher.h"
#include "BatteryStatus_Publisher.h"
#include "Cpuload_Publisher.h"
#include "DistanceSensor_Publisher.h"
#include "EstimatorStatus_Publisher.h"
#include "HomePosition_Publisher.h"
#include "IridiumsbdStatus_Publisher.h"
#include "RadioStatus_Publisher.h"
#include "SatelliteInfo_Publisher.h"
#include "SensorBaro_Publisher.h"
#include "SensorCombined_Publisher.h"
#include "SensorSelection_Publisher.h"
#include "Timesync_Publisher.h"
#include "VehicleAttitude_Publisher.h"
#include "VehicleControlMode_Publisher.h"
#include "VehicleGpsPosition_Publisher.h"
#include "VehicleLandDetected_Publisher.h"
#include "VehicleOdometry_Publisher.h"
#include "VtolVehicleStatus_Publisher.h"
#include "WindEstimate_Publisher.h"
#include "CollisionConstraints_Publisher.h"
#include "CameraCapture_Subscriber.h"
#include "CameraTrigger_Subscriber.h"
#include "CollisionReport_Subscriber.h"
#include "DebugArray_Subscriber.h"
#include "DebugKeyValue_Subscriber.h"
#include "DebugValue_Subscriber.h"
#include "DebugVect_Subscriber.h"
#include "ObstacleDistance_Subscriber.h"
#include "OpticalFlow_Subscriber.h"
#include "PositionSetpoint_Subscriber.h"
#include "PositionSetpointTriplet_Subscriber.h"
#include "Timesync_Subscriber.h"
#include "TrajectoryWaypoint_Subscriber.h"
#include "VehicleAttitudeSetpoint_Subscriber.h"
#include "VehicleLocalPositionSetpoint_Subscriber.h"
#include "VehicleTrajectoryWaypoint_Subscriber.h"
#include "OnboardComputerStatus_Subscriber.h"
#include "VehicleMocapOdometry_Subscriber.h"
#include "VehicleVisualOdometry_Subscriber.h"
#include "TrajectorySetpoint_Subscriber.h"


using CameraCapture_msg_t = px4_msgs::msg::CameraCapture;
using CameraTrigger_msg_t = px4_msgs::msg::CameraTrigger;
using CollisionReport_msg_t = px4_msgs::msg::CollisionReport;
using DebugArray_msg_t = px4_msgs::msg::DebugArray;
using DebugKeyValue_msg_t = px4_msgs::msg::DebugKeyValue;
using DebugValue_msg_t = px4_msgs::msg::DebugValue;
using DebugVect_msg_t = px4_msgs::msg::DebugVect;
using ObstacleDistance_msg_t = px4_msgs::msg::ObstacleDistance;
using OpticalFlow_msg_t = px4_msgs::msg::OpticalFlow;
using PositionSetpoint_msg_t = px4_msgs::msg::PositionSetpoint;
using PositionSetpointTriplet_msg_t = px4_msgs::msg::PositionSetpointTriplet;
using Timesync_msg_t = px4_msgs::msg::Timesync;
using TrajectoryWaypoint_msg_t = px4_msgs::msg::TrajectoryWaypoint;
using VehicleAttitudeSetpoint_msg_t = px4_msgs::msg::VehicleAttitudeSetpoint;
using VehicleLocalPositionSetpoint_msg_t = px4_msgs::msg::VehicleLocalPositionSetpoint;
using VehicleTrajectoryWaypoint_msg_t = px4_msgs::msg::VehicleTrajectoryWaypoint;
using OnboardComputerStatus_msg_t = px4_msgs::msg::OnboardComputerStatus;
using VehicleMocapOdometry_msg_t = px4_msgs::msg::VehicleMocapOdometry;
using VehicleVisualOdometry_msg_t = px4_msgs::msg::VehicleVisualOdometry;
using TrajectorySetpoint_msg_t = px4_msgs::msg::TrajectorySetpoint;
using ActuatorArmed_msg_t = px4_msgs::msg::ActuatorArmed;
using AdcReport_msg_t = px4_msgs::msg::AdcReport;
using Airspeed_msg_t = px4_msgs::msg::Airspeed;
using BatteryStatus_msg_t = px4_msgs::msg::BatteryStatus;
using Cpuload_msg_t = px4_msgs::msg::Cpuload;
using DistanceSensor_msg_t = px4_msgs::msg::DistanceSensor;
using EstimatorStatus_msg_t = px4_msgs::msg::EstimatorStatus;
using HomePosition_msg_t = px4_msgs::msg::HomePosition;
using IridiumsbdStatus_msg_t = px4_msgs::msg::IridiumsbdStatus;
using RadioStatus_msg_t = px4_msgs::msg::RadioStatus;
using SatelliteInfo_msg_t = px4_msgs::msg::SatelliteInfo;
using SensorBaro_msg_t = px4_msgs::msg::SensorBaro;
using SensorCombined_msg_t = px4_msgs::msg::SensorCombined;
using SensorSelection_msg_t = px4_msgs::msg::SensorSelection;
using Timesync_msg_t = px4_msgs::msg::Timesync;
using VehicleAttitude_msg_t = px4_msgs::msg::VehicleAttitude;
using VehicleControlMode_msg_t = px4_msgs::msg::VehicleControlMode;
using VehicleGpsPosition_msg_t = px4_msgs::msg::VehicleGpsPosition;
using VehicleLandDetected_msg_t = px4_msgs::msg::VehicleLandDetected;
using VehicleOdometry_msg_t = px4_msgs::msg::VehicleOdometry;
using VtolVehicleStatus_msg_t = px4_msgs::msg::VtolVehicleStatus;
using WindEstimate_msg_t = px4_msgs::msg::WindEstimate;
using CollisionConstraints_msg_t = px4_msgs::msg::CollisionConstraints;

class RtpsTopics {
public:
    bool init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue);
    void set_timesync(const std::shared_ptr<TimeSync>& timesync) { _timesync = timesync; };
    void publish(uint8_t topic_ID, char data_buffer[], size_t len);
    bool getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr);

private:
    /** Publishers **/
    ActuatorArmed_Publisher _ActuatorArmed_pub;
    AdcReport_Publisher _AdcReport_pub;
    Airspeed_Publisher _Airspeed_pub;
    BatteryStatus_Publisher _BatteryStatus_pub;
    Cpuload_Publisher _Cpuload_pub;
    DistanceSensor_Publisher _DistanceSensor_pub;
    EstimatorStatus_Publisher _EstimatorStatus_pub;
    HomePosition_Publisher _HomePosition_pub;
    IridiumsbdStatus_Publisher _IridiumsbdStatus_pub;
    RadioStatus_Publisher _RadioStatus_pub;
    SatelliteInfo_Publisher _SatelliteInfo_pub;
    SensorBaro_Publisher _SensorBaro_pub;
    SensorCombined_Publisher _SensorCombined_pub;
    SensorSelection_Publisher _SensorSelection_pub;
    Timesync_Publisher _Timesync_pub;
    VehicleAttitude_Publisher _VehicleAttitude_pub;
    VehicleControlMode_Publisher _VehicleControlMode_pub;
    VehicleGpsPosition_Publisher _VehicleGpsPosition_pub;
    VehicleLandDetected_Publisher _VehicleLandDetected_pub;
    VehicleOdometry_Publisher _VehicleOdometry_pub;
    VtolVehicleStatus_Publisher _VtolVehicleStatus_pub;
    WindEstimate_Publisher _WindEstimate_pub;
    CollisionConstraints_Publisher _CollisionConstraints_pub;

    /** Subscribers **/
    CameraCapture_Subscriber _CameraCapture_sub;
    CameraTrigger_Subscriber _CameraTrigger_sub;
    CollisionReport_Subscriber _CollisionReport_sub;
    DebugArray_Subscriber _DebugArray_sub;
    DebugKeyValue_Subscriber _DebugKeyValue_sub;
    DebugValue_Subscriber _DebugValue_sub;
    DebugVect_Subscriber _DebugVect_sub;
    ObstacleDistance_Subscriber _ObstacleDistance_sub;
    OpticalFlow_Subscriber _OpticalFlow_sub;
    PositionSetpoint_Subscriber _PositionSetpoint_sub;
    PositionSetpointTriplet_Subscriber _PositionSetpointTriplet_sub;
    Timesync_Subscriber _Timesync_sub;
    TrajectoryWaypoint_Subscriber _TrajectoryWaypoint_sub;
    VehicleAttitudeSetpoint_Subscriber _VehicleAttitudeSetpoint_sub;
    VehicleLocalPositionSetpoint_Subscriber _VehicleLocalPositionSetpoint_sub;
    VehicleTrajectoryWaypoint_Subscriber _VehicleTrajectoryWaypoint_sub;
    OnboardComputerStatus_Subscriber _OnboardComputerStatus_sub;
    VehicleMocapOdometry_Subscriber _VehicleMocapOdometry_sub;
    VehicleVisualOdometry_Subscriber _VehicleVisualOdometry_sub;
    TrajectorySetpoint_Subscriber _TrajectorySetpoint_sub;

    /** Msg metada Getters **/
    template <class T>
    inline uint64_t getMsgTimestamp(const T* msg) { return msg->timestamp(); }

    template <class T>
    inline uint8_t getMsgSysID(const T* msg) { return msg->sys_id(); }

    template <class T>
    inline uint8_t getMsgSeq(const T* msg) { return msg->seq(); }

    /** Msg metadata Setters **/
    template <class T>
    inline void setMsgTimestamp(T* msg, const uint64_t& timestamp) { msg->timestamp() = timestamp; }

    template <class T>
    inline void setMsgSysID(T* msg, const uint8_t& sys_id) { msg->sys_id() = sys_id; }

    template <class T>
    inline void setMsgSeq(T* msg, const uint8_t& seq) { msg->seq() = seq; }

    /**
     * @brief Timesync object ptr.
     *         This object is used to compuyte and apply the time offsets to the
     *         messages timestamps.
     */
    std::shared_ptr<TimeSync> _timesync;
};
