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

#include "RtpsTopics.h"

bool RtpsTopics::init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue)
{
    // Initialise subscribers
    std::cout << "--- Subscribers ---" << std::endl;
    if (_CameraCapture_sub.init(7, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- CameraCapture subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting CameraCapture subscriber" << std::endl;
        return false;
    }
    if (_CameraTrigger_sub.init(8, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- CameraTrigger subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting CameraTrigger subscriber" << std::endl;
        return false;
    }
    if (_CollisionReport_sub.init(9, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- CollisionReport subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting CollisionReport subscriber" << std::endl;
        return false;
    }
    if (_DebugArray_sub.init(12, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- DebugArray subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting DebugArray subscriber" << std::endl;
        return false;
    }
    if (_DebugKeyValue_sub.init(13, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- DebugKeyValue subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting DebugKeyValue subscriber" << std::endl;
        return false;
    }
    if (_DebugValue_sub.init(14, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- DebugValue subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting DebugValue subscriber" << std::endl;
        return false;
    }
    if (_DebugVect_sub.init(15, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- DebugVect subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting DebugVect subscriber" << std::endl;
        return false;
    }
    if (_ObstacleDistance_sub.init(43, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- ObstacleDistance subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting ObstacleDistance subscriber" << std::endl;
        return false;
    }
    if (_OpticalFlow_sub.init(45, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- OpticalFlow subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting OpticalFlow subscriber" << std::endl;
        return false;
    }
    if (_PositionSetpoint_sub.init(50, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- PositionSetpoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting PositionSetpoint subscriber" << std::endl;
        return false;
    }
    if (_PositionSetpointTriplet_sub.init(51, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- PositionSetpointTriplet subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting PositionSetpointTriplet subscriber" << std::endl;
        return false;
    }
    if (_Timesync_sub.init(78, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- Timesync subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting Timesync subscriber" << std::endl;
        return false;
    }
    if (_TrajectoryWaypoint_sub.init(79, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- TrajectoryWaypoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting TrajectoryWaypoint subscriber" << std::endl;
        return false;
    }
    if (_VehicleAttitudeSetpoint_sub.init(88, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- VehicleAttitudeSetpoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting VehicleAttitudeSetpoint subscriber" << std::endl;
        return false;
    }
    if (_VehicleLocalPositionSetpoint_sub.init(97, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- VehicleLocalPositionSetpoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting VehicleLocalPositionSetpoint subscriber" << std::endl;
        return false;
    }
    if (_VehicleTrajectoryWaypoint_sub.init(104, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- VehicleTrajectoryWaypoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting VehicleTrajectoryWaypoint subscriber" << std::endl;
        return false;
    }
    if (_OnboardComputerStatus_sub.init(116, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- OnboardComputerStatus subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting OnboardComputerStatus subscriber" << std::endl;
        return false;
    }
    if (_VehicleMocapOdometry_sub.init(161, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- VehicleMocapOdometry subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting VehicleMocapOdometry subscriber" << std::endl;
        return false;
    }
    if (_VehicleVisualOdometry_sub.init(162, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- VehicleVisualOdometry subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting VehicleVisualOdometry subscriber" << std::endl;
        return false;
    }
    if (_TrajectorySetpoint_sub.init(166, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- TrajectorySetpoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting TrajectorySetpoint subscriber" << std::endl;
        return false;
    }
    std::cout << "--------------------" << std::endl << std::endl;
    // Initialise publishers
    std::cout << "---- Publishers ----" << std::endl;
    if (_ActuatorArmed_pub.init()) {
        std::cout << "- ActuatorArmed publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting ActuatorArmed publisher" << std::endl;
        return false;
    }
    if (_AdcReport_pub.init()) {
        std::cout << "- AdcReport publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting AdcReport publisher" << std::endl;
        return false;
    }
    if (_Airspeed_pub.init()) {
        std::cout << "- Airspeed publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting Airspeed publisher" << std::endl;
        return false;
    }
    if (_BatteryStatus_pub.init()) {
        std::cout << "- BatteryStatus publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting BatteryStatus publisher" << std::endl;
        return false;
    }
    if (_Cpuload_pub.init()) {
        std::cout << "- Cpuload publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting Cpuload publisher" << std::endl;
        return false;
    }
    if (_DistanceSensor_pub.init()) {
        std::cout << "- DistanceSensor publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting DistanceSensor publisher" << std::endl;
        return false;
    }
    if (_EstimatorStatus_pub.init()) {
        std::cout << "- EstimatorStatus publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting EstimatorStatus publisher" << std::endl;
        return false;
    }
    if (_HomePosition_pub.init()) {
        std::cout << "- HomePosition publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting HomePosition publisher" << std::endl;
        return false;
    }
    if (_IridiumsbdStatus_pub.init()) {
        std::cout << "- IridiumsbdStatus publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting IridiumsbdStatus publisher" << std::endl;
        return false;
    }
    if (_RadioStatus_pub.init()) {
        std::cout << "- RadioStatus publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting RadioStatus publisher" << std::endl;
        return false;
    }
    if (_SatelliteInfo_pub.init()) {
        std::cout << "- SatelliteInfo publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting SatelliteInfo publisher" << std::endl;
        return false;
    }
    if (_SensorBaro_pub.init()) {
        std::cout << "- SensorBaro publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting SensorBaro publisher" << std::endl;
        return false;
    }
    if (_SensorCombined_pub.init()) {
        std::cout << "- SensorCombined publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting SensorCombined publisher" << std::endl;
        return false;
    }
    if (_SensorSelection_pub.init()) {
        std::cout << "- SensorSelection publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting SensorSelection publisher" << std::endl;
        return false;
    }
    if (_Timesync_pub.init()) {
        std::cout << "- Timesync publisher started" << std::endl;
        _timesync->start(&_Timesync_pub);
    } else {
        std::cerr << "ERROR starting Timesync publisher" << std::endl;
        return false;
    }
    if (_VehicleAttitude_pub.init()) {
        std::cout << "- VehicleAttitude publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting VehicleAttitude publisher" << std::endl;
        return false;
    }
    if (_VehicleControlMode_pub.init()) {
        std::cout << "- VehicleControlMode publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting VehicleControlMode publisher" << std::endl;
        return false;
    }
    if (_VehicleGpsPosition_pub.init()) {
        std::cout << "- VehicleGpsPosition publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting VehicleGpsPosition publisher" << std::endl;
        return false;
    }
    if (_VehicleLandDetected_pub.init()) {
        std::cout << "- VehicleLandDetected publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting VehicleLandDetected publisher" << std::endl;
        return false;
    }
    if (_VehicleOdometry_pub.init()) {
        std::cout << "- VehicleOdometry publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting VehicleOdometry publisher" << std::endl;
        return false;
    }
    if (_VtolVehicleStatus_pub.init()) {
        std::cout << "- VtolVehicleStatus publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting VtolVehicleStatus publisher" << std::endl;
        return false;
    }
    if (_WindEstimate_pub.init()) {
        std::cout << "- WindEstimate publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting WindEstimate publisher" << std::endl;
        return false;
    }
    if (_CollisionConstraints_pub.init()) {
        std::cout << "- CollisionConstraints publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting CollisionConstraints publisher" << std::endl;
        return false;
    }
    std::cout << "--------------------" << std::endl;
    return true;
}

void RtpsTopics::publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
        case 0: // ActuatorArmed
        {
            ActuatorArmed_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _ActuatorArmed_pub.publish(&st);
        }
        break;
        case 4: // AdcReport
        {
            AdcReport_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _AdcReport_pub.publish(&st);
        }
        break;
        case 5: // Airspeed
        {
            Airspeed_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _Airspeed_pub.publish(&st);
        }
        break;
        case 6: // BatteryStatus
        {
            BatteryStatus_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _BatteryStatus_pub.publish(&st);
        }
        break;
        case 11: // Cpuload
        {
            Cpuload_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _Cpuload_pub.publish(&st);
        }
        break;
        case 17: // DistanceSensor
        {
            DistanceSensor_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _DistanceSensor_pub.publish(&st);
        }
        break;
        case 24: // EstimatorStatus
        {
            EstimatorStatus_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _EstimatorStatus_pub.publish(&st);
        }
        break;
        case 29: // HomePosition
        {
            HomePosition_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _HomePosition_pub.publish(&st);
        }
        break;
        case 31: // IridiumsbdStatus
        {
            IridiumsbdStatus_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _IridiumsbdStatus_pub.publish(&st);
        }
        break;
        case 56: // RadioStatus
        {
            RadioStatus_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _RadioStatus_pub.publish(&st);
        }
        break;
        case 61: // SatelliteInfo
        {
            SatelliteInfo_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _SatelliteInfo_pub.publish(&st);
        }
        break;
        case 63: // SensorBaro
        {
            SensorBaro_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _SensorBaro_pub.publish(&st);
        }
        break;
        case 65: // SensorCombined
        {
            SensorCombined_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _SensorCombined_pub.publish(&st);
        }
        break;
        case 70: // SensorSelection
        {
            SensorSelection_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _SensorSelection_pub.publish(&st);
        }
        break;
        case 78: // Timesync
        {
            Timesync_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _timesync->processTimesyncMsg(&st);

            if (getMsgSysID(&st) == 1) {
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _Timesync_pub.publish(&st);
            }
        }
        break;
        case 87: // VehicleAttitude
        {
            VehicleAttitude_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _VehicleAttitude_pub.publish(&st);
        }
        break;
        case 92: // VehicleControlMode
        {
            VehicleControlMode_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _VehicleControlMode_pub.publish(&st);
        }
        break;
        case 94: // VehicleGpsPosition
        {
            VehicleGpsPosition_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _VehicleGpsPosition_pub.publish(&st);
        }
        break;
        case 95: // VehicleLandDetected
        {
            VehicleLandDetected_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _VehicleLandDetected_pub.publish(&st);
        }
        break;
        case 99: // VehicleOdometry
        {
            VehicleOdometry_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _VehicleOdometry_pub.publish(&st);
        }
        break;
        case 105: // VtolVehicleStatus
        {
            VtolVehicleStatus_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _VtolVehicleStatus_pub.publish(&st);
        }
        break;
        case 106: // WindEstimate
        {
            WindEstimate_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _WindEstimate_pub.publish(&st);
        }
        break;
        case 107: // CollisionConstraints
        {
            CollisionConstraints_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _CollisionConstraints_pub.publish(&st);
        }
        break;
        default:
            printf("Unexpected topic ID to publish\n");
        break;
    }
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
    bool ret = false;
    switch (topic_ID)
    {
        case 7: // CameraCapture
            if (_CameraCapture_sub.hasMsg())
            {
                CameraCapture_msg_t msg = _CameraCapture_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _CameraCapture_sub.unlockMsg();
            }
        break;
        case 8: // CameraTrigger
            if (_CameraTrigger_sub.hasMsg())
            {
                CameraTrigger_msg_t msg = _CameraTrigger_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _CameraTrigger_sub.unlockMsg();
            }
        break;
        case 9: // CollisionReport
            if (_CollisionReport_sub.hasMsg())
            {
                CollisionReport_msg_t msg = _CollisionReport_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _CollisionReport_sub.unlockMsg();
            }
        break;
        case 12: // DebugArray
            if (_DebugArray_sub.hasMsg())
            {
                DebugArray_msg_t msg = _DebugArray_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _DebugArray_sub.unlockMsg();
            }
        break;
        case 13: // DebugKeyValue
            if (_DebugKeyValue_sub.hasMsg())
            {
                DebugKeyValue_msg_t msg = _DebugKeyValue_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _DebugKeyValue_sub.unlockMsg();
            }
        break;
        case 14: // DebugValue
            if (_DebugValue_sub.hasMsg())
            {
                DebugValue_msg_t msg = _DebugValue_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _DebugValue_sub.unlockMsg();
            }
        break;
        case 15: // DebugVect
            if (_DebugVect_sub.hasMsg())
            {
                DebugVect_msg_t msg = _DebugVect_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _DebugVect_sub.unlockMsg();
            }
        break;
        case 43: // ObstacleDistance
            if (_ObstacleDistance_sub.hasMsg())
            {
                ObstacleDistance_msg_t msg = _ObstacleDistance_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _ObstacleDistance_sub.unlockMsg();
            }
        break;
        case 45: // OpticalFlow
            if (_OpticalFlow_sub.hasMsg())
            {
                OpticalFlow_msg_t msg = _OpticalFlow_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _OpticalFlow_sub.unlockMsg();
            }
        break;
        case 50: // PositionSetpoint
            if (_PositionSetpoint_sub.hasMsg())
            {
                PositionSetpoint_msg_t msg = _PositionSetpoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _PositionSetpoint_sub.unlockMsg();
            }
        break;
        case 51: // PositionSetpointTriplet
            if (_PositionSetpointTriplet_sub.hasMsg())
            {
                PositionSetpointTriplet_msg_t msg = _PositionSetpointTriplet_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _PositionSetpointTriplet_sub.unlockMsg();
            }
        break;
        case 78: // Timesync
            if (_Timesync_sub.hasMsg())
            {
                Timesync_msg_t msg = _Timesync_sub.getMsg();
                if (getMsgSysID(&msg) == 0) {
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                }
                _Timesync_sub.unlockMsg();
            }
        break;
        case 79: // TrajectoryWaypoint
            if (_TrajectoryWaypoint_sub.hasMsg())
            {
                TrajectoryWaypoint_msg_t msg = _TrajectoryWaypoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _TrajectoryWaypoint_sub.unlockMsg();
            }
        break;
        case 88: // VehicleAttitudeSetpoint
            if (_VehicleAttitudeSetpoint_sub.hasMsg())
            {
                VehicleAttitudeSetpoint_msg_t msg = _VehicleAttitudeSetpoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _VehicleAttitudeSetpoint_sub.unlockMsg();
            }
        break;
        case 97: // VehicleLocalPositionSetpoint
            if (_VehicleLocalPositionSetpoint_sub.hasMsg())
            {
                VehicleLocalPositionSetpoint_msg_t msg = _VehicleLocalPositionSetpoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _VehicleLocalPositionSetpoint_sub.unlockMsg();
            }
        break;
        case 104: // VehicleTrajectoryWaypoint
            if (_VehicleTrajectoryWaypoint_sub.hasMsg())
            {
                VehicleTrajectoryWaypoint_msg_t msg = _VehicleTrajectoryWaypoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _VehicleTrajectoryWaypoint_sub.unlockMsg();
            }
        break;
        case 116: // OnboardComputerStatus
            if (_OnboardComputerStatus_sub.hasMsg())
            {
                OnboardComputerStatus_msg_t msg = _OnboardComputerStatus_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _OnboardComputerStatus_sub.unlockMsg();
            }
        break;
        case 161: // VehicleMocapOdometry
            if (_VehicleMocapOdometry_sub.hasMsg())
            {
                VehicleMocapOdometry_msg_t msg = _VehicleMocapOdometry_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _VehicleMocapOdometry_sub.unlockMsg();
            }
        break;
        case 162: // VehicleVisualOdometry
            if (_VehicleVisualOdometry_sub.hasMsg())
            {
                VehicleVisualOdometry_msg_t msg = _VehicleVisualOdometry_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _VehicleVisualOdometry_sub.unlockMsg();
            }
        break;
        case 166: // TrajectorySetpoint
            if (_TrajectorySetpoint_sub.hasMsg())
            {
                TrajectorySetpoint_msg_t msg = _TrajectorySetpoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _TrajectorySetpoint_sub.unlockMsg();
            }
        break;
        default:
            printf("Unexpected topic ID '%hhu' to getMsg\n", topic_ID);
        break;
    }

    return ret;
}
