/*
 * Copyright (C) 2013  CoDyCo Consortium
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
 * Authors: Andrea Del Prete
 * email: andrea.delprete@iit.it
 */

#ifndef WBI_CONSTANTS_H
#define WBI_CONSTANTS_H

#include <string>

namespace wbi
{
    /** List of available joint control modes. */
    enum ControlMode
    {
        CTRL_MODE_UNKNOWN,
        CTRL_MODE_MOTOR_PWM,    // motor PWM
        CTRL_MODE_POS,          // joint position
        CTRL_MODE_DIRECT_POSITION, //joint direct position (without trajectory generator)
        CTRL_MODE_VEL,          // joint velocity
        CTRL_MODE_TORQUE       // joint torque
    };
    const int CTRL_MODE_SIZE = 6; //number of elements in the ControlMode enum

    /** List of available parameters for the various control modes. */
    enum ControlParam
    {
        CTRL_PARAM_KP,          // proportional gain of PID
        CTRL_PARAM_KI,          // integral gain of PID
        CTRL_PARAM_KD,          // derivative gain of PID
        CTRL_PARAM_OFFSET,      // offset for the control
        CTRL_PARAM_STIFFNESS,   // stiffness of impedance controller
        CTRL_PARAM_DAMPING,     // damping of impedance controller
        CTRL_PARAM_REF_VEL,     // reference velocity for trajectory generator
        CTRL_PARAM_REF_ACC      // reference acceleration for trajectory generator
    };

    /** List of available sensor types. */
    enum SensorType
    {
        // JOINT SPACE SENSORS
        SENSOR_ENCODER,         // joint encoder
        SENSOR_PWM,             // motor PWM (proportional to motor voltage)
        SENSOR_CURRENT,         // motor current
        SENSOR_TORQUE,          // joint torque

        // CARTESIAN SPACE SENSORS
        SENSOR_IMU,             // 13d inertial measurement unit (4d: axis-angle orientation[rad], 3d: linear acc: ddp[m/s^2], 3d: angular vel: dw[rad/s], magnetometer[] )
        SENSOR_FORCE_TORQUE,    // 6-axis force/torque
        SENSOR_ACCELEROMETER    // 3d linear acceleration
    };
    const int SENSOR_TYPE_SIZE = 7; //number of elements in SensorType enum

    /** Collection of data to describe a sensor type. */
    class SensorTypeDescription
    {
    public:
        SensorType id;              // id associated to this sensor type
        std::string name;           // name
        std::string description;    // description
        int dataSize;               // size of the data vector returned by a sensor reading
        bool isJointSensor;         // true if this sensor type is associated to a joint

        SensorTypeDescription(SensorType _id, std::string _name, int _dataSize, bool _isJoint, std::string _descr="");
        bool operator ==(const SensorTypeDescription &st);
    };

    /** Descriptions of the available sensor types. */
    extern const SensorTypeDescription sensorTypeDescriptions[SENSOR_TYPE_SIZE];
//    =
//    {
//    SensorTypeDescription(SENSOR_ENCODER,       "encoder",          1, true,  "Joint position"),
//    SensorTypeDescription(SENSOR_PWM,           "PWM",              1, true,  "Motor PWM"),
//    SensorTypeDescription(SENSOR_CURRENT,       "current",          1, true,  "Motor current"),
//    SensorTypeDescription(SENSOR_TORQUE,        "torque",           1, true,  "Joint torque"),
//    SensorTypeDescription(SENSOR_IMU,           "IMU",              13, false, "Inertial Measurement Unit"),
//    SensorTypeDescription(SENSOR_FORCE_TORQUE,  "force-torque",     6, false, "6-axis force torque"),
//    SensorTypeDescription(SENSOR_ACCELEROMETER, "accelerometer",    3, false, "3d linear acceleration"),
//    };

    /** List of available estimates. */
    enum EstimateType
    {
        // JOINT SPACE ESTIMATES
        ESTIMATE_JOINT_POS,                 // joint position
        ESTIMATE_JOINT_VEL,                 // joint velocity
        ESTIMATE_JOINT_ACC,                 // joint acceleration
        ESTIMATE_JOINT_TORQUE,              // joint torque
        ESTIMATE_JOINT_TORQUE_DERIVATIVE,   // joint torque derivative
        // MOTOR SPACE ESTIMATES
        ESTIMATE_MOTOR_POS,                 // motor position
        ESTIMATE_MOTOR_VEL,                 // motor velocity
        ESTIMATE_MOTOR_ACC,                 // motor acceleration
        ESTIMATE_MOTOR_TORQUE,              // motor torque
        ESTIMATE_MOTOR_TORQUE_DERIVATIVE,   // motor torque derivative
        ESTIMATE_MOTOR_PWM,                 // motor PWM (proportional to motor voltage)
        ESTIMATE_MOTOR_CURRENT,             // motor current
        // CARTESIAN SPACE ESTIMATES
        ESTIMATE_FORCE_TORQUE_SENSOR,       // 6-axis force/torque sensor
        ESTIMATE_IMU,                       // Same of sensor IMU, but filtered
        ESTIMATE_ACCELERATION,              // 3d linear acceleration
        ESTIMATE_BASE_POS,                  // position of the base of the robot
        ESTIMATE_BASE_VEL,                  // velocity of the base of the robot
        ESTIMATE_BASE_ACC,                  // acceleration of the base of the robot
        ESTIMATE_EXTERNAL_FORCE_TORQUE      // 6-axis external force/torque acting on a link
    };
    const int ESTIMATE_TYPE_SIZE = 19; //number of elements in EstimateType enum

    /** List of parameters of estimation algorithms. */
    enum EstimationParameter
    {
        ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE,
        ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD,
        ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ,
        ESTIMATION_PARAM_ENABLE_OMEGA_IMU_DOMEGA_IMU,
        ESTIMATION_PARAM_MIN_TAXEL
    };

} // end namespace

#endif

