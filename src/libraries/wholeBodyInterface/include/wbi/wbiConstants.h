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
        CTRL_MODE_VEL,          // joint velocity
        CTRL_MODE_TORQUE,       // joint torque
        CTRL_MODE_SIZE
    };

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
        SENSOR_IMU,             // 9d inertial measurement unit (angular vel: w, angular acc: dw, linear acc: ddp)
        SENSOR_FORCE_TORQUE,    // 6-axis force/torque
        SENSOR_ACCELEROMETER,   // 3d linear acceleration

        SENSOR_TYPE_SIZE
    };

    /** Collection of data to describe a sensor type. */
    class SensorTypeDescription
    {
    public:
        SensorType id;              // id associated to this sensor type
        std::string name;           // name
        std::string description;    // description
        int dataSize;               // size of the data vector returned by a sensor reading
        bool isJointSensor;         // true if this sensor type is associated to a joint

        SensorTypeDescription(SensorType _id, std::string _name, int _dataSize, bool _isJoint, std::string _descr="")
            : id(_id), name(_name), description(_descr), dataSize(_dataSize), isJointSensor(_isJoint){}
        bool operator ==(const SensorTypeDescription &st){ return st.id==this->id; }
    };

    /** Descriptions of the available sensor types. */
    const SensorTypeDescription sensorTypeDescriptions[SENSOR_TYPE_SIZE]  = 
    { 
    SensorTypeDescription(SENSOR_ENCODER,       "encoder",          1, true,  "Joint position"), 
    SensorTypeDescription(SENSOR_PWM,           "PWM",              1, true,  "Motor PWM"), 
    SensorTypeDescription(SENSOR_CURRENT,       "current",          1, true,  "Motor current"), 
    SensorTypeDescription(SENSOR_TORQUE,        "torque",           1, true,  "Joint torque"), 
    SensorTypeDescription(SENSOR_IMU,           "IMU",              9, false, "Inertial Measurement Unit"), 
    SensorTypeDescription(SENSOR_FORCE_TORQUE,  "force-torque",     6, false, "6-axis force torque"), 
    SensorTypeDescription(SENSOR_ACCELEROMETER, "accelerometer",    3, false, "3d linear acceleration"), 
    };

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
        ESTIMATE_IMU,               // 9d inertial measurement unit (angular vel: w, angular acc: dw, linear acc: ddp)
        ESTIMATE_FORCE_TORQUE,      // 6-axis force/torque
        ESTIMATE_ACCELERATION,      // 3d linear acceleration
        ESTIMATE_BASE_POS,          // position of the base of the robot
        ESTIMATE_BASE_VEL,          // velocity of the base of the robot
        ESTIMATE_BASE_ACC,          // acceleration of the base of the robot

        ESTIMATE_TYPE_SIZE
    };

    /** List of parameters of estimation algorithms. */
    enum EstimationParameter
    {
        ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE,
        ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD,
        ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ
    };
    
} // end namespace

#endif

