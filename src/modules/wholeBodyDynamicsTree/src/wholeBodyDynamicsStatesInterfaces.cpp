/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
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
 */

#include "wholeBodyDynamicsTree/wholeBodyDynamicsStatesInterface.h"

#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include <iCub/skinDynLib/common.h>

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/ResourceFinder.h>

#include <string>
#include <iostream>
#include <yarp/os/Log.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <kdl/frames_io.hpp>

#define INITIAL_TIMESTAMP -1000.0


using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

/// < \todo TODO make it a proper parameter
#define ESTIMATOR_PERIOD 10


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
wholeBodyDynamicsStatesInterface::wholeBodyDynamicsStatesInterface(const char* _name,
                                                   yarp::os::Property & _wbi_yarp_conf)
{
    sensors = new yarpWholeBodySensors(_name, _wbi_yarp_conf);              // sensor interface
    skin_contacts_port = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    skin_contacts_port->open(string("/"+string(_name)+"/skin_contacts:i").c_str());
    estimator = new yarpWholeBodyDynamicsEstimator(ESTIMATOR_PERIOD, sensors, skin_contacts_port, _wbi_yarp_conf);  // estimation thread
}

bool wholeBodyDynamicsStatesInterface::init()
{
    bool ok = sensors->init();              // initialize sensor interface
    if( !ok )
    {
        std::cerr << "wholeBodyDynamicsStatesInterface::init() failed: error in sensor initialization." << std::endl;
        close();
        return false;
    }

    ok = estimator->start();
    if( !ok )
    {
        std::cerr << "wholeBodyDynamicsStatesInterface::init() failed: error in estimator initialization." << std::endl;
        close();
        return false;
    }

    return ok; // start estimation thread
}

bool wholeBodyDynamicsStatesInterface::close()
{
    std::cout << "[INFO]wholeBodyDynamicsStatesInterface::close() : closing estimator thread" << std::endl;
    if(estimator) estimator->stop();  // stop estimator BEFORE closing sensor interface
    std::cout << "[INFO]wholeBodyDynamicsStatesInterface::close() : closing sensor interface" << std::endl;
    bool ok = (sensors ? sensors->close() : true);
    std::cout << "[INFO]wholeBodyDynamicsStatesInterface::close() : closing skin_contacts_port" << std::endl;
    if( skin_contacts_port )
    {
        skin_contacts_port->close();
        delete skin_contacts_port;
        skin_contacts_port = 0;
    }
    std::cout << "wholeBodyDynamicsStatesInterface::close() : deleting sensor interface" << std::endl;
    if(sensors) { delete sensors; sensors = 0; }
    std::cout << "wholeBodyDynamicsStatesInterface::close() : deleting estimator thread" << std::endl;
    if(estimator) { delete estimator; estimator = 0; }
    return ok;
}

bool wholeBodyDynamicsStatesInterface::addEstimate(const EstimateType et, const ID &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndAddSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:             return lockAndAddSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

int wholeBodyDynamicsStatesInterface::addEstimates(const EstimateType et, const IDList &sids)
{
    //\todo TODO properly handle dependencies
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensors(SENSOR_PWM, sids);
    case ESTIMATE_IMU:                      return lockAndAddSensors(SENSOR_IMU, sids);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndAddSensors(SENSOR_FORCE_TORQUE, sids);
    default: break;
    }
    return false;
}

bool wholeBodyDynamicsStatesInterface::removeEstimate(const EstimateType et, const ID &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndRemoveSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndRemoveSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndRemoveSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

const IDList& wholeBodyDynamicsStatesInterface::getEstimateList(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorList(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorList(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorList(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return emptyList;
}

int wholeBodyDynamicsStatesInterface::getEstimateNumber(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorNumber(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorNumber(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorNumber(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return 0;
}

bool wholeBodyDynamicsStatesInterface::getEstimate(const EstimateType et, const int numeric_id, double *data, double time, bool blocking)
{
    wbi::ID sid;
    switch(et)
    {
    case ESTIMATE_JOINT_POS:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id,estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:
        return false;
    case ESTIMATE_MOTOR_VEL:
        return getMotorVel(numeric_id, data, time, blocking);
    case ESTIMATE_MOTOR_ACC:
        return false;
    case ESTIMATE_MOTOR_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:
        return lockAndReadSensor(SENSOR_PWM, numeric_id, data, time, blocking);
    case ESTIMATE_IMU:
        return estimator->lockAndCopyElementVectorFromVector(numeric_id, estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:
        return estimator->lockAndCopyElementVectorFromVector(numeric_id, estimator->estimates.lastForceTorques, data);
    case ESTIMATE_EXTERNAL_FORCE_TORQUE:
        return estimator->lockAndCopyExternalForceTorque(numeric_id,data);
    default: break;
    }
    return false;
}

bool wholeBodyDynamicsStatesInterface::getEstimates(const EstimateType et, double *data, double time, bool blocking)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return estimator->lockAndCopyVector(estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:                return estimator->lockAndCopyVector(estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:                return estimator->lockAndCopyVector(estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:             return estimator->lockAndCopyVector(estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return estimator->lockAndCopyVector(estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:                return false;
    case ESTIMATE_MOTOR_VEL:                return getMotorVel(data, time, blocking);
    case ESTIMATE_MOTOR_ACC:                return false;
    case ESTIMATE_MOTOR_TORQUE:             return estimator->lockAndCopyVector(estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return estimator->lockAndCopyVector(estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:                return lockAndReadSensors(SENSOR_PWM, data, time, blocking);
    case ESTIMATE_IMU:                      return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastForceTorques, data);
    default: break;
    }
    return false;
}

bool wholeBodyDynamicsStatesInterface::setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    return estimator->lockAndSetEstimationParameter(et, ep, value);
}

bool wholeBodyDynamicsStatesInterface::setEstimationOffset(const EstimateType et, const ID & sid, const double *value)
{
    return estimator->lockAndSetEstimationOffset(et,sid,value);
}

bool wholeBodyDynamicsStatesInterface::getEstimationOffset(const EstimateType et, const ID & sid, double *value)
{
    return estimator->lockAndGetEstimationOffset(et,sid,value);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                    IMPLEMENTATION SPECIFIC METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************


bool wholeBodyDynamicsStatesInterface::getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    return lockAndReadExternalForces(external_forces_list);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          PRIVATE METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************

bool wholeBodyDynamicsStatesInterface::getMotorVel(double *data, double time, bool blocking)
{
    bool res = estimator->lockAndCopyVector(estimator->estimates.lastDq, data);    ///< read joint vel
    if(!res) return false;
    IDList idList = lockAndGetSensorList(SENSOR_ENCODER);
    return true;
}

bool wholeBodyDynamicsStatesInterface::getMotorVel(const int numeric_id, double *data, double time, bool blocking)
{
    ///< read joint vel
    return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
}

bool wholeBodyDynamicsStatesInterface::lockAndReadSensors(const SensorType st, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensors(st, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool wholeBodyDynamicsStatesInterface::lockAndReadSensor(const SensorType st, const int numeric_id, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensor(st, numeric_id, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool wholeBodyDynamicsStatesInterface::lockAndReadExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    estimator->mutex.wait();
    external_forces_list = estimator->estimatedLastSkinDynContacts;
    estimator->mutex.post();
    return true;
}

bool wholeBodyDynamicsStatesInterface::lockAndAddSensor(const SensorType st, const ID &sid)
{
    estimator->mutex.wait();
    bool res = sensors->addSensor(st, sid);
    estimator->mutex.post();
    return res;
}

int wholeBodyDynamicsStatesInterface::lockAndAddSensors(const SensorType st, const IDList &sids)
{
    estimator->mutex.wait();
    int res = sensors->addSensors(st, sids);
    estimator->mutex.post();
    return res;
}

bool wholeBodyDynamicsStatesInterface::lockAndRemoveSensor(const SensorType st, const ID &sid)
{
    estimator->mutex.wait();
    bool res = sensors->removeSensor(st, sid);
    estimator->mutex.post();
    return res;
}

IDList wholeBodyDynamicsStatesInterface::lockAndGetSensorList(const SensorType st)
{
    estimator->mutex.wait();
    IDList res = sensors->getSensorList(st);
    estimator->mutex.post();
    return res;
}

int wholeBodyDynamicsStatesInterface::lockAndGetSensorNumber(const SensorType st)
{
    estimator->mutex.wait();
    int res = sensors->getSensorNumber(st);
    estimator->mutex.post();
    return res;
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY DYNAMICS ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyDynamicsEstimator::yarpWholeBodyDynamicsEstimator(int _period,
                                                               yarpWholeBodySensors *_sensors,
                                                               yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * _port_skin_contacts,
                                                               yarp::os::Property & _wbi_yarp_conf
                                                              )
: RateThread(_period),
   sensors(_sensors),
   port_skin_contacts(_port_skin_contacts),
   dqFilt(0), d2qFilt(0),
   enable_omega_domega_IMU(false),
   min_taxel(0),
   wbi_yarp_conf(_wbi_yarp_conf)
{

    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
    resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

    ///< Window lengths of adaptive window filters
    dqFiltWL            = 16;
    d2qFiltWL           = 25;
    dTauJFiltWL         = 30;
    dTauMFiltWL         = 30;
    imuAngularAccelerationFiltWL = 25;

    ///< Threshold of adaptive window filters
    dqFiltTh            = 1.0;
    d2qFiltTh           = 1.0;
    dTauJFiltTh         = 0.2;
    dTauMFiltTh         = 0.2;
    imuAngularAccelerationFiltTh = 1.0;

    ///< Cut frequencies
    tauJCutFrequency    =   3.0;
    tauMCutFrequency    =   3.0;
    pwmCutFrequency     =   3.0;
    imuLinearAccelerationCutFrequency = 3.0;
    imuAngularVelocityCutFrequency    = 3.0;
    imuMagnetometerCutFrequency       = 3.0;
    forcetorqueCutFrequency           = 3.0;

    ///< Skin timestamp
    last_reading_skin_contact_list_Stamp = -1000.0;

    if( _wbi_yarp_conf.check("fixed_base") )
    {
        assume_fixed_base = true;
        std::string _fixed_link;
        _fixed_link = _wbi_yarp_conf.find("fixed_base").asString();
        if( _fixed_link == "root_link" )
        {
            fixed_link = FIXED_ROOT_LINK;
        }
        else if( _fixed_link == "l_sole" )
        {
            fixed_link = FIXED_L_SOLE;
        }
        else if( _fixed_link == "r_sole" )
        {
            fixed_link = FIXED_R_SOLE;
        }
        else
        {
            YARP_ASSERT(false);
        }
    }
    else
    {
        assume_fixed_base = false;
    }

    if( assume_fixed_base )
    {

    }
}

bool yarpWholeBodyDynamicsEstimator::threadInit()
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
    resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

    ///< create derivative filters
    dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
    dTauJFilt = new AWLinEstimator(dTauJFiltWL, dTauJFiltTh);
    dTauMFilt = new AWLinEstimator(dTauMFiltWL, dTauMFiltTh);

    ///< read sensors
    bool ok = sensors->readSensors(SENSOR_ENCODER, estimates.lastQ.data(), qStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_PWM, estimates.lastPwm.data(), 0, true);

    ///< create low pass filters
    tauJFilt    = new FirstOrderLowPassFilter(tauJCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    tauMFilt    = new FirstOrderLowPassFilter(tauMCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    pwmFilt     = new FirstOrderLowPassFilter(pwmCutFrequency, getRate()*1e-3, estimates.lastPwm);

    IDList available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    for(int ft_numeric = 0; ft_numeric < (int)available_ft_sensors.size(); ft_numeric++ )
    {
        ID wbi_id;
        available_ft_sensors.indexToID(ft_numeric,wbi_id);
        int ft_index = ft_numeric;
        sensors->readSensor(SENSOR_FORCE_TORQUE, ft_index, forcetorques[ft_index].data(), &(forcetorquesStamps[ft_index]),true );
        forcetorqueFilters[ft_index] = new FirstOrderLowPassFilter(forcetorqueCutFrequency,getRate()*1e-3,forcetorques[ft_index]); ///< low pass filter
    }

     IDList available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
     for(int numeric_imu_id = 0; numeric_imu_id < (int)available_imu_sensors.size(); numeric_imu_id++)
     {
         int imu_index = numeric_imu_id;
         //std::cout << "readSensor for IMU " << imu_index << std::endl;
         assert((int)IMUs.size() > imu_index && (int)IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
         if( sensors->readSensor(SENSOR_IMU, numeric_imu_id, IMUs[imu_index].data(), &(IMUStamps[imu_index]),true ) ) {
             imuLinearAccelerationFilters[imu_index] = new FirstOrderLowPassFilter(imuLinearAccelerationCutFrequency,getRate()*1e-3,IMUs[imu_index].subVector(4,6));  ///< linear acceleration is filtered with a low pass filter
             imuAngularVelocityFilters[imu_index] = new FirstOrderLowPassFilter(imuAngularVelocityCutFrequency,getRate()*1e-3,IMUs[imu_index].subVector(7,9));  ///< angular velocity is filtered with a low pass filter
             imuMagnetometerFilters[imu_index] = new FirstOrderLowPassFilter(imuMagnetometerCutFrequency,getRate()*1e-3,IMUs[imu_index].subVector(10,12));  ///< magnetometer readings are filtered with a low pass filter
         } else {
             std::cout << "icubWholeBodyStates: Error in reading IMU, exiting" << std::endl;
             YARP_ASSERT(false);
         }
         //std::cout << "IMU measured " << std::endl;
         //std::cout << IMUs[imu_index].toString() << std::endl;
         //std::cout << "timestamp: " << IMUStamps[imu_index] << std::endl;
     }

     //Allocating a filter for angular acceleration estimation only for IMU used in iDynTree
    imuAngularAccelerationFilt = new AWLinEstimator(imuAngularAccelerationFiltWL, imuAngularAccelerationFiltTh);

    //Allocation model
    std::string fixed_link_name;
    if( assume_fixed_base )
    {
        switch( fixed_link )
        {
            case FIXED_ROOT_LINK:
                fixed_link_name = "root_link";
            break;
            case FIXED_R_SOLE:
                fixed_link_name = "r_sole";
            break;
            case FIXED_L_SOLE:
                fixed_link_name = "l_sole";
            break;
        }
    }

    if(  !wbi_yarp_conf.check("urdf") && !wbi_yarp_conf.check("urdf_file") )
    {
        std::cerr << "yarpWholeBodyModel error: urdf not found in configuration files" << std::endl;
        return false;
    }

    std::string urdf_file;
    if( wbi_yarp_conf.check("urdf") )
    {
        urdf_file = wbi_yarp_conf.find("urdf").asString().c_str();
    }
    else
    {
        urdf_file = wbi_yarp_conf.find("urdf_file").asString().c_str();
    }

    yarp::os::ResourceFinder rf;
    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());

    std::vector<std::string> dof_serialization;
    IDList torque_estimation_list = sensors->getSensorList(SENSOR_ENCODER);
    for(int dof=0; dof < (int)torque_estimation_list.size(); dof++)
    {
        ID wbi_id;
        torque_estimation_list.indexToID(dof,wbi_id);
        dof_serialization.push_back(wbi_id.toString());
    }

    std::vector<std::string> ft_serialization;
    IDList ft_sensor_list = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    for(int ft=0; ft < (int)ft_sensor_list.size(); ft++)
    {
        ID wbi_id;
        ft_sensor_list.indexToID(ft,wbi_id);
        ft_serialization.push_back(wbi_id.toString());
    }

    model_mutex.wait();
    {
        std::cerr << "[DEBUG] Create TorqueEstimationTree with " << ft_serialization.size() << " ft sensors" << std::endl;
        if( !assume_fixed_base )
        {
            robot_estimation_model = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
        } else {
            robot_estimation_model = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,fixed_link_name);
        }
    }
    //Load mapping from skinDynLib to iDynTree links from configuration files
    model_mutex.post();

    if( !this->wbi_yarp_conf.check("IDYNTREE_SKINDYNLIB_LINKS") )
    {
        std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: IDYNTREE_SKINDYNLIB_LINKS group not found in configuration files" << std::endl;
        return false;
    }

    yarp::os::Bottle & bot =  this->wbi_yarp_conf.findGroup("IDYNTREE_SKINDYNLIB_LINKS");
    for(int i=1; i < bot.size(); i++ )
    {
        yarp::os::Bottle * map_bot = bot.get(i).asList();
        if( map_bot->size() != 2 || map_bot->get(1).asList() == NULL ||
            map_bot->get(1).asList()->size() != 3 )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: IDYNTREE_SKINDYNLIB_LINKS group is malformed (" << map_bot->toString() << ")" << std::endl;
            return false;
        }
        std::string iDynTree_link_name = map_bot->get(0).asString();
        std::string iDynTree_skinFrame_name = map_bot->get(1).asList()->get(0).asString();
        int skinDynLib_body_part = map_bot->get(1).asList()->get(1).asInt();
        int skinDynLib_link_index = map_bot->get(1).asList()->get(2).asInt();
        model_mutex.wait();
        bool ret_sdl = robot_estimation_model->addSkinDynLibAlias(iDynTree_link_name,iDynTree_skinFrame_name,skinDynLib_body_part,skinDynLib_link_index);
        model_mutex.post();
        if( !ret_sdl )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: IDYNTREE_SKINDYNLIB_LINKS link " << iDynTree_link_name
                      << " and frame " << iDynTree_skinFrame_name << " and not found in urdf model" << std::endl;
            return false;
        }
    }
    std::cerr << std::endl << "[INFO] IDYNTREE_SKINDYNLIB_LINKS correctly loaded" << std::endl;


    //Load subtree information
    link2subtree.resize(robot_estimation_model->getNrOfLinks());

    if( !this->wbi_yarp_conf.check("WBD_SUBTREES") )
    {
        std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES group not found in configuration files" << std::endl;
        return false;
    }

    yarp::os::Bottle & wbd_subtrees_bot =  this->wbi_yarp_conf.findGroup("WBD_SUBTREES");


    for(int i=1; i < wbd_subtrees_bot.size(); i++ )
    {
        yarp::os::Bottle * subtree_bot = wbd_subtrees_bot.get(i).asList();
        if( subtree_bot->size() != 2
            || subtree_bot->get(1).asList() == NULL
            || subtree_bot->get(1).asList()->size() != 2
            || subtree_bot->get(1).asList()->get(0).asList() == NULL )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES group is malformed (" << subtree_bot->toString() << ")" << std::endl;
            return false;
        }


        TorqueEstimationSubtree subtree;

        std::string subtree_name = subtree_bot->get(0).asString();
        subtree.subtree_name = subtree_name;


        yarp::os::Bottle * subtree_links_bot = subtree_bot->get(1).asList()->get(0).asList();
        for(int l=0; l < subtree_links_bot->size(); l++ )
        {

            std::string link_name = subtree_links_bot->get(l).asString();
            model_mutex.wait();
            int link_index = robot_estimation_model->getLinkIndex(link_name);
            model_mutex.post();
            if( link_index < 0 )
            {
                std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES link " << link_name << " not found in urdf model" << std::endl;
                return false;
            }
            subtree.links.push_back(link_name);
            subtree.links_numeric_ids.insert(link_index);

            int current_subtree = i-1;
            link2subtree[link_index] = current_subtree;

        }
        std::string default_contact_link_name = subtree_bot->get(1).asList()->get(1).asString();

        model_mutex.wait();
        int default_contact_link_index = robot_estimation_model->getLinkIndex(default_contact_link_name);
        model_mutex.post();
        if( default_contact_link_index < 0 )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES link " << default_contact_link_name << " not found in urdf model" << std::endl;
            return false;
        }

        if( subtree.links_numeric_ids.find(default_contact_link_index) == subtree.links_numeric_ids.end() )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES link " << default_contact_link_name
                      << " was specified as default contact for subtree " << subtree.subtree_name
                      << " but it is not present in the subtree" << std::endl;
            return false;
        }

        subtree.default_contact_link = default_contact_link_index;

        torque_estimation_subtrees.push_back(subtree);
    }

    contacts_for_given_subtree.resize(torque_estimation_subtrees.size());

    model_mutex.wait();
    std::cerr << "[DEBUG] robot_estimation_model->getSubTreeInternalDynamics().size() : " << robot_estimation_model->getSubTreeInternalDynamics().size() << std::endl;
    std::cerr << "[DEBUG] torque_estimation_subtrees.size(): " << torque_estimation_subtrees.size() << std::endl;
    YARP_ASSERT(robot_estimation_model->getSubTreeInternalDynamics().size() ==  torque_estimation_subtrees.size());
    model_mutex.post();

    std::cerr << "[INFO] WBD_SUBTREES correctly loaded with " << torque_estimation_subtrees.size() << "subtrees" << std::endl;

    IDList available_encoders = sensors->getSensorList(wbi::SENSOR_ENCODER);
    for(int i = 0; i < (int)available_encoders.size(); i++ )
    {
        ID enc;
        available_encoders.indexToID(i,enc);
        if(!(robot_estimation_model->getDOFIndex(enc.toString()) == i))
        {
            std::cerr << "[ERR] dof " << enc.toString() << " has ID " << robot_estimation_model->getDOFIndex(enc.toString())
                      << " in the dynamical model and id " << i << "in the wbi" << std::endl;
        }
        YARP_ASSERT((robot_estimation_model->getDOFIndex(enc.toString()) == i));
    }
    std::cout << "[DEBUG] yarpWholeBodyDynamicsEstimator::threadInit() terminet successfully" << std::endl;
    return ok;
}

void yarpWholeBodyDynamicsEstimator::run()
{
    run_mutex.wait();
    //Temporary workaround: wholeBodyDynamicsStatesInterface needs all the DOF present in the dynamical model
    if( sensors->getSensorNumber(wbi::SENSOR_ENCODER) != robot_estimation_model->getNrOfDOFs() )
    {
        IDList list = sensors->getSensorList(wbi::SENSOR_ENCODER);

        std::cerr << "Available encoders: " << list.toString() << std::endl;

           std::cerr << "yarpWholeBodyDynamicsEstimator::run() error: " <<
                  sensors->getSensorNumber(wbi::SENSOR_ENCODER) << " joint sensors are available, while  " <<
                  robot_estimation_model->getNrOfDOFs() << " joints are present in the model " << std::endl;
        assert(false);
        return;
    }

    ///< \todo improve robustness: what if a sensor dies or stop working? interface should warn the user
    mutex.wait();
    {
        resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
        resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
        resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

        ///< Read encoders
        if(sensors->readSensors(SENSOR_ENCODER, q.data(), qStamps.data(), false))
        {
            estimates.lastQ = q;
            AWPolyElement el;
            el.data = q;
            el.time = yarp::os::Time::now();
            estimates.lastDq = dqFilt->estimate(el);
            estimates.lastD2q = d2qFilt->estimate(el);

        }

        ///< Read force/torque sensors
        ///< \todo TODO buffer value of available_ft_sensors to avoid memory allocation (?)
        IDList available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
        for(int ft_numeric = 0; ft_numeric < (int)available_ft_sensors.size(); ft_numeric++ )
        {
            int ft_index = ft_numeric;
            if( sensors->readSensor(SENSOR_FORCE_TORQUE, ft_numeric, forcetorques[ft_index].data(), &(forcetorquesStamps[ft_index]),false ) ) {
                estimates.lastForceTorques[ft_index] = forcetorqueFilters[ft_index]->filt(forcetorques[ft_index]); ///< low pass filter
                estimates.lastForceTorques[ft_index] = estimates.lastForceTorques[ft_index] - forcetorques_offset[ft_index]; /// remove offset
            } else {
                std::cout << "wholeBodyDynamicsStatesInterface: Error in reading F/T sensors, exiting" << std::endl;
                YARP_ASSERT(false);
            }
        }

        ///< Read IMU
        ///< \todo TODO buffer value of available_imu_sensors to avoid memory allocation (?)
        ///< \todo TODO add filters for imu values ->
        IDList available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
        for(int imu_numeric = 0; imu_numeric < (int) available_imu_sensors.size(); imu_numeric++ )
        {
            int imu_index = imu_numeric;
            //std::cout << "readSensor for IMU " << imu_index << std::endl;
            assert((int)IMUs.size() > imu_index );
            assert((int)IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
            if( sensors->readSensor(SENSOR_IMU, imu_numeric, IMUs[imu_index].data(), &(IMUStamps[imu_index]),false ) ) {
                estimates.lastIMUs[imu_index].setSubvector(0,IMUs[imu_index].subVector(0,3)); ///< orientation is simply copied as already result of an estimation
                estimates.lastIMUs[imu_index].setSubvector(4,imuLinearAccelerationFilters[imu_index]->filt(IMUs[imu_index].subVector(4,6)));  ///< linear acceleration is filtered with a low pass filter
                estimates.lastIMUs[imu_index].setSubvector(7,imuAngularVelocityFilters[imu_index]->filt(IMUs[imu_index].subVector(7,9)));  ///< angular velocity is filtered with a low pass filter
                estimates.lastIMUs[imu_index].setSubvector(10,imuMagnetometerFilters[imu_index]->filt(IMUs[imu_index].subVector(10,12))); ///< magnetometer readings are filtered with a low pass filter
            } else {
                std::cout << "wholeBodyDynamicsStatesInterface: Error in reading IMU, exiting" << std::endl;
                YARP_ASSERT(false);
            }
            //std::cout << "IMU measured " << std::endl;
            //std::cout << IMUs[imu_index].toString() << std::endl;
            //std::cout << estimates.lastIMUs[imu_index].toString() << std::endl;
            //std::cout << "timestamp: " << IMUStamps[imu_index] << std::endl;
        }

        //Estimate angular acceleration only for the IMU used in iDynTree
        //std::cout << "Angular velocity used for acceleration estimation " <<  estimates.lastIMUs[0].subVector(7,9).toString() << std::endl;
        AWPolyElement el;
        el.data = omega_used_IMU = estimates.lastIMUs[0].subVector(7,9);
        el.time = yarp::os::Time::now();

        domega_used_IMU = imuAngularAccelerationFilt->estimate(el);

        ddp_used_IMU = estimates.lastIMUs[0].subVector(4,6);

        /*
        std::cout << "domega " <<  domega_used_IMU.toString() << std::endl;
        std::cout << "omega  " << omega_used_IMU.toString() << std::endl;
        std::cout << "ddp " << ddp_used_IMU.toString() << std::endl;
        */

        ///< Read skin contacts
        readSkinContacts();

        ///< Estimate joint torque sensors from force/torque measurements
        estimateExternalForcesAndJointTorques();

        ///< Filter obtained joint torque measures
        {
            // @todo Convert joint torques into motor torques
            AWPolyElement el;
            el.time = yarp::os::Time::now();

            estimates.lastTauJ = tauJ;// tauJFilt->filt(tauJ);  ///< low pass filter
            estimates.lastTauM = tauMFilt->filt(tauJ);  ///< low pass filter

            el.data = tauJ;
            estimates.lastDtauJ = dTauJFilt->estimate(el);  ///< derivative filter

            //el.data = tauM;
            estimates.lastDtauM = dTauMFilt->estimate(el);  ///< derivative filter
        }

        ///< Read motor pwm
        sensors->readSensors(SENSOR_PWM, pwm.data(),0, false);
        estimates.lastPwm = pwmFilt->filt(pwm);     ///< low pass filter
    }
    mutex.post();

    run_mutex.post();

    return;
}

void yarpWholeBodyDynamicsEstimator::readSkinContacts()
{
    skinContactList *scl = port_skin_contacts->read(false);
    if(scl)
    {
        //< \todo TODO check for envelope
        last_reading_skin_contact_list_Stamp = Time::now();
        if(scl->empty())   // if no skin contacts => leave the old contacts but reset pressure and contact list
        {
            //< \todo TODO this (using the last contacts if no contacts are detected) should be at subtree level, not at global level
            for(skinContactList::iterator it=skinContacts.begin(); it!=skinContacts.end(); it++)
            {
                it->setPressure(0.0);
                it->setActiveTaxels(0);
            }
            return;
        }

        //Probably source of crazy inefficiencies, here just to reach a working state as soon as possible \todo TODO
        map<BodyPart, skinContactList> contactsPerBp = scl->splitPerBodyPart();

        skinContacts.clear();

        // if there are more than 1 contact and less than 10 taxels are active then suppose zero moment
        for(map<BodyPart,skinContactList>::iterator it=contactsPerBp.begin(); it!=contactsPerBp.end(); it++)
        {
            if(it->second.size()>1)
            {
                for(skinContactList::iterator c=it->second.begin(); c!=it->second.end(); c++)
                {
                    if( c->getActiveTaxels()<10 )
                    {
                        c->fixMoment();
                    }

                    //Insert a contact in skinContacts only if the number of taxel is greater than ActiveTaxels
                    if( (int)c->getActiveTaxels() > min_taxel )
                    {
                        skinContacts.insert(skinContacts.end(),*c);
                    }
                }
            }
        }

        //TODO \todo add other parts
        /*
        skinContacts = contactsPerBp[LEFT_ARM];
        skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_ARM].begin(), contactsPerBp[RIGHT_ARM].end());
        skinContacts.insert(skinContacts.end(), contactsPerBp[TORSO].begin(), contactsPerBp[TORSO].end());
        */
        //skinContacts.insert(skinContacts.end(), contactsPerBp[LEFT_LEG].begin(), contactsPerBp[LEFT_LEG].end());
        //skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_LEG].begin(), contactsPerBp[RIGHT_LEG].end());
    }
    else if(Time::now()-last_reading_skin_contact_list_Stamp>SKIN_EVENTS_TIMEOUT && last_reading_skin_contact_list_Stamp!=0.0)
    {
        // if time is up, use default contact points \todo TODO
        skinContacts.clear();
    }

    //std::cout << "skinContacts: " << std::endl;
    //std::cout << skinContacts.toString() << std::endl;


    //At this point, in a way or the other skinContacts must have at least a valid contact for each subtree
    //If this is not true, we add a default contact for each subgraph
    dynContacts = skinContacts.toDynContactList();

    // std::cout << "dynContacts: " << std::endl;
    // std::cout << dynContacts.toString() << std::endl;

    for(int subtree=0; subtree < (int)contacts_for_given_subtree.size(); subtree++ )
    {
        contacts_for_given_subtree[subtree] = 0;
    }

    for(dynContactList::iterator it=dynContacts.begin();
        it != dynContacts.end(); it++ )
    {
        int skinDynLib_body_part = it->getBodyPart();
        int skinDynLib_link_index = it->getLinkNumber();
        model_mutex.wait();
        int iDynTree_link_index = -1;
        int iDynTree_frame_index=  -1;
        bool skinDynLibID_found = robot_estimation_model->skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,iDynTree_link_index,iDynTree_frame_index);
        model_mutex.post();
        // \todo TODO FIXME properly address when you find an unexpcted contact id without crashing
        if( !skinDynLibID_found )
        {
            yError("wholeBodyDynamicsStatesInterface: unexpected contact from bodyPart %d link with local id %d",skinDynLib_body_part,skinDynLib_link_index);
        }
        else
        {
            contacts_for_given_subtree[link2subtree[iDynTree_link_index]]++;
        }
    }

    for(int subtree=0; subtree < (int)torque_estimation_subtrees.size(); subtree++ )
    {
        if( contacts_for_given_subtree[subtree] == 0 )
        {
            dynContact default_contact = this->getDefaultContact(subtree);
            dynContacts.push_back(default_contact);
            //std::cout << "Adding :" << default_contact.toString() << std::endl;
            //std::cout << "dynContacts: " << std::endl;
            //std::cout << dynContacts.toString() << std::endl;
        }
    }


}



dynContact yarpWholeBodyDynamicsEstimator::getDefaultContact(const int subtree)
{
    int iDynTree_default_contact_link = torque_estimation_subtrees[subtree].default_contact_link;

    //std::cout << "default_contact_link" << default_contact_link << std::endl;
    model_mutex.wait();
    //std::cout << "Model: " << robot_estimation_model->getKDLUndirectedTree().getSerialization().toString() << std::endl;
    model_mutex.post();
    int skinDynLib_body_part = -1;
    int skinDynLib_link_index = -1;
    int iDynTree_default_contact_link_skinFrame;
    model_mutex.wait();
    YARP_ASSERT(robot_estimation_model->getSkinDynLibAlias(iDynTree_default_contact_link,iDynTree_default_contact_link_skinFrame,skinDynLib_body_part,skinDynLib_link_index));
    model_mutex.post();
    YARP_ASSERT(skinDynLib_body_part != -1);
    YARP_ASSERT(skinDynLib_link_index != -1);
    dynContact return_value = dynContact();
    return_value.setBodyPart((iCub::skinDynLib::BodyPart)skinDynLib_body_part);
    return_value.setLinkNumber(skinDynLib_link_index);
    //std::cout << return_value.toString() << std::endl;
    return return_value;
}

void getEEWrench(const iCub::iDynTree::TorqueEstimationTree & icub_model,
                 const iCub::skinDynLib::dynContact & dyn_contact,
                 bool & contact_found,
                 yarp::sig::Vector & link_wrench,
                 yarp::sig::Vector & gripper_wrench,
                 int ee_frame_idyntree_id,
                 int link_idyntree_id)
{
    //std::cout << "getEEWRench " << std::endl;
    contact_found = true;
    KDL::Wrench f_gripper, f_link, f_contact;
    KDL::Vector COP;
    YarptoKDL(dyn_contact.getCoP(),COP);
    YarptoKDL(dyn_contact.getForceMoment(),f_contact);
    //std::cout << "f_contact " << f_contact << std::endl;
    KDL::Frame H_link_contact(COP);
    f_link = H_link_contact*f_contact;
    KDLtoYarp(f_link,link_wrench);
    yarp::sig::Matrix H_gripper_link_yarp = icub_model.getPosition(ee_frame_idyntree_id,link_idyntree_id);
    YARP_ASSERT(H_gripper_link_yarp.cols() == 4 &&
                H_gripper_link_yarp.rows() == 4);
    KDL::Frame H_gripper_link;
    YarptoKDL(H_gripper_link_yarp,H_gripper_link);
    f_gripper = H_gripper_link*f_link;
    KDLtoYarp(f_gripper,gripper_wrench);
}

void yarpWholeBodyDynamicsEstimator::estimateExternalForcesAndJointTorques()
{
    //Assume that only a IMU is available

    /** \todo TODO check that serialization between wbi and iDynTree are the same */
    assert(omega_used_IMU.size() == 3);
    assert(domega_used_IMU.size() == 3);
    assert(ddp_used_IMU.size() == 3);

    if( !enable_omega_domega_IMU )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
    }

    double gravity = 9.8;

    if( assume_fixed_base )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
        ddp_used_IMU.zero();
        switch( fixed_link )
        {
            case FIXED_ROOT_LINK:
                ddp_used_IMU[2] = gravity;
            break;
            case FIXED_L_SOLE:
            case FIXED_R_SOLE:
                ddp_used_IMU[0] = gravity;
            break;
        }
    }

    model_mutex.wait();
    assert((int)estimates.lastQ.size() == robot_estimation_model->getNrOfDOFs());
    assert((int)estimates.lastDq.size() == robot_estimation_model->getNrOfDOFs());
    assert((int)estimates.lastD2q.size() == robot_estimation_model->getNrOfDOFs());

    YARP_ASSERT(robot_estimation_model->setInertialMeasure(omega_used_IMU,domega_used_IMU,ddp_used_IMU));
    (robot_estimation_model->setAng(estimates.lastQ));
    (robot_estimation_model->setDAng(estimates.lastDq));
    (robot_estimation_model->setD2Ang(estimates.lastD2q));
    for(int i=0; i < robot_estimation_model->getNrOfFTSensors(); i++ ) {
        //std::cout << "Number of F/T sensors available " << estimates.lastForceTorques.size() << std::endl;
        //std::cout << "Number of F/T sensors required by the model " << icub_model->getNrOfFTSensors() << std::endl;
        YARP_ASSERT((int)estimates.lastForceTorques.size() == robot_estimation_model->getNrOfFTSensors());
        assert(estimates.lastForceTorques[i].size() == 6);
        YARP_ASSERT(robot_estimation_model->setSensorMeasurement(i,estimates.lastForceTorques[i]));
    }
    robot_estimation_model->setContacts(dynContacts);

    /** \todo TODO avoid unlocking/locking a mutex locked in the calling function in the called function */
    /** \todo TODO use a different mutex to ensure that the dimensions of the sensors/states does not change? */
    //mutex.post();

    YARP_ASSERT(robot_estimation_model->kinematicRNEA());
    YARP_ASSERT(robot_estimation_model->estimateContactForcesFromSkin());
    YARP_ASSERT(robot_estimation_model->dynamicRNEA());
    YARP_ASSERT(robot_estimation_model->computePositions());

    estimatedLastDynContacts = robot_estimation_model->getContacts();

    //Create estimatedLastSkinDynContacts using original skinContacts list read from skinManager
    // for each dynContact find the related skinContact (if any) and set the wrench in it
    unsigned long cId;
    bool contactFound=false;

    for(unsigned int i=0; i < estimatedLastDynContacts.size(); i++)
    {
        //Workaround for bug in iCubGui
        // \todo TODO remove
        if( estimatedLastDynContacts[i].getBodyPart() == TORSO &&
            estimatedLastDynContacts[i].getLinkNumber() == 2 )
        {
            //Invert second component
            yarp::sig::Vector wrench = estimatedLastDynContacts[i].getForceMoment();
            wrench[1] = -wrench[1];
            wrench[3+1] = -wrench[3+1];
            estimatedLastDynContacts[i].setForceMoment(wrench);
        }

        cId = estimatedLastDynContacts[i].getId();
        for(unsigned int j=0; j<skinContacts.size(); j++)
        {
            if(cId == skinContacts[j].getId())
            {
                skinContacts[j].setForceMoment( estimatedLastDynContacts[i].getForceMoment() );
                contactFound = true;
                j = skinContacts.size();    // break from the inside for loop
            }
        }

        // if there is no associated skin contact, create one
        if(!contactFound)
            skinContacts.push_back(skinContact(estimatedLastDynContacts[i]));
        contactFound = false;

        //Save the total link on a
        (estimatedLastDynContacts[i].getBodyPart(),estimatedLastDynContacts[i].getLinkNumber());

    }

    //mutex.wait();

    estimatedLastSkinDynContacts = skinContacts;

    assert((int)tauJ.size() == robot_estimation_model->getNrOfDOFs());
    tauJ = robot_estimation_model->getTorques();
    model_mutex.post();

}


void deleteFirstOrderFilterVector(std::vector<iCub::ctrl::FirstOrderLowPassFilter *> & vec)
{
    for(int i=0; i < (int)vec.size(); i++ ) {
        if( vec[i]!= 0 ) { delete vec[i]; vec[i]=0; }
    }
    vec.resize(0);
}

void yarpWholeBodyDynamicsEstimator::threadRelease()
{
    if(dqFilt!=0)    { delete dqFilt;  dqFilt=0; }
    if(d2qFilt!=0)   { delete d2qFilt; d2qFilt=0; }
    if(dTauJFilt!=0) { delete dTauJFilt; dTauJFilt=0; }
    if(dTauMFilt!=0) { delete dTauMFilt; dTauMFilt=0; }     // motor torque derivative filter
    if(tauJFilt!=0)  { delete tauJFilt; tauJFilt=0; }  ///< low pass filter for joint torque
    if(tauMFilt!=0)  { delete tauMFilt; tauMFilt=0; }  ///< low pass filter for motor torque
    if(pwmFilt!=0)   { delete pwmFilt; pwmFilt=0;   }
    deleteFirstOrderFilterVector(imuLinearAccelerationFilters);
    deleteFirstOrderFilterVector(imuAngularVelocityFilters);
    deleteFirstOrderFilterVector(imuMagnetometerFilters);
    deleteFirstOrderFilterVector(forcetorqueFilters);
    if(imuAngularAccelerationFilt!=0) { delete imuAngularAccelerationFilt; imuAngularAccelerationFilt=0; }

    double avgTime, stdDev, avgTimeUsed, stdDevUsed;
    this->getEstPeriod(avgTime, stdDev);
    this->getEstUsed(avgTimeUsed, stdDevUsed);
    double period = this->getRate();
    printf("[PERFORMANCE INFORMATION][yarpWholeBodyDynamicsEstimator]:\n");
    printf("Expected period %f ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);

    return;
}

void yarpWholeBodyDynamicsEstimator::lockAndResizeAll(int n)
{
    mutex.wait();
    resizeAll(n);
    mutex.post();
}

void yarpWholeBodyDynamicsEstimator::resizeAll(int n)
{
    q.resize(n,0.0);
    qStamps.resize(n,INITIAL_TIMESTAMP);
    tauJ.resize(n,0.0);
    tauJStamps.resize(n,INITIAL_TIMESTAMP);
    pwm.resize(n,0);
    pwmStamps.resize(n,INITIAL_TIMESTAMP);
    estimates.lastQ.resize(n,0.0);
    estimates.lastDq.resize(n,0.0);
    estimates.lastD2q.resize(n,0.0);
    estimates.lastTauJ.resize(n,0.0);
    estimates.lastTauM.resize(n,0.0);
    estimates.lastDtauJ.resize(n,0.0);
    estimates.lastDtauM.resize(n,0.0);
    estimates.lastPwm.resize(n,0.0);
}

void yarpWholeBodyDynamicsEstimator::lockAndResizeFTs(int n)
{
    mutex.wait();
    resizeFTs(n);
    mutex.post();
}

void yarpWholeBodyDynamicsEstimator::resizeFTs(int n)
{
    forcetorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorques_offset.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorquesStamps.resize(n,INITIAL_TIMESTAMP);
    forcetorqueFilters.resize(n);
    estimates.lastForceTorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
}

void yarpWholeBodyDynamicsEstimator::lockAndResizeIMUs(int n)
{
    mutex.wait();
    resizeIMUs(n);
    mutex.post();
}

void yarpWholeBodyDynamicsEstimator::resizeIMUs(int n)
{
    IMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
    IMUStamps.resize(n,INITIAL_TIMESTAMP);
    imuAngularVelocityFilters.resize(n);
    imuLinearAccelerationFilters.resize(n);
    imuMagnetometerFilters.resize(n);
    estimates.lastIMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyVector(const Vector &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest, src.data(), sizeof(double)*src.size());
    mutex.post();
    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyVectorElement(int index, const Vector &src, double *dest)
{
    mutex.wait();
    dest[0] = src[index];
    mutex.post();
    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyElementVectorFromVector(int index, const std::vector<Vector> &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest,src[index].data(),sizeof(double)*src[index].size());
    mutex.post();
    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyVectorOfVectors(const std::vector<Vector> &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    for(int i=0, offset = 0; i < (int)src.size(); i++) {
        memcpy(dest+offset,src[i].data(),sizeof(double)*src[i].size());
        offset += src[i].size();
    }
    mutex.post();
    return true;
}

void copyVector(const yarp::sig::Vector & src, double * dest)
{
    memcpy(dest,src.data(),src.size()*sizeof(double));
}

bool yarpWholeBodyDynamicsEstimator::lockAndCopyExternalForceTorque(int link_index, double * dest)
{
    if( link_index < 0 ||
        link_index >= robot_estimation_model->getNrOfLinks() )
    {
        return false;
    }

    KDL::Wrench ext_f = robot_estimation_model->getExternalForceTorqueKDL(link_index,link_index,link_index);

    for(int i=0; i < 6; i++ )
    {
        dest[i] = ext_f[i];
    }

    return true;
}

bool yarpWholeBodyDynamicsEstimator::lockAndSetEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    bool res = false;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_JOINT_VEL:
    case ESTIMATE_MOTOR_VEL:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setVelFiltParams(((int*)value)[0], dqFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setVelFiltParams(dqFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_JOINT_ACC:
    case ESTIMATE_MOTOR_ACC:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setAccFiltParams(((int*)value)[0], d2qFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setAccFiltParams(d2qFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_JOINT_TORQUE:
        if(ep==ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ)
            res = setTauJCutFrequency(((double*)value)[0]);
        else if(ep==wbi::ESTIMATION_PARAM_ENABLE_OMEGA_IMU_DOMEGA_IMU)
            res = setEnableOmegaDomegaIMU(*((bool*)value));
        else if(ep==wbi::ESTIMATION_PARAM_MIN_TAXEL)
            res = setMinTaxel(*((int*)value));
        break;

    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setDtauJFiltParams(((int*)value)[0], dTauJFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setDtauJFiltParams(dTauJFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_MOTOR_TORQUE:
        if(ep==ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ)
            res = setTauMCutFrequency(((double*)value)[0]);
        break;

    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
        if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE)
            res = setDtauMFiltParams(((int*)value)[0], dTauMFiltTh);
        else if(ep==ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD)
            res = setDtauMFiltParams(dTauMFiltWL, ((double*)value)[0]);
        break;

    case ESTIMATE_MOTOR_PWM:
        if(ep==ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ)
            res = setPwmCutFrequency(((double*)value)[0]);
        break;

    case ESTIMATE_IMU: ///< \todo TODO
    case ESTIMATE_FORCE_TORQUE_SENSOR: ///< \todo TODO
    case ESTIMATE_JOINT_POS:
    case ESTIMATE_MOTOR_POS:
    default: break;
    }
    mutex.post();
    return res;
}

bool yarpWholeBodyDynamicsEstimator::lockAndSetEstimationOffset(const EstimateType et, const ID & sid, const double *value)
{
    bool res = true;
    int ft_index;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_FORCE_TORQUE_SENSOR: ///< \todo TODO
        sensors->getSensorList(SENSOR_FORCE_TORQUE).idToIndex(sid,ft_index);
        memcpy(forcetorques_offset[ft_index].data(), (double*)value, sizeof(double)*sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize);
        break;
    default:
        break;
    }
    mutex.post();
    return res;
}

bool yarpWholeBodyDynamicsEstimator::lockAndGetEstimationOffset(const EstimateType et, const ID & sid, double *value)
{
    bool res = true;
    int ft_index;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_FORCE_TORQUE_SENSOR: ///< \todo TODO
        sensors->getSensorList(SENSOR_FORCE_TORQUE).idToIndex(sid,ft_index);
        memcpy(value, forcetorques_offset[ft_index].data(), sizeof(double)*sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize);
        break;
    default:
        break;
    }
    mutex.post();
    return res;
}



bool yarpWholeBodyDynamicsEstimator::setVelFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    dqFiltWL = windowLength;
    dqFiltTh = threshold;
    if(dqFilt!=NULL)
    {
        AWPolyList list = dqFilt->getList();
        dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dqFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyDynamicsEstimator::setAccFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    d2qFiltWL = windowLength;
    d2qFiltTh = threshold;
    if(d2qFilt!=NULL)
    {
        AWPolyList list = d2qFilt->getList();
        d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            d2qFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyDynamicsEstimator::setDtauJFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    dTauJFiltWL = windowLength;
    dTauJFiltTh = threshold;
    if(dTauJFilt!=NULL)
    {
        AWPolyList list = dTauJFilt->getList();
        dTauJFilt = new AWLinEstimator(windowLength, threshold);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dTauJFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyDynamicsEstimator::setDtauMFiltParams(int windowLength, double threshold)
{
    if(windowLength<1 || threshold<=0.0)
        return false;
    dTauMFiltWL = windowLength;
    dTauMFiltTh = threshold;
    if(dTauMFilt!=NULL)
    {
        AWPolyList list = dTauMFilt->getList();
        dTauMFilt = new AWLinEstimator(windowLength, threshold);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dTauMFilt->feedData(*it);
    }
    return true;
}

bool yarpWholeBodyDynamicsEstimator::setTauJCutFrequency(double fc)
{
    return tauJFilt->setCutFrequency(fc);
}

bool yarpWholeBodyDynamicsEstimator::setTauMCutFrequency(double fc)
{
    return tauMFilt->setCutFrequency(fc);
}

bool yarpWholeBodyDynamicsEstimator::setPwmCutFrequency(double fc)
{
    return pwmFilt->setCutFrequency(fc);
}

bool yarpWholeBodyDynamicsEstimator::setEnableOmegaDomegaIMU(bool _enabled_omega_domega_IMU)
{
    enable_omega_domega_IMU = _enabled_omega_domega_IMU;
    return true;
}

bool yarpWholeBodyDynamicsEstimator::setMinTaxel(const int _min_taxel)
{
    if( _min_taxel < 0 )
    {
        return false;
    }
    min_taxel = _min_taxel;
    return true;
}
