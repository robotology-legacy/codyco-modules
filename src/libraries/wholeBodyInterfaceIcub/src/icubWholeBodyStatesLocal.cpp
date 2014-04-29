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

#include "wbiIcub/icubWholeBodyStatesLocal.h"
#include "wbiIcub/icubWholeBodySensors.h"
#include <iCub/skinDynLib/common.h>
#include <yarp/os/Time.h>
#include <string>
#include <iostream>
#include <yarp/os/Log.h>

#define INITIAL_TIMESTAMP -1000.0


using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

/// < \todo TODO make it a proper parameter
#define ESTIMATOR_PERIOD 10

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyStatesLocal::icubWholeBodyStatesLocal(const char* _name, const char* _robotName, iCub::iDynTree::iCubTree_version_tag icub_version)
{
    sensors = new icubWholeBodySensors(_name, _robotName);              // sensor interface
    skin_contacts_port = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    skin_contacts_port->open(string("/"+string(_name)+"/skin_contacts:i").c_str());
    estimator = new icubWholeBodyDynamicsEstimator(ESTIMATOR_PERIOD, sensors, skin_contacts_port,icub_version);  // estimation thread
}

#ifdef CODYCO_USES_URDFDOM
icubWholeBodyStatesLocal::icubWholeBodyStatesLocal(const char* _name,
                                                   const char* _robotName,
                                                   iCub::iDynTree::iCubTree_version_tag icub_version,
                                                   std::string urdf_file)
{
    sensors = new icubWholeBodySensors(_name, _robotName);              // sensor interface
    skin_contacts_port = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    skin_contacts_port->open(string("/"+string(_name)+"/skin_contacts:i").c_str());
    estimator = new icubWholeBodyDynamicsEstimator(ESTIMATOR_PERIOD, sensors, skin_contacts_port,icub_version,urdf_file);  // estimation thread
}

#endif

bool icubWholeBodyStatesLocal::init()
{
    bool ok = sensors->init();              // initialize sensor interface
    if( !ok ) { std::cerr << "icubWholeBodyStatesLocal::init() failed: error in sensor initialization." << std::endl; return false; }
    ok = estimator->start();
    if( !ok ) { std::cerr << "icubWholeBodyStatesLocal::init() failed: error in estimator initialization." << std::endl; return false; }
    return ok; // start estimation thread
}

bool icubWholeBodyStatesLocal::close()
{
    std::cout << "icubWholeBodyStatesLocal::close() : closing estimator thread" << std::endl;
    if(estimator) estimator->stop();  // stop estimator BEFORE closing sensor interface
    std::cout << "icubWholeBodyStatesLocal::close() : closing sensor interface" << std::endl;
    bool ok = (sensors ? sensors->close() : true);
    std::cout << "icubWholeBodyStatesLocal::close() : deleting sensor interface" << std::endl;
    if(sensors) { delete sensors; sensors = 0; }
    std::cout << "icubWholeBodyStatesLocal::close() : deleting estimator thread" << std::endl;
    if(estimator) { delete estimator; estimator = 0; }
    return ok;
}

bool icubWholeBodyStatesLocal::addEstimate(const EstimateType et, const LocalId &sid)
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
    case ESTIMATE_FORCE_TORQUE:             return lockAndAddSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

int icubWholeBodyStatesLocal::addEstimates(const EstimateType et, const LocalIdList &sids)
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
    case ESTIMATE_FORCE_TORQUE:             return lockAndAddSensors(SENSOR_FORCE_TORQUE, sids);
    default: break;
    }
    return false;
}

bool icubWholeBodyStatesLocal::removeEstimate(const EstimateType et, const LocalId &sid)
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
    case ESTIMATE_FORCE_TORQUE:             return lockAndRemoveSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

const LocalIdList& icubWholeBodyStatesLocal::getEstimateList(const EstimateType et)
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
    case ESTIMATE_FORCE_TORQUE:             return sensors->getSensorList(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return emptyList;
}

int icubWholeBodyStatesLocal::getEstimateNumber(const EstimateType et)
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
    case ESTIMATE_FORCE_TORQUE:             return sensors->getSensorNumber(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return 0;
}

bool icubWholeBodyStatesLocal::getEstimate(const EstimateType et, const LocalId &sid, double *data, double time, bool blocking)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:
        return false;
    case ESTIMATE_MOTOR_VEL:
        return getMotorVel(sid, data, time, blocking);
    case ESTIMATE_MOTOR_ACC:
        return false;
    case ESTIMATE_MOTOR_TORQUE:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(sid), estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:
        return lockAndReadSensor(SENSOR_PWM, sid, data, time, blocking);
    case ESTIMATE_IMU:
        return estimator->lockAndCopyElementVectorFromVector(sensors->getSensorList(SENSOR_IMU).localToGlobalId(sid), estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE:
        return estimator->lockAndCopyElementVectorFromVector(sensors->getSensorList(SENSOR_FORCE_TORQUE).localToGlobalId(sid), estimator->estimates.lastForceTorques, data);
    default: break;
    }
    return false;
}

bool icubWholeBodyStatesLocal::getEstimates(const EstimateType et, double *data, double time, bool blocking)
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
    case ESTIMATE_FORCE_TORQUE:             return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastForceTorques, data);
    default: break;
    }
    return false;
}

bool icubWholeBodyStatesLocal::setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    return estimator->lockAndSetEstimationParameter(et, ep, value);
}

bool icubWholeBodyStatesLocal::setEstimationOffset(const EstimateType et, const LocalId & sid, const double *value)
{
    return estimator->lockAndSetEstimationOffset(et,sid,value);
}

bool icubWholeBodyStatesLocal::getEstimationOffset(const EstimateType et, const LocalId & sid, double *value)
{
    return estimator->lockAndGetEstimationOffset(et,sid,value);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                    IMPLEMENTATION SPECIFIC METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************


bool icubWholeBodyStatesLocal::getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    return lockAndReadExternalForces(external_forces_list);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          PRIVATE METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************

bool icubWholeBodyStatesLocal::getMotorVel(double *data, double time, bool blocking)
{
    bool res = estimator->lockAndCopyVector(estimator->estimates.lastDq, data);    ///< read joint vel
    if(!res) return false;
    LocalIdList idList = lockAndGetSensorList(SENSOR_ENCODER);
    int i=0;
    FOR_ALL_OF(itBp, itJ, idList)   ///< manage coupled joints
    {
        if(itBp->first == LEFT_ARM && *itJ==0)          // left arm shoulder
            data[i] = data[i];
        else if(itBp->first == LEFT_ARM && *itJ==1)     // left arm shoulder
            data[i] = data[i];
        else if(itBp->first == LEFT_ARM && *itJ==2)     // left arm shoulder
            data[i] = data[i];
        else if(itBp->first == RIGHT_ARM && *itJ==0)    // right arm shoulder
            data[i] = data[i];
        else  if(itBp->first == RIGHT_ARM && *itJ==1)   // right arm shoulder
            data[i] = data[i];
        else if(itBp->first == RIGHT_ARM && *itJ==2)    // right arm shoulder
            data[i] = data[i];
        else if(itBp->first == TORSO && *itJ==1)        // torso
            data[i] = data[i];
        else if(itBp->first == TORSO && *itJ==2)        // torso
            data[i] = data[i];
        i++;
    }
    return true;
}

bool icubWholeBodyStatesLocal::getMotorVel(const LocalId &lid, double *data, double time, bool blocking)
{
    ///< read joint vel
    return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(lid), estimator->estimates.lastDq, data);
}

bool icubWholeBodyStatesLocal::lockAndReadSensors(const SensorType st, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensors(st, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool icubWholeBodyStatesLocal::lockAndReadSensor(const SensorType st, const LocalId sid, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensor(st, sid, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool icubWholeBodyStatesLocal::lockAndReadExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    estimator->mutex.wait();
    external_forces_list = estimator->estimatedLastSkinDynContacts;
    estimator->mutex.post();
    return true;
}

bool icubWholeBodyStatesLocal::lockAndAddSensor(const SensorType st, const LocalId &sid)
{
    estimator->mutex.wait();
    bool res = sensors->addSensor(st, sid);
    estimator->mutex.post();
    return res;
}

int icubWholeBodyStatesLocal::lockAndAddSensors(const SensorType st, const LocalIdList &sids)
{
    estimator->mutex.wait();
    int res = sensors->addSensors(st, sids);
    estimator->mutex.post();
    return res;
}

bool icubWholeBodyStatesLocal::lockAndRemoveSensor(const SensorType st, const LocalId &sid)
{
    estimator->mutex.wait();
    bool res = sensors->removeSensor(st, sid);
    estimator->mutex.post();
    return res;
}

LocalIdList icubWholeBodyStatesLocal::lockAndGetSensorList(const SensorType st)
{
    estimator->mutex.wait();
    LocalIdList res = sensors->getSensorList(st);
    estimator->mutex.post();
    return res;
}

int icubWholeBodyStatesLocal::lockAndGetSensorNumber(const SensorType st)
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
icubWholeBodyDynamicsEstimator::icubWholeBodyDynamicsEstimator(int _period,
                                                               icubWholeBodySensors *_sensors,
                                                               yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * _port_skin_contacts,
                                                               iCub::iDynTree::iCubTree_version_tag _icub_version)
: RateThread(_period),
   sensors(_sensors),
   port_skin_contacts(_port_skin_contacts),
   dqFilt(0), d2qFilt(0),
   icub_version(_icub_version),
   enable_omega_domega_IMU(false)
{
    #ifdef CODYCO_USES_URDFDOM
    use_urdf = false;
    #endif

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
}

#ifdef CODYCO_USES_URDFDOM
icubWholeBodyDynamicsEstimator::icubWholeBodyDynamicsEstimator(int _period,
                                                               icubWholeBodySensors *_sensors,
                                                               yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * _port_skin_contacts,
                                                               iCub::iDynTree::iCubTree_version_tag _icub_version,
                                                               std::string urdf_file)
: RateThread(_period),
   sensors(_sensors),
   port_skin_contacts(_port_skin_contacts),
   dqFilt(0), d2qFilt(0),
   icub_version(_icub_version),
   enable_omega_domega_IMU(false),
   use_urdf(true),
   urdf_file_name(urdf_file)
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
}
#endif

bool icubWholeBodyDynamicsEstimator::threadInit()
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

    LocalIdList available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    FOR_ALL_OF(itBp, itS, available_ft_sensors)
    {
        LocalId loc_id(itBp->first,*itS);
        int ft_index = available_ft_sensors.localToGlobalId(loc_id);
        bool ret = sensors->readSensor(SENSOR_FORCE_TORQUE, loc_id, forcetorques[ft_index].data(), &(forcetorquesStamps[ft_index]),true );
        forcetorqueFilters[ft_index] = new FirstOrderLowPassFilter(forcetorqueCutFrequency,getRate()*1e-3,forcetorques[ft_index]); ///< low pass filter
    }

     LocalIdList available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
     FOR_ALL_OF(itBp, itS, available_imu_sensors)
     {
         LocalId loc_id(itBp->first,*itS);
         int imu_index = available_imu_sensors.localToGlobalId(loc_id);
         //std::cout << "readSensor for IMU " << imu_index << std::endl;
         assert(IMUs.size() > imu_index && IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
         if( sensors->readSensor(SENSOR_IMU, loc_id, IMUs[imu_index].data(), &(IMUStamps[imu_index]),true ) ) {
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

    model_mutex.wait();
    #ifdef CODYCO_USES_URDFDOM
    if( use_urdf )
    {
        icub_model = new iCub::iDynTree::iCubTree(urdf_file_name,icub_version);
    }
    else
    {
        icub_model = new iCub::iDynTree::iCubTree(icub_version);
    }
    #else
    icub_model = new iCub::iDynTree::iCubTree(icub_version);
    #endif
    model_mutex.post();
    return ok;
}

void icubWholeBodyDynamicsEstimator::run()
{
    run_mutex.wait();
    //Temporary workaround: icubWholeBodyStatesLocal needs all the DOF present in the dynamical model
    if( sensors->getSensorNumber(wbi::SENSOR_ENCODER) != icub_model->getNrOfDOFs() )
    {
        LocalIdList list = sensors->getSensorList(wbi::SENSOR_ENCODER);

        std::cerr << "Available sensors: " << std::endl;

        for(LocalIdList::const_iterator it = list.begin(); it != list.end(); it++ ) {
            for(int j=0; j < it->second.size(); j++ ) {
                std::cerr << "Bp: " << it->first << " index " << it->second[j] << std::endl;
            }
        }
           std::cerr << "icubWholeBodyDynamicsEstimator::run() error: " <<
                  sensors->getSensorNumber(wbi::SENSOR_ENCODER) << " joint sensors are available, while  " <<
                  icub_model->getNrOfDOFs() << " joints are present in the model " << std::endl;
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
            el.time = qStamps[0];
            estimates.lastDq = dqFilt->estimate(el);
            estimates.lastD2q = d2qFilt->estimate(el);

        }

        ///< Read force/torque sensors
        ///< \todo TODO buffer value of available_ft_sensors to avoid memory allocation (?)
        LocalIdList available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
        FOR_ALL_OF(itBp, itS, available_ft_sensors)
        {
            LocalId loc_id(itBp->first,*itS);
            int ft_index = available_ft_sensors.localToGlobalId(loc_id);
            if( sensors->readSensor(SENSOR_FORCE_TORQUE, loc_id, forcetorques[ft_index].data(), &(forcetorquesStamps[ft_index]),false ) ) {
                estimates.lastForceTorques[ft_index] = forcetorqueFilters[ft_index]->filt(forcetorques[ft_index]); ///< low pass filter
                estimates.lastForceTorques[ft_index] = estimates.lastForceTorques[ft_index] - forcetorques_offset[ft_index]; /// remove offset
            } else {
                std::cout << "icubWholeBodyStatesLocal: Error in reading F/T sensors, exiting" << std::endl;
                YARP_ASSERT(false);
            }
        }

        ///< Read IMU
        ///< \todo TODO buffer value of available_imu_sensors to avoid memory allocation (?)
        ///< \todo TODO add filters for imu values ->
        LocalIdList available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
        FOR_ALL_OF(itBp, itS, available_imu_sensors)
        {
            LocalId loc_id(itBp->first,*itS);
            int imu_index = available_imu_sensors.localToGlobalId(loc_id);
            //std::cout << "readSensor for IMU " << imu_index << std::endl;
            assert(IMUs.size() > imu_index );
            assert(IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
            if( sensors->readSensor(SENSOR_IMU, loc_id, IMUs[imu_index].data(), &(IMUStamps[imu_index]),false ) ) {
                estimates.lastIMUs[imu_index].setSubvector(0,IMUs[imu_index].subVector(0,3)); ///< orientation is simply copied as already result of an estimation
                estimates.lastIMUs[imu_index].setSubvector(4,imuLinearAccelerationFilters[imu_index]->filt(IMUs[imu_index].subVector(4,6)));  ///< linear acceleration is filtered with a low pass filter
                estimates.lastIMUs[imu_index].setSubvector(7,imuAngularVelocityFilters[imu_index]->filt(IMUs[imu_index].subVector(7,9)));  ///< angular velocity is filtered with a low pass filter
                estimates.lastIMUs[imu_index].setSubvector(10,imuMagnetometerFilters[imu_index]->filt(IMUs[imu_index].subVector(10,12))); ///< magnetometer readings are filtered with a low pass filter
            } else {
                std::cout << "icubWholeBodyStatesLocal: Error in reading IMU, exiting" << std::endl;
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
        el.time = IMUStamps[0];

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

void icubWholeBodyDynamicsEstimator::readSkinContacts()
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


        // if there are more than 1 contact and less than 10 taxels are active then suppose zero moment
        for(map<BodyPart,skinContactList>::iterator it=contactsPerBp.begin(); it!=contactsPerBp.end(); it++)
            if(it->second.size()>1)
                for(skinContactList::iterator c=it->second.begin(); c!=it->second.end(); c++)
                    if(c->getActiveTaxels()<10)
                        c->fixMoment();

        //TODO \todo add other parts
        skinContacts = contactsPerBp[LEFT_ARM];
        skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_ARM].begin(), contactsPerBp[RIGHT_ARM].end());
        skinContacts.insert(skinContacts.end(), contactsPerBp[TORSO].begin(), contactsPerBp[TORSO].end());
        //skinContacts.insert(skinContacts.end(), contactsPerBp[LEFT_LEG].begin(), contactsPerBp[LEFT_LEG].end());
        //skinContacts.insert(skinContacts.end(), contactsPerBp[RIGHT_LEG].begin(), contactsPerBp[RIGHT_LEG].end());
    }
    else if(Time::now()-last_reading_skin_contact_list_Stamp>SKIN_EVENTS_TIMEOUT && last_reading_skin_contact_list_Stamp!=0.0)
    {
        // if time is up, use default contact points \todo TODO
        skinContacts.clear();
    }

    //At this point, in a way or the other skinContacts must have at least a valid contact for each subtree
    //If this is not true, we add a default contact for each subgraph
    map<BodyPart, skinContactList> contactsPerBp = skinContacts.splitPerBodyPart();

    dynContacts = skinContacts.toDynContactList();

    //Ugly, but if we depend on skinContact data structure we have to do in this way
    //default contact torso
    if( contactsPerBp[TORSO].size() == 0 ) {
        dynContacts.push_back(getDefaultContact(TORSO_SUBTREE));
    }

    if( contactsPerBp[RIGHT_ARM].size() == 0 ) {
        dynContacts.push_back(getDefaultContact(RIGHT_ARM_SUBTREE));
    }

    if( contactsPerBp[LEFT_ARM].size() == 0 ) {
        dynContacts.push_back(getDefaultContact(LEFT_ARM_SUBTREE));
    }

    if( contactsPerBp[RIGHT_LEG].size() == 0 ) {
        /// \todo TODO handle v1 and v2 legs
        dynContacts.push_back(getDefaultContact(RIGHT_LEG_SUBTREE));
        dynContacts.push_back(getDefaultContact(RIGHT_FOOT_SUBTREE));
    }

    if( contactsPerBp[LEFT_LEG].size() == 0 ) {
        /// \todo TODO handle v1 and v2 legs
        dynContacts.push_back(getDefaultContact(LEFT_LEG_SUBTREE));
        dynContacts.push_back(getDefaultContact(LEFT_FOOT_SUBTREE));
    }

}



dynContact icubWholeBodyDynamicsEstimator::getDefaultContact(const iCubSubtree icub_subtree)
{
    dynContact return_value;
    switch( icub_subtree ) {
        case RIGHT_ARM_SUBTREE:
            //Copied by iDynContactSolver::computeExternalContacts
            return_value = dynContact(RIGHT_ARM,6,Vector(3,0.0));
            break;
        case LEFT_ARM_SUBTREE:
            //Copied by iDynContactSolver::computeExternalContacts
            return_value = dynContact(LEFT_ARM,6,Vector(3,0.0));
            break;
        case TORSO_SUBTREE:
            return_value = dynContact(TORSO,0,Vector(3,0.0));
            break;
        case RIGHT_LEG_SUBTREE:
            return_value = dynContact(RIGHT_LEG,3,Vector(3,0.0)); //Random contact
            break;
        case LEFT_LEG_SUBTREE:
            return_value = dynContact(LEFT_LEG,3,Vector(3,0.0));  //Random contact
            break;
        case RIGHT_FOOT_SUBTREE:
            //Copied by wholeBodyDynamics run() method
            return_value = dynContact(RIGHT_LEG,5,Vector(3,0.0));
            break;
        case LEFT_FOOT_SUBTREE:
            //Copied by wholeBodyDynamics run() method
            return_value = dynContact(LEFT_LEG,5,Vector(3,0.0));
            break;
        default:
            break;
    }
    return return_value;
}

void icubWholeBodyDynamicsEstimator::estimateExternalForcesAndJointTorques()
{
    //Assume that only a IMU is available

    /** \todo TODO check that serialization between wbi and iDynTree are the same */
    assert(omega_used_IMU.size() == 3);
    assert(domega_used_IMU.size() == 3);
    assert(ddp_used_IMU.size() == 3);

    if( !enable_omega_domega_IMU ) {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
    }

    model_mutex.wait();
    assert(estimates.lastQ.size() == icub_model->getNrOfDOFs());
    assert(estimates.lastDq.size() == icub_model->getNrOfDOFs());
    assert(estimates.lastD2q.size() == icub_model->getNrOfDOFs());

    YARP_ASSERT(icub_model->setInertialMeasure(omega_used_IMU,domega_used_IMU,ddp_used_IMU));
    (icub_model->setAng(estimates.lastQ));
    (icub_model->setDAng(estimates.lastDq));
    (icub_model->setD2Ang(estimates.lastD2q));
    for(int i=0; i < icub_model->getNrOfFTSensors(); i++ ) {
        //std::cout << "Number of F/T sensors available " << estimates.lastForceTorques.size() << std::endl;
        //std::cout << "Number of F/T sensors required by the model " << icub_model->getNrOfFTSensors() << std::endl;
        YARP_ASSERT(estimates.lastForceTorques.size() == icub_model->getNrOfFTSensors());
        assert(estimates.lastForceTorques[i].size() == 6);
        YARP_ASSERT(icub_model->setSensorMeasurement(i,estimates.lastForceTorques[i]));
    }
    icub_model->setContacts(dynContacts);

    /** \todo TODO avoid unlocking/locking a mutex locked in the calling function in the called function */
    /** \todo TODO use a different mutex to ensure that the dimensions of the sensors/states does not change? */
    //mutex.post();

    YARP_ASSERT(icub_model->kinematicRNEA());
    YARP_ASSERT(icub_model->estimateContactForces());
    YARP_ASSERT(icub_model->dynamicRNEA());

    estimatedLastDynContacts = icub_model->getContacts();

    //Create estimatedLastSkinDynContacts using original skinContacts list read from skinManager
    // for each dynContact find the related skinContact (if any) and set the wrench in it
    unsigned long cId;
    bool contactFound=false;
    for(unsigned int i=0; i < estimatedLastDynContacts.size(); i++)
    {
        cId = estimatedLastDynContacts[i].getId();
        for(unsigned int j=0; j<skinContacts.size(); j++)
        {
            if(cId == skinContacts[j].getId())
            {
                skinContacts[j].setForceMoment( dynContacts[i].getForceMoment() );
                contactFound = true;
                j = skinContacts.size();    // break from the inside for loop
            }
        }
        // if there is no associated skin contact, create one
        if(!contactFound)
            skinContacts.push_back(skinContact(estimatedLastDynContacts[i]));
        contactFound = false;
    }

    //mutex.wait();

    estimatedLastSkinDynContacts = skinContacts;

    assert((int)tauJ.size() == icub_model->getNrOfDOFs());
    tauJ = icub_model->getTorques();
    model_mutex.post();

}

void deleteFirstOrderFilterVector(std::vector<iCub::ctrl::FirstOrderLowPassFilter *> & vec)
{
    for(int i=0; i < (int)vec.size(); i++ ) {
        if( vec[i]!= 0 ) { delete vec[i]; vec[i]=0; }
    }
    vec.resize(0);
}

void icubWholeBodyDynamicsEstimator::threadRelease()
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
    return;
}

void icubWholeBodyDynamicsEstimator::lockAndResizeAll(int n)
{
    mutex.wait();
    resizeAll(n);
    mutex.post();
}

void icubWholeBodyDynamicsEstimator::resizeAll(int n)
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

void icubWholeBodyDynamicsEstimator::lockAndResizeFTs(int n)
{
    mutex.wait();
    resizeFTs(n);
    mutex.post();
}

void icubWholeBodyDynamicsEstimator::resizeFTs(int n)
{
    forcetorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorques_offset.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorquesStamps.resize(n,INITIAL_TIMESTAMP);
    forcetorqueFilters.resize(n);
    estimates.lastForceTorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
}

void icubWholeBodyDynamicsEstimator::lockAndResizeIMUs(int n)
{
    mutex.wait();
    resizeIMUs(n);
    mutex.post();
}

void icubWholeBodyDynamicsEstimator::resizeIMUs(int n)
{
    IMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
    IMUStamps.resize(n,INITIAL_TIMESTAMP);
    imuAngularVelocityFilters.resize(n);
    imuLinearAccelerationFilters.resize(n);
    imuMagnetometerFilters.resize(n);
    estimates.lastIMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
}

bool icubWholeBodyDynamicsEstimator::lockAndCopyVector(const Vector &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest, src.data(), sizeof(double)*src.size());
    mutex.post();
    return true;
}

bool icubWholeBodyDynamicsEstimator::lockAndCopyVectorElement(int index, const Vector &src, double *dest)
{
    mutex.wait();
    dest[0] = src[index];
    mutex.post();
    return true;
}

bool icubWholeBodyDynamicsEstimator::lockAndCopyElementVectorFromVector(int index, const std::vector<Vector> &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest,src[index].data(),sizeof(double)*src[index].size());
    mutex.post();
    return true;
}

bool icubWholeBodyDynamicsEstimator::lockAndCopyVectorOfVectors(const std::vector<Vector> &src, double *dest)
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

bool icubWholeBodyDynamicsEstimator::lockAndSetEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
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
    case ESTIMATE_FORCE_TORQUE: ///< \todo TODO
    case ESTIMATE_JOINT_POS:
    case ESTIMATE_MOTOR_POS:
    default: break;
    }
    mutex.post();
    return res;
}

bool icubWholeBodyDynamicsEstimator::lockAndSetEstimationOffset(const EstimateType et, const LocalId & sid, const double *value)
{
    bool res = true;
    int ft_index;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_FORCE_TORQUE: ///< \todo TODO
        ft_index = sensors->getSensorList(SENSOR_FORCE_TORQUE).localToGlobalId(sid);
        memcpy(forcetorques_offset[ft_index].data(), (double*)value, sizeof(double)*sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize);
        break;
    default:
        break;
    }
    mutex.post();
    return res;
}

bool icubWholeBodyDynamicsEstimator::lockAndGetEstimationOffset(const EstimateType et, const LocalId & sid, double *value)
{
    bool res = true;
    int ft_index;
    mutex.wait();
    switch(et)
    {
    case ESTIMATE_FORCE_TORQUE: ///< \todo TODO
        ft_index = sensors->getSensorList(SENSOR_FORCE_TORQUE).localToGlobalId(sid);
        memcpy(value, forcetorques_offset[ft_index].data(), sizeof(double)*sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize);
        break;
    default:
        break;
    }
    mutex.post();
    return res;
}



bool icubWholeBodyDynamicsEstimator::setVelFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyDynamicsEstimator::setAccFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyDynamicsEstimator::setDtauJFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyDynamicsEstimator::setDtauMFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyDynamicsEstimator::setTauJCutFrequency(double fc)
{
    return tauJFilt->setCutFrequency(fc);
}

bool icubWholeBodyDynamicsEstimator::setTauMCutFrequency(double fc)
{
    return tauMFilt->setCutFrequency(fc);
}

bool icubWholeBodyDynamicsEstimator::setPwmCutFrequency(double fc)
{
    return pwmFilt->setCutFrequency(fc);
}

bool icubWholeBodyDynamicsEstimator::setEnableOmegaDomegaIMU(bool _enabled_omega_domega_IMU)
{
    enable_omega_domega_IMU = _enabled_omega_domega_IMU;
    return true;
}
