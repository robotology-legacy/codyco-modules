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

#include "wbiIcub/wholeBodyInterfaceIcub.h"
#include <iCub/skinDynLib/common.h>
#include <yarp/os/Time.h>
#include <string>


using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

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
icubWholeBodyStates::icubWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow)
{
    sensors = new icubWholeBodySensors(_name, _robotName);              // sensor interface
    estimator = new icubWholeBodyEstimator(ESTIMATOR_PERIOD, sensors);  // estimation thread
}

bool icubWholeBodyStates::init()
{
    bool ok = sensors->init();              // initialize sensor interface
    return ok ? estimator->start() : false; // start estimation thread
}

bool icubWholeBodyStates::close()
{
    if(estimator)   estimator->stop();  // stop estimator BEFORE closing sensor interface
    bool ok = (sensors ? sensors->close() : true);
    if(sensors)     delete sensors;
    if(estimator)   delete estimator;
    return ok;
}

bool icubWholeBodyStates::addEstimate(const EstimateType et, const LocalId &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndAddSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE:             return lockAndAddSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}
        
int icubWholeBodyStates::addEstimates(const EstimateType et, const LocalIdList &sids)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_TORQUE, sids);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensors(SENSOR_PWM, sids);
    case ESTIMATE_IMU:                      return lockAndAddSensors(SENSOR_IMU, sids);
    case ESTIMATE_FORCE_TORQUE:             return lockAndAddSensors(SENSOR_FORCE_TORQUE, sids);
    default: break;
    }
    return false;
}

bool icubWholeBodyStates::removeEstimate(const EstimateType et, const LocalId &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_TORQUE, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndRemoveSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndRemoveSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE:             return lockAndRemoveSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}
        
const LocalIdList& icubWholeBodyStates::getEstimateList(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorList(SENSOR_TORQUE);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorList(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorList(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorList(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE:             return sensors->getSensorList(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return emptyList;
}
        
int icubWholeBodyStates::getEstimateNumber(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_TORQUE);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorNumber(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorNumber(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE:             return sensors->getSensorNumber(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return 0;
}

bool icubWholeBodyStates::getEstimate(const EstimateType et, const LocalId &sid, double *data, double time, bool blocking)
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
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_TORQUE).localToGlobalId(sid), estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_TORQUE).localToGlobalId(sid), estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:        
        return false;
    case ESTIMATE_MOTOR_VEL:        
        return getMotorVel(sid, data, time, blocking);
    case ESTIMATE_MOTOR_ACC:        
        return false;
    case ESTIMATE_MOTOR_TORQUE:     
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_TORQUE).localToGlobalId(sid), estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:     
        return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_TORQUE).localToGlobalId(sid), estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:        
        return lockAndReadSensor(SENSOR_PWM, sid, data, time, blocking);
    case ESTIMATE_IMU:              
        return lockAndReadSensor(SENSOR_IMU, sid, data, time, blocking);
    case ESTIMATE_FORCE_TORQUE:     
        return lockAndReadSensor(SENSOR_FORCE_TORQUE, sid, data, time, blocking);
    default: break;
    }
    return false;
}

bool icubWholeBodyStates::getEstimates(const EstimateType et, double *data, double time, bool blocking)
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
    case ESTIMATE_IMU:                      return lockAndReadSensors(SENSOR_IMU, data, time, blocking);
    case ESTIMATE_FORCE_TORQUE:             return lockAndReadSensors(SENSOR_FORCE_TORQUE, data, time, blocking);
    default: break;
    }
    return false;
}

bool icubWholeBodyStates::setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    return estimator->lockAndSetEstimationParameter(et, ep, value);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          PRIVATE METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************

bool icubWholeBodyStates::getMotorVel(double *data, double time, bool blocking)
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

bool icubWholeBodyStates::getMotorVel(const LocalId &lid, double *data, double time, bool blocking)
{
    ///< read joint vel
    return estimator->lockAndCopyVectorElement(sensors->getSensorList(SENSOR_ENCODER).localToGlobalId(lid), estimator->estimates.lastDq, data);
}

bool icubWholeBodyStates::lockAndReadSensors(const SensorType st, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensors(st, data, 0, blocking); 
    estimator->mutex.post();
    return res;
}

bool icubWholeBodyStates::lockAndReadSensor(const SensorType st, const LocalId sid, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensor(st, sid, data, 0, blocking); 
    estimator->mutex.post();
    return res;
}

bool icubWholeBodyStates::lockAndAddSensor(const SensorType st, const LocalId &sid)
{
    estimator->mutex.wait();
    bool res = sensors->addSensor(st, sid); 
    estimator->mutex.post();
    return res;
}

int icubWholeBodyStates::lockAndAddSensors(const SensorType st, const LocalIdList &sids)
{
    estimator->mutex.wait();
    int res = sensors->addSensors(st, sids);
    estimator->mutex.post();
    return res;
}

bool icubWholeBodyStates::lockAndRemoveSensor(const SensorType st, const LocalId &sid)
{
    estimator->mutex.wait();
    bool res = sensors->removeSensor(st, sid);
    estimator->mutex.post();
    return res;
}

LocalIdList icubWholeBodyStates::lockAndGetSensorList(const SensorType st)
{
    estimator->mutex.wait();
    LocalIdList res = sensors->getSensorList(st); 
    estimator->mutex.post();
    return res;
}

int icubWholeBodyStates::lockAndGetSensorNumber(const SensorType st)
{
    estimator->mutex.wait();
    int res = sensors->getSensorNumber(st);
    estimator->mutex.post();
    return res;
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyEstimator::icubWholeBodyEstimator(int _period, icubWholeBodySensors *_sensors)
: RateThread(_period), sensors(_sensors), dqFilt(0), d2qFilt(0)
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    ///< Window lengths of adaptive window filters
    dqFiltWL            = 16;
    d2qFiltWL           = 25;
    dTauJFiltWL         = 30;
    dTauMFiltWL         = 30;
    ///< Threshold of adaptive window filters
    dqFiltTh            = 1.0;      
    d2qFiltTh           = 1.0;
    dTauJFiltTh         = 0.2;
    dTauMFiltTh         = 0.2;
    ///< Cut frequencies
    tauJCutFrequency    =   3.0;
    tauMCutFrequency    =   3.0;
}

bool icubWholeBodyEstimator::threadInit()
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    ///< create derivative filters
    dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
    dTauJFilt = new AWLinEstimator(dTauJFiltWL, dTauJFiltTh);
    dTauMFilt = new AWLinEstimator(dTauMFiltWL, dTauMFiltTh);
    ///< read sensors
    bool ok = sensors->readSensors(SENSOR_ENCODER, estimates.lastQ.data(), qStamps.data(), true);
    ok = ok && sensors->readSensors(SENSOR_TORQUE, estimates.lastTauJ.data(), tauJStamps.data(), true);
    ///< create low pass filters
    tauJFilt    = new FirstOrderLowPassFilter(tauJCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    tauMFilt    = new FirstOrderLowPassFilter(tauMCutFrequency, getRate()*1e-3, estimates.lastTauJ);
    return ok;
}

void icubWholeBodyEstimator::run()
{
    mutex.wait();
    {
        resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
        
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
        
        ///< Read joint torque sensors
        if(sensors->readSensors(SENSOR_TORQUE, tauJ.data(), tauJStamps.data(), false))
        {
            // @todo Convert joint torques into motor torques
            AWPolyElement el;
            el.time = yarp::os::Time::now();

            estimates.lastTauJ = tauJFilt->filt(tauJ);  ///< low pass filter
            estimates.lastTauM = tauMFilt->filt(tauJ);  ///< low pass filter

            el.data = tauJ;
            estimates.lastDtauJ = dTauJFilt->estimate(el);  ///< derivative filter

            //el.data = tauM;
            estimates.lastDtauM = dTauMFilt->estimate(el);  ///< derivative filter
        }
    }
    mutex.post();
    
    return;
}

void icubWholeBodyEstimator::threadRelease()
{
    // this causes a memory access violation (to investigate)
    //if(dqFilt!=NULL)    delete dqFilt;
    //if(d2qFilt!=NULL)   delete d2qFilt;
    return;
}

void icubWholeBodyEstimator::lockAndResizeAll(int n)
{
    mutex.wait();
    resizeAll(n);
    mutex.post();
}

void icubWholeBodyEstimator::resizeAll(int n)
{
    q.resize(n);
    qStamps.resize(n);
    tauJ.resize(n);
    tauJStamps.resize(n);
    estimates.lastQ.resize(n);
    estimates.lastDq.resize(n);
    estimates.lastD2q.resize(n);
    estimates.lastTauJ.resize(n);
    estimates.lastTauM.resize(n);
    estimates.lastDtauJ.resize(n);
    estimates.lastDtauM.resize(n);
}

bool icubWholeBodyEstimator::lockAndCopyVector(const Vector &src, double *dest)
{
    if(dest==0)
        return false;
    mutex.wait();
    memcpy(dest, src.data(), sizeof(double)*src.size());
    mutex.post();
    return true;
}

bool icubWholeBodyEstimator::lockAndCopyVectorElement(int index, const Vector &src, double *dest)
{
    mutex.wait();
    dest[0] = src[index];
    mutex.post();
    return true;
}

bool icubWholeBodyEstimator::lockAndSetEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    bool res;
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
    case ESTIMATE_IMU:
    case ESTIMATE_FORCE_TORQUE:
    case ESTIMATE_JOINT_POS:
    case ESTIMATE_MOTOR_POS:    
    default: break;
    }
    mutex.post();
    return res;
}

bool icubWholeBodyEstimator::setVelFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyEstimator::setAccFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyEstimator::setDtauJFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyEstimator::setDtauMFiltParams(int windowLength, double threshold)
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

bool icubWholeBodyEstimator::setTauJCutFrequency(double fc)
{
    return tauJFilt->setCutFrequency(fc);
}

bool icubWholeBodyEstimator::setTauMCutFrequency(double fc)
{
    return tauMFilt->setCutFrequency(fc);
}
