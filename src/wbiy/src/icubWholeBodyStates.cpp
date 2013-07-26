/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email: marco.randazzo@iit.it
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

#include "wbiy/wbiy.h"
#include <iCub/skinDynLib/common.h>
#include <string>


using namespace std;
using namespace wbi;
using namespace wbiy;
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
    vector<string> bodyPartNames(BodyPart_s, BodyPart_s + sizeof(BodyPart_s) / sizeof(string) );
    sensors = new yarpWholeBodySensors(_name, _robotName, bodyPartNames);
    estimator = new icubWholeBodyEstimator(ESTIMATOR_PERIOD, sensors);
}

bool icubWholeBodyStates::init()
{
    bool ok = sensors->init();
    return ok ? estimator->start() : false;
}

bool icubWholeBodyStates::close()
{
    estimator->stop();  // stop estimator BEFORE closing sensor interface
    bool ok = sensors->close();
    delete sensors;
    delete estimator;
    return ok;
}

bool icubWholeBodyStates::getQ(double *q, double time, bool wait){                  return estimator->getQ(q); }
bool icubWholeBodyStates::getDq(double *dq, double time, bool wait){                return estimator->getDq(dq); }
bool icubWholeBodyStates::getDqMotors(double *dqM, double time, bool wait){         return false; }
bool icubWholeBodyStates::getD2q(double *d2q, double time, bool wait){              return estimator->getD2q(d2q); }
bool icubWholeBodyStates::getPwm(double *pwm, double time, bool wait){              return sensors->readPwm(pwm, 0, wait); }
bool icubWholeBodyStates::getInertial(double *inertial, double time, bool wait){    return sensors->readInertial(inertial, 0, wait); }
bool icubWholeBodyStates::getFTsensors(double *ftSens, double time, bool wait){     return sensors->readFTsensors(ftSens, 0, wait); }
bool icubWholeBodyStates::getTorques(double *tau, double time, bool wait){          return false; }


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyEstimator::icubWholeBodyEstimator(int _period, yarpWholeBodySensors *_sensors)
: RateThread(_period), sensors(_sensors), dqFilt(0), d2qFilt(0)
{
    resizeAll(sensors->getDoFs());
}

bool icubWholeBodyEstimator::threadInit()
{
    resizeAll(sensors->getDoFs());
    dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
    sensors->readEncoders(lastQ.data(), qStamps.data(), true);
    return true;
}

void icubWholeBodyEstimator::run()
{
    mutex.wait();
    {
        if(sensors->readEncoders(q.data(), qStamps.data(), false))
        {
            lastQ = q;
            AWPolyElement el;
            el.data = q;
            el.time = qStamps[0];
            lastDq = dqFilt->estimate(el);
            lastD2q = d2qFilt->estimate(el);
        }
    }
    mutex.post();
    
    return;
}

void icubWholeBodyEstimator::threadRelease()
{
    if(dqFilt!=NULL)    delete dqFilt;
    if(d2qFilt!=NULL)   delete d2qFilt;
    return;
}

void icubWholeBodyEstimator::setVelFiltParams(int windowLength, double threshold)
{
    dqFiltWL = windowLength;
    dqFiltTh = threshold;
    if(dqFilt!=NULL)
    {
        AWPolyList list = dqFilt->getList();
        dqFilt = new AWLinEstimator(dqFiltWL, dqFiltTh);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            dqFilt->feedData(*it);
    }
}

void icubWholeBodyEstimator::setAccFiltParams(int windowLength, double threshold)
{
    d2qFiltWL = windowLength;
    d2qFiltTh = threshold;
    if(d2qFilt!=NULL)
    {
        AWPolyList list = d2qFilt->getList();
        d2qFilt = new AWQuadEstimator(d2qFiltWL, d2qFiltTh);
        for(AWPolyList::iterator it=list.begin(); it!=list.end(); it++)
            d2qFilt->feedData(*it);
    }
        
}

void icubWholeBodyEstimator::lockAndResizeAll(int n)
{
    mutex.wait();
    {
        q.resize(n);
        qStamps.resize(n);
        lastQ.resize(n);
        lastDq.resize(n);
        lastD2q.resize(n);
    }
    mutex.post();
}

void icubWholeBodyEstimator::resizeAll(int n)
{
    q.resize(n);
    qStamps.resize(n);
    lastQ.resize(n);
    lastDq.resize(n);
    lastD2q.resize(n);
}

bool icubWholeBodyEstimator::copyVector(const Vector &src, double *dest)
{
    if(dest==0)
        return false;
    memcpy(dest, src.data(), sizeof(double)*src.size());
    return true;
}

bool icubWholeBodyEstimator::lockAndCopyVector(const Vector &src, double *dest)
{
    bool res = false;
    mutex.wait();
    {
        res = copyVector(src, dest);
    }
    mutex.post();
    return res;
}

bool icubWholeBodyEstimator::removeJoint(const LocalId &j)
{
    bool res;
    mutex.wait();
    {
        if(res = sensors->removeJoint(j))
            resizeAll(sensors->getDoFs());
    }
    mutex.post();
    return res;
}

bool icubWholeBodyEstimator::addJoint(const LocalId &j)
{
    bool res;
    mutex.wait();
    {
        if(res = sensors->addJoint(j))
            resizeAll(sensors->getDoFs());
    }
    mutex.post();
    return res;
}

int icubWholeBodyEstimator::addJoints(const LocalIdList &j)
{
    int res;
    mutex.wait();
    {
        if((res = sensors->addJoints(j))>0)
            resizeAll(sensors->getDoFs());
    }
    mutex.post();
    return res;
}

bool icubWholeBodyEstimator::getQ(double *q){       return lockAndCopyVector(lastQ, q); }
bool icubWholeBodyEstimator::getDq(double *dq){     return lockAndCopyVector(lastDq, dq); }
bool icubWholeBodyEstimator::getD2q(double *d2q){   return lockAndCopyVector(lastD2q, d2q);}
