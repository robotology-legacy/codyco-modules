/**
 * Copyright (C) 2014 CoDyCo
 * @author: Francesco Romano
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

#include "wholeBodyReach/MinJerkTrajGen.h"
#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/sig/Vector.h>

using namespace wholeBodyReach;
using namespace Eigen;
        
MinJerkTrajGen::MinJerkTrajGen(int size, double sampleTime, double trajDuration)
: _size(size)
, _sampleTime(sampleTime)
, _trajDuration(trajDuration)
, _minimumJerkGenerator(0)
, _posEig(size)
, _velEig(size)
, _accEig(size)
, _setPointYarp(size)
, _initialValueYarp(_size)
{
    _minimumJerkGenerator = new iCub::ctrl::minJerkTrajGen(_size, _sampleTime, _trajDuration);
}

MinJerkTrajGen::~MinJerkTrajGen()
{
    if (_minimumJerkGenerator) {
        delete _minimumJerkGenerator;
        _minimumJerkGenerator = 0;
    }
}

MinJerkTrajGen* MinJerkTrajGen::clone() const
{
    MinJerkTrajGen* newObject = new MinJerkTrajGen(_size, _sampleTime, _trajDuration);
    return newObject;
}

bool MinJerkTrajGen::initializeTimeParameters(double sampleTime, double trajDuration)
{
    _sampleTime = sampleTime;
    _trajDuration = trajDuration;
    bool result;
    result = _minimumJerkGenerator->setT(trajDuration);
    result = result && _minimumJerkGenerator->setTs(sampleTime);
    return result;
}

double MinJerkTrajGen::setTrajectoryDuration(double trajDuration)
{
    _trajDuration = trajDuration;
    return _minimumJerkGenerator->setT(trajDuration);
}

double MinJerkTrajGen::setSampleTime(double sampleTime)
{
    _sampleTime = sampleTime;
    return _minimumJerkGenerator->setTs(sampleTime);
}

bool MinJerkTrajGen::init(VectorConst initialValue)
{
    assert(_minimumJerkGenerator!=NULL);
    assert(initialValue.size()==_size);
    for(int i=0; i<_size; i++)
        _initialValueYarp(i)=initialValue(i);
    _minimumJerkGenerator->init(_initialValueYarp);
    return true;
}

bool MinJerkTrajGen::computeNextValues()
{
    assert(_minimumJerkGenerator!=NULL);
    _minimumJerkGenerator->computeNextValues(_setPointYarp);
    return true;
}

bool MinJerkTrajGen::computeNextValues(VectorConst setPoint)
{
    assert(_minimumJerkGenerator!=NULL);
    assert(setPoint.size()==_size);
    for (int i = 0; i < _size; i++)
        _setPointYarp(i) = setPoint(i);
    _minimumJerkGenerator->computeNextValues(_setPointYarp);
    return true;
}

VectorConst MinJerkTrajGen::getPos()
{
    assert(_minimumJerkGenerator!=NULL);
    const yarp::sig::Vector& pos = _minimumJerkGenerator->getPos();
    for(int i=0;i<_size;i++)
        _posEig(i) = pos(i);
    return _posEig;
}

VectorConst MinJerkTrajGen::getVel()
{
    assert(_minimumJerkGenerator!=NULL);
    const yarp::sig::Vector& vel = _minimumJerkGenerator->getVel();
    for(int i=0;i<_size;i++)
        _velEig(i) = vel(i);
    return _velEig;
}

VectorConst MinJerkTrajGen::getAcc()
{
    assert(_minimumJerkGenerator!=NULL);
    const yarp::sig::Vector& acc = _minimumJerkGenerator->getAcc();
    for(int i=0;i<_size;i++)
        _accEig(i) = acc(i);
    return _accEig;
}

