/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#include <wholeBodyReach/wbiAbstractTasks.h>

using namespace wholeBodyReach;
using namespace std;
using namespace Eigen;
using namespace wbi;
using namespace paramHelp;


/********************************************************************************************/
/***************************************  WbiPDTask *****************************************/
/********************************************************************************************/
void WbiPDTask::linkParameterKp(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_Kp = paramId;
    paramHelper->linkParam(paramId, _Kp.data());
    paramHelper->registerParamValueChangedCallback(paramId, this);
    if(_automaticCriticallyDamped)
        _Kd = 2*_Kp.cwiseSqrt();
}

void WbiPDTask::linkParameterKd(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _Kd.data());
}

void WbiPDTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id == _paramId_Kp)
    {
        if(_automaticCriticallyDamped)
            _Kd = 2*_Kp.cwiseSqrt();
    }
}

bool WbiPDTask::setProportionalGain(VectorConst Kp)
{
    if(!checkGainSize(Kp)) return false;
    _Kp = Kp;
    if(_automaticCriticallyDamped)
        _Kd = 2*_Kp.cwiseSqrt();
    return true;
}

bool WbiPDTask::setDerivativeGain(VectorConst Kd)
{
    if(!checkGainSize(Kd)) return false;
    _Kd = Kd;
    if(_automaticCriticallyDamped)
        _Kp = 0.5*_Kd.array().pow(2);
    return true;
}

bool WbiPDTask::setGainSize(int gainSize)
{
    if(gainSize<0) return false;
    _gainSize = gainSize;
    return true;
}

void WbiPDTask::setAutomaticCriticallyDamped(bool value)
{
    _automaticCriticallyDamped = value;
    if(value)
        _Kd = 2*_Kp.cwiseSqrt();
}


/*********************************************************************************************************/
/******************************************* UTILITIES ***************************************************/
/*********************************************************************************************************/

void wholeBodyReach::compute6DError(const wbi::Frame& H, const wbi::Frame& H_des, Eigen::VectorRef res)
{
    assert(res.size()>=6);
    Vector4d aa;
    Rotation Re = H.R; // Re = R_des * R.transposed();
    Re.setToInverse();
    H_des.R.rotateInPlace(Re);
    
    Re.getAxisAngle(aa.data());
    res[0] = H_des.p[0]-H.p[0];
    res[1] = H_des.p[1]-H.p[1];
    res[2] = H_des.p[2]-H.p[2];
    res[3] = aa[3] * aa[0];
    res[4] = aa[3] * aa[1];
    res[5] = aa[3] * aa[2];
}

void wholeBodyReach::computeOrientationError(const wbi::Rotation& R, const wbi::Rotation& R_des, Eigen::VectorRef res)
{
    assert(res.size()>=3);
    Vector4d aa;
    Rotation Re = R; // Re = R_des * R.transposed();
    Re.setToInverse();
    R_des.rotateInPlace(Re);
    Re.getAxisAngle(aa.data());
    res[0] = aa[3] * aa[0];
    res[1] = aa[3] * aa[1];
    res[2] = aa[3] * aa[2];
}
