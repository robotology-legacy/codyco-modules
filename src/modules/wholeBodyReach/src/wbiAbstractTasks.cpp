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


/********************************************************************************************/
/***************************************  WbiPDTask *****************************************/
/********************************************************************************************/
void WbiPDTask::linkParameterKp(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_Kp = paramId;
    paramHelper->linkParam(paramId, _Kp.data());
    paramHelper->registerParamValueChangedCallback(paramId, this);
}

void WbiPDTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id == _paramId_Kp)
    {
        if(_automaticCriticallyDamped)
            _Kd = 2*_Kp.cwiseSqrt();
    }
    else
        cout<<"A callback is registered but not managed for the parameter "<<pp->name<<endl;
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
