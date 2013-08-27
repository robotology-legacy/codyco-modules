/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelp.h>

#include <yarp/os/Vocab.h>

#include <limits.h>
#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <cassert>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace paramHelp;

//*************************************************************************************************************************
bool ParamIOType::canRead(){ return value==PARAM_OUTPUT || value==PARAM_OUT_STREAM || value==PARAM_IN_OUT || value==PARAM_IN_OUT_STREAM || PARAM_IN_STREAM; }
bool ParamIOType::canWrite(){ return value==PARAM_INPUT || value==PARAM_IN_STREAM || value==PARAM_IN_OUT || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::isStreaming(){ return value==PARAM_OUT_STREAM || value==PARAM_IN_STREAM || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::isStreamingOut(){ return value==PARAM_OUT_STREAM || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::isStreamingIn(){ return value==PARAM_IN_STREAM || value==PARAM_IN_OUT_STREAM; }

//*************************************************************************************************************************
bool ParamHelperBase::close()
{
    if(portInStream){  portInStream->interrupt();  portInStream->close();  delete portInStream;  portInStream=0; }
    if(portOutStream){ portOutStream->interrupt(); portOutStream->close(); delete portOutStream; portOutStream=0; }
    portInfo.close();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::linkParam(int id, void *v)
{
    if(!hasParam(id) || v==0) return false;
    assert(paramValues[id]!=NULL);
    void *currentValue = paramValues[id];   // store current value of the parameter
    paramValues[id] = v;                    // point the parameter to the new variable
    setParam(id, currentValue);             // set the new variable to the current value of the parameter
    // delete previously allocated memory
    switch(paramList[id].type)
    {
    case PARAM_DATA_FLOAT:  delete[] (double*)currentValue;     break;
    case PARAM_DATA_INT:    delete[] (int*)currentValue;        break;
    case PARAM_DATA_BOOL:   delete[] (bool*)currentValue;       break;
    case PARAM_DATA_STRING: delete[] (string*)currentValue;     break;
    }
    
    return true;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//************************************************  BASE METHODS  *********************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

bool ParamHelperBase::setParam(int paramId, const void *v)
{
    // if the value to set doesn't satisfy the constraints, then return
    if(!hasParam(paramId) || !checkParamConstraints(paramId, v)) return false; 
    
    bool res = true;
    ParamDescription *pd = &paramList[paramId];
    for(int i=0; i<pd->size.size; i++)
    {
        switch(pd->type)
        {
            case PARAM_DATA_FLOAT:  *paramValue<double>(pd->id, i) = ((double*)v)[i]; break;
            case PARAM_DATA_INT:    *paramValue<int>(pd->id, i)    = ((int*)v)[i]; break;
            case PARAM_DATA_BOOL:   *paramValue<bool>(pd->id, i)   = ((bool*)v)[i]; break;
            case PARAM_DATA_STRING: *paramValue<string>(pd->id, i) = ((string*)v)[i]; break;
            default: res = false;
        }
    }
    return res;
}

//*************************************************************************************************************************
bool ParamHelperBase::addParams(const ParamDescription *pdList, int size)
{
    bool res = true;
    for(int i=0; i<size; i++)
        res = res && addParam(pdList[i]);
    return res;
}

//*************************************************************************************************************************
bool ParamHelperBase::addCommands(const CommandDescription *cdList, int size)
{
    bool res = true;
    for(int i=0; i<size; i++)
        res = res && addCommand(cdList[i]);
    return res;
}

//*************************************************************************************************************************
bool ParamHelperBase::addParam(const ParamDescription &pd)
{
    if(hasParam(pd.id)) return false;   // there exists a parameter with the same id
    paramList[pd.id] = pd;
    paramValues[pd.id] = NULL;
    switch(pd.type)
    {
    case PARAM_DATA_FLOAT:  paramValues[pd.id] = new double[pd.size.size];      break;
    case PARAM_DATA_INT:    paramValues[pd.id] = new int[pd.size.size];         break;
    case PARAM_DATA_BOOL:   paramValues[pd.id] = new bool[pd.size.size];        break;
    case PARAM_DATA_STRING: paramValues[pd.id] = new string[pd.size.size];      break;
    default:                return false;   // unknown data type
    }
    setParam(pd.id, pd.defaultValue);   // set the variable to the default value
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::addCommand(const CommandDescription &cd)
{
    if(hasCommand(cd.id)) return false;   // there exists a command with the same id
    cmdList[cd.id] = cd;
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::checkParamConstraints(int id, const Bottle &v, Bottle &reply)
{
    // check size
    if(!paramList[id].size.freeSize && v.size() != paramList[id].size.size)
    {
        reply.addString(("Wrong parameter size, expected "+toString(paramList[id].size.size)+" found "+toString(v.size())).c_str());
        return false;
    }
    // check bounds
    bool HLB = paramList[id].bounds.hasLowerBound;
    bool HUB = paramList[id].bounds.hasUpperBound;
    if(HLB || HUB)
    {
        double LB = paramList[id].bounds.lowerBound;
        double UB = paramList[id].bounds.upperBound;
        double vi;
        for(int i=0; i<v.size(); i++)
        {
            vi = v.get(i).asDouble();   // bounds make sense only for float or int values, so "asDouble()" should work fine
            if(HLB && vi<LB)
            {
                reply.addString(("Parameter out of range, lower bound "+toString(LB)+", value "+toString(vi)).c_str());
                return false;
            }
            if(HUB && vi>UB)
            {
                reply.addString(("Parameter out of range, upper bound "+toString(UB)+", value "+toString(vi)).c_str());
                return false;
            }
        }
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::checkParamConstraints(int id, const void *v)
{
    if(v==0) return false;

    // check bounds
    bool HLB = paramList[id].bounds.hasLowerBound;
    bool HUB = paramList[id].bounds.hasUpperBound;
    if(HLB || HUB)
    {
        double LB = paramList[id].bounds.lowerBound;
        double UB = paramList[id].bounds.upperBound;
        double vi;
        for(int i=0; i<paramList[id].size.size; i++)
        {
            if(paramList[id].type == PARAM_DATA_FLOAT)
                vi = ((double*)v)[i];
            else if(paramList[id].type == PARAM_DATA_INT)
                vi = ((int*)v)[i];
            else
                return true;    // bounds make sense only for float or int values
            if(HLB && vi<LB)    return false;
            if(HUB && vi>UB)    return false;
        }
    }
    return true;
}

//*************************************************************************************************************************
void ParamHelperBase::logMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[ParamHelper] %s\n", s.c_str());
}