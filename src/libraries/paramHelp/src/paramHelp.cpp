/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelp.h>

#include <limits.h>
#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <iomanip>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace paramHelp;

//*************************************************************************************************************************
bool ParamHelper::addParam(const ParamDescription &pd)
{
    if(hasParam(pd.id)) return false;

    paramList[pd.id] = pd;
    bool res = true;
    switch(pd.type)
    {
        case PARAM_TYPE_FLOAT:  paramValues[pd.id] = new double;    break;
        case PARAM_TYPE_INT:    paramValues[pd.id] = new int;       break;
        case PARAM_TYPE_BOOL:   paramValues[pd.id] = new bool;      break;
        case PARAM_TYPE_STRING: paramValues[pd.id] = new string;    break;
        default: res = false;
    }
    
    return res;
}

//*************************************************************************************************************************
bool ParamHelper::addParams(const ParamDescription *pdList, int size)
{
    bool res = true;
    for(int i=0; i<size; i++)
        res = res && addParam(pdList[i]);
    return res;
}

//*************************************************************************************************************************
bool ParamHelper::respond(const Bottle& cmd, Bottle& reply)
{
    bool isSetCmd=true;
    int paramId;
    Bottle v;
    if(!identifyCommand(cmd, isSetCmd, paramId, v))    // identify the command and, if it is a "set", put the value in v
        return false;
    if(isSetCmd)
        return setParam(paramId, v, reply);
    return getParam(paramId, reply);
}

//*************************************************************************************************************************
bool ParamHelper::linkParam(int id, void *v)
{
    if(!hasParam(id)) return false;
    paramValues[id] = v;
    return true;
}

//*************************************************************************************************************************
bool ParamHelper::getParam(int id, Bottle &v)
{
    if(!hasParam(id)) return false;

    bool res = true;
    mutex.wait();   // take the mutex before reading the parameter value
    {
        ParamDescription *pd = &paramList[id];
        for(int i=0; i<pd->size.size; i++)
        {
            switch(pd->type)
            {
                case PARAM_TYPE_FLOAT:  v.addDouble(*paramValue<double>(pd->id, i)); break;
                case PARAM_TYPE_INT:    v.addInt(*paramValue<int>(pd->id, i)); break;
                case PARAM_TYPE_BOOL:   v.addInt(*paramValue<bool>(pd->id, i)); break;
                case PARAM_TYPE_STRING: v.addString(paramValue<string>(pd->id, i)->c_str()); break;
                default: res = false;
            }
        }
    }
    mutex.post();
    return res;
}

//*************************************************************************************************************************
bool ParamHelper::setParam(int paramId, const Bottle &v, Bottle &reply)
{
    if(!hasParam(paramId))
        return false;
    if(!checkParamConstraints(paramId, v, reply))    // if the value to set doesn't satisfy the constraints, then return
        return true;
    
    bool res;
    mutex.wait();   // take the mutex before modifying the parameter value
    {
        ParamDescription *pd = &paramList[paramId];
        for(int i=0; i<v.size(); i++)
        {
            switch(pd->type)
            {
                case PARAM_TYPE_FLOAT:  *paramValue<double>(pd->id, i) = v.get(i).asDouble(); break;
                case PARAM_TYPE_INT:    *paramValue<int>(pd->id, i)    = v.get(i).asInt(); break;
                case PARAM_TYPE_BOOL:   *paramValue<bool>(pd->id, i)   = v.get(i).asBool(); break;
                case PARAM_TYPE_STRING: *paramValue<string>(pd->id, i) = v.get(i).asString().c_str(); break;
                default: res = false;
            }
        }
    }
    mutex.post();
    return res;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//*********************************************  PRIVATE METHODS  *********************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

//*************************************************************************************************************************
bool ParamHelper::identifyCommand(const Bottle &cmd, bool &isSetCmd, int &paramId, Bottle &params)
{
    if(cmd.size()<2) return false;  // check minimum size of command
    
    // check whether it is a "set" or a "get" (if neither, then return false)
    if(cmd.get(0).asString() == "set")
        isSetCmd = true;
    else if(cmd.get(0).asString() != "get")
        return false;
    else
        isSetCmd = false;

    Bottle cmdTail = cmd.tail();    // remove first element of command Bottle (i.e. "set" or "get")
    string word;
	int wordCounter;
	bool found;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
    {
		stringstream stream(it->second.name);
		found = true;
        wordCounter = 0;

		while(stream>>word)
        {
			if (cmdTail.get(wordCounter).asString() != word.c_str())
            {
				found=false;
				break;
			}
			wordCounter++;
		}
		if(found)
        {
			paramId = it->second.id;
            for(int k=wordCounter; k<cmdTail.size(); k++)
                params.add(cmdTail.get(k));
			return true;
		}
	}

	return false;
}

//*************************************************************************************************************************
bool ParamHelper::checkParamConstraints(int id, const Bottle &v, Bottle &reply)
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