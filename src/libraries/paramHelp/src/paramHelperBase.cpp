/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelperBase.h>

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
    if(!hasParam(id) || v==0) 
        return false;
    paramList[id]->linkToVariable(v);
    return true;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//************************************************  BASE METHODS  *********************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//
//bool ParamHelperBase::setParam(int paramId, const void *v)
//{
//    // if the value to set doesn't satisfy the constraints, then return
//    if(!hasParam(paramId) || !checkParamConstraints(paramId, v)) return false; 
//    
//    //paramList[paramId]->fromString(v);
//    
//    return true;
//}

//*************************************************************************************************************************
bool ParamHelperBase::addParams(const ParamProxyInterface *const *pdList, int size)
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
bool ParamHelperBase::addParam(const ParamProxyInterface* pd)
{
    if(hasParam(pd->id)) 
    {
        printf("[ParamHelperBase::addParam()]: Parameter %s has the same id of parameter %s\n", 
            paramList[pd->id]->name.c_str(), pd->name.c_str());
        return false;   // there exists a parameter with the same id
    }
    paramList[pd->id] = pd->clone();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::addCommand(const CommandDescription &cd)
{
    if(hasCommand(cd.id))
    {
        printf("[ParamHelperBase::addCommand()]: Command %s has the same id of command %s\n", 
            cmdList[cd.id].name.c_str(), cd.name.c_str());
        return false;   // there exists a command with the same id
    }
    cmdList[cd.id] = cd;
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::checkParamConstraints(int id, const Bottle &v, Bottle &reply)
{
    if(!hasParam(id))
    {
        reply.addString("There is no parameter with the specified id: ");
        reply.addInt(id);
        return false;
    }
    return paramList[id]->checkConstraints(v, &reply);
}

//*************************************************************************************************************************
void ParamHelperBase::logMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[ParamHelper] %s\n", s.c_str());
}