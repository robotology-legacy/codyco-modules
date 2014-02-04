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
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <cassert>

#if defined(WIN32) || defined(_WIN32) 
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
 #endif

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace paramHelp;

//*************************************************************************************************************************
ParamHelperBase::ParamHelperBase()
{
    initDone         = false;
    portInStream    = 0;
    portOutStream   = 0;
    portOutMonitor  = 0;
}

//*************************************************************************************************************************
bool ParamHelperBase::closePorts()
{
    if(portInStream){   portInStream->interrupt();   portInStream->close();   delete portInStream;   portInStream=0;   }
    if(portOutStream){  portOutStream->interrupt();  portOutStream->close();  delete portOutStream;  portOutStream=0;  }
    if(portOutMonitor){ portOutMonitor->interrupt(); portOutMonitor->close(); delete portOutMonitor; portOutMonitor=0; }
    portInfo.close();
    initDone = false;
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::deleteParameters()
{
    ///< delete all the cloned ParamProxyInterfaces
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); ++it)
    {
        delete it->second;
        it->second = 0;
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::linkParam(int id, void *v, int newSize)
{
    if(!hasParam(id) || v==0) 
        return false;
    paramList[id]->linkToVariable(v, newSize);
    return true;
}

//*************************************************************************************************************************
bool ParamHelperBase::registerParamSizeChangedCallback(int id, ParamSizeObserver *observer)
{
    if(!hasParam(id)) return false;
    paramSizeObs[id] = observer;
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
        logMsg(strapp("[addParam] Parameter ",paramList[pd->id]->name," has the same id of parameter ",pd->name), MSG_ERROR);
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
        logMsg(strapp("[addCommand] Command ",cmdList[cd.id].name," has the same id of command ",cd.name), MSG_ERROR);
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
bool ParamHelperBase::writeParamsOnFile(string filename, int *paramIds, int paramNumber)
{
    ofstream file(filename.c_str(), ios::out | ios::app); ///< append content at the end of the file
    if(!file.is_open())
    {
        logMsg("[writeParamsOnFile] Error while opening file "+filename, MSG_ERROR);
        return false;
    }

    time_t rawtime;
    time (&rawtime);
    file<<"%File written on "<< ctime(&rawtime)<< endl;
    ///< if no parameter is specified, write all of them
    if(paramNumber<=0 || paramIds==0)
    {
        for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            file<< it->second->name<<"\t"<<it->second->getAsString()<<endl;
    }
    else
    {
        ParamProxyInterface *ppi;
        for(int i=0; i<paramNumber; i++)
        {
            if(!hasParam(paramIds[i]))
            {
                logMsg(strapp("[writeParamsOnFile] There exists no parameter with id", paramIds[i]), MSG_ERROR);
                continue;
            }
            ppi = paramList[paramIds[i]];
            file<< ppi->name<<"\t"<<ppi->getAsString()<<endl;
        }
    }

    ///< close the file and return
    char the_path[256];
    GetCurrentDir(the_path, 255);
    logMsg(strapp("Written file ", the_path,"\\",filename), MSG_INFO);
    file<<endl;
    file.close();
    return true;
}

//*************************************************************************************************************************
void ParamHelperBase::logMsg(const string &s, MsgType type) const
{
    if(type>=MSG_DEBUG)
        printf("[ParamHelper] %s\n", s.c_str());
}
