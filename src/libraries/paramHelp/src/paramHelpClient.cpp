/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelpClient.h>

#include <yarp\os\Vocab.h>

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


ParamHelperClient::ParamHelperClient(const ParamDescription *pdList, int pdListSize, const CommandDescription *cdList, int cdListSize)
{
    if(pdList!=NULL) addParams(pdList, pdListSize);
    if(cdList!=NULL) addCommands(cdList, cdListSize);
}

//*************************************************************************************************************************
ParamHelperClient::~ParamHelperClient()
{
    // delete all allocated memory
    close();
    /** Do not delete memory associated to parameters, because for the moment I assume
      * that all the parameters are linked to external variables */
}

//*************************************************************************************************************************
bool ParamHelperClient::init(string localName, string remoteName, Bottle &reply)
{
    string remotePortInStreamName   = "/" + remoteName + PORT_IN_STREAM_SUFFIX;
    string remotePortOutStreamName  = "/" + remoteName + PORT_OUT_STREAM_SUFFIX;
    string remotePortInfoName       = "/" + remoteName + PORT_OUT_INFO_SUFFIX;
    string remotePortRpcName        = "/" + remoteName + PORT_RPC_SUFFIX;
    string portInStreamName         = "/" + localName + remotePortInStreamName;
    string portOutStreamName        = "/" + localName + remotePortOutStreamName;
    string portInfoName             = "/" + localName + "/" + remoteName + PORT_IN_INFO_SUFFIX;
    string portRpcName              = "/" + localName + remotePortRpcName;
    portInStream = new BufferedPort<Bottle>();
    portOutStream = new BufferedPort<Bottle>();
    bool res = portInStream->open(portInStreamName.c_str())
            && portOutStream->open(portOutStreamName.c_str())
            && portInfo.open(portInfoName.c_str())
            && portRpc.open(portRpcName.c_str());
    if(!res) reply.addString(("Error while opening ports ("+portInStreamName+", "+portOutStreamName+", "+portInfoName+", "+portRpcName+").").c_str());

    res = res && Network::connect(remotePortOutStreamName.c_str(),  portInStreamName.c_str());
    if(!res) reply.addString(("Error while trying to connect to "+remotePortOutStreamName).c_str());
    res = res && Network::connect(portOutStreamName.c_str(),        remotePortInStreamName.c_str());
    if(!res) reply.addString(("Error while trying to connect to "+remotePortInStreamName).c_str());
    res = res && Network::connect(portInfoName.c_str(),             remotePortInfoName.c_str());
    if(!res) reply.addString(("Error while trying to connect to "+remotePortInfoName).c_str());
    res = res && Network::connect(portRpcName.c_str(),              remotePortRpcName.c_str());
    if(!res) reply.addString(("Error while trying to connect to "+remotePortRpcName).c_str());

    return res;
}
//*************************************************************************************************************************
bool ParamHelperClient::close()
{
    ParamHelperBase::close();
    portRpc.close();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperClient::readInfoMessage(Bottle &b, bool blockingRead)
{
    return portInfo.read(b);
}

//*************************************************************************************************************************
bool ParamHelperClient::sendStreamParams()
{
    Bottle out;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second.ioType.isStreamingIn())
        {
            ParamDescription *pd = &(it->second);
            if(paramValues[pd->id]==NULL){ logMsg("Parameter "+pd->name+" has no associated variable.", MSG_ERROR); return false; }
            Bottle &b = out.addList();
            for(int i=0; i<pd->size.size; i++)
                switch(pd->type)
                {
                    case PARAM_DATA_FLOAT:  b.addDouble(*paramValue<double>(pd->id, i)); break;
                    case PARAM_DATA_INT:    b.addInt(*paramValue<int>(pd->id, i)); break;
                    case PARAM_DATA_BOOL:   b.addInt(*paramValue<bool>(pd->id, i)); break;  // the class Bottle has no "addBool" method
                    case PARAM_DATA_STRING: b.addString(paramValue<string>(pd->id, i)->c_str()); break;
                }
        }
    portOutStream->prepare() = out;
    portOutStream->write();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperClient::readStreamParams(bool blockingRead)
{
    Bottle *in = portInStream->read(blockingRead);
    if(in==NULL) return false;
    int j=0;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second.ioType.isStreamingOut())
        {
            ParamDescription *pd = &(it->second);
            if(paramValues[pd->id]==NULL){  logMsg("readStreamParams, parameter "+pd->name+" has no associated variable.", MSG_ERROR);  return false; }
            if(j >= in->size()){            logMsg("readStreamParams, unexpected bottle size: "+toString(in->size()), MSG_ERROR);       return false; }
            Bottle *b = in->get(j).asList();
            j++;
            if(pd->size.size != b->size())
            { 
                logMsg("readStreamParams, unexpected size of "+pd->name+": "+toString(b->size())+"!="+toString(pd->size.size), MSG_ERROR);
                logMsg("readStreamParams, value read for "+pd->name+": "+b->toString().c_str(), MSG_DEBUG);
                return false; 
            }

            for(int i=0; i<pd->size.size; i++)
                switch(pd->type)
                {
                    case PARAM_DATA_FLOAT:  (*paramValue<double>(pd->id, i))    = b->get(i).asDouble();         break;
                    case PARAM_DATA_INT:    (*paramValue<int>(pd->id, i))       = b->get(i).asInt();            break;
                    case PARAM_DATA_BOOL:   (*paramValue<bool>(pd->id, i))      = b->get(i).asBool();           break;
                    case PARAM_DATA_STRING: (*paramValue<string>(pd->id, i))    = b->get(i).asString().c_str(); break;
                }
        }
    return true;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//************************************************  PRIVATE METHODS  ******************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
