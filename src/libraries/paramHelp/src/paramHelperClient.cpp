/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelpUtil.h>
#include <paramHelp/paramHelperClient.h>

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


ParamHelperClient::ParamHelperClient(const ParamProxyInterface *const *pdList,  int pdListSize, 
                                     const CommandDescription *cdList,          int cdListSize)
{
    if(pdList!=NULL) addParams(pdList, pdListSize);
    if(cdList!=NULL) addCommands(cdList, cdListSize);
}

//*************************************************************************************************************************
ParamHelperClient::~ParamHelperClient()
{
    // delete all allocated memory
    close();
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
    if(!res)
    {
        reply.addString(("Error while opening ports ("+portInStreamName+", "+portOutStreamName+", "+portInfoName+", "+portRpcName+").").c_str());
        return false;
    }

    res = res && Network::connect(remotePortOutStreamName.c_str(), portInStreamName.c_str());
    if(!res) 
    {
        reply.addString(("Error while trying to connect to "+remotePortOutStreamName).c_str());
        return false;
    }

    res = res && Network::connect(portOutStreamName.c_str(), remotePortInStreamName.c_str());
    if(!res) 
    {
        reply.addString(("Error while trying to connect to "+remotePortInStreamName).c_str());
        return false;
    }

    res = res && Network::connect(portInfoName.c_str(), remotePortInfoName.c_str());
    if(!res)
    {
        reply.addString(("Error while trying to connect to "+remotePortInfoName).c_str());
        return false;
    }

    res = res && Network::connect(portRpcName.c_str(), remotePortRpcName.c_str());
    if(!res)
    {
        reply.addString(("Error while trying to connect to "+remotePortRpcName).c_str());
        return false;
    }

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
    ///< Streaming parameters are inserted inside a Bottle with the following format:
    ///< (PARAM_ID0 VALUE0 VALUE1 ... VALUEN) ... (PARAM_IDM VALUE0 ... VALUEN)
    Bottle out;
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second->ioType.isStreamingIn())
        {
            Bottle &b = out.addList();
            b.addInt(it->second->id);
            it->second->getAsBottle(b);
        }
    portOutStream->prepare() = out;
    portOutStream->write();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperClient::readStreamParams(bool blockingRead)
{
    Bottle *in = portInStream->read(blockingRead);
    if(in==NULL || in->size()==0) return false;
    Bottle reply;
    for(int i=0; i<in->size(); i++)
    {
        if(!in->get(i).isList())
        {
            logMsg(strcat("[readStreamParams] Value ",i," of read Bottle is not a list! Skipping it."), MSG_ERROR);
            continue;
        }
        Bottle *b = in->get(i).asList();
        if(b->size()<1)
        {
            logMsg(strcat("[readStreamParams] Value ",i," of read Bottle is an empty list! Skipping it."), MSG_ERROR);
            continue;
        }
        if(!b->get(0).isInt())
        {
            logMsg(strcat("[readStreamParams] 1st element of value ",i," of read Bottle is not an int! Skipping it."), MSG_ERROR);
            continue;
        }
        int paramId = b->get(0).asInt();
        if(!paramList[paramId]->set(b->tail(), &reply))
        {
            logMsg("[readStreamParams] "+reply.toString(), MSG_ERROR);
            reply.clear();
        }
    }
    return true;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//************************************************  PRIVATE METHODS  ******************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
