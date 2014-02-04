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
    closePorts();
    deleteParameters();
}

//*************************************************************************************************************************
bool ParamHelperClient::init(string localName, string remoteName, Bottle &reply)
{
    string remotePortInStreamName   = "/" + remoteName + PORT_IN_STREAM_SUFFIX;
    string remotePortOutStreamName  = "/" + remoteName + PORT_OUT_STREAM_SUFFIX;
    string remotePortInfoName       = "/" + remoteName + PORT_OUT_INFO_SUFFIX;
    string remotePortRpcName        = "/" + remoteName + PORT_RPC_SUFFIX;

    ///< Check whether the remote ports exist, if not return false
    if(!Network::exists(remotePortInStreamName.c_str()) || !Network::exists(remotePortOutStreamName.c_str()) ||
        !Network::exists(remotePortInfoName.c_str()) || !Network::exists(remotePortRpcName.c_str()))
    {
        reply.addString("One of the remote port does not exist");
        return false;
    }

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
        closePorts();
        return false;
    }

    res = res && Network::connect(remotePortOutStreamName.c_str(), portInStreamName.c_str());
    if(!res) 
    {
        reply.addString(("Error while trying to connect to "+remotePortOutStreamName).c_str());
        closePorts();
        return false;
    }

    res = res && Network::connect(portOutStreamName.c_str(), remotePortInStreamName.c_str());
    if(!res) 
    {
        reply.addString(("Error while trying to connect to "+remotePortInStreamName).c_str());
        closePorts();
        return false;
    }

    res = res && Network::connect(portInfoName.c_str(), remotePortInfoName.c_str());
    if(!res)
    {
        reply.addString(("Error while trying to connect to "+remotePortInfoName).c_str());
        closePorts();
        return false;
    }

    res = res && Network::connect(portRpcName.c_str(), remotePortRpcName.c_str());
    if(!res)
    {
        reply.addString(("Error while trying to connect to "+remotePortRpcName).c_str());
        closePorts();
        return false;
    }

    initDone = res;
    return res;
}

//*************************************************************************************************************************
bool ParamHelperClient::close()
{
    closePorts();
    portRpc.close();
    deleteParameters();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperClient::readInfoMessage(Bottle &b, bool blockingRead)
{
    if(initDone)
        return portInfo.read(b);
    return false;
}

//*************************************************************************************************************************
bool ParamHelperClient::sendStreamParams()
{
    if(!initDone)
        return false;
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
    if(!initDone)
        return false;
    Bottle *in = portInStream->read(blockingRead);
    if(in==NULL || in->size()==0) return false;
    Bottle reply;
    for(int i=0; i<in->size(); i++)
    {
        if(!in->get(i).isList())
        {
            logMsg(strapp("[readStreamParams] Value ",i," of read Bottle is not a list! Skipping it."), MSG_ERROR);
            continue;
        }
        Bottle *b = in->get(i).asList();
        if(b->size()<1)
        {
            logMsg(strapp("[readStreamParams] Value ",i," of read Bottle is an empty list! Skipping it."), MSG_ERROR);
            continue;
        }
        if(!b->get(0).isInt())
        {
            logMsg(strapp("[readStreamParams] 1st element of Bottle ",i," is not an int (",b->get(0).toString().c_str()
                ,")! Skipping it."), MSG_ERROR);
            continue;
        }
        int paramId = b->get(0).asInt();
        if(!paramList[paramId]->set(b->tail(), &reply))
        {
            std::string reply_str(reply.toString().c_str());
            logMsg("[readStreamParams] "+reply_str, MSG_ERROR);
            reply.clear();
        }
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperClient::setRpcParam(int paramId, Bottle *reply)
{
    if(!initDone)
        return false;
    ///< check that the parameter exists and it can be written
    if(!hasParam(paramId)){ logMsg(strapp("[setRpcParam] There is no param with id ",paramId), MSG_ERROR); return false; }
    ParamProxyInterface *ppi = paramList[paramId];
    if(!ppi->ioType.canWrite()){ logMsg(strapp("[setRpcParam] Parameter ",ppi->name," cannot be written"),MSG_ERROR); return false;}

    ///< prepare the Bottle to send
    Bottle outBottle;
    string word;
    stringstream stream(ppi->name);
    outBottle.addString("set");
	while(stream>>word)
        outBottle.addString(word.c_str());
    ppi->getAsBottle(outBottle);

    ///< send the Bottle
    bool res;
    if(reply==0)
        res = portRpc.write(outBottle);
    else
        res = portRpc.write(outBottle, *reply);
    if(!res) logMsg("[setRpcParam] Error while send rpc msg", MSG_ERROR);
    return res;
}

//*************************************************************************************************************************
bool ParamHelperClient::getRpcParam(int paramId, Bottle *reply)
{
    if(!initDone)
        return false;
    ///< check that the parameter exists and it can be written
    if(!hasParam(paramId)){ logMsg(strapp("[getRpcParam] There is no param with id ",paramId), MSG_ERROR); return false; }
    ParamProxyInterface *ppi = paramList[paramId];
    if(!ppi->ioType.canRead()){ logMsg(strapp("[getRpcParam] Parameter ",ppi->name," cannot be read"),MSG_ERROR); return false;}

    ///< prepare the Bottle to send
    Bottle outBottle;
    string word;
    stringstream stream(ppi->name);
    outBottle.addString("get");
	while(stream>>word)
        outBottle.addString(word.c_str());

    ///< send the Bottle
    Bottle value;
    if(!portRpc.write(outBottle, value)){ logMsg("[getRpcParam] Error while send rpc msg", MSG_ERROR); return false; }

    ///< if parameter size is about to change, perform callbacks
    if(ppi->size.freeSize && (value.size()!=ppi->size.size) && paramSizeObs[paramId]!=NULL)
        paramSizeObs[paramId]->parameterSizeChanged(ppi, value.size());

    ///< read the reply with the parameter value
    if(!ppi->set(value, reply))
    { 
        logMsg(strapp("[getRpcParam] Error while setting read value of ", ppi->name,": ",value.toString().c_str()), MSG_ERROR);
        return false;
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperClient::sendRpcCommand(int cmdId, Bottle *params, Bottle *reply)
{
    if(!initDone)
        return false;
    ///< check that the command exists
    if(!hasCommand(cmdId)){ logMsg(strapp("[sendRpcCommand] There is no command with id ",cmdId), MSG_ERROR); return false; }
    CommandDescription &cd = cmdList[cmdId];

    ///< prepare the Bottle to send
    Bottle outBottle;
    string word;
    stringstream stream(cd.name);
	while(stream>>word)
        outBottle.addString(word.c_str());
    if(params!=0)
        for(int i=0; i<params->size(); i++)
            outBottle.add(params->get(i));

    ///< send the Bottle
    bool res;
    if(reply==0)
        res = portRpc.write(outBottle);
    else
        res = portRpc.write(outBottle, *reply);
    if(!res) logMsg("[sendRpcCommand] Error while send rpc msg", MSG_ERROR);
    return res;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//************************************************  PRIVATE METHODS  ******************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
