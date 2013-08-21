/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelpServer.h>

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


//*************************************************************************************************************************
ParamHelperServer::ParamHelperServer(const ParamDescription *pdList, int pdListSize, const CommandDescription *cdList, int cdListSize)
{
    if(pdList!=NULL) addParams(pdList, pdListSize);
    if(cdList!=NULL) addCommands(cdList, cdListSize);
}
//*************************************************************************************************************************
ParamHelperServer::~ParamHelperServer()
{
    // delete all allocated memory
    close();
    /** Do not delete memory associated to parameters, because for the moment I assume
      * that all the parameters are linked to external variables */
}

//*************************************************************************************************************************
bool ParamHelperServer::init(string moduleName)
{
    string portInStreamName     = "/" + moduleName + PORT_IN_STREAM_SUFFIX;
    string portOutStreamName    = "/" + moduleName + PORT_OUT_STREAM_SUFFIX;
    string portInfoName         = "/" + moduleName + PORT_OUT_INFO_SUFFIX;
    portInStream = new BufferedPort<Bottle>();
    portOutStream = new BufferedPort<Bottle>();
    bool res = portInStream->open(portInStreamName.c_str())
            && portOutStream->open(portOutStreamName.c_str())
            && portInfo.open(portInfoName.c_str());
    return res;
}

//*************************************************************************************************************************
bool ParamHelperServer::processRpcCommand(const Bottle& cmd, Bottle& reply)
{
    // try to identify the rpc command contained in 'cmd'
    bool isSetCmd, isGetCmd;
    int id;
    Bottle v;
    if(!identifyCommand(cmd, isSetCmd, isGetCmd, id, v))    // identify the command and, if put anything after it in v
        return false;
    
    if(isSetCmd)
        setParam(id, v, reply);
    else if(isGetCmd)
        getParam(id, reply);
    else if(cmdObs[id] != NULL)
        cmdObs[id]->commandReceived(cmdList[id], v, reply);

    return true;
}

//*************************************************************************************************************************
void ParamHelperServer::getHelpMessage(Bottle &b)
{
    b.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
    if(cmdList.size()>0)
    {
        b.addString("List of the module commands and descriptions:");
        for(map<int,CommandDescription>::iterator it=cmdList.begin(); it!=cmdList.end(); it++)
			b.addString( (" - "+it->second.name+": "+it->second.description).c_str() );
    }
    if(paramList.size()>0)
    {
        b.addString("To get a parameter type 'get <param_name>'");
        b.addString("To set a parameter type 'set <param_name> <param_value>'");
        b.addString("List of the parameter names and descriptions:");
		for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
			b.addString( (" - "+it->second.name+": "+it->second.description).c_str() );
    }
}

//*************************************************************************************************************************
bool ParamHelperServer::sendStreamParams()
{
    Bottle out;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second.ioType.isStreamingOut())
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
bool ParamHelperServer::readStreamParams(bool blockingRead)
{
    Bottle *in = portInStream->read(blockingRead);
    if(in==NULL) return false;
    int j=0;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second.ioType.isStreamingIn())
        {
            ParamDescription *pd = &(it->second);
            Bottle *b = in->get(j).asList();
            
            if(paramValues[pd->id]==NULL){  logMsg("Parameter "+pd->name+" has no associated variable.", MSG_ERROR);        return false; }
            if(j >= in->size()){            logMsg("readStreamParams, unexpected bottle size.", MSG_ERROR);                 return false; }
            if(pd->size.size != b->size()){ logMsg("readStreamParams, unexpected size of parameter "+pd->name, MSG_ERROR);  return false; }

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
bool ParamHelperServer::registerParamCallback(int id, ParamObserver *observer)
{
    if(!hasParam(id)) return false;
    paramObs[id] = observer;
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::registerCommandCallback(int id, CommandObserver *observer)
{
    if(!hasCommand(id)) return false;
    cmdObs[id] = observer;
    return true;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//************************************************  PRIVATE METHODS  ******************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

bool ParamHelperServer::getParam(int id, Bottle &v)
{
    if(!hasParam(id)){                      v.addString("Parameter id not found.");                     return false; }
    if(paramValues[id]==NULL){              v.addString("This parameter has no associated variable.");  return false; }
    if(!paramList[id].ioType.canRead()){    v.addString("This parameter can not be read");              return false; }

    bool res = true;
    ParamDescription *pd = &paramList[id];
    for(int i=0; i<pd->size.size; i++)
    {
        switch(pd->type)
        {
            case PARAM_DATA_FLOAT:  v.addDouble(*paramValue<double>(pd->id, i)); break;
            case PARAM_DATA_INT:    v.addInt(*paramValue<int>(pd->id, i)); break;
            case PARAM_DATA_BOOL:   v.addInt(*paramValue<bool>(pd->id, i)); break;  // the class Bottle has no "addBool" method
            case PARAM_DATA_STRING: v.addString(paramValue<string>(pd->id, i)->c_str()); break;
            default: res = false;
        }
    }
    return res;
}

//*************************************************************************************************************************
bool ParamHelperServer::setParam(int id, const Bottle &v, Bottle &reply)
{
    // if the parameter doesn't exist or the value to set doesn't satisfy the constraints, then return
    if(!hasParam(id)){              reply.addString(("Parameter "+paramList[id].name+" cannot be written.").c_str());           return false; }
    if(paramValues[id]==NULL){      reply.addString(("Parameter "+paramList[id].name+" has no associated variable.").c_str());  return false; }
    if(!checkParamConstraints(id, v, reply)){                                                                                   return false; }
    if(!paramList[id].ioType.canWrite()){   reply.addString(("Parameter "+paramList[id].name+" cannot be written.").c_str());   return false; }
    
    bool res = true;
    ParamDescription *pd = &paramList[id];     // TODO: deal with free size parameters
    for(int i=0; i<v.size(); i++)
    {
        switch(pd->type)
        {
            case PARAM_DATA_FLOAT:  *paramValue<double>(pd->id, i) = v.get(i).asDouble(); break;
            case PARAM_DATA_INT:    *paramValue<int>(pd->id, i)    = v.get(i).asInt(); break;
            case PARAM_DATA_BOOL:   *paramValue<bool>(pd->id, i)   = v.get(i).asBool(); break;
            case PARAM_DATA_STRING: *paramValue<string>(pd->id, i) = v.get(i).asString().c_str(); break;
            default: res = false;
        }
    }
    
    if(res)
    {
        reply.addString(("Parameter "+paramList[id].name+" has been set.").c_str());
        // if an observer is registered, then perform the callback
        if(paramObs[id]!=NULL)
            paramObs[id]->parameterUpdated(paramList[id]);
        return true;
    }
    
    reply.addString(("Parameter "+paramList[id].name+" has not been set: unknown data type.").c_str());
    return false;
}

//*************************************************************************************************************************
bool ParamHelperServer::identifyCommand(const Bottle &cmd, bool &isSetCmd, bool &isGetCmd, int &id, Bottle &params)
{
    if(cmd.size()==0) return false;  // check minimum size of command
    
    // check whether it is a "set" or a "get"
    if(cmd.get(0).asString() == "set"){         isSetCmd = true;  isGetCmd = false; }
    else if(cmd.get(0).asString() == "get"){    isSetCmd = false; isGetCmd = true;  }
    else{                                       isSetCmd = false; isGetCmd = false; }

    string word;
	int wordCounter;
	bool found;

    if(isSetCmd || isGetCmd)
    {
        Bottle cmdTail = cmd.tail();    // remove first element of command Bottle (i.e. "set" or "get")
        for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        {
		    stringstream stream(it->second.name);
		    found = true;
            wordCounter = 0;

		    while(stream>>word)
            {
                if (cmdTail.get(wordCounter).asString() != word.c_str()){ found=false; break; }
			    wordCounter++;
		    }
		    if(found)
            {
			    id = it->second.id;
                for(int k=wordCounter; k<cmdTail.size(); k++)
                    params.add(cmdTail.get(k));
			    return true;
		    }
	    }
        return false;
    }

    for(map<int,CommandDescription>::iterator it=cmdList.begin(); it!=cmdList.end(); it++)
    {
		stringstream stream(it->second.name);
		found = true;
        wordCounter = 0;

		while(stream>>word)
        {
			if (cmd.get(wordCounter).asString() != word.c_str()){ found=false; break; }
			wordCounter++;
		}
		if(found)
        {
			id = it->second.id;
            for(int k=wordCounter; k<cmd.size(); k++)
                params.add(cmd.get(k));
			return true;
		}
	}
	return false;
}
