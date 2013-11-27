/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelpUtil.h>
#include <paramHelp/paramHelperServer.h>

#include <yarp/os/Vocab.h>

#include <limits.h>
#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <cassert>
#include <algorithm>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace paramHelp;


//*************************************************************************************************************************
ParamHelperServer::ParamHelperServer(const ParamProxyInterface *const *pdList, int pdListSize, const CommandDescription *cdList, int cdListSize)
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
void ParamHelperServer::initializeParams(ResourceFinder &rf, Bottle &reply)
{
    //Value *v;
    Bottle temp;
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
    {
        string paramName = it->second->name;
        replace( paramName.begin(), paramName.end(), ' ', '_'); // replace "white spaces" with "underscores"
        Bottle &b = rf.findGroup(paramName.c_str());
        if(!b.isNull())
        {
            ///< check whether this is the parameter value of a list of section names, which contain the parameter values
            if(paramList[it->second->id]->checkConstraints(b.tail()))
            {
                printf("Param %s is simple\n", it->second->name.c_str());
                temp = b.tail();
            }
            else
            {
                printf("Param %s is structured\n", it->second->name.c_str());
                temp.clear();
                for(int i=1; i<b.size(); i++)
                {
                    ConstString key = b.get(i).asString();
                    Bottle &sb = rf.findGroup(key);     ///< find all sections related to this parameter
                    if(!sb.isNull())
                    {
                        //printf("Looking for subparameter %s I found this Bottle: %s\n", key.c_str(), sb.toString().c_str());
                        temp.addList() = sb.tail();
                    }
                }
            }

            setParam(it->second->id, temp, reply, true);
        }
    }
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
    CommandType cmdType;
    int id;
    Bottle v;
    if(!identifyCommand(cmd, cmdType, id, v))    // identify the command and, put anything after it in v
        return false;
    
    switch(cmdType)
    {
    case COMMAND_SET:       setParam(id, v, reply);     break;
    case COMMAND_SET_ONE:   setOneParam(id, v, reply);  break;
    case COMMAND_SET_ALL:   setAllParam(id, v, reply);  break;
    case COMMAND_GET:       getParam(id, reply);        break;
    case COMMAND_GET_ONE:   getOneParam(id, v, reply);  break;
    case COMMAND_GENERIC:   cmdObs[id]->commandReceived(cmdList[id], v, reply);
    }

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
        b.addString(" - get <param_name>: get the value of the parameter <param_name>");
        b.addString(" - get <param_name> <i>: get the <i>-th element of <param_name>");
        b.addString(" - set <param_name> <param_values>: set <param_name> to <param_values>");
        b.addString(" - set one <param_name> <i> <param_value>: set the <i>-th element of <param_name>");
        b.addString(" - set all <param_name> <param_value>: set all the elements of <param_name> to the same value");
        b.addString("List of the configuration parameters:");
		for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            if(it->second->ioType.value==PARAM_CONFIG)
			    b.addString( (" - "+it->second->name+": "+it->second->description).c_str() );
        b.addString("List of the streaming parameters:");
		for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            if(it->second->ioType.isStreaming())
			    b.addString( (" - "+it->second->name+": "+it->second->description).c_str() );
        b.addString("List of the rpc parameters:");
		for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            if(it->second->ioType.value == PARAM_IN_OUT)
			    b.addString( (" - "+it->second->name+": "+it->second->description).c_str() );
    }
}

//*************************************************************************************************************************
bool ParamHelperServer::sendStreamParams()
{
    Bottle out;
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second->ioType.isStreamingOut())
        {
            Bottle &b = out.addList();
            b.addString(it->second->name);  // add the name of the parameter
            it->second->getAsBottle(b);     // add the value of the parameter
        }
    portOutStream->prepare() = out;
    portOutStream->write();
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::readStreamParams(bool blockingRead)
{
    // TODO: manage variable size input streaming params
    Bottle *in = portInStream->read(blockingRead);
    if(in==NULL) return false;
    Bottle reply;
    bool res;
    for(int i=0; i<in->size(); i++)
    {
        if(!in->get(i).isList())
        {
            logMsg("[ParamHelperServer::readStreamParams] Value ",i," is not a Bottle. Skipping it.", MSG_ERROR);
            continue;
        }
        Bottle *b = in->get(i).asList();
        if(b->size()==0)
        {
            logMsg("[ParamHelperServer::readStreamParams] Value ",i," is an empty Bottle. Skipping it.", MSG_ERROR);
            continue;
        }
        int parId = b->get(0).asInt();
        if(!hasParam(parId))
        {
            logMsg("[ParamHelperServer::readStreamParams] Value ",i," refers to a nonexisting parameter with id ",parId, MSG_ERROR);
            continue;
        }
        res = paramList[parId]->set(b->tail(), &reply);
        if(res==false)
            logMsg("[ParamHelperServer::readStreamParams] Param ",parId,reply.toString().c_str(), MSG_ERROR);
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::registerParamValueChangedCallback(int id, ParamValueObserver *observer)
{
    if(!hasParam(id)) return false;
    paramValueObs[id] = observer;
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::registerParamSizeChangedCallback(int id, ParamSizeObserver *observer)
{
    if(!hasParam(id)) return false;
    paramSizeObs[id] = observer;
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

bool ParamHelperServer::getParam(int id, Bottle &reply)
{
    if(!hasParam(id)){                      reply.addString("Parameter id not found.");         return false; }
    if(!paramList[id]->ioType.canRead()){   reply.addString("This parameter can not be read");  return false; }
    paramList[id]->getAsBottle(reply);
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::getOneParam(int id, const Bottle &v, Bottle &reply)
{
    if(!hasParam(id)){              reply.addString("Error: parameter id not found.");          return false; }
    ParamProxyInterface *pd = paramList[id];
    if(!pd->ioType.canRead()){      reply.addString("Error: this parameter can not be read.");  return false; }
    if(v.size()==0){                reply.addString("Error: index not specified.");             return false; }
    if(!v.get(0).isInt()){          reply.addString("Error: specified index is not an int.");   return false; }
    int i = v.get(0).asInt();
    if(i<0 || i>=pd->size.size){    reply.addString("Error: specified index is out of range."); return false; }
    pd->getAsBottle(reply, i);
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::setParam(int id, const Bottle &v, Bottle &reply, bool init)
{
    // if the parameter doesn't exist or the value to set doesn't satisfy the constraints, then return
    if(!hasParam(id)){              reply.addString(("Id of parameter "+paramList[id]->name+" not found.").c_str()); return false; }
    if(!checkParamConstraints(id, v, reply)){                                                                        return false; }
    if(!init && !paramList[id]->ioType.canWrite()){ reply.addString(("Parameter "+paramList[id]->name+" cannot be written.").c_str());   return false; }
    
    ///< if parameter size is about to change, perform callbacks
    if(paramList[id]->size.freeSize && paramList[id]->size != v.size() && paramSizeObs[id]!=NULL)
    {
        paramSizeObs[id]->parameterSizeChanged(paramList[id], v.size());
    }
    bool res = paramList[id]->set(v, &reply);
    // TODO: deal with free size parameters
    
    if(res)
    {
        reply.addString(("Parameter "+paramList[id]->name+" set to "+v.toString().c_str()).c_str());
        // if an observer is registered, then perform the callback
        if(paramValueObs[id]!=NULL)
            paramValueObs[id]->parameterUpdated(paramList[id]);
        return true;
    }
    
    reply.addString(("Parameter "+paramList[id]->name+" has not been set.").c_str());
    return false;
}

//*************************************************************************************************************************
bool ParamHelperServer::setOneParam(int id, const Bottle &v, Bottle &reply)
{
    // Check a lot of things
    ParamProxyInterface *pd = paramList[id];
    if(!hasParam(id)){              reply.addString(("Error: parameter "+pd->name+" does not exist.").c_str());               return false; }
    if(!pd->ioType.canWrite()){     reply.addString(("Error: parameter "+pd->name+" cannot be written.").c_str());            return false; }
    if(v.size()<=1){ reply.addString("Error: wrong command format (correct format is 'set one <param_name> <index> <value>').");        return false; }
    if(!v.get(0).isInt()){          reply.addString("Error: specified index is not an integer value.");                                 return false; }
    if(v.get(0).asInt()>=pd->size.size){                        reply.addString("Error: specified index is out of range.");             return false; }
    int index = v.get(0).asInt();
    bool res = pd->set(v.tail(), index, &reply);
    
    if(res)
    {
        reply.addString(("Parameter "+pd->name+" has been set.").c_str());
        // if an observer is registered, then perform the callback
        if(paramValueObs[id]!=NULL)
            paramValueObs[id]->parameterUpdated(paramList[id]);
    }
    return res;
}

//*************************************************************************************************************************
bool ParamHelperServer::setAllParam(int id, const Bottle &v, Bottle &reply)
{
    // Check a lot of things
    ParamProxyInterface *pd = paramList[id];
    if(!hasParam(id)){              reply.addString(("Error: parameter "+pd->name+" does not exist.").c_str());     return false; }
    if(!pd->ioType.canWrite()){     reply.addString(("Error: parameter "+pd->name+" cannot be written.").c_str());  return false; }
    if(v.size()==0){                reply.addString("Error: a value to set was not specified.");                    return false; }
    
    bool res = true;
    for(int i=0; i<pd->size.size; i++)
        res = res && pd->set(v, i, &reply);
    
    if(res)
    {
        reply.addString(("Parameter "+pd->name+" has been set.").c_str());
        // if an observer is registered, then perform the callback
        if(paramValueObs[id]!=NULL)
            paramValueObs[id]->parameterUpdated(pd);
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::identifyCommand(const Bottle &cmd, CommandType &cmdType, int &id, Bottle &params)
{
    if(cmd.size()==0) return false;  // check minimum size of command
    
    // identify the type of command
    if(cmd.get(0).asString() == "set")
    {
        if(cmd.size()<3)    // set <param_name> <param_value>
            return false;
        if(cmd.get(1).isString() && cmd.get(1).asString()=="one")
            cmdType=COMMAND_SET_ONE;
        else if(cmd.get(1).isString() && cmd.get(1).asString()=="all")
            cmdType=COMMAND_SET_ALL;
        else
            cmdType=COMMAND_SET;
    }
    else if(cmd.get(0).asString() == "get")
    {
        if(cmd.size()<2)    // get <param_name>
            return false;
        cmdType=COMMAND_GET;
    }
    else
        cmdType=COMMAND_GENERIC;

    string word;
	int wordCounter;
	bool found;

    if(cmdType!=COMMAND_GENERIC)
    {
        // remove first element of command Bottle (i.e. "set" or "get")
        Bottle cmdTail = cmd.tail();    
        // if necessary, remove also 2nd element of command Bottle (i.e. "one" or "all")
        if(cmdType==COMMAND_SET_ALL || cmdType==COMMAND_SET_ONE || cmdType==COMMAND_GET_ONE)
            cmdTail = cmdTail.tail();
        // look for a parameter name
        for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        {
		    stringstream stream(it->second->name);
		    found = true;
            wordCounter = 0;

		    while(stream>>word)
            {
                if (cmdTail.get(wordCounter).asString() != word.c_str()){ found=false; break; }
			    wordCounter++;
		    }
		    if(found)
            {
			    id = it->second->id;
                for(int k=wordCounter; k<cmdTail.size(); k++)
                    params.add(cmdTail.get(k));
                if(cmdType==COMMAND_GET && params.size()>0)
                    cmdType=COMMAND_GET_ONE;
			    return true;
		    }
	    }
        return false;
    }

    // look for a command name
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
    // nothing has been found => return false
	return false;
}
