/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelpServer.h>

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
void ParamHelperServer::initializeParams(ResourceFinder &rf, Bottle &reply)
{
    Value *v;
    Bottle temp;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
    {
        string paramName = it->second.name;
        replace( paramName.begin(), paramName.end(), ' ', '_');
        if (rf.check(paramName.c_str(), v))
        {
            if(v->isList())
                temp = *(v->asList());  // if v is a Bottle => cast it into a Bottle
            else
            {
                temp.clear();           // otherwise create a 1-element Bottle
                temp.add(v);
            }
            printf("Setting parameter %s to %s\n", paramName.c_str(), temp.toString().c_str());
            setParam(it->second.id, temp, reply, true);
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
		for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            if(it->second.ioType.value==PARAM_CONFIG)
			    b.addString( (" - "+it->second.name+": "+it->second.description).c_str() );
        b.addString("List of the streaming parameters:");
		for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            if(it->second.ioType.isStreaming())
			    b.addString( (" - "+it->second.name+": "+it->second.description).c_str() );
        b.addString("List of the rpc parameters:");
		for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
            if(it->second.ioType.value == PARAM_IN_OUT)
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
            if(paramValues[pd->id]==NULL){  logMsg("Parameter "+pd->name+" has no associated variable.", MSG_ERROR);        return false; }
            if(j >= in->size()){            logMsg("readStreamParams, unexpected bottle size.", MSG_ERROR);                 return false; }
            Bottle *b = in->get(j).asList();
            j++;
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

bool ParamHelperServer::getParam(int id, Bottle &reply)
{
    if(!hasParam(id)){                      reply.addString("Parameter id not found.");                     return false; }
    if(paramValues[id]==NULL){              reply.addString("This parameter has no associated variable.");  return false; }
    if(!paramList[id].ioType.canRead()){    reply.addString("This parameter can not be read");              return false; }

    bool res = true;
    ParamDescription *pd = &paramList[id];
    for(int i=0; i<pd->size.size; i++)
    {
        switch(pd->type)
        {
            case PARAM_DATA_FLOAT:  reply.addDouble(*paramValue<double>(pd->id, i)); break;
            case PARAM_DATA_INT:    reply.addInt(*paramValue<int>(pd->id, i)); break;
            case PARAM_DATA_BOOL:   reply.addInt(*paramValue<bool>(pd->id, i)); break;  // the class Bottle has no "addBool" method
            case PARAM_DATA_STRING: reply.addString(paramValue<string>(pd->id, i)->c_str()); break;
            default: res = false;
        }
    }
    return res;
}

//*************************************************************************************************************************
bool ParamHelperServer::getOneParam(int id, const Bottle &v, Bottle &reply)
{
    if(!hasParam(id)){              reply.addString("Error: parameter id not found.");                      return false; }
    if(paramValues[id]==NULL){      reply.addString("Error: this parameter has no associated variable.");   return false; }
    ParamDescription *pd = &paramList[id];
    if(!pd->ioType.canRead()){      reply.addString("Error: this parameter can not be read.");              return false; }
    if(v.size()==0){                reply.addString("Error: index not specified.");                         return false; }
    if(!v.get(0).isInt()){          reply.addString("Error: specified index is not an int.");               return false; }
    int i = v.get(0).asInt();
    if(i<0 || i>=pd->size.size){    reply.addString("Error: specified index is out of range.");             return false; }

    switch(pd->type)
    {
        case PARAM_DATA_FLOAT:  reply.addDouble(*paramValue<double>(pd->id, i)); break;
        case PARAM_DATA_INT:    reply.addInt(*paramValue<int>(pd->id, i)); break;
        case PARAM_DATA_BOOL:   reply.addInt(*paramValue<bool>(pd->id, i)); break;  // the class Bottle has no "addBool" method
        case PARAM_DATA_STRING: reply.addString(paramValue<string>(pd->id, i)->c_str()); break;
    }
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::setParam(int id, const Bottle &v, Bottle &reply, bool init)
{
    // if the parameter doesn't exist or the value to set doesn't satisfy the constraints, then return
    if(!hasParam(id)){              reply.addString(("Parameter "+paramList[id].name+" cannot be written.").c_str());           return false; }
    if(paramValues[id]==NULL){      reply.addString(("Parameter "+paramList[id].name+" has no associated variable.").c_str());  return false; }
    if(!checkParamConstraints(id, v, reply)){                                                                                   return false; }
    if(!init && !paramList[id].ioType.canWrite()){ reply.addString(("Parameter "+paramList[id].name+" cannot be written.").c_str());   return false; }
    
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
bool ParamHelperServer::setOneParam(int id, const Bottle &v, Bottle &reply)
{
    // Check a lot of things
    ParamDescription *pd = &paramList[id];
    if(!hasParam(id)){              reply.addString(("Error: parameter "+paramList[id].name+" does not exist.").c_str());               return false; }
    if(paramValues[id]==NULL){      reply.addString(("Error: parameter "+paramList[id].name+" has no associated variable.").c_str());   return false; }
    if(!pd->ioType.canWrite()){     reply.addString(("Error: parameter "+paramList[id].name+" cannot be written.").c_str());            return false; }
    if(v.size()<=1){ reply.addString("Error: wrong command format (correct format is 'set one <param_name> <index> <value>').");        return false; }
    if(!v.get(0).isInt()){          reply.addString("Error: specified index is not an integer value.");                                 return false; }
    if(v.get(0).asInt()>=pd->size.size){                        reply.addString("Error: specified index is out of range.");             return false; }
    if(pd->type==PARAM_DATA_STRING && !v.get(1).isString()){    reply.addString("Error: specified value is not a string.");             return false; }
    else if(pd->type==PARAM_DATA_BOOL && !v.get(1).isBool()){   reply.addString("Error: specified value is not a bool.");               return false; }
    if(pd->type==PARAM_DATA_FLOAT || pd->type==PARAM_DATA_INT)
        if(!(v.get(1).isInt() || v.get(1).isDouble()) || !pd->bounds.checkBounds(v.get(1).asDouble()))
        { 
            reply.addString(("Value out of the allowed range: "+paramList[id].bounds.toString()).c_str()); 
            return false; 
        }
    
    
    int index = v.get(0).asInt();
    switch(pd->type)
    {
        case PARAM_DATA_FLOAT:  *paramValue<double>(pd->id, index) = v.get(1).asDouble();           break;
        case PARAM_DATA_INT:    *paramValue<int>(pd->id, index)    = v.get(1).asInt();              break;
        case PARAM_DATA_BOOL:   *paramValue<bool>(pd->id, index)   = v.get(1).asBool();             break;
        case PARAM_DATA_STRING: *paramValue<string>(pd->id, index) = v.get(1).asString().c_str();   break;
    }
    
    reply.addString(("Parameter "+paramList[id].name+" has been set.").c_str());
    // if an observer is registered, then perform the callback
    if(paramObs[id]!=NULL)
        paramObs[id]->parameterUpdated(paramList[id]);
    return true;
}

//*************************************************************************************************************************
bool ParamHelperServer::setAllParam(int id, const Bottle &v, Bottle &reply)
{
    // Check a lot of things
    ParamDescription *pd = &paramList[id];
    if(!hasParam(id)){              reply.addString(("Error: parameter "+paramList[id].name+" does not exist.").c_str());               return false; }
    if(paramValues[id]==NULL){      reply.addString(("Error: parameter "+paramList[id].name+" has no associated variable.").c_str());   return false; }
    if(!pd->ioType.canWrite()){     reply.addString(("Error: parameter "+paramList[id].name+" cannot be written.").c_str());            return false; }
    if(v.size()==0){                reply.addString("Error: a value to set was not specified.");                                        return false; }
    if(pd->type==PARAM_DATA_STRING && !v.get(0).isString()){    reply.addString("Error: specified value is not a string.");             return false; }
    else if(pd->type==PARAM_DATA_BOOL && !v.get(0).isBool()){   reply.addString("Error: specified value is not a bool.");               return false; }
    if(pd->type==PARAM_DATA_FLOAT || pd->type==PARAM_DATA_INT)
        if(!(v.get(0).isInt() || v.get(0).isDouble()) || !pd->bounds.checkBounds(v.get(0).asDouble()))
        { 
            reply.addString(("Value out of the allowed range: "+paramList[id].bounds.toString()).c_str()); 
            return false; 
        }
    
    
    Value &val = v.get(0);
    for(int i=0; i<pd->size.size; i++)
        switch(pd->type)
        {
            case PARAM_DATA_FLOAT:  *paramValue<double>(pd->id, i) = val.asDouble();           break;
            case PARAM_DATA_INT:    *paramValue<int>(pd->id, i)    = val.asInt();              break;
            case PARAM_DATA_BOOL:   *paramValue<bool>(pd->id, i)   = val.asBool();             break;
            case PARAM_DATA_STRING: *paramValue<string>(pd->id, i) = val.asString().c_str();   break;
        }
    
    reply.addString(("Parameter "+paramList[id].name+" has been set.").c_str());
    // if an observer is registered, then perform the callback
    if(paramObs[id]!=NULL)
        paramObs[id]->parameterUpdated(paramList[id]);
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
