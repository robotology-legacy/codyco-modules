/*
 * Copyright (C) 2013 CODYCO
 * Author: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 */

#include <paramHelp/paramHelp.h>

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

bool ParamIOType::canRead(){ return value==PARAM_OUTPUT || value==PARAM_OUT_STREAM || value==PARAM_IN_OUT || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::canWrite(){ return value==PARAM_INPUT || value==PARAM_IN_STREAM || value==PARAM_IN_OUT || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::isStreaming(){ return value==PARAM_OUT_STREAM || value==PARAM_IN_STREAM || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::isStreamingOut(){ return value==PARAM_OUT_STREAM || value==PARAM_IN_OUT_STREAM; }
bool ParamIOType::isStreamingIn(){ return value==PARAM_IN_STREAM || value==PARAM_IN_OUT_STREAM; }

//*************************************************************************************************************************
ParamHelper::ParamHelper(const ParamDescription *pdList, int pdListSize, const CommandDescription *cdList, int cdListSize)
{
    if(pdList!=NULL) addParams(pdList, pdListSize);
    if(cdList!=NULL) addCommands(cdList, cdListSize);
}
//*************************************************************************************************************************
ParamHelper::~ParamHelper()
{
    // delete all allocated memory
    close();
    for(map<int,void*>::iterator it=paramValues.begin(); it!=paramValues.end(); it++)
        if(it->second != NULL)
            delete[] it->second;
}
//*************************************************************************************************************************
bool ParamHelper::init(string moduleName)
{
    string portInStreamName     = "/" + moduleName + PORT_IN_STREAM_SUFFIX;
    string portOutStreamName    = "/" + moduleName + PORT_OUT_STREAM_SUFFIX;
    portInStream = new BufferedPort<Bottle>();
    portOutStream = new BufferedPort<Bottle>();
    bool res = portInStream->open(portInStreamName.c_str())
            && portOutStream->open(portOutStreamName.c_str());
    return res;
}
//*************************************************************************************************************************
bool ParamHelper::close()
{
    if(portInStream){  portInStream->interrupt();  portInStream->close();  delete portInStream;  portInStream=0; }
    if(portOutStream){ portOutStream->interrupt(); portOutStream->close(); delete portOutStream; portOutStream=0; }
    return true;
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
bool ParamHelper::addCommands(const CommandDescription *cdList, int size)
{
    bool res = true;
    for(int i=0; i<size; i++)
        res = res && addCommand(cdList[i]);
    return res;
}

//*************************************************************************************************************************
bool ParamHelper::respond(const Bottle& cmd, Bottle& reply)
{
    // if cmd contains "help", then write the help message in reply
    if(cmd.size()>0 && cmd.get(0).asString()=="help")
    {
        reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
        if(paramList.size()>0)
        {
            reply.addString("To get a parameter type 'get <param_name>'");
            reply.addString("To set a parameter type 'set <param_name> <param_value>'");
            reply.addString("List of the parameter names and descriptions:");
		    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
			    reply.addString( ("- "+it->second.name+": "+it->second.description).c_str() );
        }
        if(cmdList.size()>0)
        {
            reply.addString("List of the module commands and descriptions:");
            for(map<int,CommandDescription>::iterator it=cmdList.begin(); it!=cmdList.end(); it++)
			    reply.addString( ("- "+it->second.name+": "+it->second.description).c_str() );
        }
        return true;
    }

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
        cmdObs[id]->commandReceived(cmdList[id], v);

    return true;
}

//*************************************************************************************************************************
bool ParamHelper::sendStreamParams()
{
    // assume that who is calling this has alread taken the mutex
    Bottle out;
    for(map<int,ParamDescription>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second.ioType.isStreamingOut())
        {
            ParamDescription *pd = &(it->second);
            for(int i=0; i<pd->size.size; i++)
                switch(pd->type)
                {
                    case PARAM_DATA_FLOAT:  out.addDouble(*paramValue<double>(pd->id, i)); break;
                    case PARAM_DATA_INT:    out.addInt(*paramValue<int>(pd->id, i)); break;
                    case PARAM_DATA_BOOL:   out.addInt(*paramValue<bool>(pd->id, i)); break;  // the class Bottle has no "addBool" method
                    case PARAM_DATA_STRING: out.addString(paramValue<string>(pd->id, i)->c_str()); break;
                }
        }
    portOutStream->prepare() = out;
    portOutStream->write();
    return true;
}

//*************************************************************************************************************************
bool ParamHelper::readStreamParams(bool blockingRead)
{
    return true;
}

//*************************************************************************************************************************
bool ParamHelper::linkParam(int id, void *v)
{
    if(!hasParam(id) || v==0) return false;
    assert(paramValues[id]!=NULL);
    delete[] paramValues[id];   // delete previously allocated memory
    paramValues[id] = v;
    setParam(id, paramList[id].defaultValue);   // set the variable pointed by v to the default value (if any)
    return true;
}

//*************************************************************************************************************************
bool ParamHelper::registerCallback(int id, ParamObserver *observer)
{
    if(!hasParam(id)) return false;
    paramObs[id] = observer;
    return true;
}

//*************************************************************************************************************************
bool ParamHelper::registerCallback(int id, CommandObserver *observer)
{
    if(!hasCommand(id)) return false;
    cmdObs[id] = observer;
    return true;
}

//*************************************************************************************************************************
//*************************************************************************************************************************
//*********************************************  PRIVATE METHODS  *********************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

bool ParamHelper::getParam(int id, Bottle &v)
{
    if(!hasParam(id)){                      v.addString("Parameter id not found.");         return false; }
    if(!paramList[id].ioType.canRead()){    v.addString("This parameter can not be read");  return false; }

    bool res = true;
    mutex.wait();   // take the mutex before reading the parameter value
    {
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
    }
    mutex.post();
    return res;
}

//*************************************************************************************************************************
bool ParamHelper::setParam(int id, const Bottle &v, Bottle &reply)
{
    // if the parameter doesn't exist or the value to set doesn't satisfy the constraints, then return
    if(!hasParam(id)){                      reply.addString(("Parameter "+paramList[id].name+" cannot be written.").c_str());   return false; }
    if(!checkParamConstraints(id, v, reply)){                                                                                   return false; }
    if(!paramList[id].ioType.canWrite()){   reply.addString(("Parameter "+paramList[id].name+" cannot be written.").c_str());   return false; }
    
    bool res = true;
    mutex.wait();   // take the mutex before modifying the parameter value
    {
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
    }
    mutex.post();

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
bool ParamHelper::addParam(const ParamDescription &pd)
{
    if(hasParam(pd.id)) return false;   // there exists a parameter with the same id

    // create the parameter
    paramList[pd.id] = pd;
    bool res = true;
    switch(pd.type)
    {
        case PARAM_DATA_FLOAT:  paramValues[pd.id] = new double[pd.size.size];    break;
        case PARAM_DATA_INT:    paramValues[pd.id] = new int[pd.size.size];       break;
        case PARAM_DATA_BOOL:   paramValues[pd.id] = new bool[pd.size.size];      break;
        case PARAM_DATA_STRING: paramValues[pd.id] = new string[pd.size.size];    break;
        default: res = false;
    }
    
    // initialize parameter
    if(res && pd.defaultValue!=0) 
        setParam(pd.id, pd.defaultValue);
    
    return res;
}

//*************************************************************************************************************************
bool ParamHelper::addCommand(const CommandDescription &cd)
{
    if(hasCommand(cd.id)) return false;   // there exists a command with the same id
    cmdList[cd.id] = cd;
    return true;
}

//*************************************************************************************************************************
bool ParamHelper::identifyCommand(const Bottle &cmd, bool &isSetCmd, bool &isGetCmd, int &id, Bottle &params)
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

//*************************************************************************************************************************
bool ParamHelper::checkParamConstraints(int id, const void *v)
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
bool ParamHelper::setParam(int paramId, const void *v)
{
    // if the value to set doesn't satisfy the constraints, then return
    if(!hasParam(paramId) || !checkParamConstraints(paramId, v)) return false; 
    
    bool res = true;
    mutex.wait();   // take the mutex before modifying the parameter value
    {
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
    }
    mutex.post();
    return res;
}
