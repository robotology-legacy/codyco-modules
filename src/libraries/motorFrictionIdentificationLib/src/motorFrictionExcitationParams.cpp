/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <motorFrictionIdentificationLib/motorFrictionExcitationParams.h>

using namespace motorFrictionExcitation;
using namespace yarp::os;

// ************************************************************************************************
FreeMotionExcitation::FreeMotionExcitation()
{
    const ParamProxyInterface *const freeMotionExcitationParamDescr[FREE_MOTION_EXCITATION_PARAM_ID_SIZE] =
    {
    //                          NAME                           ID                          SIZE                 CONSTRAINTS                              I/O ACCESS          DEFAULT VALUE          DESCRIPTION
    new ParamProxyBasic<int>(   "joint id",                    PARAM_ID_JOINT_ID,          PARAM_SIZE_FREE,     ParamBilatBounds<int>(0,30),             PARAM_CONFIG,       0,                     "Id(s) of the joint(s) to excite"),          
    new ParamProxyBasic<double>("initial joint configuration", PARAM_ID_INIT_Q,            PARAM_SIZE_FREE,     ParamBilatBounds<double>(-360,360),      PARAM_CONFIG,       0,                     "Initial configuration of all joints before starting excitation"),
    new ParamProxyBasic<double>("a",                           PARAM_ID_A,                 PARAM_SIZE_FREE,     ParamBilatBounds<double>(0,10),          PARAM_CONFIG,       0,                     "Linear coefficient of PWM sinusoid amplitude"),
    new ParamProxyBasic<double>("a0",                          PARAM_ID_A0,                PARAM_SIZE_FREE,     ParamBilatBounds<double>(0,100),         PARAM_CONFIG,       0,                     "Initial amplitude of PWM sinusoid signal"),
    new ParamProxyBasic<double>("w",                           PARAM_ID_W,                 PARAM_SIZE_FREE,     ParamBilatBounds<double>(0,5),           PARAM_CONFIG,       0,                     "Frequency of PWM sinusoid signal"),
    new ParamProxyBasic<double>("joint limit thresh",          PARAM_ID_JOINT_LIM_THR,     PARAM_SIZE_FREE,     ParamBilatBounds<double>(1,100),         PARAM_CONFIG,       0,                     "Max distance to joint limit before stopping excitation"),
    new ParamProxyBasic<double>("fric param covar thresh",     PARAM_ID_FRIC_PAR_COV_THR,  PARAM_SIZE_FREE,     ParamLowerBound<double>(0),              PARAM_CONFIG,       0,                     "Max variance of identified friction parameters"),
    new ParamProxyBasic<double>("ki",                          PARAM_ID_POS_INT_GAIN,      PARAM_SIZE_FREE,     ParamBilatBounds<double>(-0.1,0.1),      PARAM_CONFIG,       0,                     "Gain of the posiion integral used to correct the pwm offset")
    };
    addParams(freeMotionExcitationParamDescr, FREE_MOTION_EXCITATION_PARAM_ID_SIZE);

    ///< resize arrays
    jointId.resize(1);
    initialJointConfiguration.resize(1);
    a.resize(1);
    a0.resize(1);
    w.resize(1);
    jointLimitThresh.resize(1);
    fricParamCovarThresh.resize(1);
    ki.resize(1);

    ///< link parameters to variables
    paramList[PARAM_ID_JOINT_ID]->linkToVariable(jointId.data());
    paramList[PARAM_ID_INIT_Q]->linkToVariable(initialJointConfiguration.data());
    paramList[PARAM_ID_A]->linkToVariable(a.data());
    paramList[PARAM_ID_A0]->linkToVariable(a0.data());
    paramList[PARAM_ID_W]->linkToVariable(w.data());
    paramList[PARAM_ID_JOINT_LIM_THR]->linkToVariable(jointLimitThresh.data());
    paramList[PARAM_ID_FRIC_PAR_COV_THR]->linkToVariable(fricParamCovarThresh.data());
    paramList[PARAM_ID_POS_INT_GAIN]->linkToVariable(ki.data());
}

//FreeMotionExcitation::FreeMotionExcitation(const FreeMotionExcitation &rhs)
//{
//    paramList = rhs.paramList;
//    jointId = rhs.jointId;
//    initialJointConfiguration = rhs.initialJointConfiguration;
//    a = rhs.a;
//    a0 = rhs.a0;
//    w = rhs.w;
//    jointLimitThresh = rhs.jointLimitThresh;
//    fricParamCovarThresh = rhs.fricParamCovarThresh;
//    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
//        relinkParam(it->first);
//}
    
// ************************************************************************************************
FreeMotionExcitation& FreeMotionExcitation::operator=(const FreeMotionExcitation& rhs)
{
    paramList = rhs.paramList;
    jointId = rhs.jointId;
    initialJointConfiguration = rhs.initialJointConfiguration;
    a = rhs.a;
    a0 = rhs.a0;
    w = rhs.w;
    jointLimitThresh = rhs.jointLimitThresh;
    fricParamCovarThresh = rhs.fricParamCovarThresh;
    ki = rhs.ki;
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        relinkParam(it->first);
    return *this;
}

// ************************************************************************************************
bool FreeMotionExcitation::setFromValue(const Value &v)
{
    //printf("FreeMotionExcitation::setFromValue called with input:\n%s\n", v.toString().c_str());

    ///< v should be a list containing a list for each parameter (i.e. 7 lists).
    ///< Each of these 8 lists should have the parameter's name as first element
    ///< and the parameter's value as tail.
    if(!v.isList())
        return false;
    Bottle *b = v.asList();
    if(b==NULL || b->size()==0)
        return false;

    Bottle reply;
    bool res = true;
    for(int d=0;d<b->size();d++)
    {
        Value &vd = b->get(d);
        if(!vd.isList())
        {
            printf("Error setting FreeMotionExcitation subparam. Value %d is not a list: %s\n", d, vd.toString().c_str());
            res = false;
            continue;
        }
        Bottle *bd = vd.asList();
        if(bd->get(0).isString())   ///< first element is the param name
        {
            Bottle bTail = bd->tail();  ///< the tail can be either a list or a one-element list with the only element being a list itself
            if(bTail.size()>0 && bTail.get(0).isList()) ///< if 1st element is a list then unpack it
                res = res && setSubParam(bd->get(0).asString().c_str(), *bTail.get(0).asList(), &reply);
            else
                res = res && setSubParam(bd->get(0).asString().c_str(), bTail, &reply);
            if(reply.size()!=0)
            {
                printf("Error setting FreeMotionExcitation subparam %s: %s\n", bd->get(0).asString().c_str(), reply.toString().c_str());
                reply.clear();
            }
        }
    }
    return res;
}

// ************************************************************************************************
bool FreeMotionExcitation::setSubParam(const char *key, const Bottle &value, Bottle *reply)
{
    //printf("Going to set FreeMotionExcitation subparam %s to: %s\n", key, value.toString().c_str());
    ///< replace "underscores" with "white spaces"
    string paramName(key);
    replace( paramName.begin(), paramName.end(), '_', ' ');
    
    ///< look for a parameter with name==key and try to set the new value
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
        if(it->second->name.compare(paramName)==0)
        {
            ///< if the new size of the parameter is different THEN resize the associated array
            if(it->second->size.freeSize && it->second->size != value.size())
                resizeParam(it->first, value.size());
            bool res = it->second->set(value, reply);
            
            Bottle b;
            it->second->getAsBottle(b);
            //printf("Subparam %s set to %s\n", key, b.toString().c_str());
            return res;
        }
    
    ///< if no parameter has been found then return false
    if(reply!=NULL)
        reply->addString(("Param name not found: "+paramName).c_str());
    return false;
}

// ************************************************************************************************
Value FreeMotionExcitation::getAsValue()
{
    Value *v = Value::makeList();
    Bottle *b = v->asList();
    for(map<int,ParamProxyInterface*>::iterator it=paramList.begin(); it!=paramList.end(); it++)
    {
        Bottle &b1 = b->addList();
        b1.addString(it->second->name.c_str());
        it->second->getAsBottle(b1);
    }
    return *v;
}

// ************************************************************************************************
void FreeMotionExcitation::resizeParam(int paramId, int newSize)
{
    //printf("Param %s changed size to %d\n", paramList[paramId]->name.c_str(), newSize);
    switch(paramId)
    {
    case PARAM_ID_JOINT_ID:         
        jointId.resize(newSize); break;
    case PARAM_ID_INIT_Q:           
        initialJointConfiguration.resize(newSize); break;
    case PARAM_ID_A:                
        a.resize(newSize); break;
    case PARAM_ID_A0:               
        a0.resize(newSize); break;
    case PARAM_ID_W:                
        w.resize(newSize); break;
    case PARAM_ID_JOINT_LIM_THR:    
        jointLimitThresh.resize(newSize); break;
    case PARAM_ID_FRIC_PAR_COV_THR: 
        fricParamCovarThresh.resize(newSize); break;
    case PARAM_ID_POS_INT_GAIN: 
        ki.resize(newSize); break;
    default:    
        printf("Unexpected param ID in FreeMotionExcitation::resizeParam: %d\n", paramId); 
        return;
    }
    relinkParam(paramId, newSize);
}

// ************************************************************************************************
void FreeMotionExcitation::relinkParam(int paramId, int newSize)
{
    switch(paramId)
    {
    case PARAM_ID_JOINT_ID:         
        return paramList[PARAM_ID_JOINT_ID]->linkToVariable(jointId.data(), newSize);
    case PARAM_ID_INIT_Q:           
        return paramList[PARAM_ID_INIT_Q]->linkToVariable(initialJointConfiguration.data(), newSize);
    case PARAM_ID_A:                
        return paramList[PARAM_ID_A]->linkToVariable(a.data(), newSize);
    case PARAM_ID_A0:               
        return paramList[PARAM_ID_A0]->linkToVariable(a0.data(), newSize);
    case PARAM_ID_W:                
        return paramList[PARAM_ID_W]->linkToVariable(w.data(), newSize);
    case PARAM_ID_JOINT_LIM_THR:    
        return paramList[PARAM_ID_JOINT_LIM_THR]->linkToVariable(jointLimitThresh.data(), newSize);
    case PARAM_ID_FRIC_PAR_COV_THR: 
        return paramList[PARAM_ID_FRIC_PAR_COV_THR]->linkToVariable(fricParamCovarThresh.data(), newSize);
    case PARAM_ID_POS_INT_GAIN: 
        return paramList[PARAM_ID_POS_INT_GAIN]->linkToVariable(ki.data(), newSize);
    default:    
        printf("Unexpected param ID in FreeMotionExcitation::relinkParam: %d\n", paramId); 
    }
}

// ************************************************************************************************
bool FreeMotionExcitation::addParams(const ParamProxyInterface *const *pdList, int size)
{
    for(int i=0;i<size;i++)
        paramList[pdList[i]->id] = pdList[i]->clone();
    return true;
}

// ************************************************************************************************
string FreeMotionExcitation::toString()
{
    return getAsValue().toString().c_str();
}

// ************************************************************************************************
// ***************************************** ContactExcitation ************************************
// ************************************************************************************************
string ContactExcitation::toString() const
{
    return strcat("Joint id (",jointId.transpose(),"); Initial joint configuration (", 
        initialJointConfiguration.transpose(), "); Param covariance threshold (", paramCovarThresh.transpose(),")");
}

// ************************************************************************************************
bool ContactExcitation::set(const Bottle &value, Bottle &reply)
{
    // value should be a Bottle of 3 elements, each being a Bottle itself
    Bottle *subparam;
    bool res = true;
    for(int i=0; i<value.size(); i++)
    {
        if(!value.get(i).isList())
        {
            reply.addString(strcat("Element ",i," was expected to be a list but it is not: ",value.get(i).toString()));
            continue;
        }
        subparam = value.get(i).asList();
        if(!(subparam->size()>0 && subparam->get(0).isString()))
        {
            reply.addString(strcat("First element of Bottle is not the subparameter name: ",subparam->toString()));
            continue;
        }
        res = res && setSubparam(subparam->get(0).asString(), subparam->tail(), reply);
    }
    return res;
}

// ************************************************************************************************
bool ContactExcitation::setSubparam(const string &name, const Bottle &v, Bottle &reply)
{
    Bottle value = v;
    if(value.get(0).isList())   ///< if the value is a list then unpack it
        value = *v.get(0).asList();
    bool res = false;
    //printf("Set subparam %s to %s which has %d elements\n", name.c_str(), value.toString().c_str(), value.size());
    if(name=="joint_id")
    {
        res = true;
        jointId.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isInt())
            {
                res = false;
                reply.addString(strcat("The value ",j," of the parameter ",name," is not of the expected type"));
                continue;
            }
            jointId[j] = value.get(j).asInt();
        }
    }
    else if(name=="initial_joint_configuration")
    {
        res = true;
        initialJointConfiguration.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!(value.get(j).isInt() || value.get(j).isDouble()))
            {
                res = false;
                reply.addString(strcat("The value ",j," of the parameter ",name," is not of the expected type"));
                continue;
            }
            initialJointConfiguration[j] = value.get(j).asDouble();
        }
    }
    else if(name=="param_covar_thresh")
    {
        res = true;
        paramCovarThresh.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strcat("The value ",j," of the parameter ",name," is not of the expected type"));
                continue;
            }
            paramCovarThresh[j] = value.get(j).asDouble();
        }
    }
    return res;
}

// ************************************************************************************************
// ************************************* ContactExcitationList ************************************
// ************************************************************************************************
bool ContactExcitationList::readFromConfigFile(ResourceFinder &rf, Bottle &reply)
{
    Bottle &b = rf.findGroup("contact_excitation");
    if(b.isNull())
    {
        reply.addString("Section contactExcitation not found");
        return false;
    }

    bool res = true;
    resize(b.size()-1);
    for(int i=1; i<b.size(); i++)
    {
        ConstString key = b.get(i).asString();
        Bottle &sb = rf.findGroup(key);     ///< find the i-th section "contact_excitation"
        if(sb.isNull())
        {
            reply.addString(strcat("Could not find section ",key));
            continue;
        }
        if(!sb.get(0).isString())   ///< first element is the section name
        {
            reply.addString(strcat("Error parsing contact_excitation param, section name not found: ",sb.get(0).toString()));
            continue;
        }
        if(sb.size()<2)
        {
            reply.addString("Section "+sb.get(0).asString()+" does not have a content");
            continue;
        }
        ///< the tail can be either a list or a one-element list with the only element being a list itself
        this->operator[](i-1).set(sb.tail(), reply);
    }
    return res;
}

// ************************************************************************************************
string ContactExcitationList::toString() const
{
    string res;
    for(unsigned int i=0; i<size(); i++)
        res += this->operator[](i).toString()+"\n";
    return res;
}