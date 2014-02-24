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
// ************************************************************************************************


//FreeMotionExcitation();
bool FreeMotionExcitation::set(const yarp::os::Bottle &value, yarp::os::Bottle &reply)
{
    // value should be a Bottle of 3 elements, each being a Bottle itself
    Bottle *subparam;
    bool res = true;
    for(int i=0; i<value.size(); i++)
    {
        if(!value.get(i).isList())
        {
            reply.addString(strapp("Element ",i," was expected to be a list but it is not: ",value.get(i).toString()).c_str());
            continue;
        }
        subparam = value.get(i).asList();
        if(!(subparam->size()>0 && subparam->get(0).isString()))
        {
            reply.addString(strapp("First element of Bottle is not the subparameter name: ",subparam->toString()).c_str());
            continue;
        }
        bool currentResult = setSubparam(subparam->get(0).asString().c_str(), subparam->tail(), reply);
        res = res && currentResult;
    }
    return res;
}
bool FreeMotionExcitation::setSubparam(const std::string &name, const yarp::os::Bottle &v, yarp::os::Bottle &reply)
{
    Bottle value = v;
    if(value.get(0).isList())   ///< if the value is a list then unpack it
        value = *v.get(0).asList();
    bool res = false;
    
    if(name=="joint_id")
    {
        res = true;
        jointId.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isInt())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
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
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            initialJointConfiguration[j] = value.get(j).asDouble();
        }
    }
    else if(name=="fric_param_covar_thresh")
    {
        res = true;
        fricParamCovarThresh.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            fricParamCovarThresh[j] = value.get(j).asDouble();
        }
    }
    else if(name=="a")
    {
        res = true;
        a.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            a[j] = value.get(j).asDouble();
        }
    }
    else if(name=="a0")
    {
        res = true;
        a0.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            a0[j] = value.get(j).asDouble();
        }
    }
    else if(name=="w")
    {
        res = true;
        w.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            w[j] = value.get(j).asDouble();
        }
    }
    else if(name=="joint_limit_thresh")
    {
        res = true;
        jointLimitThresh.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            jointLimitThresh[j] = value.get(j).asDouble();
        }
    }
    else if(name=="ki")
    {
        res = true;
        ki.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isDouble())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
                continue;
            }
            ki[j] = value.get(j).asDouble();
        }
    }
    
    return res;
}
std::string FreeMotionExcitation::toString() const
{
    return strapp("Joint id (",jointId.transpose(),"); Initial joint configuration (",
                  initialJointConfiguration.transpose(), "); Param covariance threshold (", fricParamCovarThresh.transpose()) + strapp("); a(", a.transpose(), "); a0(", a0.transpose()) + strapp("); w(", w.transpose(), "); ki:(", ki.transpose(), ")");
}

// ************************************************************************************************
// ********************************** FreeMotionExcitationList ************************************
// ************************************************************************************************
bool FreeMotionExcitationList::readFromConfigFile(ResourceFinder &rf, Bottle &reply)
{
    Bottle &b = rf.findGroup("free_motion_excitation");
    if(b.isNull())
    {
        reply.addString("Section freeMotionExcitation not found");
        return false;
    }
    bool res = true;
    resize(b.size()-1);
    for(int i=1; i<b.size(); i++)
    {
        ConstString key = b.get(i).asString();
        Bottle &sb = rf.findGroup(key);     ///< find the i-th section "free_motion_excitation"
        if(sb.isNull())
        {
            reply.addString(strapp("Could not find section ",key).c_str());
            continue;
        }
        if(!sb.get(0).isString())   ///< first element is the section name
        {
            reply.addString(strapp("Error parsing free_motion_excitation param, section name not found: ",sb.get(0).toString()).c_str());
            continue;
        }
        if(sb.size()<2)
        {
            std::string section_name = sb.get(0).asString().c_str();
            reply.addString(("Section "+section_name+" does not have a content").c_str());
            continue;
        }
        ///< the tail can be either a list or a one-element list with the only element being a list itself
        this->operator[](i-1).set(sb.tail(), reply);
    }
    return res;
}

// ************************************************************************************************
string FreeMotionExcitationList::toString() const
{
    string res;
    for(unsigned int i=0; i<size(); i++)
        res += this->operator[](i).toString()+"\n";
    return res;
}

// ************************************************************************************************
// ***************************************** ContactExcitation ************************************
// ************************************************************************************************
string ContactExcitation::toString() const
{
    return strapp("Joint id (",jointId.transpose(),"); Initial joint configuration (", 
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
            reply.addString(strapp("Element ",i," was expected to be a list but it is not: ",value.get(i).toString()).c_str());
            continue;
        }
        subparam = value.get(i).asList();
        if(!(subparam->size()>0 && subparam->get(0).isString()))
        {
            reply.addString(strapp("First element of Bottle is not the subparameter name: ",subparam->toString()).c_str());
            continue;
        }
        res = res && setSubparam(subparam->get(0).asString().c_str(), subparam->tail(), reply);
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
    if(name=="joint_id")
    {
        res = true;
        jointId.resize(value.size());
        for(int j=0; j<value.size(); j++)
        {
            if(!value.get(j).isInt())
            {
                res = false;
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
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
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
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
                reply.addString(strapp("The value ",j," of the parameter ",name," is not of the expected type").c_str());
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
            reply.addString(strapp("Could not find section ",key).c_str());
            continue;
        }
        if(!sb.get(0).isString())   ///< first element is the section name
        {
            reply.addString(strapp("Error parsing contact_excitation param, section name not found: ",sb.get(0).toString()).c_str());
            continue;
        }
        if(sb.size()<2)
        {
            std::string section_name = sb.get(0).asString().c_str();
            reply.addString(("Section "+section_name+" does not have a content").c_str());
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
