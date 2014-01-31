/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Andrea Del Prete
 * email: andrea.delprete@iit.it
 */

#ifndef __PARAMPROXYBASIC_H__
#define __PARAMPROXYBASIC_H__

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <paramHelp/paramProxyInterface.h>
#include <paramHelp/paramHelpUtil.h>


namespace paramHelp
{

/**
 * Abstract representation of a constraint on a parameter.
 */
template<class T>
class ParamConstraint
{
public:
    /** Return true iff the specified value satisfies the constraint. */
    virtual bool checkConstraint(const T v) const { return true; }
};

/************************************************************************************************//**
// Lower bound on a parameter value
// *************************************************************************************************/
template<class T>
class ParamLowerBound : public ParamConstraint<T>
{
    T lowerBound;        ///< value of the bound
public:
    ParamLowerBound(const T _lowBound): lowerBound(_lowBound) {}
    bool checkConstraint(const T v) const { return (v)>lowerBound; }
};

/************************************************************************************************//**
// Upper bound on a parameter value
// *************************************************************************************************/
template<class T>
class ParamUpperBound : public ParamConstraint<T>
{
    T upperBound;        ///< value of the bound
public:
    ParamUpperBound(const T _upBound): upperBound(_upBound) {}
    bool checkConstraint(const T v) const { return (v)<upperBound; }
};

/************************************************************************************************//**
// Double bound on a parameter value
// *************************************************************************************************/
template<class T>
class ParamBilatBounds : public ParamConstraint<T>
{
    T       lowerBound, upperBound;
public:
    ParamBilatBounds(const T low, const T up): lowerBound(low), upperBound(up) {}
    
    bool checkConstraint(const T v) const { return (v)<upperBound && (v)>lowerBound; }
    
    std::string toString()
    { return std::string("[")+paramHelp::toString(lowerBound)+", "+paramHelp::toString(upperBound)+"]"; }
};


/** 
 * Template class for parameters of basic types such as int, double or string.
 * This template can be used with any type for which the template functions castFromValue 
 * and castToValue work correctly.
 */
template<class T>
class ParamProxyBasic: public ParamProxyInterface
{
    T                   *value;                     ///< pointer to the variable containing the value of this parameter
    bool                linkedToExternalVariable;   ///< true if the parameter has been linked to an external variable
    const T             *defaultValue;              ///< pointer to the (array of( default value of this parameter
    ParamConstraint<T>  constraints;                ///< constraints on the values that the parameter can take

    void init(const T *_defaultValue)
    {
        linkedToExternalVariable = false;
        value = new T[size];                ///< allocate memory
        defaultValue = _defaultValue;
        if(defaultValue==0)
            return;
        for(int i=0;i<size;i++)
            value[i] = defaultValue[i];     ///< copy default value
    }

public:
    
    /** Constructor of an empty parameter. */
    ParamProxyBasic(): ParamProxyInterface(), linkedToExternalVariable(false){}
    
    /** Constructor of a simple parameter proxy (e.g. int, float or string). 
     * @param _name Name of the parameter.
     * @param _id Unique id associated to the parameter.
     * @param _constraints Constraints on the values that the parameter can take.
     * @param _ioType Specifies whether the parameter can be read and/or written and if it is of streaming type.
     * @param _defaultValue Pointer to the array of default values for this parameter.
     * @param _descr Description of the parameter (used for giving instructions to users). */
    ParamProxyBasic(const std::string &_name, int _id, ParamSize _size, const ParamConstraint<T> &_constraints, 
                    ParamIOType _ioType, const T *_defaultValue, const std::string &_descr="")
        :ParamProxyInterface(_name, _id, _size, _ioType, _descr), constraints(_constraints)
    {
        init(_defaultValue);
    }

    /** Constructor of a simple parameter proxy (e.g. int, float or string). 
     * @param _name Name of the parameter.
     * @param _id Unique id associated to the parameter.
     * @param _ioType Specifies whether the parameter can be read and/or written and if it is of streaming type.
     * @param _defaultValue Pointer to the array of default values for this parameter.
     * @param _descr Description of the parameter (used for giving instructions to users). */
    ParamProxyBasic(const std::string &_name, int _id, ParamSize _size, ParamIOType _ioType, 
                    const T *_defaultValue, const std::string &_descr="")
        :ParamProxyInterface(_name, _id, _size, _ioType, _descr)
    {
        init(_defaultValue);
    }

    ~ParamProxyBasic()
    {
        ///< if the parameter is not linked to an external variable
        ///< then delete the previously allocated memory used to store the value of the parameter
        if(!linkedToExternalVariable)
        {
            delete[] value;
        }
    }

    virtual ParamProxyInterface* clone() const
    {
        ParamProxyInterface* res = new ParamProxyBasic<T>(name, id, size, constraints, ioType, defaultValue, description);
        return res;
    }

    /** Link the parameter to the variable pointed by var. 
     * Once a parameter is linked to an external variable, the proxy does not manage the parameter's
     * memory any longer. If the parameter's size changes, whoever has control over the variable
     * linked to the parameter must resize the variable (e.g. using the paramSizeChangedCallback)
     * and, if necessary, re-link the parameter to the new variable memory location.
     * @param var Pointer to the variable to link to this parameter.
     * @param newSize New size of the parameter (and associated variable).
     * @note The current value of the parameter is copied inside var. */
    virtual void linkToVariable(void *var, int newSize=-1)
    {
        if(newSize<0)
        {
            T* currentValue = value;        ///< store pointer to old variable
            value = (T*) var;               ///< replace pointer with new variable's address
            
            if(!linkedToExternalVariable)   ///< delete memory iff memory is not externally managed
            {
                for(int i=0;i<size;i++)
                    value[i] = currentValue[i]; ///< copy current value inside new variable
                delete[] currentValue;          ///< delete old variable
            }
        }
        else    ///< if the parameter changed size do not copy old value in new variable
        {
            if(!linkedToExternalVariable)   ///< delete memory iff memory is not externally managed
                delete[] value;
            value = (T*) var;
            size = newSize;
        }
        linkedToExternalVariable = true;
    }

    /** Get the current value of the parameter in Bottle format. 
     * @param index Index of the element of the parameter to read, if index is out of range it reads all the elements. */
    virtual void getAsBottle(yarp::os::Bottle &b, int index=-1) const
    {
        if(index>=0 && index<size)
            return b.add(castToValue(value[index]));
        for(int i=0;i<size;i++)
            b.add(castToValue(value[i]));
    }

    /** Set the parameter to the specified value. Return true if the operation succeeded. */
    virtual bool set(const yarp::os::Bottle &nv, yarp::os::Bottle *reply=0)
    {
        ///< check the Bottle is not empty
        if(nv.size()==0)
        {
            if(reply!=NULL)
                reply->addString(("The Bottle with the new value for the parameter "+name+" is empty.").c_str());
            return false;
        }
        
        ///< if nv is a one-element list THEN unpack it
        const yarp::os::Bottle *newValue;
        if(nv.get(0).isList())
        {
            //printf("Parameter %s, 1st element of this Bottle should be a Bottle: %s\n", name.c_str(), nv.toString().c_str());
            newValue = nv.get(0).asList();
        }
        else
            newValue = &nv;
        
        ///< check the size
        if(!size.freeSize && newValue->size()!=size)
        {
            if(reply!=NULL) {
                std::string reply_str = strapp("Wrong size of parameter ",name," (expected ",size,", found ",newValue->size());
                reply->addString(reply_str.c_str());
            }
            return false;
        }

        ///< check the constraints
        if(!checkConstraints(*newValue, reply))
            return false;

        ///< if the parameter is not linked to any external variable and if size changed then resize variable
        if(!linkedToExternalVariable && size != newValue->size())    
        {
            delete[] value;                 ///< deallocate old memory
            size.size = newValue->size();   ///< update variable size
            value = new T[size];            ///< allocate new memory
        }

        ///< set the new value
        for(int i=0;i<size;i++)
            value[i] = castFromValue<T>(newValue->get(i));

        return true;
    }

    /** Set one element of the parameter to the specified value. Return true if the operation succeeded. */
    virtual bool set(const yarp::os::Bottle &newValue, int index, yarp::os::Bottle *reply=0) 
    {
        if(index<0 || index>=size)
        {
            if(reply!=NULL) {
                std::string reply_str = strapp("Index out of bound. Index=",index,", parameter size=",size);
                reply->addString(reply_str.c_str());
            }
            return false;
        }
        if(!checkConstraints(newValue, reply))
            return false;
        value[index] = castFromValue<T>(newValue.get(0));
        return true;
    }

    /** Return true if the specified value satisfies this parameter's constraints. 
     * Note that the number of values in the bottle does not have to match the number of elements of the parameter.
     * @param newValue Bottle containing the value(s) to check. */
    virtual bool checkConstraints(const yarp::os::Bottle &newValue, yarp::os::Bottle *reply=0) const
    {
        T vi;
        for(int i=0;i<newValue.size();i++)
        {
            vi = castFromValue<T>(newValue.get(i));
            if(!constraints.checkConstraint(vi))
            {
                if(reply!=NULL)
                    reply->addString((std::string("Value ")+castToValue(vi).toString().c_str()+" does not satisfy constraints.").c_str());
                return false;
            }
        }
        return true;
    }

    virtual std::string getAsString() const
    {
        if(size==0)
            return std::string("");
        std::stringstream ss;
        ss<< value[0];
        for(int i=1; i<size; i++)
            ss<<" "<<value[i];
        return ss.str();
    }

    ///** Return true if the specified value satisfies this parameter's constraints. 
    // * @param newValue Array of values to check. */
    //virtual bool checkConstraints(const T *newValue, yarp::os::Bottle *reply=0) const
    //{
    //    T vi;
    //    for(int i=0;i<size;i++)
    //    {
    //        vi = newValue[i];
    //        if(!constraints.checkConstraint(vi))
    //        {
    //            if(reply!=NULL)
    //                reply->addString("Value "+toString(vi)+" does not satisfy constraints.");
    //            return false;
    //        }
    //    }
    //    return true;
    //}

    /** Set the parameter to the specified value. Return true if the operation succeeded. */
    //virtual bool set(const void *newValue, yarp::os::Bottle *reply=0)
    //{
    //    T *newValueTyped = (T*)newValue;
    //    if(!checkConstraints(newValueTyped, reply))
    //        return false;

    //    for(int i=0;i<size;i++)
    //        value[i] = newValueTyped[i];
    //    return true;
    //}
};


/** 
 * Template class for parameters of class types, i.e. user defined classes.
 * This template can be used with any class that defines the methods getAsValue
 * and setFromValue.
 */
template<class T>
class ParamProxyClass: public ParamProxyInterface
{
    T                   *value;          ///< pointer to the variable containing the value of this parameter
    bool                linkedToExternalVariable;   ///< true if the parameter has been linked to an external variable
    const T             *defaultValue;   ///< pointer to the (array of( default value of this parameter
    ParamConstraint<T>  constraints;     ///< constraints on the values that the parameter can take

    void init(const T *_defaultValue)
    {
        linkedToExternalVariable = false;
        value = new T[size];            ///< initialize memory
        defaultValue = _defaultValue;
        if(_defaultValue==0)
            return;
        for(int i=0;i<size;i++)
            value[i] = defaultValue[i]; ///< copy default value
    }

public:
    
    /** Constructor of an empty parameter. */
    ParamProxyClass(): ParamProxyInterface(), linkedToExternalVariable(false){}
    
    /** Constructor of a simple parameter proxy (e.g. int, float or string). 
     * @param _name Name of the parameter.
     * @param _id Unique id associated to the parameter.
     * @param _constraints Constraints on the values that the parameter can take.
     * @param _ioType Specifies whether the parameter can be read and/or written and if it is of streaming type.
     * @param _defaultValue Pointer to the array of default values for this parameter.
     * @param _descr Description of the parameter (used for giving instructions to users). */
    ParamProxyClass(const std::string &_name, int _id, ParamSize _size, const ParamConstraint<T> &_constraints, 
                    ParamIOType _ioType, const T *_defaultValue, const std::string &_descr="")
        :ParamProxyInterface(_name, _id, _size, _ioType, _descr), constraints(_constraints)
    {
        init(_defaultValue);
    }

    /** Constructor of a simple parameter proxy (e.g. int, float or string). 
     * @param _name Name of the parameter.
     * @param _id Unique id associated to the parameter.
     * @param _ioType Specifies whether the parameter can be read and/or written and if it is of streaming type.
     * @param _defaultValue Pointer to the array of default values for this parameter.
     * @param _descr Description of the parameter (used for giving instructions to users). */
    ParamProxyClass(const std::string &_name, int _id, ParamSize _size, ParamIOType _ioType, 
                    const T *_defaultValue, const std::string &_descr="")
        :ParamProxyInterface(_name, _id, _size, _ioType, _descr)
    {
        init(_defaultValue);
    }

    virtual ParamProxyInterface* clone() const
    {
        return new ParamProxyClass<T>(name, id, size, constraints, ioType, defaultValue, description);
    }

    /** Link the parameter to the variable pointed by var. */
    virtual void linkToVariable(void *var, int newSize=-1)
    {
        T* currentValue = value;        ///< store pointer to old variable
        value = (T*) var;               ///< replace pointer with new variable's address
        for(int i=0;i<size;i++)
            value[i] = currentValue[i]; ///< copy current value inside new variable
        delete[] currentValue;          ///< delete old variable
        linkedToExternalVariable = true;
    }

    /** Get the current value of the parameter in Bottle format. 
     * @param index Index of the element of the parameter to read, if index is out 
     *              of range it reads all the elements. 
     */
    virtual void getAsBottle(yarp::os::Bottle &b, int index=-1) const
    {
        if(index>=0 && index<size)
            return b.add(value[index].getAsValue());
        for(int i=0;i<size;i++)
            b.add(value[i].getAsValue());
    }

    /** Set the parameter to the specified value. Return true if the operation succeeded. */
    virtual bool set(const yarp::os::Bottle &newValue, yarp::os::Bottle *reply=0)
    {
        if(!size.freeSize && newValue.size()!=size)
        {
            if(reply!=NULL)
                reply->addString(("Wrong size of parameter "+name+" (expected "+paramHelp::toString(size)+
                    ", found "+paramHelp::toString(newValue.size())).c_str());
            return false;
        }
        if(!checkConstraints(newValue, reply))
            return false;

        ///< if the parameter is not linked to any external variable and if size changed then resize variable
        if(!linkedToExternalVariable && size != newValue.size())
        {
            delete[] value;                 ///< deallocate old memory
            size.size = newValue.size();    ///< update variable size
            value = new T[size];            ///< allocate new memory
        }
        for(int i=0;i<size;i++)
            value[i].setFromValue(newValue.get(i));
        return true;
    }

    /** Set one element of the parameter to the specified value. Return true if the operation succeeded. */
    virtual bool set(const yarp::os::Bottle &newValue, int index, yarp::os::Bottle *reply=0) 
    {
        if(index<0 || index>=size)
        {
            if(reply!=NULL)
                reply->addString(("Index out of bound. Index="+paramHelp::toString(index)+
                    ", parameter size="+paramHelp::toString(size)).c_str());
            return false;
        }
        if(!checkConstraints(newValue, reply))
            return false;
        value[index].setFromValue(newValue.get(0));
        return true;
    }

    /** Return true if the specified value satisfies this parameter's constraints. 
     * Note that the number of values in the bottle does not have to match the number of elements of the parameter.
     * @param newValue Bottle containing the value(s) to check. */
    virtual bool checkConstraints(const yarp::os::Bottle &newValue, yarp::os::Bottle *reply=0) const
    {
        T vi;
        for(int i=0;i<newValue.size();i++)
        {
            if(!vi.setFromValue(newValue.get(i)))   ///< if the conversion from Value to T fails return false
                return false;
            if(!constraints.checkConstraint(vi))
            {
                if(reply!=NULL) { 
                    std::string val_str(vi.getAsValue().toString().c_str());
                    reply->addString(("Value "+val_str+" does not satisfy constraints.").c_str());
                }
                return false;
            }
        }
        return true;
    }
};

    
}//end namespace paramHelp


#endif



