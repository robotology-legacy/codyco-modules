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

#ifndef __PARAMPROXYINTERFACE_H__
#define __PARAMPROXYINTERFACE_H__

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>



namespace paramHelp
{

/************************************************************************************************//**
// This enum defines the I/O types of parameters: 
// PARAM_CONFIG: can only be set when launching the module (either from command line or from configuration file)
// PARAM_INPUT: can be written from the rpc port, but not read
// PARAM_OUTPUT: can be read from the rpc port, but not written
// PARAM_IN_OUT: can be both written and read from the rpc port
// PARAM_IN_STREAM: can be written (from either rpc or the input streaming port), and read from the rpc port only
// PARAM_OUT_STREAM: can be read (from eitaher rpc or the output streaming port), but not written
// PARAM_IN_OUT_STREAM: can be both written and read from either rpc or the streaming ports
// PARAM_MONITOR: can be read from both the monitor port and the rpc port
// *************************************************************************************************/
enum ParamIOTypeEnum
{ 
    PARAM_IO_UNKNOWN,       ///< unknown parameter input/output type
    PARAM_CONFIG,           ///< standard configuration parameter, e.g. module name, thread period
    PARAM_INPUT,            ///< probably USELESS
    PARAM_OUTPUT,           ///< probably USELESS
    PARAM_IN_OUT,           ///< standard rpc parameter, e.g. control gain, filter frequency
    PARAM_IN_STREAM,        ///< input from other modules
    PARAM_OUT_STREAM,       ///< output to other modules
    PARAM_IN_OUT_STREAM,    ///< input from some modules that is also of interest for other modules
    PARAM_MONITOR,          ///< output streaming on monitoring port
    PARAM_IO_TYPE_SIZE      ///< number of elements of this enum
};

/** Class representing a parameter I/O type. */
class ParamIOType
{
public:
    ParamIOTypeEnum value;
    ParamIOType(ParamIOTypeEnum _value=PARAM_IO_UNKNOWN): value(_value) {}

    inline bool canRead() const
    { return value==PARAM_OUTPUT || value==PARAM_OUT_STREAM || value==PARAM_IN_OUT || value==PARAM_IN_OUT_STREAM || PARAM_IN_STREAM; }
    inline bool canWrite() const
    { return value==PARAM_INPUT || value==PARAM_IN_STREAM || value==PARAM_IN_OUT || value==PARAM_IN_OUT_STREAM; }
    inline bool isStreaming() const
    { return value==PARAM_OUT_STREAM || value==PARAM_IN_STREAM || value==PARAM_IN_OUT_STREAM; }
    inline bool isStreamingOut() const
    { return value==PARAM_OUT_STREAM || value==PARAM_IN_OUT_STREAM; }
    inline bool isStreamingIn() const
    { return value==PARAM_IN_STREAM || value==PARAM_IN_OUT_STREAM; }
    inline bool isMonitoring() const
    { return value==PARAM_MONITOR; }
};


/************************************************************************************************//**
// Class defining the dimension of a parameter
// *************************************************************************************************/
class ParamSize
{
public:
    int size;       ///< parameter dimension (if size is free, this is the default size)
    bool freeSize;  ///< true: the parameter size is free, false it is fixed

    ParamSize(int s=1, bool free=false): size(s), freeSize(free) {}
    operator int() const { return size; } ///< conversion from ParamSize to int (this allows to use ParamSize as an int)
};

const ParamSize PARAM_SIZE_FREE(1, true);   ///< to be used when a parameter has free dimension


/************************************************************************************************//**
// Interface for the proxy of a parameter.
// For any type of parameter (e.g. int, double, string) there has to be a class implementing
// all the methods defined by this interface. 
// *************************************************************************************************/
class ParamProxyInterface
{
public:
    std::string     name;           ///< name of the parameter (can be used as an identifier, alternatively to id)
    std::string     description;    ///< meaning of the parameter displayed in help messages
    int             id;             ///< unique identifier of the parameter
    ParamSize       size;           ///< dimension
    ParamIOType     ioType;         ///< access level (input/output, streaming/rpc)


    ParamProxyInterface():id(-1), size(PARAM_SIZE_FREE), ioType(PARAM_IO_UNKNOWN) {}

    ParamProxyInterface(const std::string &_name, int _id, ParamSize _size, ParamIOType _ioType, const std::string &_descr)
        :name(_name), description(_descr), id(_id), size(_size), ioType(_ioType) {}

    /** Clone this object. This method is used to create a copy of a subclass of ParamProxyInterface
     * so as to avoid "object slicing". In more details, the subclasses of ParamProxyinterface may
     * have additional member variables, which would not be copied by the copy constructor of
     * ParamProxyInterface. To avoid copying only part of the object members, the copy is performed
     * using the clone() method. Since this method is polymorphic it can copy all the member variables
     * of the specific subclass. */
    virtual ParamProxyInterface* clone() const = 0;

    /** Link the parameter to the variable pointed by var. 
     * @param var Pointer to the first element of the array of variables containing the value of this parameter. */
    virtual void linkToVariable(void *var, int newSize=-1) = 0;

    /** Get the current value of the parameter in Bottle format. 
     * @param b Output bottle containing the parameter value(s).
     * @param index Index of the element to get, if less than 0 it means 'all the elements'. */
    virtual void getAsBottle(yarp::os::Bottle &b, int index=-1) const = 0;

    /** Set the parameter to the specified value. Return true if the operation succeeded. */
    virtual bool set(const yarp::os::Bottle &value, yarp::os::Bottle *reply=0) = 0;

    /** Set one element of the parameter to the specified value. 
     * @param value Bottle containing the new value of the parameter.
     * @param index Element of the array to set.
     * @param reply Bottle in which eventual error messages may be inserted.
     * @return true if the operation succeeded. */
    virtual bool set(const yarp::os::Bottle &value, int index, yarp::os::Bottle *reply=0) = 0;

    /** Return true if the specified value satisfies this parameter's constraints. */
    virtual bool checkConstraints(const yarp::os::Bottle &value, yarp::os::Bottle *reply=0) const = 0;

    /** Get the current value of the parameter in string format. */
    virtual std::string getAsString() const
    {
        yarp::os::Bottle b;
        getAsBottle(b);
        return b.toString().c_str();
    }

    /** Set the parameter to the specified value. Return true if the operation succeeded. */
    //virtual bool set(const void *newValue, yarp::os::Bottle *reply=0) = 0;
    /** Set the parameter to the specified value. Return true if the operation succeeded. */
    //virtual bool set(const std::string &value, std::string *reply=0) = 0;
    /** Return true if the specified value satisfies this parameter's constraints. */
    //virtual bool checkConstraints(const std::string &value, std::string *reply=0) = 0;
};


/************************************************************************************************//**
// Description of an RPC command
// *************************************************************************************************/
class CommandDescription
{
public:
    std::string     name;           ///< name of the command
    std::string     description;    ///< meaning of the command displayed in help messages
    int             id;             ///< unique identifier of the command
    
    CommandDescription(): name(""), description(""), id(-1) {}
    
    CommandDescription(const std::string &_name, int _id, const std::string &_descr="")
        :name(_name), description(_descr), id(_id){}
};


/************************************************************************************************//**
// Abstract class to be implemented for getting callbacks when a parameter value is changed.
// *************************************************************************************************/
class ParamValueObserver
{
public:
    /** Method called just after the parameter value has changed.
     * @param pd Description of the parameter. */
    virtual void parameterUpdated(const ParamProxyInterface *pd) = 0;
};


/************************************************************************************************//**
// Abstract class to be implemented for getting callbacks when a parameter's size is about to change.
// *************************************************************************************************/
class ParamSizeObserver
{
public:
    /** Method called every time the parameter's size is about to change.
     * @param pd Description of the parameter. 
     * @param newSize New size of the parameter. 
     * @note This callback is thought to allow the resize of the variable linked to the parameter (if any). */
    virtual void parameterSizeChanged(const ParamProxyInterface *pd, int newSize) = 0;
};


/************************************************************************************************//**
// Abstract class to be implemented for getting callbacks when a command is received.
// *************************************************************************************************/
class CommandObserver
{
public:
    /** Method called every time a command is received.
     * @param cd Description of the command
     * @param params Parameters received after the command (if any) 
     * @param reply Reply to the command */
    virtual void commandReceived(const CommandDescription &cd, const yarp::os::Bottle &params, yarp::os::Bottle &reply) = 0;
};

    
}//end namespace paramHelp


#endif
