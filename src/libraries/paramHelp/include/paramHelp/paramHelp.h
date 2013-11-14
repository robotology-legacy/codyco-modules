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
/**
 * \defgroup paramHelp paramHelp
 *
 * @ingroup codyco_libraries
 *
 * Classes for simplifying the management of the parameters of YARP modules.
 * The two main classes of this library are ParamHelperServer and ParamHelperClient.

 * \section ParamHelperServer_sec Param Helper Server
 *
 * ParamHelperServer can be used to simplify these operations:
 * - reading parameters from a configuration file
 * - reading input parameters from a streaming YARP port
 * - writing output parameters on a streaming YARP port
 * - setting/getting parameters through an RPC YARP port
 *
 * To automatize these operations ParamHelperServer needs a description of the parameters,
 * in the form of a vector of ParamDescription objects. A ParamDescription contains this information:
 * - name:           name of the parameter (can be used as an identifier, alternatively to id)
 * - description:    meaning of the parameter (displayed in rpc port help messages)
 * - id:             unique integer identifier of the parameter
 * - type:           data type (int, float, string), see ParamDataType
 * - size:           dimension of the parameter vector (1 if the parameter is a single value), see ParamSize
 * - bounds:         constraints on the parameter values (makes sense only for numbers), see ParamBounds
 * - ioType:         access level (input/output, streaming/rpc), see ParamIOTypeEnum
 * - defaultValue:   default value of the parameter
 *
 * Besides the description of the parameters, ParamHelperServer needs to know where the parameters are 
 * stored in memory (i.e. the address of the variable containing the value of the parameter).
 * To link a parameter to a variable the user can use the method ParamHelperServer::linkParam().
 * If necessary, the user can also get a callback every time a parameter is set; this is done by
 * calling ParamHelperServer::registerParamCallback(). The object that gets the callback has to 
 * inherit the abstract class ParamObserver.
 *
 * After the initial configuration, the user can use this class in these ways:
 * - Any time an rpc message is received, process it calling the method processRpcCommand()
 * - To read the input streaming parameter call the method readStreamParams()
 * - To write the output streaming parameter call the method sendStreamParams()
 * - To send sporadic messages (about the module status) call the method sendInfoMessage()
 *
 * If multiple threads use an instance of this class, they can coordinate by using the methods
 * lock() and unlock(), which take and release the mutex associated to the object.
 *
 * \subsection rpc_command RPC Commands
 * 
 * The ParamHelperServer can also help managing rpc commands such as 'start', 'stop', 'quit', 'help'.
 * The priciples are similar to what just explained regarding the parameter management.
 * An rpc command is described by an object of the class CommandDescription.
 * To register a callback for an rpc command, use ParamHelperServer::registerCommandCallback().
 *
 * \subsection open_ports Open YARP Ports
 * The ParamHelperServer opens four YARP ports when ParamHelperServer::init() is called:
 * - "/<module_name>/stream:i": port from which to read the input streaming parameters
 * - "/<module_name>/stream:o": port on which to send the output streaming parameters
 * - "/<module_name>/info:o":   port on which to write output info messages
 * - "/<module_name>/rpc":      port for rpc communication
 *
 * \section tested_os_sec Tested OS
 * Windows, Linux
 *
 * \section dep_sec Dependencies
 * YARP
 *
 * \author Andrea Del Prete - andrea.delprete@iit.it
 *
 * Copyright (C) 2013-.. CODYCO
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 **/

#ifndef __PARAMHELP_H__
#define __PARAMHELP_H__

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

/** Convert a generic variable into a string. */
template <class T> inline std::string toString(const T& t)
{ std::ostringstream ss; ss << t; return ss.str(); }

/** Convert a generic vector into a string */
template <class T> inline std::string toString(const std::vector<T>& v, const char *separator=" ")
{ std::ostringstream s; std::copy(v.begin(), v.end(), std::ostream_iterator<T>(s, separator)); return s.str(); }

/** Print the specified Bottle. */
void printBottle(const yarp::os::Bottle &b);


/************************************************************************************************//**
// This enum defines the possible data types of parameters
// *************************************************************************************************/
enum ParamDataType{ PARAM_DATA_UNKNOWN, PARAM_DATA_FLOAT, PARAM_DATA_INT, PARAM_DATA_BOOL, PARAM_DATA_STRING, PARAM_DATATYPE_SIZE };
const std::string ParamDataType_desc[PARAM_DATATYPE_SIZE] = { "UNKNOWN PARAM TYPE", "FLOAT", "INT", "BOOL", "STRING" };

/************************************************************************************************//**
// This enum defines the I/O types of parameters: 
// PARAM_CONFIG: can only be set when launching the module (either from command line or from configuration file)
// PARAM_INPUT: can be written from the rpc port, but not read
// PARAM_OUTPUT: can be read from the rpc port, but not written
// PARAM_IN_OUT: can be both written and read from the rpc port
// PARAM_IN_STREAM: can be written (from either rpc or the input streaming port), and read from the rpc port only
// PARAM_OUT_STREAM: can be read (from either rpc or the output streaming port), but not written
// PARAM_IN_OUT_STREAM: can be both written and read from either rpc or the streaming ports
// *************************************************************************************************/
enum ParamIOTypeEnum
{ 
    PARAM_IO_UNKNOWN, 
    PARAM_CONFIG,           ///< standard configuration parameter, e.g. module name, thread period
    PARAM_INPUT,            ///< probably USELESS
    PARAM_OUTPUT,           ///< probably USELESS
    PARAM_IN_OUT,           ///< standard rpc parameter, e.g. control gain, filter frequency
    PARAM_IN_STREAM,        ///< input from other modules
    PARAM_OUT_STREAM,       ///< output to other modules
    PARAM_IN_OUT_STREAM,    ///< input from some modules that is also of interest for other modules
    PARAM_IO_TYPE_SIZE 
};
class ParamIOType
{
public:
    ParamIOTypeEnum value;
    ParamIOType(ParamIOTypeEnum _value=PARAM_IO_UNKNOWN): value(_value) {}
    bool canRead();
    bool canWrite();
    bool isStreaming();
    bool isStreamingOut();
    bool isStreamingIn();
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
// Single bound on a parameter value
// *************************************************************************************************/
class ParamBound
{
public:
    double bound;   ///< value of the bound
    bool hasBound;  ///< false if the bound is at infinity
    ParamBound(bool _hasBound, double _bound=0.0): hasBound(_hasBound), bound(_bound) {}
    ParamBound(double _bound): hasBound(true), bound(_bound) {}
};
const ParamBound PARAM_BOUND_INF(false);   ///< no bound


/************************************************************************************************//**
// Double bound on a parameter value
// *************************************************************************************************/
class ParamBounds
{
public:
    double  upperBound,     lowerBound;
    bool    hasUpperBound,  hasLowerBound;
    ParamBounds(double low, double up): lowerBound(low), upperBound(up), hasUpperBound(true), hasLowerBound(true) {}
    ParamBounds(ParamBound low, double up): lowerBound(low.bound), upperBound(up), hasUpperBound(true), hasLowerBound(low.hasBound) {}
    ParamBounds(double low, ParamBound up): lowerBound(low), upperBound(up.bound), hasUpperBound(up.hasBound), hasLowerBound(true) {}
    ParamBounds(ParamBound low, ParamBound up): lowerBound(low.bound), upperBound(up.bound), hasUpperBound(up.hasBound), hasLowerBound(low.hasBound) {}

    /** Return true if the specified value respects the bounds, false otherwise. */
    bool checkBounds(double v){ return (!hasUpperBound || v<=upperBound) && (!hasLowerBound || v>=lowerBound); }
    std::string toString()
    { return std::string("[")+(hasLowerBound?paramHelp::toString(lowerBound):"-INF")+", "+(hasUpperBound?paramHelp::toString(upperBound):"INF")+"]"; }
};
const ParamBounds PARAM_BOUNDS_INF(PARAM_BOUND_INF, PARAM_BOUND_INF);   ///< no bounds


/************************************************************************************************//**
// Description of a parameter
// *************************************************************************************************/
class ParamDescription
{
public:
    std::string     name;           ///< name of the parameter (can be used as an identifier, alternatively to id)
    std::string     description;    ///< meaning of the parameter displayed in help messages
    int             id;             ///< unique identifier of the parameter
    ParamDataType   type;           ///< data type
    ParamSize       size;           ///< dimension
    ParamBounds     bounds;         ///< constraints on the range of values that the parameter can take
    ParamIOType     ioType;         ///< access level (input/output, streaming/rpc)
    const void      *defaultValue;  ///< default value of the parameter

    ParamDescription()
        : id(-1), type(PARAM_DATA_UNKNOWN), size(PARAM_SIZE_FREE), bounds(PARAM_BOUNDS_INF), ioType(PARAM_IO_UNKNOWN) {}
    
    ParamDescription(const std::string &_name, int _id, ParamDataType _type, ParamSize _size, ParamBounds _bounds=PARAM_BOUNDS_INF, 
        ParamIOTypeEnum _ioType=PARAM_IN_OUT, const void *_value=0, const std::string &_descr="")
        :name(_name), id(_id), type(_type), size(_size), bounds(_bounds), ioType(_ioType), defaultValue(_value), description(_descr) {}
    
    ///< This constructor takes an 'int' rather than a ParamSize (just to simplify notation)
    ParamDescription(const std::string &_name, int _id, ParamDataType _type, int _size, ParamBounds _bounds=PARAM_BOUNDS_INF, 
        ParamIOTypeEnum _ioType=PARAM_IN_OUT, const void *_value=0, const std::string &_descr="")
        :name(_name), id(_id), type(_type), size(ParamSize(_size)), bounds(_bounds), 
        ioType(_ioType), defaultValue(_value), description(_descr) {}
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
    
    CommandDescription(): name(""), id(-1), description("") {}
    
    CommandDescription(const std::string &_name, int _id, const std::string &_descr="")
        :name(_name), id(_id), description(_descr) {}
};


/************************************************************************************************//**
// Abstract class to be implemented for getting callbacks when a parameter value is changed.
// *************************************************************************************************/
class ParamObserver
{
public:
    /** Method called every time the parameter value is changed.
     * @param pd Description of the parameter. */
    virtual void parameterUpdated(const ParamDescription &pd) = 0;
};


/************************************************************************************************//**
// Abstract class to be implemented for getting callbacks when a command is received.
// *************************************************************************************************/
class CommandObserver
{
public:
    /** Method called every time the command is received.
     * @param cd Description of the command
     * @param params Parameters received after the command (if any) 
     * @param reply Reply to the command */
    virtual void commandReceived(const CommandDescription &cd, const yarp::os::Bottle &params, yarp::os::Bottle &reply) = 0;
};



/** Suffixes of the ports opened by the ParamHelper class. */
static const char* PORT_IN_STREAM_SUFFIX    = "/stream:i";
static const char* PORT_OUT_STREAM_SUFFIX   = "/stream:o";
static const char* PORT_OUT_INFO_SUFFIX     = "/info:o";
static const char* PORT_IN_INFO_SUFFIX      = "/info:i";
static const char* PORT_RPC_SUFFIX          = "/rpc";

/**
  * Base class for the ParamHelperClient and ParamHelperServer.
  */
class ParamHelperBase
{
protected:
    std::map<int, ParamDescription>     paramList;      ///< list of parameter descriptions
    std::map<int, void*>                paramValues;    ///< list of pointers to parameter values
    std::map<int, CommandDescription>   cmdList;        ///< list of command descriptions

    yarp::os::BufferedPort<yarp::os::Bottle>    *portInStream;  ///< input port for streaming data
    yarp::os::BufferedPort<yarp::os::Bottle>    *portOutStream; ///< output port for streaming data
    yarp::os::Port                              portInfo;       ///< port for sporadic info messages

    /** Check whether the specified value satisfy the constraints on the specified parameter.
     * @param id Id of the parameter (input)
     * @param v Value of the parameter (input)
     * @param reply An error message is added to this bottle if a constraint is violated (output)
     * @return True if the constraints are satisfied, false otherwise. */
    bool checkParamConstraints(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);
    bool checkParamConstraints(int id, const void *v);

    /** Add the specified parameter to the list of managed parameters. 
     * If a default value is specified, the parameter is initialized to that value.
     * @param pd Description of the parameter to add
     * @return True if the operation succeeded, false otherwise (parameter id conflict) */
    bool addParam(const ParamDescription &pd);
    bool addParams(const ParamDescription *pdList, int size);
    bool addCommands(const CommandDescription *cdList, int size);
    bool addCommand(const CommandDescription &cd);

    /** Check whether a parameter with the specified id exists.
     * @param id Id of the parameter
     * @return True if the parameter exists, false otherwise. */
    bool hasParam(int id){ return paramList.find(id)!=paramList.end(); }
    bool hasCommand(int id){ return cmdList.find(id)!=cmdList.end(); }

    /** This version of setParam is used for the initialization and does not perform callbacks.
      * @param id Id of the parameter
      * @param v Pointer to the variable containing the new value of the parameter
      * @return True if the operation succeeded, false otherwise. */
    bool setParam(int id, const void *v);

    /** Get a pointer to the element index of the parameter with the specified id. */
    template <class T> inline T* paramValue(int id, int index){ return ((T*)paramValues[id]) +index; }

    enum MsgType{ MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR };
    void logMsg(const std::string &s, MsgType type=MSG_INFO);

public:
    /** Close the ports opened during the initialization phase (see init method). */
    virtual bool close();
    
    /** Link the parameter with the specified id to the specified variable v, so that
      * every time that the parameter is set, the value of the specified variable is updated.
      * If the parameter already has a value (e.g. the deafault value), the variable pointed by v is set to that value.
      * @param id Id of the parameter
      * @param v Pointer to the variable that has to contain the parameter value
      * @return True if the operation succeeded, false otherwise. */
    virtual bool linkParam(int id, void *v);

    /** Send the output streaming parameters.
      * @return True if the operation succeeded, false otherwise */
    virtual bool sendStreamParams() = 0;

    /** Read the input streaming parameters.
      * @param blockingRead If true the reading is blocking (it waits until data arrive), otherwise it is not
      * @return True if the operation succeeded, false otherwise */
    virtual bool readStreamParams(bool blockingRead=false) = 0;
};

    
}//end namespace paramHelp


#endif



