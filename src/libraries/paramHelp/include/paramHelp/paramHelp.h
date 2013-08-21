/**
 * \defgroup paramHelp paramHelp
 *
 * @ingroup codyco_libraries
 *
 * Classes for simplifying the management of the module parameters.
 *
 * \section dep_sec Dependencies
 * YARP
 *
 * \section intro_sec Description
 *
 *
 * \section tested_os_sec Tested OS
 *
 * Windows
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


// *************************************************************************************************
// This enum defines the possible data types of parameters
// *************************************************************************************************
enum ParamDataType{ PARAM_DATA_UNKNOWN, PARAM_DATA_FLOAT, PARAM_DATA_INT, PARAM_DATA_BOOL, PARAM_DATA_STRING, PARAM_DATATYPE_SIZE };
const std::string ParamDataType_desc[PARAM_DATATYPE_SIZE] = { "UNKNOWN PARAM TYPE", "FLOAT", "INT", "BOOL", "STRING" };

// *************************************************************************************************
// This enum defines the I/O types of parameters: 
// PARAM_INPUT: the parameter can be written from the rpc port, but not read
// PARAM_OUTPUT: the parameter can be read from the rpc port, but not written
// PARAM_IN_OUT: the parameter can be both written and read from the rpc port
// PARAM_IN_STREAM: the parameter can be written (from either rpc or the input streaming port), but not read
// PARAM_OUT_STREAM: the parameter can be read (from either rpc or the output streaming port), but not written
// PARAM_IN_OUT_STREAM: the parameter can be both written and read from the rpc port (from either rpc or the streaming ports)
// *************************************************************************************************
enum ParamIOTypeEnum
{ PARAM_IO_UNKNOWN, PARAM_INPUT, PARAM_OUTPUT, PARAM_IN_OUT, PARAM_IN_STREAM, PARAM_OUT_STREAM, PARAM_IN_OUT_STREAM, PARAM_IO_TYPE_SIZE };
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


// *************************************************************************************************
// Class defining the dimension of a parameter
// *************************************************************************************************
class ParamSize
{
public:
    int size;       // parameter dimension (if size is free, this is the default size)
    bool freeSize;  // true: the parameter size is free, false it is fixed
    ParamSize(int s=1, bool free=false): size(s), freeSize(free) {}
};
const ParamSize PARAM_SIZE_FREE(1, true);   // to be used when a parameter has free dimension


// *************************************************************************************************
// Single bound on a parameter value
// *************************************************************************************************
class ParamBound
{
public:
    double bound;   // value of the bound
    bool hasBound;  // false if the bound is at infinity
    ParamBound(bool _hasBound, double _bound=0.0): hasBound(_hasBound), bound(_bound) {}
    ParamBound(double _bound): hasBound(true), bound(_bound) {}
};
const ParamBound PARAM_BOUND_INF(false);   // no bound


// *************************************************************************************************
// Double bound on a parameter value
// *************************************************************************************************
class ParamBounds
{
public:
    double  upperBound,     lowerBound;
    bool    hasUpperBound,  hasLowerBound;
    ParamBounds(double low, double up): lowerBound(low), upperBound(up), hasUpperBound(true), hasLowerBound(true) {}
    ParamBounds(ParamBound low, double up): lowerBound(low.bound), upperBound(up), hasUpperBound(true), hasLowerBound(low.hasBound) {}
    ParamBounds(double low, ParamBound up): lowerBound(low), upperBound(up.bound), hasUpperBound(up.hasBound), hasLowerBound(true) {}
    ParamBounds(ParamBound low, ParamBound up): lowerBound(low.bound), upperBound(up.bound), hasUpperBound(up.hasBound), hasLowerBound(low.hasBound) {}
};
const ParamBounds PARAM_BOUNDS_INF(PARAM_BOUND_INF, PARAM_BOUND_INF);   // no bounds


// *************************************************************************************************
// Description of a parameter
// *************************************************************************************************
class ParamDescription
{
public:
    std::string     name;           // name of the parameter (can be used as an identifier, alternatively to id)
    std::string     description;    // meaning of the parameter displayed in help messages
    int             id;             // unique identifier of the parameter
    ParamDataType   type;           // data type
    ParamSize       size;           // dimension
    ParamBounds     bounds;         // constraints on the range of values that the parameter can take
    ParamIOType     ioType;         // access level (input/output, streaming/rpc)
    const void      *defaultValue;  // default value of the parameter

    ParamDescription()
        : id(-1), type(PARAM_DATA_UNKNOWN), size(PARAM_SIZE_FREE), bounds(PARAM_BOUNDS_INF), ioType(PARAM_IO_UNKNOWN) {}
    
    ParamDescription(const std::string &_name, int _id, ParamDataType _type, ParamSize _size, ParamBounds _bounds=PARAM_BOUNDS_INF, 
        ParamIOTypeEnum _ioType=PARAM_IN_OUT, const void *_value=0, const std::string &_descr="")
        :name(_name), id(_id), type(_type), size(_size), bounds(_bounds), ioType(_ioType), defaultValue(_value), description(_descr) {}
    
    // This constructor takes an 'int' rather than a ParamSize (just to simplify notation)
    ParamDescription(const std::string &_name, int _id, ParamDataType _type, int _size, ParamBounds _bounds=PARAM_BOUNDS_INF, 
        ParamIOTypeEnum _ioType=PARAM_IN_OUT, const void *_value=0, const std::string &_descr="")
        :name(_name), id(_id), type(_type), size(ParamSize(_size)), bounds(_bounds), 
        ioType(_ioType), defaultValue(_value), description(_descr) {}
};


// *************************************************************************************************
// Description of an RPC command
// *************************************************************************************************
class CommandDescription
{
public:
    std::string     name;           // name of the command
    std::string     description;    // meaning of the command displayed in help messages
    int             id;             // unique identifier of the command
    
    CommandDescription(): name(""), id(-1), description("") {}
    
    CommandDescription(const std::string &_name, int _id, const std::string &_descr="")
        :name(_name), id(_id), description(_descr) {}
};


// *************************************************************************************************
// Abstract class to be implemented for getting callbacks when a parameter value is changed.
// *************************************************************************************************
class ParamObserver
{
public:
    /** Method called every time the parameter value is changed.
     * @param pd Description of the parameter. */
    virtual void parameterUpdated(const ParamDescription &pd) = 0;
};


// *************************************************************************************************
// Abstract class to be implemented for getting callbacks when a command is received.
// *************************************************************************************************
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
    std::map<int, ParamDescription>     paramList;      // list of parameter descriptions
    std::map<int, void*>                paramValues;    // list of pointers to parameter values
    std::map<int, CommandDescription>   cmdList;        // list of command descriptions

    yarp::os::BufferedPort<yarp::os::Bottle>    *portInStream;  // input port for streaming data
    yarp::os::BufferedPort<yarp::os::Bottle>    *portOutStream; // output port for streaming data
    yarp::os::Port                              portInfo;       // port for sporadic info messages

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



