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
     * @param params Parameters received after the command (if any) */
    virtual void commandReceived(const CommandDescription &cd, const yarp::os::Bottle &params) = 0;
};



static const char* PORT_IN_STREAM_SUFFIX    = "/stream:i";
static const char* PORT_OUT_STREAM_SUFFIX   = "/stream:o";
// *************************************************************************************************
// Parameter helper.
// By default (set/get) rpc commands are managed by calling the method 'respond',
// whereas streaming parameters are read/sent by calling readStreamParams/sendStreamParams.
// *************************************************************************************************
class ParamHelper
{
    yarp::os::Semaphore                 mutex;          // mutex for the access to the parameter values
    
    std::map<int, ParamDescription>     paramList;      // list of parameter descriptions
    std::map<int, void*>                paramValues;    // list of pointers to parameter values
    std::map<int, ParamObserver*>       paramObs;       // list of pointers to parameter observers

    std::map<int, CommandDescription>   cmdList;        // list of command descriptions
    std::map<int, CommandObserver*>     cmdObs;         // list of pointers to command observers

    yarp::os::BufferedPort<yarp::os::Bottle>    *portInStream;  // input port for streaming data
    yarp::os::BufferedPort<yarp::os::Bottle>    *portOutStream; // output port for streaming data
    
    /** Identify the specified rpc command.
     * @param cmd The rpc command (input)
     * @param isSetCmd True if the cmd contains a "set" command, false otherwise (output)
     * @param isGetCmd True if the cmd contains a "get" command, false otherwise (output)
     * @param id Id of either the parameter (if the command is a set/get) or the command (output)
     * @param v Bottle containing everything that is after the identified command in cmd (output)
     * @return True if the command has been identified, false otherwise. */
    bool identifyCommand(const yarp::os::Bottle &cmd, bool &isSetCmd, bool &isGetCmd, int &paramId, yarp::os::Bottle &v);

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
    bool addCommand(const CommandDescription &cd);

    /** Set the value of the parameter with the specified id.
     * @param id Id of the parameter to set
     * @param v Bottle containing the value of the parameter
     * @param reply Bottle into which to write the response of the operation.
     * @return True if the operation succeeded, false otherwise. */
    bool setParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);

    /** This version of setParam is used for the initialization and does not perform callbacks.
      * @param id Id of the parameter
      * @param v Pointer to the variable containing the new value of the parameter
      * @return True if the operation succeeded, false otherwise. */
    bool setParam(int id, const void *v);

    /** Get the value of the parameter with the specified id
     * @param id Id of the parameter (input)
     * @param v Bottle inside which the value of the parameter is put (output)
     * @return True if the operation succeeded, false otherwise */
    bool getParam(int id, yarp::os::Bottle &v);

    /** Check whether a parameter with the specified id exists.
     * @param id Id of the parameter
     * @return True if the parameter exists, false otherwise. */
    bool hasParam(int id){ return paramList.find(id)!=paramList.end(); }
    bool hasCommand(int id){ return cmdList.find(id)!=cmdList.end(); }

    /** Convert a bottle into a vector of int.
     * @param b The bottle to convert (input)
     * @param x The pointer to the first element of the int vector (output)
     * @param maxSize The max size of the int vector, -1 means no size limit (input)
     * @return True if the operation succeeded, false otherwise. */
    bool bottleToParam(const yarp::os::Bottle &b, int* x, int maxSize=-1);

    /** Get a pointer to the element index of the parameter with the specified id. */
    template <class T> inline T* paramValue(int id, int index){ return ((T*)paramValues[id]) +index; }

    void logMsg(const std::string &s);

public:
    /** Constructor.
      * @param pdList
      * @param pdListSize
      * @param cdList
      * @param cdListSize
      */
    ParamHelper(const ParamDescription *pdList=0, int pdListSize=0, const CommandDescription *cdList=0, int cdListSize=0);

    // Destructor
    ~ParamHelper();

    /** Open four ports:
      * - "/moduleName/rpc": Rpc Port for synchronous set/get operations on module parameters
      * - "/moduleName/info:o": Output Port for sporadic message regarding the module status
      * - "/moduleName/stream:i": Input BufferedPort<Bottle> for asynchronous input streaming data
      * - "/moduleName/stream:o": Output BufferedPort<Bottle> for asynchronous output streaming data 
      * @param moduleName Name of the module, used as stem for all the port names 
      * @return True if the initialization succeeded, false otherwise. */
    bool init(std::string moduleName);

    bool close();

    /** Respond to the specified rpc command
     * @param cmd The rpc command (input)
     * @param reply The reply to the rpc command (output)
     * @return bool True if the command is recognized (regardless of the success of the operation), false otherwise. */
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    
    bool addParams(const ParamDescription *pdList, int size);

    bool addCommands(const CommandDescription *cdList, int size);
    
    /** Link the parameter with the specified id to the specified variable v, so that
      * every time that the parameter is set, the value of the specified variable is updated.
      * If the parameter already has a value (e.g. the deafault value), the variable pointed by v is set to that value.
      * @param id Id of the parameter
      * @param v Pointer to the variable that has to contain the parameter value
      * @return True if the operation succeeded, false otherwise. */
    bool linkParam(int id, void *v);

    /** Register a callback on the parameter with the specified id.
      * After the callback is registered, every time the parameter value is set
      * the observer is notified through a call to its method "parameterUpdated".
      * The callback is performed after the new value of the parameter is set and after releasing the mutex.
      * @param id Id of the parameter
      * @param observer Object to notify when the parameter changes value
      * @return True if the operation succeeded, false otherwise. 
      * @note If an observer was already registered, it is overwritten by the new one. */
    bool registerCallback(int id, ParamObserver *observer);

    /** Register a callback for an rpc command.
      * @param id The id of the command
      * @param observer Object to call when the command is received
      * @return True if the operation succeeded, false otherwise. */
    bool registerCallback(int id, CommandObserver *observer);

    /** Send the output streaming parameters.
      * @return True if the operation succeeded, false otherwise */
    bool sendStreamParams();

    /** Send the input streaming parameters.
      * @param blockingRead If true the reading is blocking (it waits until data arrive), otherwise it is not
      * @return True if the operation succeeded, false otherwise */
    bool readStreamParams(bool blockingRead=false);
    
    /** Take the mutex that regulates the access to all the parameters.
      * When the mutex is taken the parameter values are not updated. 
      * @note Some methods do not take the mutex (i.e. linkParam, registerCallback, addParams). */
    void lock(){    mutex.wait(); }
    
    /** Release the mutex that regulates the access to all the parameters. */
    void unlock(){  mutex.post(); }
};
    
}//end namespace paramHelp


#endif



