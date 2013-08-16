/*
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \defgroup paramHelp paramHelp
 *
 * @ingroup codyco_libraries
 *
 * Classes for simplifying the management of the module parameters,
 * with special attention to set/get rpc commands.
 *
 *
 * \section dep_sec Dependencies
 
 *
 * \section intro_sec Description
 *
 *
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
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
#include <string>
#include <sstream>
#include <map>



namespace paramHelp
{

/** Convert a generic variable into a string. */
template <class T> inline std::string toString(const T& t){ std::stringstream ss; ss << t; return ss.str(); }

// *************************************************************************************************
// This enum defines the types of parameters
// *************************************************************************************************
enum ParamType{ PARAM_TYPE_UNKNOWN, PARAM_TYPE_FLOAT, PARAM_TYPE_INT, PARAM_TYPE_BOOL, PARAM_TYPE_STRING, PARAM_TYPE_SIZE };
const std::string ParamType_desc[PARAM_TYPE_SIZE] = { "UNKNOWN PARAM TYPE", "FLOAT", "INT", "BOOL", "STRING" };


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
    std::string     name;
    int             id;
    ParamType       type;
    ParamSize       size;
    ParamBounds     bounds;

    ParamDescription(): id(-1), type(PARAM_TYPE_UNKNOWN), size(PARAM_SIZE_FREE), bounds(PARAM_BOUNDS_INF) {}
    ParamDescription(std::string _name, int _id, ParamType _type, ParamSize _size, ParamBounds _bounds=PARAM_BOUNDS_INF)
        :name(_name), id(_id), type(_type), size(_size), bounds(_bounds) {}
};



// *************************************************************************************************
// Parameter helper
// *************************************************************************************************
class ParamHelper
{
    yarp::os::Semaphore                 mutex;          // mutex for the access to this object
    std::map<int, ParamDescription>     paramList;      // list of parameter descriptions
    std::map<int, void*>                paramValues;    // list of pointers to parameter values
    
    /** Identify the specified rpc command.
     * @param cmd The rpc command (input)
     * @param isSetCmd True if the cmd contains a "set" command, false otherwise (output)
     * @param paramId Id of the parameter that the rpc command refers to (output)
     * @param v Bottle containing everything that is after the identified command in cmd (output)
     * @return True if the command has been identified, false otherwise. */
    bool identifyCommand(const yarp::os::Bottle &cmd, bool &isSetCmd, int &paramId, yarp::os::Bottle &v);

    /** Check whether the specified value satisfy the constraints on the specified parameter.
     * @param id Id of the parameter (input)
     * @param v Value of the parameter (input)
     * @param reply An error message is added to this bottle if a constraint is violated (output)
     * @return True if the constraints are satisfied, false otherwise. */
    bool checkParamConstraints(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);

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
    /** Respond to the specified rpc command
     * @param cmd The rpc command (input)
     * @param reply The reply to the rpc command (output)
     * @return bool True if the command is recognized (regardless of the success of the operation), false otherwise. */
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);

    /** Add the specified parameter to the list of managed parameters. 
     * @param pd Description of the parameter to add
     * @return True if the operation succeeded, false otherwise (parameter id conflict) */
    bool addParam(const ParamDescription &pd);
    bool addParams(const ParamDescription *pdList, int size);
    
    /** Check whether a parameter with the specified id exists.
     * @param id Id of the parameter
     * @return True if the parameter exists, false otherwise. */
    bool hasParam(int id){ return paramList.find(id)!=paramList.end(); }
    
    /** Get the value of the parameter with the specified id
     * @param id Id of the parameter (input)
     * @param v Bottle inside which the value of the parameter is put (output)
     * @return True if the operation succeeded, false otherwise */
    bool getParam(int id, yarp::os::Bottle &v);
    
    /** Set the value of the parameter with the specified id.
     * @param id Id of the parameter to set
     * @param v Bottle containing the value of the parameter
     * @param reply Bottle into which to write the response of the operation.
     * @return True if a parameter with the specified id exists, false otherwise. */
    bool setParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);
    
    /** Link the parameter with the specified id with the specified variable, so that
      * every time the parameter is set, the value of the specified variable is updated.
      * @param id Id of the parameter
      * @param v Pointer to the variable that has to contain the parameter value
      * @return True if the operation succeeded, false otherwise. */
    bool linkParam(int id, void *v);
    
    void lock(){    mutex.wait(); }
    void unlock(){  mutex.post(); }
};
    
}//end namespace paramHelp


#endif



