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

#ifndef __PARAMHELPSERVER_H__
#define __PARAMHELPSERVER_H__

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <paramHelp/paramHelp.h>


namespace paramHelp
{

// *************************************************************************************************
// Parameter helper (server side).
// To use this class you need an initialization phase, which consists of the following steps:
// 1) Instantiate an object of this class specifying the parameters and the commands of the module
// 2) Link each parameter to a variable, calling the method 'linkParam'
// 3) Register a callback for each command, calling the method 'registerCommandCallback'
// 4) If necessary, register a callback for some parameters, , calling the method 'registerParamCallback'
// 5) Call the method 'init' (actually it can be called in any moment)
//
// After the initializion, you can use this class in these ways:
// 1) Any time an rpc message is received, process it calling the method 'processRpcCommand'
// 2) To read the input streaming parameter call the method 'readStreamParams'
// 3) To write the output streaming parameter call the method 'sendStreamParams'
// 4) To send sporadic messages (about the module status) call the method 'sendInfoMessage'
//
// If multiple threads use an instance of this class, they can coordinate by using the methods
// 'lock' and 'unlock', which take and release the mutex associated to the object.
// *************************************************************************************************
class ParamHelperServer: public ParamHelperBase
{
    yarp::os::Semaphore                 mutex;          // mutex for the access to the parameter values    
    std::map<int, ParamObserver*>       paramObs;       // list of pointers to parameter observers
    std::map<int, CommandObserver*>     cmdObs;         // list of pointers to command observers
    
    /** Identify the specified rpc command.
     * @param cmd The rpc command (input)
     * @param isSetCmd True if the cmd contains a "set" command, false otherwise (output)
     * @param isGetCmd True if the cmd contains a "get" command, false otherwise (output)
     * @param id Id of either the parameter (if the command is a set/get) or the command (output)
     * @param v Bottle containing everything that is after the identified command in cmd (output)
     * @return True if the command has been identified, false otherwise. */
    bool identifyCommand(const yarp::os::Bottle &cmd, bool &isSetCmd, bool &isGetCmd, int &paramId, yarp::os::Bottle &v);

    /** Set the value of the parameter with the specified id.
     * @param id Id of the parameter to set
     * @param v Bottle containing the value of the parameter
     * @param reply Bottle into which to write the response of the operation.
     * @return True if the operation succeeded, false otherwise. */
    bool setParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);

    /** Get the value of the parameter with the specified id
     * @param id Id of the parameter (input)
     * @param v Bottle inside which the value of the parameter is put (output)
     * @return True if the operation succeeded, false otherwise */
    bool getParam(int id, yarp::os::Bottle &v);

    /** Convert a bottle into a vector of int.
     * @param b The bottle to convert (input)
     * @param x The pointer to the first element of the int vector (output)
     * @param maxSize The max size of the int vector, -1 means no size limit (input)
     * @return True if the operation succeeded, false otherwise. */
    bool bottleToParam(const yarp::os::Bottle &b, int* x, int maxSize=-1);

public:
    /** Constructor.
      * @param pdList List of the module parameters
      * @param pdListSize
      * @param cdList List of the module commands
      * @param cdListSize
      */
    ParamHelperServer(const ParamDescription *pdList=0, int pdListSize=0, const CommandDescription *cdList=0, int cdListSize=0);

    // Destructor
    ~ParamHelperServer();

    /** Open 4 ports (at the moment only 3 ports because the rpc is assumed to be already opened) :
      * - "/moduleName/rpc": Rpc Port for synchronous set/get operations on module parameters
      * - "/moduleName/info:o": Output Port for sporadic message regarding the module status
      * - "/moduleName/stream:i": Input BufferedPort<Bottle> for asynchronous input streaming data
      * - "/moduleName/stream:o": Output BufferedPort<Bottle> for asynchronous output streaming data 
      * @param moduleName Name of the module, used as stem for all the port names 
      * @return True if the initialization succeeded, false otherwise. */
    bool init(std::string moduleName);

    /** Register a callback on the parameter with the specified id.
      * After the callback is registered, every time the parameter value is set
      * the observer is notified through a call to its method "parameterUpdated".
      * The callback is performed after the new value of the parameter is set and after releasing the mutex.
      * @param id Id of the parameter
      * @param observer Object to notify when the parameter changes value
      * @return True if the operation succeeded, false otherwise. 
      * @note If an observer was already registered, it is overwritten by the new one. */
    bool registerParamCallback(int id, ParamObserver *observer);

    /** Register a callback for an rpc command.
      * @param id The id of the command
      * @param observer Object to call when the command is received
      * @return True if the operation succeeded, false otherwise. */
    bool registerCommandCallback(int id, CommandObserver *observer);

    /** Process the specified rpc command
     * @param cmd The rpc command (input)
     * @param reply The reply to the rpc command (output)
     * @return bool True if the command is recognized (regardless of the success of the operation), false otherwise. */
    bool processRpcCommand(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);

    /** Send the specified message on the info port.
      * @param b Message to send
      * @return True if the operation succeeded, false otherwise */
    bool sendInfoMessage(yarp::os::Bottle &b){ return portInfo.write(b); }

    /** Send the output streaming parameters.
      * @return True if the operation succeeded, false otherwise */
    bool sendStreamParams();

    /** Read the input streaming parameters.
      * @param blockingRead If true the reading is blocking (it waits until data arrive), otherwise it is not
      * @return True if the operation succeeded, false otherwise */
    bool readStreamParams(bool blockingRead=false);

    /** Get the help message for the rpc port
      * @param b The help message is put into this bottle */
    void getHelpMessage(yarp::os::Bottle &b);
    
    /** Take the mutex associated to this object.
      * @note The management of the concurrent accesses to this object is left to the user. */
    void lock(){    mutex.wait(); }
    
    /** Release the mutex associated to this object. */
    void unlock(){  mutex.post(); }
};
    
}//end namespace paramHelp


#endif



