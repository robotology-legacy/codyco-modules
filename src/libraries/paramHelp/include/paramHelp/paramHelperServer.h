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

#ifndef __PARAMHELPSERVER_H__
#define __PARAMHELPSERVER_H__

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <paramHelp/paramHelperBase.h>


namespace paramHelp
{

/************************************************************************************************//**
// Parameter helper (server side).
// To use this class you need an initialization phase, which consists of the following steps:
// 1) Instantiate an object of this class specifying the parameters and the commands of the module
// 2) Link each parameter to a variable, calling the method linkParam()
// 3) Register a callback for each command, calling the method registerCommandCallback()
// 4) If necessary, register a callback for some parameters, calling the method registerParamCallback()
// 5) Call the method init() (actually it can be called at any moment)
//
// After the initializion, you can use this class in these ways:
// 1) Any time an rpc message is received, process it calling the method processRpcCommand()
// 2) To read the input streaming parameter call the method readStreamParams()
// 3) To write the output streaming parameter call the method sendStreamParams()
// 4) To send sporadic messages (about the module status) call the method sendInfoMessage()
//
// If multiple threads use an instance of this class, they can coordinate by using the methods
// lock() and unlock(), which take and release the mutex associated to the object.
// *************************************************************************************************/
class ParamHelperServer: public ParamHelperBase
{
    ///< Define the different types of rpc commands
    enum CommandType
    { 
        COMMAND_GET,        ///< get the values of a parameter
        COMMAND_GET_ONE,    ///< get one of the values of a parameter
        COMMAND_SET,        ///< set the values of a parameter by specifying all the parameter values
        COMMAND_SET_ALL,    ///< set all the values of a parameter to the same value
        COMMAND_SET_ONE,    ///< set one of the values of a parameter
        COMMAND_GENERIC     ///< generic rpc command that is neither a 'set' nor a 'get'
    };

    yarp::os::Semaphore                 mutex;          ///< mutex for the access to the parameter values    
    std::map<int, ParamValueObserver*>  paramValueObs;  ///< list of pointers to parameter value observers
    std::map<int, CommandObserver*>     cmdObs;         ///< list of pointers to command observers
    
    /** Identify the specified rpc command.
     * @param cmd The rpc command (input)
     * @param cmdType Type of command contained in cmd (output)
     * @param id Id of either the parameter (if the command is a set/get) or the command (output)
     * @param v Bottle containing everything that is after the identified command in cmd (output)
     * @return True if the command has been identified, false otherwise. */
    bool identifyCommand(const yarp::os::Bottle &cmd, CommandType &cmdType, int &id, yarp::os::Bottle &v);

    /** Set the value of the parameter with the specified id.
     * @param id Id of the parameter to set
     * @param v Bottle containing the value of the parameter
     * @param reply Bottle into which to write the response of the operation
     * @param init True if the parameter is being initialized (e.g. from config file), false otherwise
     * @return True if the operation succeeded, false otherwise. */
    bool setParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply, bool init=false);
    bool setOneParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);
    bool setAllParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);

    /** Get the value of the parameter with the specified id
     * @param id Id of the parameter (input)
     * @param v Bottle inside which the value of the parameter is put (output)
     * @return True if the operation succeeded, false otherwise */
    bool getParam(int id, yarp::os::Bottle &reply);
    bool getOneParam(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);

    /** Convert a bottle into a vector of int.
     * @param b The bottle to convert (input)
     * @param x The pointer to the first element of the int vector (output)
     * @param maxSize The max size of the int vector, -1 means no size limit (input)
     * @return True if the operation succeeded, false otherwise. */
    bool bottleToParam(const yarp::os::Bottle &b, int* x, int maxSize=-1);

public:
    /** Constructor.
      * @param pdList Array of const pointers to const ParamProxyInterface containing a description of the parameters to add.
      * @param pdListSize Size of the array pdList.
      * @param cdList List of the module commands.
      * @param cdListSize Size of the array cdList.
      */
    ParamHelperServer(const ParamProxyInterface * const * pdList=0, int pdListSize=0, const CommandDescription *cdList=0, int cdListSize=0);

    // Destructor.
    virtual ~ParamHelperServer();

    /** Initialize the module parameters reading the value from the specified resource finder.
      * @param rf Resource finder used to read the parameter values from configuration file or command line
      * @param reply Output message containing information about the initialization (e.g. what went wrong) */
    void initializeParams(yarp::os::ResourceFinder &rf, yarp::os::Bottle &reply);

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
    bool registerParamValueChangedCallback(int id, ParamValueObserver *observer);

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
    inline void lock(){    mutex.wait(); }
    
    /** Release the mutex associated to this object. */
    inline void unlock(){  mutex.post(); }
};
    
}//end namespace paramHelp


#endif



