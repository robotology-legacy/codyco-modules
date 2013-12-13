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
 * \subsection sec_config_file Configuration File
 *
 * ParamHelperServer can automatically read the values of the parameters from a configuration file.
 * However, any white space in the name of the parameters is expected to be replaced with an underscore '_'
 * in the configuration file. For instance, if a parameter name is 'kp com', ParamHelperServer will look
 * for a parameter 'kp_com' in the configuration file. This is motivated by the behavior of the yarp 
 * ResourceFinder, which works only with parameters whose name does not contain white spaces.
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

#ifndef __PARAMHELPERBASE_H__
#define __PARAMHELPERBASE_H__

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <paramHelp/paramHelpUtil.h>
#include <paramHelp/paramProxyInterface.h>


namespace paramHelp
{

/**
  * Base class for the ParamHelperClient and ParamHelperServer.
  */
class ParamHelperBase
{
protected:
    bool                initDone;                        ///< true after successfull initialization
    std::map<int, ParamProxyInterface*> paramList;      ///< list of parameter proxies
    std::map<int, CommandDescription>   cmdList;        ///< list of command descriptions

    yarp::os::BufferedPort<yarp::os::Bottle>    *portInStream;      ///< input port for streaming data
    yarp::os::BufferedPort<yarp::os::Bottle>    *portOutStream;     ///< output port for streaming data
    yarp::os::BufferedPort<yarp::os::Bottle>    *portOutMonitor;    ///< output port for monitoring with Yarpscope
    yarp::os::Port                              portInfo;           ///< port for sporadic info messages

    std::map<int, ParamSizeObserver*>   paramSizeObs;   ///< list of pointers to parameter size observers

    /** Check whether the specified value satisfy the constraints on the specified parameter.
     * @param id Id of the parameter (input)
     * @param v Value of the parameter (input)
     * @param reply An error message is added to this bottle if a constraint is violated (output)
     * @return True if the constraints are satisfied, false otherwise. */
    bool checkParamConstraints(int id, const yarp::os::Bottle &v, yarp::os::Bottle &reply);
    //bool checkParamConstraints(int id, const void *v);

    /** Add the specified parameter to the list of managed parameters. 
     * If a default value is specified, the parameter is initialized to that value.
     * @param pd Description of the parameter to add
     * @return True if the operation succeeded, false otherwise (parameter id conflict) */
    bool addParam(const ParamProxyInterface *pd);
    
    /** Add the specified parameters to the list of managed parameters. 
     * If a default value is specified, the parameter is initialized to that value.
     * @param pdList Array of const pointers to const ParamProxyInterface containing a description of the parameters to add.
     * @return True if the operation succeeded, false otherwise (parameter id conflict). 
     * @note This method clones the paramProxyInterface contained in pdList. */
    bool addParams(const ParamProxyInterface *const *pdList, int size);
    bool addCommand(const CommandDescription &cd);
    bool addCommands(const CommandDescription *cdList, int size);

    /** Check whether a parameter with the specified id exists.
     * @param id Id of the parameter
     * @return True if the parameter exists, false otherwise. */
    inline bool hasParam(int id) const   { return paramList.find(id)!=paramList.end(); }
    inline bool hasCommand(int id) const { return cmdList.find(id)!=cmdList.end(); }

    /** This version of setParam is used for the initialization and does not perform callbacks.
      * @param id Id of the parameter
      * @param v Pointer to the variable containing the new value of the parameter
      * @return True if the operation succeeded, false otherwise. */
    //bool setParam(int id, const void *v);

    enum MsgType{ MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR };
    
    void logMsg(const std::string &s, MsgType type=MSG_INFO) const;

public:
    /** Default constructor. */
    ParamHelperBase();

    inline bool isInitDone(){ return initDone; }

    /** Close the ports opened during the initialization phase (see init method). */
    virtual bool closePorts();
    
    /** Delete all the memory allocated when cloning the ParamProxyInterfaces. */
    virtual bool deleteParameters();

    virtual bool close(){ return closePorts() && deleteParameters(); }

    /* COMMENTS ON THE METHOD linkParam:
     * PROBLEM:
     * Linking the parameter through a pointer to the variable complicates the management of 
     * variable size parameters. When the new value of the parameter has a different size the memory
     * needs to be reallocated, but the paramProxy cannot do it because it has only the pointer to
     * the current memory. 
     * SOLUTION:
     * Rather than passing the pointer to the memory (i.e. a void*) you could pass a pointer to
     * a generic class T that has to implement the method resize and the operator []. This is
     * true for Yarp, Eigen and stl vectors, so it is a reasonable constraint. However, it is
     * not true for standard C arrays, and I wonder whether I should let the old way still 
     * viable for constant-size parameters.
     */

    /** Link the parameter with the specified id to the variable pointed by v, so that
      * every time that the parameter is set, the value of the specified variable is updated.
      * If the parameter already has a value (e.g. the default value), the variable pointed 
      * by v is set to that value.
      * @param id Id of the parameter.
      * @param v Pointer to the variable that has to contain the parameter value.
      * @param newSize New size of the parameter (it applies only for free size parameters).
      * @return True if the operation succeeded, false otherwise. */
    virtual bool linkParam(int id, void *v, int newSize=-1);

    /*** @return A pointer to the proxy of the parameter with the specified id. */
    virtual ParamProxyInterface* getParamProxy(int id){ return paramList[id]; }

    /** Send the output streaming parameters.
      * @return True if the operation succeeded, false otherwise */
    virtual bool sendStreamParams() = 0;

    /** Read the input streaming parameters.
      * @param blockingRead If true the reading is blocking (it waits until data arrive), otherwise it is not
      * @return True if the operation succeeded, false otherwise */
    virtual bool readStreamParams(bool blockingRead=false) = 0;

    /** Register a callback on the parameter with the specified id.
      * After the callback is registered, every time the parameter size changes
      * the observer is notified through a call to its method "parameterSizeChanged".
      * The callback is performed before setting the new value of the parameter,
      * so that the variable can be resized.
      * @param id Id of the parameter.
      * @param observer Object to notify when the parameter changes size.
      * @return True if the operation succeeded, false otherwise. 
      * @note If an observer was already registered, it is overwritten by the new one. */
    bool registerParamSizeChangedCallback(int id, ParamSizeObserver *observer);

    /** Write the specified parameters to file. If no parameters are specified, write all of them. 
     * @param filename name of the text file on which to write.
     * @param paramIds Pointer to an array of parameter IDs.
     * @param paramNumber Size of the array paramIds. 
     * @return True if the operation succeeded, false otherwise. */
    virtual bool writeParamsOnFile(std::string filename, int *paramIds=0, int paramNumber=0);
};

    
}//end namespace paramHelp


#endif
