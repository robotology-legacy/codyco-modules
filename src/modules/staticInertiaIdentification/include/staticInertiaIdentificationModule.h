/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef STATIC_INERTIA_IDENTIFICATION_MODULE_H
#define STATIC_INERTIA_IDENTIFICATION_MODULE_H

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include "staticInertiaIdentification_IDLServer.h"
#include "staticInertiaIdentificationThread.h"

#include <wbiIcub/wholeBodyInterfaceIcub.h>


using namespace std;
using namespace yarp::os;
using namespace wbi;

class staticInertiaIdentificationModule: public RFModule, public staticInertiaIdentification_IDLServer
{
    /* module parameters */
    string  moduleName;
    string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

    Port                rpcPort;        // a port to handle rpc messages
    staticInertiaIdentificationThread*   siiThread;     // locomotion control thread
    wbiIcub::icubWholeBodyStatesLocal* estimationInterface; // interface to communicate with the robot

public:
    staticInertiaIdentificationModule();

    bool attach(yarp::os::Port &source);          // Attach the module to a RPC port
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    double getPeriod(){ return period;  }
    bool updateModule();

    /** RPC methods (Thrift) */
    /**
     * Start the estimation.
     * @return true/false on success/failure
     */
    virtual bool start();
/**
 * stop the estimation.
 * @return true/false on success/failure
 */
  virtual bool stop();
/**
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();
};



#endif
//empty line to make gcc happy

