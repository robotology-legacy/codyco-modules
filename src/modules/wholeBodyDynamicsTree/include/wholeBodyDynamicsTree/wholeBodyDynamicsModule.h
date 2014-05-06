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

#ifndef __WHOLE_BODY_DYNAMICS_TREE_MODULE_H__
#define __WHOLE_BODY_DYNAMICS_TREE_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include "../wholeBodyDynamics_IDLServer.h"

#include <wholeBodyDynamicsTree/wholeBodyDynamicsThread.h>

#include <wbiIcub/wholeBodyInterfaceIcub.h>


using namespace std;
using namespace yarp::os;
using namespace wbi;

class wholeBodyDynamicsModule: public RFModule, public wholeBodyDynamics_IDLServer
{
    /* module parameters */
    string  moduleName;
    string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

    Port                rpcPort;        // a port to handle rpc messages
    wholeBodyDynamicsThread*   wbdThread;     // locomotion control thread
    wbiIcub::icubWholeBodyStatesLocal* estimationInterface; // interface to communicate with the robot

public:
    wholeBodyDynamicsModule();

    bool attach(yarp::os::Port &source);          // Attach the module to a RPC port
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    double getPeriod(){ return period;  }
    bool updateModule();

    /** RPC methods (Thrift) */
    /**
      * Calibrate the force/torque sensors
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feets)
      * @param nr_of_samples argument to specify the number of samples to use for calibration
      * @return true/false on success/failure
      */
    virtual bool calib(const std::string& calib_code, const int32_t nr_of_samples);

    virtual bool calibOnDoubleSupport(const std::string& calib_code, const int32_t nr_of_samples);

    /**
     * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
     * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
     * @return true/false on success/failure
     */
    virtual bool resetOffset(const std::string& calib_code);


    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    virtual bool quit();
};



#endif
//empty line to make gcc happy

