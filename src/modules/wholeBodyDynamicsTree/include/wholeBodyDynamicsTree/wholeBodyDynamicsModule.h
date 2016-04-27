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

#include "wholeBodyDynamics_IDLServer.h"

#include <wholeBodyDynamicsTree/wholeBodyDynamicsThread.h>

#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>

class wholeBodyDynamicsModule: public yarp::os::RFModule, public wholeBodyDynamics_IDLServer
{
    /* module parameters */
    std::string  moduleName;
    std::string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

    yarp::os::Port                 rpcPort;        // a port to handle rpc messages
    wholeBodyDynamicsThread*     wbdThread;     // locomotion control thread
    yarpWbi::yarpWholeBodySensors* sensors;

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

    virtual bool calibStanding(const std::string& calib_code, const int32_t nr_of_samples);

    virtual bool calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples);

    virtual bool calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples);
    /**
     * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
     * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
     * @return true/false on success/failure
     */
    virtual bool resetOffset(const std::string& calib_code);

  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a link that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_link the name of the link that should be initially fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
    virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link);

  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_link the name of the new link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
    virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link);


    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    virtual bool quit();
};



#endif
//empty line to make gcc happy

