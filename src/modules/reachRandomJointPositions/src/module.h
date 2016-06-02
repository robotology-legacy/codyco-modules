/*
* Copyright (C) 2014 ...
* Author: ...
* email: ...
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

#ifndef REACHRANDOM_JOINT_POSITIONS_MODULE_H
#define REACHRANDOM_JOINT_POSITIONS_MODULE_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/CalibratorInterfaces.h>

#include <yarp/sig/Vector.h>



using namespace std;
using namespace yarp::os;

struct controlledJoint
{
public:
    double lower_limit;
    double upper_limit;
    double delta;
};

class desiredPositions
{
public:
    yarp::sig::Vector pos;
    double waiting_time;
    desiredPositions(yarp::sig::Vector _pos, double _waiting_time):
    pos(_pos), waiting_time(_waiting_time)
    {}
};

class reachRandomJointPositionsModule: public RFModule
{
    /* module parameters */
    string moduleName;
    string robotName;
    double period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;
    
    enum { RANDOM, GRID_VISIT, GRID_MAPPING_WITH_RETURN } mode;
    bool boringModeInitialized;

    double static_pose_period;
    double return_point_waiting_period;
    double elapsed_time; // time passed from when the desired pose was reached
    double ref_speed;

    std::vector< std::string     > controlledJointNames;
    std::vector< controlledJoint > controlledJoints;
    yarp::sig::Vector commandedPositions;
    double desired_waiting_time;
    std::vector<desiredPositions> listOfDesiredPositions;
    
    int next_desired_position;
    yarp::sig::Vector originalPositions;
    yarp::sig::Vector originalRefSpeeds;
    bool jointInitialized;

    yarp::dev::PolyDriver * driver;
    yarp::dev::IPositionControl * pos;
    yarp::dev::IEncoders * encs;
    yarp::dev::IControlLimits * lims;
    yarp::dev::IRemoteCalibrator * calib;

    void close_drivers();

public:
    reachRandomJointPositionsModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();

    /**
     * Get a new desired position.
     *
     * @param[out] desired_pos vector of desired position (in YARP units, i.e. deg for revolute and m for primatic joints
     * @param[out] desired_parked_time time to spend parked in this position
     * @return false when the position to reach have been all reached.
     */
    bool getNewDesiredPosition(yarp::sig::Vector & desired_pos,
                               double & desired_parked_time);
    bool updateModule();

};


#endif
//empty line to make gcc happy
