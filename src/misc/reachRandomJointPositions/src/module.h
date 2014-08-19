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

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits2.h>



using namespace std;
using namespace yarp::os;

struct controlledJoint
{
public:
    std::string part_name;
    int axis_number;
    double lower_limit;
    double upper_limit;
};

class reachRandomJointPositionsModule: public RFModule
{
    /* module parameters */
    string moduleName;
    string robotName;
    double period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;

    double new_position_period;
    double elapsed_time; // time passed from last position command
    double ref_speed;

    std::vector< controlledJoint > controlledJoints;

    std::map<std::string,yarp::dev::PolyDriver *> drivers;
    std::map<std::string,yarp::dev::IPositionControl *> pos;
    std::map<std::string,yarp::dev::IEncoders *>encs;
    std::map<std::string,yarp::dev::IControlLimits *>lims;

    void close_drivers();

public:
    reachRandomJointPositionsModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();
    bool updateModule();

};


#endif
//empty line to make gcc happy
