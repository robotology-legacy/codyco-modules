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
    std::string part_name;
    int axis_number;
    double lower_limit;
    double upper_limit;
    double delta;
};

class desiredPositions
{
public:
    yarp::sig::Vector pos;
    double waiting_time;
    bool is_return_point;
    typedef enum { ROW_START, ROW_IN, ROW_STOP, ROW_OUT } RowBoundary_t;
    RowBoundary_t rowBoundary;
    desiredPositions(yarp::sig::Vector _pos, double _waiting_time,
                     bool _is_return_point=false, RowBoundary_t _rowBoundary=ROW_OUT):
    pos(_pos),
    waiting_time(_waiting_time),
    is_return_point(_is_return_point),
    rowBoundary(_rowBoundary)
    {}
    std::string toString()
    {
        std::stringstream ss;

        ss << "pos: " << this->pos.toString()
        << "\t wait: " << this->waiting_time
        << "\t return: " << this->is_return_point
        << "\t row boundary: " << this->rowBoundary
        << std::endl;

        return ss.str();
    }
};

class reachRandomJointPositionsModule: public RFModule
{
    /* module parameters */
    string moduleName;
    string robotName;
    double period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;
    
    enum { RANDOM, GRID_VISIT, GRID_MAPPING, GRID_MAPPING_WITH_RETURN } mode;

    double static_pose_period;
    double return_point_waiting_period;
    double elapsed_time; // time passed from when the desired pose was reached
    double ref_speed;

    std::vector< controlledJoint > controlledJoints;
    yarp::sig::Vector commandedPositions;
    double desired_waiting_time;
    std::vector<desiredPositions> listOfDesiredPositions;
    yarp::os::BufferedPort<yarp::os::Bottle> isTheRobotInReturnPoint;
    yarp::os::BufferedPort<yarp::os::Bottle> useFurtherPosForFitting;
    bool is_desired_point_return_point;
    bool keep_fitting_after_desired_point;
    yarp::os::Stamp timestamp;
    double remoteLatchedTimestamp;
    double localLatchedTimestamp;

    int next_desired_position;
    yarp::sig::Vector originalPositions;
    yarp::sig::Vector originalRefSpeeds;
    bool jointInitialized;

    std::map<std::string,yarp::dev::PolyDriver *> drivers;
    std::map<std::string,yarp::dev::IPositionControl *> pos;
    std::map<std::string,yarp::dev::IEncoders *>encs;
    std::map<std::string,yarp::dev::IControlLimits *>lims;
    std::map<std::string,yarp::dev::IRemoteCalibrator *> calib;

    void close_drivers();

public:
    reachRandomJointPositionsModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();
    bool getNewDesiredPosition(yarp::sig::Vector & desired_pos, double & desired_parked_time,
                               bool & is_return_point, bool & keep_fitting_after_desired_point);
    bool updateModule();

private:
    /*
     * Helper for generating the vector of desired positions:
     * draws a raw moving one
     * joint while the other one is fixed
     */
    bool drawRow(yarp::sig::Vector center, int movingJointIdx, int fixedJointStep,
                 bool withReturn, bool flagReturn);

    /*
     * Synchronise th timestamp with the one from iCub "State_ext:o" port envelope
     */
    bool latchTimestampSync();
};


#endif
//empty line to make gcc happy
