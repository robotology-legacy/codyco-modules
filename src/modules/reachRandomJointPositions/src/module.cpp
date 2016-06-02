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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include <cmath>

#include "module.h"



using namespace yarp::dev;



reachRandomJointPositionsModule::reachRandomJointPositionsModule()
{
    period = 1;

}

void reachRandomJointPositionsModule::close_drivers()
{
    if(jointInitialized)
    {
        for(int jnt=0; jnt < originalPositions.size(); jnt++ )
        {
            pos->setRefSpeed(jnt,originalRefSpeeds[jnt]);
            calib->homingSingleJoint(jnt);
        }
    }

    driver->close();

    delete driver;

    driver = 0;
}


bool reachRandomJointPositionsModule::configure(ResourceFinder &rf)
{
    jointInitialized = false;
    
    std::cout << rf.toString() << std::endl;

    if( rf.check("robot") )
    {
        robotName = rf.find("robot").asString().c_str();
    }
    else
    {
        robotName = "icub";
    }

    std::string mode_cfg = rf.find("mode").asString().c_str();
    if( mode_cfg == "random" )
    {
        mode = RANDOM;
    } 
    else if( mode_cfg == "gridVisit" )
    {
        mode = GRID_VISIT;
    }
    else if( mode_cfg == "gridMappingWithReturn" )
    {
        mode = GRID_MAPPING_WITH_RETURN;
    }
    else
    {
        std::cerr << "[ERR] reachRandomJointPositionsModule: mode " << mode_cfg << "is not available, exiting." << std::endl;
        std::cerr << "[ERR] existing modes: random, gridVisit, gridMappingWithReturn" << std::endl;
    }

    
    if( rf.check("localPortPrefix") )
    {
        moduleName = rf.find("localPortPrefix").asString().c_str();
    }

    static_pose_period = rf.check("static_pose_period",1.0).asDouble();
    return_point_waiting_period = rf.check("return_point_waiting_period",5.0).asDouble();
    elapsed_time = 0.0;
    ref_speed = rf.check("ref_speed",3.0).asDouble();
    period = rf.check("period",1.0).asDouble();


    if ( !rf.check("joints") )
    {
        fprintf(stderr, "Please specify the name and joints of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--joints ((part_name axis_number lower_limit upper_limit) ... )\n");
        return false;
    }

    if( !rf.check("remoteControlBoards") || !rf.find("remoteControlBoards").isList() )
    {
        fprintf(stderr, "Please specify the remote controlboards of the robot\n");
        fprintf(stderr, "--remoteControlBoards (/icubSim/left_leg,/icubSim/left_leg) \n");
        return false;
    }

    yarp::os::Bottle & jnts = rf.findGroup("joints");

    if( jnts.isNull() )
    {
        fprintf(stderr, "Please specify the name and joints of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--joints ((part_name axis_number lower_limit upper_limit) ... )\n");
        return false;
    }

    int nrOfControlledJoints = jnts.size()-1;

    std::cout << "Found " << nrOfControlledJoints << " joint to control" << std::endl;

    controlledJoints.resize(nrOfControlledJoints);
    controlledJointNames.resize(nrOfControlledJoints);
    commandedPositions.resize(nrOfControlledJoints,0.0);
    originalPositions.resize(nrOfControlledJoints);
    originalRefSpeeds.resize(nrOfControlledJoints);

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        yarp::os::Bottle * jnt_ptr = jnts.get(jnt+1).asList();
        if( jnt_ptr == 0 || jnt_ptr->isNull() || !(jnt_ptr->size() == 4)  )
        {
            fprintf(stderr, "Malformed configuration file (joint %d)\n",jnt);
            return false;
        }

        std::cout << jnt_ptr->toString() << std::endl;

        controlledJointNames[jnt] = jnt_ptr->get(0).asString().c_str();
        controlledJoint new_joint;
        new_joint.lower_limit = jnt_ptr->get(1).asDouble();
        new_joint.upper_limit = jnt_ptr->get(2).asDouble();
        new_joint.delta = jnt_ptr->get(3).asDouble();

        controlledJoints[jnt] = new_joint;
    }

    // Open the controlboard
    Property options;
    options.put("device","remotecontrolboardremapper");

    Bottle axesNames;
    Bottle & axesList = axesNames.addList();
    for(size_t jnt=0; jnt < controlledJointNames.size(); jnt++)
    {
        axesList.addString(controlledJointNames[jnt]);
    }

    options.put("axesNames",axesNames.get(0));
    options.put("remoteControlBoards",rf.find("remoteControlBoards"));
    options.put("localPortPrefix",rf.find("localPortPrefix"));

    driver = new PolyDriver(options);

    bool ok = true;
    ok = ok && driver->view(encs);
    ok = ok && driver->view(pos);
    ok = ok && driver->view(lims);
    ok = ok && driver->view(calib);

    if(!ok)
    {
        fprintf(stderr, "Error in opening the remote controlboards\n");
        close_drivers();
        return false;
    }

    encs->getEncoders(originalPositions.data());
    pos->getRefSpeeds(originalRefSpeeds.data());

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        pos->setRefSpeed(jnt,ref_speed);
    }
    
    jointInitialized = true;

    //Configure
    if( mode == GRID_MAPPING_WITH_RETURN )
    {
        listOfDesiredPositions.resize(0,desiredPositions(yarp::sig::Vector(),0.0));
        next_desired_position = 0;
        //Generate vector of desired positions 
        if( controlledJoints.size() != 2)
        {
            std::cerr << "GRID_MAPPING_WITH_RETURN mode available only for two controlled joints" << std::endl;
            close_drivers();
            return false;
        }
        
        yarp::sig::Vector center(2), lower_left(2),lower_right(2),upper_left(2),upper_right(2);
        std::vector<int> semi_nr_of_lines(2,0);
        lower_left[0] = lower_right[0] = controlledJoints[0].lower_limit;
        upper_left[0] = upper_right[0] = controlledJoints[0].upper_limit;
        lower_left[1] = upper_left[1] = controlledJoints[1].lower_limit;
        lower_right[1] = upper_right[0] = controlledJoints[1].upper_limit;
        center[0] =  (controlledJoints[0].lower_limit+controlledJoints[0].upper_limit)/2; 
        center[1] =  (controlledJoints[1].lower_limit+controlledJoints[1].upper_limit)/2; 
        semi_nr_of_lines[0] = ceil((controlledJoints[0].upper_limit-center[0])/controlledJoints[0].delta);
        semi_nr_of_lines[1] = ceil((controlledJoints[1].upper_limit-center[1])/controlledJoints[1].delta);
        //Start at the center of the workspace
        listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period));
        
        for(int i=0; i < (int)semi_nr_of_lines[0]; i++ ) 
        {
            //Draw upper row
            yarp::sig::Vector row_center(2), row_lower(2), row_upper(2);
            row_upper[0] = row_lower[0] = row_center[0] = center[0]+i*controlledJoints[0].delta;
            row_center[1] = center[1];
            row_lower[1] = controlledJoints[1].lower_limit;
            row_upper[1] = controlledJoints[1].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period));

            //Draw lower row
            row_upper[0] = row_lower[0] = row_center[0] = center[0]-i*controlledJoints[0].delta;
            row_center[1] = center[1];
            row_lower[1] = controlledJoints[1].lower_limit;
            row_upper[1] = controlledJoints[1].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period));

        }
        for(int j=0; j < (int)semi_nr_of_lines[1]; j++ )
        {
            //Draw upper row
            yarp::sig::Vector row_center(2), row_lower(2), row_upper(2);
            row_upper[1] = row_lower[1] = row_center[1] = center[1]+j*controlledJoints[1].delta;
            row_center[0] = center[0];
            row_lower[0] = controlledJoints[0].lower_limit;
            row_upper[0] = controlledJoints[0].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period));

            //Draw lower row
            row_upper[1] = row_lower[1] = row_center[1] = center[1]-j*controlledJoints[1].delta;
            row_center[0] = center[0];
            row_lower[0] = controlledJoints[0].lower_limit;
            row_upper[0] = controlledJoints[0].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period));
        }
    }

    return true;
}


bool reachRandomJointPositionsModule::interruptModule()
{
    return true;
}

bool reachRandomJointPositionsModule::close()
{
    close_drivers();
    return true;
}

/**
 * 
 */
bool reachRandomJointPositionsModule::getNewDesiredPosition(yarp::sig::Vector & desired_pos, double & desired_parked_time)
{
    switch(mode)
    {
        case GRID_MAPPING_WITH_RETURN:
            if( next_desired_position >= 0 && next_desired_position < listOfDesiredPositions.size() )
            {
                desired_pos = listOfDesiredPositions[next_desired_position].pos;
                desired_parked_time = listOfDesiredPositions[next_desired_position].waiting_time;
                next_desired_position++;
                return true;
            }
            else
            {
                return false;
            }
        break;
        default:
            std::cerr << "[ERR] reachRandomJointPositionsModule: unknown mode " << mode << ", exiting" << std::endl;
            return false;
        break;
    }
    
}

bool reachRandomJointPositionsModule::updateModule()
{
    //Check that all desired position have been reached
    bool dones=true;
    for(int jnt=0; jnt < controlledJoints.size(); jnt++ )
    {
        bool done=true;
        pos->checkMotionDone(jnt,&done);
        dones = dones && done;
    }
    
    if(dones)
    {
        elapsed_time += getPeriod();
    }
    
    if( elapsed_time > desired_waiting_time )
    {
        elapsed_time = 0.0;
        //set a new position for the controlled joints
        bool new_position_available = getNewDesiredPosition(commandedPositions,desired_waiting_time);
        if( !new_position_available )
        {
            //no new position available, exiting
            return false;
        }
        
        //Set a new position for the controlled joints
        bool boring_overflow = true;
        for(int jnt=0; jnt < controlledJoints.size(); jnt++ )
        {
            double low = controlledJoints[jnt].lower_limit;
            double up = controlledJoints[jnt].upper_limit;
            //Set desired position, depending on the mode
            {
                if( !boringModeInitialized )
                {
                    commandedPositions[jnt] = low;
                }
                else
                {
                    if(boring_overflow)
                    {
                        commandedPositions[jnt]=commandedPositions[jnt]+controlledJoints[jnt].delta;
                        boring_overflow = false;
                        if( commandedPositions[jnt] > up )
                        {
                            commandedPositions[jnt] = low;
                            boring_overflow = true;
                        }
                    }
                }
            }
            std::cout  << "Send new desired position: " << commandedPositions[jnt] << " to joint " << controlledJointNames[jnt] << std::endl;
            pos->positionMove(jnt,commandedPositions[jnt]);
        }
        boringModeInitialized = true;
    } 
    
    return true;
}

double reachRandomJointPositionsModule::getPeriod()
{
    return period;
}
