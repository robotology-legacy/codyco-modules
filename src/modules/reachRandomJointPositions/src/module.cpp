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



reachRandomJointPositionsModule::reachRandomJointPositionsModule():
period(1),
timestamp()
{
}

void reachRandomJointPositionsModule::close_drivers()
{
    std::map<string,PolyDriver*>::iterator it;
    if(jointInitialized)
    {
        for(int jnt=0; jnt < int(originalPositions.size()); jnt++ )
        {
            std::string part = controlledJoints[jnt].part_name;
            int axis = controlledJoints[jnt].axis_number;
            pos[part]->setRefSpeed(axis,originalRefSpeeds[jnt]);
            if( !calib[part]->homingSingleJoint(axis) )
            {
                std::cerr << "[ERR] reachRandomJointPositionsModule: homing part " << part << ", axis " << axis << " failed!" << std::endl;
            }
        }
    }
    for(it=drivers.begin(); it!=drivers.end(); it++ )
    {
        if(it->second)
        {
            it->second->close();
            it->second = 0;
        }
    }
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
    else if( mode_cfg == "gridMapping")
    {
        mode = GRID_MAPPING;
    }
    else if( mode_cfg == "gridMappingWithReturn" )
    {
        mode = GRID_MAPPING_WITH_RETURN;
    }
    else
    {
        std::cerr << "[ERR] reachRandomJointPositionsModule: mode " << mode_cfg << "is not available, exiting." << std::endl;
        std::cerr << "[ERR] existing modes: random, gridVisit, gridMapping, gridMappingWithReturn" << std::endl;
    }

    
    if( rf.check("local") )
    {
        moduleName = rf.find("local").asString().c_str();
    }

    static_pose_period = rf.check("static_pose_period",1.0).asDouble();
    return_point_waiting_period = rf.check("return_point_waiting_period",5.0).asDouble();
    elapsed_time = 0.0;
    ref_speed = rf.check("ref_speed",3.0).asDouble();
    period = rf.check("period",1.0).asDouble();

    //Check ports collecting position flagging data
    if( rf.check("returnFlagPort") )
    {
        this->returnFlagPort = rf.find("returnFlagPort").asString();
    }
    if( rf.check("fittingFlagPort") )
    {
        this->fittingFlagPort = rf.find("fittingFlagPort").asString();
    }

    if ( !rf.check("joints") )
    {
        fprintf(stderr, "Please specify the name and joints of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--joints ((part_name axis_number lower_limit upper_limit) ... )\n");
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
    commandedPositions.resize(nrOfControlledJoints,0.0);
    originalPositions.resize(nrOfControlledJoints);
    originalRefSpeeds.resize(nrOfControlledJoints);

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        yarp::os::Bottle * jnt_ptr = jnts.get(jnt+1).asList();
        if( jnt_ptr == 0 || jnt_ptr->isNull() || !(jnt_ptr->size() == 4 || jnt_ptr->size() == 5)  )
        {
            fprintf(stderr, "Malformed configuration file (joint %d)\n",jnt);
            return false;
        }

        std::cout << jnt_ptr->toString() << std::endl;

        controlledJoint new_joint;
        new_joint.part_name = jnt_ptr->get(0).asString().c_str();
        new_joint.axis_number = jnt_ptr->get(1).asInt();
        new_joint.lower_limit = jnt_ptr->get(2).asDouble();
        new_joint.upper_limit = jnt_ptr->get(3).asDouble();
        if( mode != RANDOM ) 
        {
            new_joint.delta = jnt_ptr->get(4).asDouble();
        }

        controlledJoints[jnt] = new_joint;
    }

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        if( drivers.count(controlledJoints[jnt].part_name) == 1 )
        {
            //parts controlboard already opened, continue
            continue;
        }

        std::string part_name = controlledJoints[jnt].part_name;

        //Open the polydrivers
        std::string remotePort="/";
          remotePort+=robotName;
          remotePort+="/";
          remotePort+= part_name;

        std::string localPort="/"+moduleName+remotePort;
        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", localPort.c_str());   //local port names
        options.put("remote", remotePort.c_str());         //where we connect to

       // create a device
       PolyDriver * new_driver = new PolyDriver(options);
       if( !new_driver->isValid() )
       {
           fprintf(stderr, "[ERR] Error in opening %s part\n",remotePort.c_str());
           close_drivers();
           return false;
       }

       bool ok = true;

       IEncoders * new_encs;
       IPositionControl * new_poss;
       IControlLimits * new_lims;
       IRemoteCalibrator * new_calib;
       ok = ok && new_driver->view(new_encs);
       ok = ok && new_driver->view(new_poss);
       ok = ok && new_driver->view(new_lims);
       ok = ok && new_driver->view(new_calib);

       if(!ok)
       {
           fprintf(stderr, "Error in opening %s part\n",remotePort.c_str());
           close_drivers();
           return false;
       }

       drivers[part_name] = new_driver;
       pos[part_name] = new_poss;
       encs[part_name] = new_encs;
       lims[part_name] = new_lims;
       calib[part_name] = new_calib;
    }

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        std::string part = controlledJoints[jnt].part_name;
        int axis = controlledJoints[jnt].axis_number;
        encs[part]->getEncoder(axis,originalPositions.data()+jnt);
        pos[part]->getRefSpeed(axis,originalRefSpeeds.data()+jnt);
        pos[part]->setRefSpeed(axis,ref_speed);
    }
    
    jointInitialized = true;

    //Configure
    switch(mode)
    {
        case GRID_MAPPING:
        case GRID_MAPPING_WITH_RETURN:
        {
            yarp::sig::Vector center(2), row_center(2), row_lower(2), row_upper(2);
            std::vector<int> semi_nr_of_lines(2,0);

            is_desired_point_return_point = false;
            listOfDesiredPositions.resize(0,desiredPositions(yarp::sig::Vector(),0.0));
            next_desired_position = 0;

            //Generate vector of desired positions
            if( controlledJoints.size() != 2)
            {
                std::cerr << "GRID_MAPPING_WITH_RETURN mode available only for two controlled joints" << std::endl;
                close_drivers();
                return false;
            }

            center[0] = (controlledJoints[0].lower_limit+controlledJoints[0].upper_limit)/2;
            center[1] = (controlledJoints[1].lower_limit+controlledJoints[1].upper_limit)/2;
            semi_nr_of_lines[0] = ceil((controlledJoints[0].upper_limit-center[0])/controlledJoints[0].delta);
            semi_nr_of_lines[1] = ceil((controlledJoints[1].upper_limit-center[1])/controlledJoints[1].delta);

            //Start at the center of the workspace
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,
                                                              true,desiredPositions::ROW_OUT));
            bool return2center = (mode == GRID_MAPPING_WITH_RETURN);

            //Draw rows moving second joint, for each step of first joint position
            this->drawRow(center, 1, 0, return2center, false); //Draw center row
            for(int i = 1; i < semi_nr_of_lines[0]+1; i++)
            {
                this->drawRow(center, 1, i, return2center, true); //Draw upper row

                this->drawRow(center, 1, -i, return2center, false); //Draw lower row
            }

            //Return to center position for eventual drift checking by external module
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,
                                                              true,desiredPositions::ROW_OUT));

            //Draw rows moving first joint, for each step of second joint position
            this->drawRow(center, 0, 0, return2center, false); //Draw center row
            for(int j = 1; j < semi_nr_of_lines[1]+1; j++ )
            {
                this->drawRow(center, 0, j, return2center, true); //Draw upper row

                this->drawRow(center, 0, -j, return2center, false); //Draw lower row
            }

            //Return to center position for eventual drift checking by external module
            listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,
                                                              true,desiredPositions::ROW_OUT));

            //Print list of desired positions
            std::cout  << "List of desired positions: " << std::endl;
            for(int i=0; i<int(listOfDesiredPositions.size()); i++)
            {
                std::cout << listOfDesiredPositions[i].toString() << std::endl;
            }

            isTheRobotInReturnPoint.open("/"+moduleName+"/"+controlledJoints[0].part_name+"/isTheRobotInReturnPoint:o");
            useFurtherPosForFitting.open("/"+moduleName+"/"+controlledJoints[0].part_name+"/useFurtherPosForFitting:o");
            
            break;
        }

        case RANDOM:
        case GRID_VISIT:
            break;

        default:
            break;
    }

    //Latch the remote timestamp for synchronising local one
    this->latchTimestampSync();

    //Wait for "isTheRobotInReturnPoint:o" and "useFurtherPosForFitting:o" to be
    //connected to another port
    if( !returnFlagPort.empty() )
    {
        while(!Network::isConnected(isTheRobotInReturnPoint.getName(), returnFlagPort)) {};
    }
    if( !fittingFlagPort.empty() )
    {
        while(!Network::isConnected(useFurtherPosForFitting.getName(), fittingFlagPort)) {};
    }

    return true;
}


bool reachRandomJointPositionsModule::interruptModule()
{
    return this->close();
}

bool reachRandomJointPositionsModule::close()
{
    close_drivers();
    if( Network::isConnected(isTheRobotInReturnPoint.getName(), returnFlagPort) )
    {
        Network::disconnect(isTheRobotInReturnPoint.getName(), returnFlagPort);
    }
    if( Network::isConnected(useFurtherPosForFitting.getName(), fittingFlagPort) )
    {
        Network::disconnect(useFurtherPosForFitting.getName(), fittingFlagPort);
    }

    if( !isTheRobotInReturnPoint.isClosed() ) {isTheRobotInReturnPoint.close();}
    if( !useFurtherPosForFitting.isClosed() ) {useFurtherPosForFitting.close();}
    return true;
}

/**
 * 
 */
bool reachRandomJointPositionsModule::getNewDesiredPosition(yarp::sig::Vector & desired_pos, double & desired_parked_time,
                                                            bool & is_return_point, bool & keep_fitting_after_desired_point)
{
    switch(mode)
    {
        case GRID_MAPPING:
        case GRID_MAPPING_WITH_RETURN:
            if( next_desired_position >= 0 && next_desired_position < int(listOfDesiredPositions.size()) )
            {
                desired_pos = listOfDesiredPositions[next_desired_position].pos;
                desired_parked_time = listOfDesiredPositions[next_desired_position].waiting_time;
                is_return_point = listOfDesiredPositions[next_desired_position].is_return_point;
                if( listOfDesiredPositions[next_desired_position].rowBoundary == desiredPositions::ROW_START
                   || listOfDesiredPositions[next_desired_position].rowBoundary == desiredPositions::ROW_IN )
                {
                    keep_fitting_after_desired_point = true;
                }
                else
                {
                    keep_fitting_after_desired_point = false;
                }
                next_desired_position++;
                return true;
            }
            else
            {
                return false;
            }

        case RANDOM:
        case GRID_VISIT:
            std::cerr << "[ERR] reachRandomJointPositionsModule: mode not implemented " << mode << ", exiting" << std::endl;
            return false;

        default:
            std::cerr << "[ERR] reachRandomJointPositionsModule: unknown mode " << mode << ", exiting" << std::endl;
            return false;
    }
    
}

bool reachRandomJointPositionsModule::updateModule()
{
    //Check that all desired position have been reached
    bool dones=true;
    for(int jnt=0; jnt < int(controlledJoints.size()); jnt++ )
    {
        bool done=true;
        pos[controlledJoints[jnt].part_name]->checkMotionDone(controlledJoints[jnt].axis_number,
                                                              &done);
        dones = dones && done;
    }
    
    if(dones)
    {
        //Update elapsed time
        elapsed_time += getPeriod();

        //Update the timestamp and publish status for external modules using sensor data
        this->timestamp.update(Time::now()
                               - this->localLatchedTimestamp
                               + this->remoteLatchedTimestamp);
        if( !isTheRobotInReturnPoint.isClosed() )
        {
            isTheRobotInReturnPoint.prepare().clear();
            isTheRobotInReturnPoint.prepare().addInt(int(is_desired_point_return_point));
            isTheRobotInReturnPoint.setEnvelope(this->timestamp);
            isTheRobotInReturnPoint.write();
        }

        if( !useFurtherPosForFitting.isClosed() )
        {
            useFurtherPosForFitting.prepare().clear();
            useFurtherPosForFitting.prepare().addInt(int(keep_fitting_after_desired_point));
            useFurtherPosForFitting.setEnvelope(this->timestamp);
            useFurtherPosForFitting.write();
        }
    }

    if( elapsed_time > desired_waiting_time )
    {
        elapsed_time = 0.0;
        //Get a new desired position
        if( !getNewDesiredPosition(commandedPositions,
                                   desired_waiting_time,
                                   is_desired_point_return_point,
                                   keep_fitting_after_desired_point) )
        {
            //no new position available, exiting
            return false;
        }

        //Set the position for the controlled joints
        for(int jnt=0; jnt < int(controlledJoints.size()); jnt++ )
        {
            std::string part = controlledJoints[jnt].part_name;
            int axis = controlledJoints[jnt].axis_number;

            //Send desired position to robot
            std::cout  << "Send new desired position: " << commandedPositions[jnt]
            << " to joint " << part <<  " " << axis << std::endl;
            pos[part]->positionMove(axis,commandedPositions[jnt]);
        }
    }
    
    return true;
}

double reachRandomJointPositionsModule::getPeriod()
{
    return period;
}

bool reachRandomJointPositionsModule::drawRow(yarp::sig::Vector center, int movingJointIdx, int fixedJointStep,
                                              bool withReturn, bool flagReturn)
{
    yarp::sig::Vector row_upper(2), row_lower(2), row_center(2);

    //only two joints supported for this function
    if(movingJointIdx > 1)
    {
        std::cerr << "[ERR] Mode available only for 2 controlled joints." << std::endl;
        return false;
    }

    int fixedJointIdx = 1 - movingJointIdx; // define fixed joint index

    row_center[fixedJointIdx] = center[fixedJointIdx]+fixedJointStep*controlledJoints[fixedJointIdx].delta;
    row_upper[fixedJointIdx] = row_lower[fixedJointIdx] = row_center[fixedJointIdx];

    row_center[movingJointIdx] = center[movingJointIdx];
    row_lower[movingJointIdx] = this->controlledJoints[movingJointIdx].lower_limit;
    row_upper[movingJointIdx] = this->controlledJoints[movingJointIdx].upper_limit;

    this->listOfDesiredPositions.push_back(desiredPositions(row_center,this->static_pose_period,false,desiredPositions::ROW_START));
    this->listOfDesiredPositions.push_back(desiredPositions(row_lower,this->static_pose_period,false,desiredPositions::ROW_IN));
    this->listOfDesiredPositions.push_back(desiredPositions(row_upper,this->static_pose_period,false,desiredPositions::ROW_IN));
    this->listOfDesiredPositions.push_back(desiredPositions(row_center,this->static_pose_period,false,desiredPositions::ROW_STOP));
    if( withReturn )
    {
        this->listOfDesiredPositions.push_back(desiredPositions(center,this->return_point_waiting_period,flagReturn,desiredPositions::ROW_OUT));
    }

    return true;
}

bool reachRandomJointPositionsModule::latchTimestampSync()
{
    //Open State_ext:o port and connect to anonymous port
    std::string part_name = this->controlledJoints[0].part_name;
    yarp::os::BufferedPort<yarp::os::Bottle> aReader;
    aReader.open("/"+this->moduleName+"/"+part_name+"/latch:i");
    std::string remotePort= "/" + robotName + "/" + part_name + "/stateExt:o";
    if(!Network::connect(remotePort, aReader.getName()))
    {
        std::cerr << "[ERR] Could not connect to remote source port " + remotePort + "!";
        return false;
    }

    //Read and latch remote timestamp
    yarp::os::Stamp remoteTimestamp;
    if( !aReader.read() )
    {
        std::cerr << "[ERR] Could not read remote source port " + remotePort + "!";
        return false;
    }
    aReader.getEnvelope(remoteTimestamp);
    this->remoteLatchedTimestamp = remoteTimestamp.getTime();

    //Read and latch local current (arbitrary) timestamp
    this->localLatchedTimestamp = Time::now();

    //Disconnect and close port
    Network::disconnect(remotePort, aReader.getName());
    aReader.close();

    return true;
}

