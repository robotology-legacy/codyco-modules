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

#include "module.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;



reachRandomJointPositionsModule::reachRandomJointPositionsModule()
{
    period = 1;

}

void reachRandomJointPositionsModule::close_drivers()
{
    std::map<string,PolyDriver*>::iterator it;
    if(jointInitialized)
    {
        for(int jnt=0; jnt < originalPositions.size(); jnt++ )
        {
            std::string part = controlledJoints[jnt].part_name;
            int axis = controlledJoints[jnt].axis_number;
            pos[part]->setRefSpeed(axis,originalRefSpeeds[jnt]);
            pos[part]->positionMove(axis,originalPositions[jnt]);
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

    boringMode = rf.check("boringMode");
    boringModeInitialized = false;

    if( rf.check("local") )
    {
        moduleName = rf.find("local").asString().c_str();
    }

    static_pose_period = rf.check("static_pose_period",10.0).asDouble();
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
        if( boringMode ) 
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
           fprintf(stderr, "Error in opening %s part\n",remotePort.c_str());
           close_drivers();
           return false;
       }

       bool ok = true;

       IEncoders * new_encs;
       IPositionControl * new_poss;
       IControlLimits * new_lims;
       ok = ok && new_driver->view(new_encs);
       ok = ok && new_driver->view(new_poss);
       ok = ok && new_driver->view(new_lims);

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

bool reachRandomJointPositionsModule::updateModule()
{
    //std::cout << "Update module called " << std::endl;
    //std::cout << "static_pose_period: " << static_pose_period << std::endl;

    bool dones=true;
    for(int jnt=0; jnt < controlledJoints.size(); jnt++ )
    {
        bool done=true;
        std::string part = controlledJoints[jnt].part_name;
        int axis = controlledJoints[jnt].axis_number;
        pos[part]->checkMotionDone(axis,&done);
        dones = dones && done;
    }
    
    if(dones)
    {
        //std::cout << "elapsed_time: " << elapsed_time << std::endl;
        elapsed_time += getPeriod();
        //std::cout << "elapsed_time: " << elapsed_time << std::endl;
    }
    

    
    if( elapsed_time > static_pose_period )
    {
        elapsed_time = 0.0;
        //Set a new position for the controlled joints
        bool boring_overflow = true;
        for(int jnt=0; jnt < controlledJoints.size(); jnt++ )
        {
            std::string part = controlledJoints[jnt].part_name;
            int axis = controlledJoints[jnt].axis_number;
            double low = controlledJoints[jnt].lower_limit;
            double up = controlledJoints[jnt].upper_limit;
            //Set desired position, depending on the mode
            if( !boringMode )
            {
                double des_pos = yarp::os::Random::uniform(low,up);
                commandedPositions[jnt] = des_pos;
            }
            else
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
            std::cout  << "Send new desired position: " << commandedPositions[jnt] << " to joint " << part <<  " " << axis << std::endl;
            pos[part]->positionMove(axis,commandedPositions[jnt]);
        }
        boringModeInitialized = true;
    } 
    else
    {
    }

    return true;
}

double reachRandomJointPositionsModule::getPeriod()
{
    return period;
}
