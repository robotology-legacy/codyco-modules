/* 
 * Copyright (C) 2016 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Silvio Traversaro
 * email:  so@iit.it
 * website: www.robotcub.org
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

/** 
\defgroup wholeBodyDynamics3 wholeBodyDynamics3
 
@ingroup codyco_module
 
A module that uses the wholeBodyDynamicsDevice with network wrappers.
 
Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 
Author: Silvio Traversaro

CopyPolicy: Released under the terms of the GNU LGPL v2.1+.

\section intro_sec Description
 
This module allows to run the wholeBodyDynamics device outside
of the yarprobotinterface.
 
\section lib_sec Dependencies
- YARP libraries. 
- The \ref iDynTree library.
- The \ref wholeBodyDynamicsDevice device.
 
\section usage_sec Usage 
Follow this steps: 
-# Launch the simulation.
-# Launch the \ref wholeBodyDynamics3 module.

\section parameters_sec Parameters
This is an example configuration file, commented inline.
~~~
# This is the prefix for the yarp port opened by the robots,
# it is mainly used as a parameter for the configuration of the
# network devices specified later in the configuration file,
# for more on this check out http://www.yarp.it/yarp_config_files.html#yarp_config_file_environment
robot           icub
# This is the prefix of all the ports opened by the module
name            wholeBodyDynamics
# This is the list of the devices that will be opened by the module
# The configuration for each device will be searched in the group with the same name
# in this configuration file (for more on "groups", see http://www.yarp.it/yarp_config_files.html#yarp_config_file_lists)
# All this devices will be passed to the `attachAll` method of the device whose configuration specified in the wholeBodyDynamicsDevice parameter.
devices (left_leg,right_leg,right_arm,left_arm,torso,head,torso_vsens,left_leg_vsens,right_leg_vsens,left_arm_vsens,right_arm_vsens,l_arm_ft_sensor,r_arm_ft_sensor,r_foot_ft_sensor,l_foot_ft_sensor,r_leg_ft_sensor,l_leg_ft_sensor,imu)
# Device to spawn and to attach (using the attachAll method) to all the devices specified in the `devices` list.
wholeBodyDynamicsDevice wholeBodyDynamicsDevice.ini

# Controlboards
# This are the controlboard devices, used for reading joint sensors
# (such as encoders) measurements
[left_leg]
device remote_controlboard
local /${name}/left_leg
remote /${robot}/left_leg

[right_leg]
device remote_controlboard
local /${name}/right_leg
remote /${robot}/right_leg

[right_arm]
device remote_controlboard
local /${name}/right_arm
remote /${robot}/right_arm

[left_arm]
device remote_controlboard
local /${name}/left_arm
remote /${robot}/left_arm

[torso]
device remote_controlboard
local /${name}/torso
remote /${robot}/torso

[head]
device remote_controlboard
local /${name}/head
remote /${robot}/head

# Virtual sensors devices
# This are the virtual sensors device, used to provide
# the estimated joint torques to the low level joint torque control
# of the robot (if any).
# If the robot is not designed to read this kind of estimate,
# this device can be removed.
[torso_vsens]
device virtualAnalogClient
local /${name}/torso/Torques:o
remote /${robot}/joint_vsens/torso:i
AxisName (torso_yaw,torso_roll,torso_pitch)
virtualAnalogSensorInteger 4

[left_leg_vsens]
device virtualAnalogClient
local /${name}/left_leg/Torques:o
remote /${robot}/joint_vsens/left_leg:i
AxisName (l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,l_ankle_pitch,l_ankle_roll)
virtualAnalogSensorInteger 2

[right_leg_vsens]
device virtualAnalogClient
local /${name}/right_leg/Torques:o
remote /${robot}/joint_vsens/right_leg:i
AxisName (r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)
virtualAnalogSensorInteger 2

[left_arm_vsens]
device virtualAnalogClient
local /${name}/left_arm/Torques:o
remote /${robot}/joint_vsens/left_arm:i
AxisName (l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,l_wrist_prosup)
virtualAnalogSensorInteger 1

[right_arm_vsens]
device virtualAnalogClient
local /${name}/right_arm/Torques:o
remote /${robot}/joint_vsens/right_arm:i
AxisName (r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,r_wrist_prosup)
virtualAnalogSensorInteger 1

# IMU
# Device to read the measurements of an Inertial Measurement Device
[imu]
device genericSensorClient
local /${name}/imu
remote /${robot}/inertial


# Six Axis Force/Torque sensors
# Devices to read the measurements of Six Axis F/T sensors embedded into the robot
[l_arm_ft_sensor]
device analogsensorclient
local /${name}/l_arm_ft_sensor
remote /${robot}/left_arm/analog:o

[r_arm_ft_sensor]
device analogsensorclient
local /${name}/r_arm_ft_sensor
remote /${robot}/right_arm/analog:o

[l_leg_ft_sensor]
device analogsensorclient
local /${name}/l_leg_ft_sensor
remote /${robot}/left_leg/analog:o

[r_leg_ft_sensor]
device analogsensorclient
local /${name}/r_leg_ft_sensor
remote /${robot}/right_leg/analog:o

[l_foot_ft_sensor]
device analogsensorclient
local /${name}/l_foot_ft_sensor
remote /${robot}/left_foot/analog:o

[r_foot_ft_sensor]
device analogsensorclient
local /${name}/r_foot_ft_sensor
remote /${robot}/right_foot/analog:o
~~~
 
\section tested_os_sec Tested OS
Linux

\author Silvio Traversaro
*/ 


#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/Wrapper.h>

#include <vector>

#include <cstdlib>

using namespace yarp::dev;
using namespace yarp::os;

/************************************************************************/
class wholeBodyDynamics3Module: public RFModule
{
protected:
    PolyDriver wholeBodyDynamicsDevice;
    IMultipleWrapper* iwrap;
    PolyDriverList usedDevices;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        // Check required data
        if( !rf.check("wholeBodyDynamicsDevice")
            || !rf.check("devices") || !(rf.find("devices").isList()) )
        {
            yError() << "wholeBodyDynamics3 : missing required parameters wholeBodyDynamicsDevice and devices";
            return false;
        }


        // Open the wholeBodyDynamics device, then get all other devices and
        // pass them to the attachAll method
        Property wholeBodyDynamicsDeviceOptions;
        wholeBodyDynamicsDeviceOptions.fromConfigFile(rf.findFile("wholeBodyDynamicsDevice"));


        // Open all devices
        bool ok = wholeBodyDynamicsDevice.open(wholeBodyDynamicsDeviceOptions);
        ok = ok && wholeBodyDynamicsDevice.view(iwrap);

        if( !ok )
        {
            yError() << "wholeBodyDynamics3 : error in open wholeBodyDynamicsDevice";
            return false;
        }

        Bottle * devices = rf.find("devices").asList();

        for(int dev=0; dev < devices->size(); dev++)
        {
            std::string devKey = devices->get(dev).asString();

            // Open the property
            PolyDriver * devPD = new PolyDriver();

            bool ok = devPD->open(rf.findGroup(devKey));

            if( !ok || !(devPD->isValid()) )
            {
                yError() << "wholeBodyDynamics3 : error in opening device " << devKey;
                this->close();
                return false;
            }

            // Add to PolyDriverList
            usedDevices.push(devPD,devKey.c_str());
        }

        // Attach the sensors devices to the wholeBodyDynamics
        ok = iwrap->attachAll(usedDevices);

        if( !ok )
        {
            yError() << "wholeBodyDynamics3 : error in attachAll";
            return false;
        }

        return true;
    }


    /************************************************************************/
    bool close()
    {
        // Call detach all from wholeBodyDynamics
        iwrap->detachAll();

        // Close wholeBodyDynamicsDevice
        wholeBodyDynamicsDevice.close();

        // Close all other devices
        for(int dev = 0; dev < usedDevices.size(); dev++)
        {
            if( usedDevices[dev]->poly )
            {
                usedDevices[dev]->poly->close();
                delete usedDevices[dev]->poly;
                usedDevices[dev]->poly = 0;
            }
        }

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP server not available!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("wholeBodyDynamics3");
    rf.setDefaultConfigFile("wholeBodyDynamics3.ini");         //default config file name.
    rf.configure(argc,argv);

    wholeBodyDynamics3Module mod;
    return mod.runModule(rf);
}



