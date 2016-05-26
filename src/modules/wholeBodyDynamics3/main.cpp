/* 
 * Copyright (C) 2016 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
 
A module that uses the wholeBodyDynamics device
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU LGPL v2.1+.

\section intro_sec Description
 
This module allows to run the wholeBodyDynamics device outside
of the yarprobotinterface.
 
\section lib_sec Dependencies
- YARP libraries. 
- The \ref iDynTree library.
- The \ref wholeBodyDynamics device.
 
\section usage_sec Usage 
Follow this steps: 
-# Launch the simulation.
-# Launch the \ref wholeBodyDynamics3 module.

\section parameters_sec Parameters
 
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



