#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include "quaternionEKFModule.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace filter;

int main(int argc, char* argv[]) 
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("quaternionEKF");
    rf.setDefaultConfigFile("quaternionEKFModule.ini");
    rf.configure(argc, argv);
    
    if(rf.check("help")) {
        yInfo() << "\tParameters";
        yInfo() << "\t--from            :[quaternionEKFModule.ini] Name of .ini file for configuration";
        yInfo() << "\t--robot           :[icub] Robot name. Other options: icubGazeboSim";
        yInfo() << "\t--rate            :[10] Thread period (ms)";
        yInfo() << "\t--mode            :[online] or offline";
        yInfo() << "\t--autoconnect     :[true] or false";
        yInfo() << "\t--calib           :[false] The module enters in calibration mode for which it asks you to put the accelerometer at 0 degrees or 90 degrees. Once this orientation is achieved, the user needs to hit ENTER for data to be collected";
        yInfo() << "\t--usingXSens      :[false] When true, the module will an XSens IMU attached to the computer for testing the algorithm.";
        yInfo() << "\t--verbose         :[false] Verbose level.";
        yInfo() << "\t--sensorPortName  :[/icub/left_foot_inertial/analog:o] Specifies the sensor port name to be used.";
        yInfo() << "\t--using2acc       :[false] Using two accelerometers for direct angle measurement from only accelerometers plus low pass filtering of the raw measurements.";
        return 0;
    }
    
    yarp::os::Network yarpNetwork;
    
    if (!yarpNetwork.checkNetwork())
    {
        yError("YARP Network is not available. The module will shut down now...");
        return -1;
    }
    
    quaternionEKFModule module;
    return module.runModule(rf);
}
