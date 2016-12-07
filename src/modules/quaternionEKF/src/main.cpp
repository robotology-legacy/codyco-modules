#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <iomanip>

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
        printf("\n");
        printf("PARAMETERS\n");
        printf("--from            :[quaternionEKFModule.ini] Name of .ini file for configuration\n");
        printf("--robot           :[icub] Robot name. Other options: icubSim\n");
        printf("--rate            :[10] Thread period (ms)\n");
        printf("--local           :[quaternionEKFModule] Module name used to prepend ports opened by this module\n");
        printf("--calib           :[false] The module enters in calibration mode for which it asks you \n\
                                  to put the accelerometer at 0 degrees or 90 degrees. Once this \n\
                                  orientation is achieved, the user needs to hit ENTER for data to be collected\n");
        printf("--autoconnect     :[true] or false\n");
        printf("--mode            :[online] or offline\n");
        printf("--usingXSens      :[false] When true, the module will an XSens IMU attached to the \n\
                                  computer (USB) for testing the algorithm.\n");
        printf("--usingEKF        :[true] When true, the module will use a quaternion-based Extended \n\
                                  Kalman Filter approach to estimate orientation of the world ref.\n\
                                  frame in the sensor reference frame based on gyrscope and \n\
                                  accelerometer readings.\n");
	printf("--inWorldRefFrame :[false] When true, the estimation published in /[local]/filteredOrientationEuler:o\n\
                                  is expressed in the Earth reference frame. When [false] in the sensor\n\
                                  reference frame.\n");
        printf("--usingSkin       :[true] (using2acc=false, usingXSens=false, mode=online) When true,\n\
                                  it is assumed that the right foot of the robot has a palm skin \n\
                                  patch attached in order to use its gyroscope, plus the accelerometer \n\
                                  in the MTB  board of the right foot.\n");
        printf("--using2acc       :[false] Using two accelerometers for direct angle measurement from \n\
                                  only accelerometers plus low pass filtering of the raw measurements.\n");
        printf("--debugGyro       :[false] When true, the module opens a port called /[local]/rawGyroMeas:o \n\
                                  streaming the parsed gyroscope measurement from the MTB port.\n");
        printf("--debugAcc        :[false] When true, the module opens a port called /[local]/rawAccMeas:o \n\
                                  streaming the parsed accelerometer measurement from the MTB port.\n");
        printf("--verbose         :[false] When true, prints many debugging messages.\n");
        printf("--sensorPortName  :[/icub/right_leg/inertialMTB] Specifies the sensor port name to be used.\n");
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
