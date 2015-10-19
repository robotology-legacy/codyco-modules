#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <iomanip>

#include "WholeBodyEstimatorModule.h"

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char* argv[]) 
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("wholeBodyEstimator");
    rf.setDefaultConfigFile("wholeBodyEstimator.ini");
    rf.configure(argc, argv);
    
    if(rf.check("help")) {
        printf("\n");
        printf("\PARAMETERS\n");
        printf("--period          :[10] Thread period in miliseconds\n.");
        printf("--from            :[wholeBodyEstimator.ini] Name of .ini file for configuration\n");
        printf("--robot           :[icub] Robot name. Other options: icubGazeboSim\n");
        printf("--rate            :[10] Thread period (ms)\n");
        printf("--local           :[quaternionEKFModule] Module name used to prepend ports opened by this module\n");
        printf("--verbose         :[false] When true, prints many debugging messages.\n");
        return 0;
    }
    
    yarp::os::Network yarpNetwork;
    
    double timeout = 10.0;
    if (!yarpNetwork.checkNetwork(timeout))
    {
        yError("YARP Network is not available. The module will shut down now...");
        // Standardized exit code
        return 1;
    }
    
    WholeBodyEstimatorModule module;
    return module.runModule(rf);
}
