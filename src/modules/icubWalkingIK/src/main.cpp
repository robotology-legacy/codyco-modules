#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include "iCubWalkingIKModule.h"

int main (int argc, char **argv)
{
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5)) {
        yError("YARP network is not available");
        return -1;
    }

    yarp::os::ResourceFinder rf;

    rf.setVerbose(true);
    rf.setDefaultConfigFile("iCubWalkingIKModule.ini");
    rf.setDefaultContext("iCubWalkingIKModule");
    rf.configure(argc,argv);

    if (rf.check("help")) {
        std::cout << "Parameters of this module: " << std::endl;
        std::cout << "\t--robot                 :[icub] Robot name (icubGazeboSim or icub)" << std::endl;
        std::cout << "\t--rate                  :[10] Module period in ms" << std::endl;
        std::cout << "\t--local                 :Prefix of the port to be opened by this module" << std::endl;
        std::cout << "\t--com-traj-csv          :[comTraj_iCubGenova01] csv file name without extension" << std::endl;
        std::cout << "\t--walking-pattern-csv   :[WalkTrajectories] csv directory with walking trajectory " << std::endl;
        std::cout << "\t--verbose               :[false] Debugging messages are printed when true." << std::endl;
    }

    iCubWalkingIKModule module;
    return module.runModule(rf);
}
