
#include "Coordinator.h"
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;

int main(int argc, char **argv) {

    Network yarp;
    if (!Network::checkNetwork())
    {
        yError("YARP server not available!");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("codycoTrajGenDemoY2");
    rf.setDefaultConfigFile("default.ini");
    rf.configure(argc,argv);

    codyco::y2::Coordinator module;

    return module.runModule(rf);
}
