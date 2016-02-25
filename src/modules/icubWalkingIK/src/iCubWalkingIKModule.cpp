#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include "iCubWalkingIKModule.h"


using namespace yarp::os;
using namespace yarpWbi;

iCubWalkingIKModule::iCubWalkingIKModule() {}

bool iCubWalkingIKModule::configure(ResourceFinder &rf) {
    return true;
}

bool iCubWalkingIKModule::updateModule() {
    return true;
}

bool iCubWalkingIKModule::close() {
    return true;
}
