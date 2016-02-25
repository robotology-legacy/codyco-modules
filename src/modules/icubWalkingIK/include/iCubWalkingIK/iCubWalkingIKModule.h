#ifndef ICUB_WALK_IK_MODULE_H
#define ICUB_WALK_IK_MODULE_H

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "iCubWalkingIKThread.h"

class iCubWalkingIKModule: public yarp::os::RFModule {
private:
    double                          m_period;
    iCubWalkingIKThread            *thread;
public:
    iCubWalkingIKModule();
    bool configure(yarp::os::ResourceFinder &rf);
    double getPeriod(){ return m_period; }
    bool updateModule();
    bool close();
};

#endif
