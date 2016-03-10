#ifndef SCRIPTMODULE_H
#define SCRIPTMODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Property.h>
#include <iostream>
#include <string.h>

#include "robotDriver.h"
#include "workingThread.h"

class scriptModule: public yarp::os::RFModule
{
protected:
    yarp::os::Port      rpcPort;
    std::string         name;
    std::string         contextPath;
    bool                verbose;
    robotDriver         robot;
    WorkingThread       thread;
    yarp::os::ResourceFinder      rfCopy;
  
public:
    scriptModule();

    bool   configure(yarp::os::ResourceFinder &rf);
    bool   respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
    bool   close();
    double getPeriod()    { return 1.0;  };
    bool   updateModule() { return true; };
};


#endif // SCRIPTMODULE_H
