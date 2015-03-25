#include "ManagerModule.h"
#include "ManagerThread.h"

#include <yarp/os/Time.h>

using yarp::os::Time;


namespace codyco {

    ManagerModule::ManagerModule():thr(0) {}
    ManagerModule::~ManagerModule()
    {
        if (thr) {
            delete thr;
            thr = 0;
        }
    }

    bool ManagerModule::configure(yarp::os::ResourceFinder &rf)
    {
        Time::turboBoost();

        thr = new ManagerThread(getName().c_str(), rf);
        if (!thr) return false;
        if (!thr->start())
        {
            delete thr;
            thr = 0;
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

        return true;
    }

    bool ManagerModule::close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        if (thr) {
            thr->stop();
            delete thr;
            thr = 0;
        }

        return true;
    }

    double ManagerModule::getPeriod()    { return 1.0;  }
    bool   ManagerModule::updateModule() { return true; }

    bool ManagerModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        if (command.get(0).asString()=="start") {
            reply.addString("Enabling grasping");
            thr->enableGrasping();
        } else if (command.get(0).asString()=="stop") {
            reply.addString("Disabling grasping");
            thr->disableGrasping();
        }
        return true;
    }

}
