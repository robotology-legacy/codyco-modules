// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[]) 
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    bool isSimulator = false;

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    if (params.check("isSimulator")) {
        isSimulator = params.find("isSimulator").asBool();
    }
    
    std::string robotName=params.find("robot").asString().c_str();
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/right_leg";

    std::string localPorts="/codyco_deliv";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IControlMode *controlMode;
    IOpenLoopControl *pwm;
    ITorqueControl *torque;

    bool ok;
    ok = robotDevice.view(controlMode);
    if (isSimulator) {
        ok = ok && robotDevice.view(torque);
    }
    else {
        ok = ok && robotDevice.view(pwm);
    }
    

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int hip = 0, knee = 3;

    if (isSimulator) {
        controlMode->setTorqueMode(hip);
        controlMode->setTorqueMode(knee);

        torque->setRefTorque(hip, 0);
        torque->setRefTorque(knee, 0);
    }
    else {
        controlMode->setOpenLoopMode(hip);
        controlMode->setOpenLoopMode(knee);

        pwm->setOutput(hip, 0);
        pwm->setOutput(knee, 0);
    }
    robotDevice.close();
    
    return 0;
}
