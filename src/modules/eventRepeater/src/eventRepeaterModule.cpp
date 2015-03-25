/*
 * Copyright (C) 2015 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
*/


#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include <iostream>
#include <eventRepeater.h>


class eventRepeaterModule : public eventRepeater, public yarp::os::RFModule {
private:
    yarp::os::Port rpcPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPort;

public:
    // Thrift Interface Implementation
    virtual bool sendEvent(const std::string& event)
    {
        yarp::os::Bottle& b = outputPort.prepare();
        b.clear();
        b.addString(event.c_str());
        outputPort.write();

        return true;
    }

    virtual bool se(const std::string& event)
    {
        return sendEvent(event);
    }

    // RFModule implementation

    bool attach(yarp::os::Port &source)
    {
        return this->yarp().attachAsServer(source);
    }

    bool configure( yarp::os::ResourceFinder &rf )
    {
        std::string moduleName = rf.check("name",
                yarp::os::Value("eventRepeater"),
                "module name (string)").asString().c_str();
        setName(moduleName.c_str());

        std::string slash="/";

        attach(rpcPort);

        // Opening rpc port
        std::string cmdPortName= "/";
        cmdPortName+= getName();
        cmdPortName += "/rpc";
        if (!rpcPort.open(cmdPortName.c_str())) {
            yError() << getName() << ": Unable to open port " << cmdPortName;
            return false;
        }

        // Opening output streaming port
        std::string outputPortName= "/";
        outputPortName+= getName();
        outputPortName += "/events:o";
        if (!outputPort.open(outputPortName.c_str())) {
            yError() << getName() << ": Unable to open port " << outputPortName;
            return false;
        }

        // Send all events
        outputPort.writeStrict();

        return true;
    }

    bool updateModule()
    {
        return true;
    }

    bool close()
    {
      rpcPort.close();
      return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Error: yarp server does not seem available";
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("eventRepeater");
    rf.configure(argc, argv);

    eventRepeaterModule mod;

    return mod.runModule(rf);
}
