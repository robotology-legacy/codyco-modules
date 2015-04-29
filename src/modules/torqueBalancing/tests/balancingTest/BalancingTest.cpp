// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "BalancingTest.h"

#include <Plugin.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <vector>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;


// prepare the plugin
PREPARE_PLUGIN(BalancingTest)

BalancingTest::BalancingTest() : YarpTestCase("BalancingTest") {
}

BalancingTest::~BalancingTest() { }

bool BalancingTest::setup(yarp::os::Property &property) {

    // initialization goes here ...
    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    duration = property.find("duration").asDouble();
    pollingStep = property.find("pollingStep").asDouble();

    // Open rpc connection to torqueBalancing
    RTF_ASSERT_ERROR_IF(rpcPort.open("/balancingTest/rpc"),
                        "error in opening rpc port");
    RTF_ASSERT_ERROR_IF(Network::connect(rpcPort.getName(),"/torqueBalancing/rpc"),
                        "could not connect to remote port, torqueBalancing module not available");

    // Open right leg controlboard
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/balancingTest/ctrlBoard");                 //local port names
    options.put("remote", "/icubGazeboSim/right_leg");         //where we connect to

    RTF_ASSERT_ERROR_IF(monitorDD.open(options),
                        "error in opening remote_controlboard");

    RTF_ASSERT_ERROR_IF(monitorDD.view(ctrlMode),
                        "error in opening IControlMode2 interface");

    RTF_ASSERT_ERROR_IF(monitorDD.view(iEncs),
                        "error in opening IEncoders interface");

    return true;
}

void BalancingTest::tearDown() {
    Network::disconnect("/balancingTest", rpcPort.getName());
    rpcPort.close();
    monitorDD.close();
}

bool isEqual(int * data, int size, int testValue) {
    for(int i=0; i < size; i++ ) {
        if( data[i] != testValue ) {
            return false;
        }
    }
    return true;
}

void BalancingTest::run() {

    // Allocate vector for control modes
    int nrOfAxes;

    RTF_ASSERT_ERROR_IF(iEncs->getAxes(&nrOfAxes),
                        "error in reading number of axes");

    RTF_ASSERT_ERROR_IF(nrOfAxes > 0,
                        "error, negative or zero number of axes");

    std::vector<int> controlModes(nrOfAxes);

    RTF_ASSERT_ERROR_IF(ctrlMode->getControlModes(controlModes.data()),
                        "error in getControlModes");

    RTF_ASSERT_ERROR_IF(isEqual(controlModes.data(),controlModes.size(),VOCAB_CM_POSITION),
                        "error : robot leg is not in VOCAB_CM_POSITION before starting the controller");

    RTF_TEST_REPORT("Starting balancing");
    Bottle cmd;
    cmd.addString("start");
    Bottle response;
    RTF_ASSERT_ERROR_IF(rpcPort.write(cmd,response),
                        "error in sending start command to torqueBalancing");
    RTF_TEST_REPORT( Asserter::format("Response to command start : %s",response.toString().c_str()));


    double initialTimeStamp = yarp::os::Time::now();
    double elapsedTime = 0.0;
    do
    {
        yarp::os::Time::delay(pollingStep);

        RTF_TEST_REPORT("Check control mode");

        RTF_ASSERT_ERROR_IF(isEqual(controlModes.data(),controlModes.size(),VOCAB_CM_TORQUE),
                        "error : robot leg is not in VOCAB_CM_TORQUE after starting the controller, probably the limit have been reached");

       elapsedTime = yarp::os::Time::now() - initialTimeStamp;
    } while( elapsedTime < duration );

    RTF_TEST_REPORT("Stop balancing");
    yarp::os::Bottle cmdStop;
    cmdStop.addString("stop");
    yarp::os::Bottle responseStop;
    RTF_ASSERT_ERROR_IF(rpcPort.write(cmd,response),
                        "error in sending stop command to torqueBalancing");
    RTF_TEST_REPORT( Asserter::format("Response to command stop : %s",response.toString().c_str()));

    RTF_ASSERT_ERROR_IF(isEqual(controlModes.data(),controlModes.size(),VOCAB_CM_TORQUE),
                        "error : robot leg is not in VOCAB_CM_POSITION after stopping the controller");
}

