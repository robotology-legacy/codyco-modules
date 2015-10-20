// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "matlabTorqueBalancingTest.h"

#include <rtf/dll/Plugin.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <vector>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;


// prepare the plugin
PREPARE_PLUGIN(matlabTorqueBalancingTest)

matlabTorqueBalancingTest::matlabTorqueBalancingTest() : YarpTestCase("matlabTorqueBalancingTest")
{
}

matlabTorqueBalancingTest::~matlabTorqueBalancingTest()
{
}

bool matlabTorqueBalancingTest::setup(yarp::os::Property &property)
{
    std::cout << "Waiting for gazebo " << std::endl;
    yarp::os::Time::delay(10);

    // initialization goes here ...
    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    if(property.check("duration"))
    {
        duration = property.find("duration").asDouble();
    }
    else
    {
        duration = 100;
    }

    if(property.check("duration"))
    {
        pollingStep = property.find("pollingStep").asDouble();
    }
    else
    {
        pollingStep = 5;
    }

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

void matlabTorqueBalancingTest::tearDown() {
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

void matlabTorqueBalancingTest::run() {

    // Allocate vector for control modes
    int nrOfAxes;

    RTF_ASSERT_ERROR_IF(iEncs->getAxes(&nrOfAxes),
                        "error in reading number of axes");

    RTF_ASSERT_ERROR_IF(nrOfAxes > 0,
                        "error, negative or zero number of axes");

    std::vector<int> controlModes(nrOfAxes);

    RTF_ASSERT_ERROR_IF(ctrlMode->getControlModes(controlModes.data()),
                        "error in getControlModes");

    double initialTimeStamp = yarp::os::Time::now();
    double elapsedTime = 0.0;
    bool testSuccess = false;
    do
    {
        yarp::os::Time::delay(pollingStep);

        RTF_TEST_REPORT("Check control mode");

        if(!isEqual(controlModes.data(),controlModes.size(),VOCAB_CM_TORQUE) )
        {
            if( !testSuccess )
            {
                RTF_TEST_REPORT("Leg still not in position mode");
            }
            else // if testSuccess
            {
                RTF_TEST_REPORT("Leg switched back from torque mode, test failed");
                testSuccess = false;
                break;
            }
        }
        else
        {
            RTF_TEST_REPORT("Low-level controller switched to torque, balancing running!");
            testSuccess = true;
        }

       elapsedTime = yarp::os::Time::now() - initialTimeStamp;
    } while( elapsedTime < duration );

    RTF_ASSERT_ERROR_IF(testSuccess,
                        "matlabTorqueBalancingTest failed, low-level controller do not switched to torque"
                        " or switched back to position control");
}

