/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for wholeBodyInterfaceYarp.
 */
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>
#include <kdl_format_io/urdf_import.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include <wbi/wbi.h>

#include <yarp/os/LogStream.h>

#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

#include <kdl_codyco/treeinertialparameters.hpp>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace Eigen;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarp;
    yarp::os::ResourceFinder rf;
    rf.setVerbose (true);
    rf.setDefaultConfigFile ("wbi_conf_file.ini");
    rf.setDefaultContext ("icubGazeboSim");

    rf.configure (argc,argv);

    Property yarpWbiOptions;

    printf("Finding configuration file\n");
    std::string wbiConfFile = rf.findFile("wbi_conf_file.ini");
    yarpWbiOptions.fromConfigFile(wbiConfFile);
    printf("Configuration file in: %s", wbiConfFile.c_str());

    // Create new yarpWholeBodyInterface object
    std::string moduleName = "yarpWholeBodyInterface";
    yarpWholeBodyInterface* robotInterface;
    robotInterface = new yarpWholeBodyInterface (moduleName.c_str(), yarpWbiOptions);

    // Add joints to the robotInterface
    IDList robotMainJoints;
    std::string robotMainJointsList = "ICUB_DYNAMIC_MODEL_JOINTS";
    if (!loadIdListFromConfig(robotMainJointsList, yarpWbiOptions, robotMainJoints)) {
      printf("[ERR] yarpWholeBodyInterfaceTest: Impossible to load from ID List from Configuration file\n");
      return EXIT_FAILURE;
    }
    robotInterface->addJoints(robotMainJoints);

    if( robotInterface->getJointList().size() != robotMainJointsList.size() )
    {
        std::cerr << " robotInterface->getJointList() : " << std::endl;
        std::cerr << robotInterface->getJointList().toString() << std::endl;
        std::cerr << " is different from robotMainJoints : " << std::endl;
        std::cerr << robotMainJoints.toString() << std::endl;
    }

    // Initialize interface
    if (!robotInterface->init()) {
      printf("[ERR] Whole body interface object did not initialize correctly\n");
      return EXIT_FAILURE;
    }

    robotInterface->setControlMode(CTRL_MODE_POS);

    Time::delay (0.5);

    // Get robot DOF
    int DOF = robotInterface->getDoFs();

    // Initialize variables
    Vector q(DOF), dq(DOF), d2q(DOF);

    // Get robot estimates

    robotInterface->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data(), -1.0, false);

    printf("Joint Angles: %s \n", q.toString().c_str());
    // Set control mode
    if (!robotInterface->setControlMode(wbi::CTRL_MODE_DIRECT_POSITION)) {
      printf("[ERR] Position control mode could not be set\n");
      return EXIT_FAILURE;
    }



    robotInterface->close();
    delete robotInterface;

    printf("Main returning...\n");
    return 0;
}


