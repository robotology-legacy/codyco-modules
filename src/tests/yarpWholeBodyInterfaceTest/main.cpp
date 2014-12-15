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

// #include <kdl_codyco/treeinertialparameters.hpp>
// #include <../../external/orocos_kdl/python_orocos_kdl/PyKDL/kinfam.sip>

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
    Eigen::Matrix<double, 6, Dynamic, Eigen::RowMajor> jacobian;

    Network yarp;
    yarp::os::ResourceFinder rf;
    rf.setVerbose (true);
    rf.setDefaultConfigFile ("yarpWholeBodyInterface.ini");

    Property options;
    options.fromCommand(argc,argv);

    std::string robotName;
    if(options.check("robot")) {
        robotName = options.find("robot").asString();
    } else {
        fprintf(stderr,"[WARN] No robot name specified. Using default double_pendulum\n");
        robotName = "icubGazeboSim";
    }

    rf.configure (argc,argv);

    Property yarpWbiOptions;

    printf("Finding configuration file\n");
    std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
    yarpWbiOptions.fromConfigFile(wbiConfFile);
    printf("Configuration  file in: %s", wbiConfFile.c_str());

    // Create new yarpWholeBodyInterface object
    std::string moduleName = "yarpWholeBodyInterface";
    yarpWholeBodyInterface* robotInterface;
    robotInterface = new yarpWholeBodyInterface (moduleName.c_str(), yarpWbiOptions);

    // Add joints to the robotInterface
    IDList robotMainJoints;
    yarp::os::Value wbi_id_list = yarpWbiOptions.find("wbi_id_list");
    std::string robotMainJointsList = wbi_id_list.toString();
    if (!loadIdListFromConfig(robotMainJointsList, yarpWbiOptions, robotMainJoints)) {
      printf("[ERR] yarpWholeBodyInterfaceTest: Impossible to load from ID List from Configuration file\n");
      return EXIT_FAILURE;
    }
    robotInterface->addJoints(robotMainJoints);

    if( robotInterface->getJointList().size() != robotMainJoints.size() )
    {
        std::cerr << "ROBOT_DOF from robotInterface->getJointlist(): " << robotInterface->getJointList().size() << std::endl;
        std::cerr << "ROBOT_DOF from robotMainJoints.size(): " << robotMainJoints.size() << std::endl;
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


