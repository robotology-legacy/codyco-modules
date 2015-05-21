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

#include <time.h>


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
        robotName = "double_pendulum";
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
    std::string robotMainJointsList = "ROBOT_MAIN_JOINTS";
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
    
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd esaZero = Eigen::VectorXd::Zero(6);
    wbi::Frame frame = wbi::Frame::identity();
    double grav[3];
    grav[0] = grav[1] = 0;
    grav[2] = -9.81;
    Eigen::VectorXd out = Eigen::VectorXd::Zero(31);
    Eigen::VectorXd first = Eigen::VectorXd::Zero(31);
    
    robotInterface->computeGeneralizedBiasForces(pos.data(), frame, pos.data(), esaZero.data(), grav, first.data());
    
    
    while(1) {
     
        robotInterface->computeGeneralizedBiasForces(pos.data(), frame, pos.data(), esaZero.data(), grav, out.data());
        
        if (first.norm() - out.norm() > 0) {
            std::cerr << "Error\n" << out.transpose() << "\nVS\n" << first.transpose() << "\n\n";
        }
        
	    double delay_in_s = 0.01;
		yarp::os::Time::delay(delay_in_s);
    }
    
    
    
    robotInterface->close();
    delete robotInterface;

    printf("Main returning...\n");
    return 0;
}


