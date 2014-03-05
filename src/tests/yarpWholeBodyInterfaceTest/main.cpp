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

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>

#include <wbiIcub/yarpWholeBodyInterface.h>
#include <wbiIcub/icubWholeBodySensors.h>
#include <wbi/wbi.h>

#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace Eigen;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarp; 
    Property options;
    options.fromCommand(argc,argv);
    
    std::string robotName;
    if(options.check("robot")) {
      robotName = options.find("robot").asString();
    } else {
      robotName = "double_pendulum";
    }
    
    std::string urdfFile;
    if(options.check("urdf")) {
        urdfFile = options.find("urdf").asString();
    } else {
        std::cerr << "Error: --urdf option not specified" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string paramFile;
    if(options.check("from")) {
        paramFile = options.find("from").asString();
    } else {
        std::cerr << "Error: --from option not specified" << std::endl;
        return EXIT_FAILURE;
    }
    
    Property wbi_options;
    wbi_options.fromConfigFile(paramFile.c_str());
    
    // TEST WHOLE BODY INTERFACE
    std::string localName = "wbiTest";
    std::cout << "Creating yarpWholeBodyInterface with robotName " << robotName << " " << localName << std::endl;
    wholeBodyInterface *double_pendulum = new yarpWholeBodyInterface(localName.c_str(),robotName.c_str(),urdfFile.c_str(),wbi_options);
    //wholeBodyStates still not ready, we use directly the wholeBodySensor
    iWholeBodySensors * double_pendulum_sensor = new icubWholeBodySensors(localName.c_str(),robotName.c_str(),wbi_options);
    
    std::cout << "yarpWholeBodyInterface created, adding joints" << std::endl;
    //(the pendulum has only one body part whose ID is 0
    double_pendulum->addJoints(LocalIdList(0,0,1));
    double_pendulum_sensor->addSensors(wbi::SENSOR_ENCODER,LocalIdList(0,0,1));
    std::cout << "Joints added, calling init method" <<  std::endl;

    if(!double_pendulum->init())
        return -1;
    
    if(!double_pendulum_sensor->init())
        return -1;
    
    Time::delay(0.5);
    
    int dof = double_pendulum->getDoFs();
    printf("Joint list: %s\n", double_pendulum->getJointList().toString().c_str());
    printf("Number of DoFs: %d\n", dof);
    
    Vector q(dof), dq(dof), d2q(dof);
    double_pendulum_sensor->readSensors(wbi::SENSOR_ENCODER, q.data(),0,true);
    Vector refSpeed(dof, CTRL_DEG2RAD*10.0), qd = q;
    qd += 15.0*CTRL_DEG2RAD;
    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
    printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());
    double_pendulum->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
    double_pendulum->setControlReference(qd.data());
    
    int j = 0;

    for(int i=0; i<30; i++)
    {
        Vector com(7,0.0);
        wbi::Frame world2base;
        world2base.identity();
        
        Time::delay(1);
        double_pendulum_sensor->readSensors(SENSOR_ENCODER, q.data());
        printf("(Q):   %.2f \n", CTRL_RAD2DEG*q(j));
        
        yarp::sig::Matrix mass_matrix(6+dof,6+dof);
        mass_matrix.zero();
        
        wbi::Frame id = wbi::Frame::identity();
        double_pendulum->computeMassMatrix(q.data(),id,mass_matrix.data());
        
        std::cout << "Joint mass matrix: " << std::endl;
        std::cout << mass_matrix.submatrix(6,6+dof-1,6,6+dof-1).toString() << std::endl;
    }
    
    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());

    qd -= CTRL_DEG2RAD*15.0;
    double_pendulum->setControlReference(qd.data());

    Time::delay(1.0);
    printf("Test finished..");
    
    double_pendulum->close();
    double_pendulum_sensor->close();
    
    delete double_pendulum;
    delete double_pendulum_sensor;
    
    printf("Main returning...\n");
    return 0;
}


