/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2007 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for wholeBodyInterfaceYarp.
 */
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>

#include <wbiy/wbiy.h>

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
using namespace wbiy;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarp; 
    
    // TEST WHOLE BODY INTERFACE
    wholeBodyInterface *icub = new icubWholeBodyInterface("testSensInt", "icubSim");
    icub->addJoints(LocalIdList(RIGHT_ARM,0,1,2,3,4));
    icub->addJoints(LocalIdList(LEFT_ARM,0,1,2,3,4));
    icub->addJoints(LocalIdList(TORSO,0,1,2));
    if(!icub->init())
        return 0;
    
    int dof = icub->getDoFs();
    Vector q(dof);
    printf("DoF of the interface: %d\n", dof);
    icub->getQ(q.data());
    printf("Q: %s\n", q.toString(1).c_str());
    
    printf("Main returning...\n");
    return 0;
}


