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
    vector<string> bodyPartNames(BodyPart_s, BodyPart_s + sizeof(BodyPart_s) / sizeof(string) );
    iWholeBodySensors *is = new yarpWholeBodySensors("testSensInt", "icubSim", bodyPartNames);
    is->addJoint(LocalId(LEFT_ARM, 0));
    vector<int> jList(2);
    jList[0] = 1;
    jList[1] = 3;
    is->addJoints(LocalIdList(LEFT_ARM, jList));
    if(!is->init())
    {
        printf("Error while initializing sensor interface.\n");
    }
    printf("DoF = %d\n", is->getDoFs());
    
    wholeBodyInterface *icub = new icubWholeBodyInterface("testSensInt", "icubSim");
    
    printf("Main returning...\n");
    return 0;
}


