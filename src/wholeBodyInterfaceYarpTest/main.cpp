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
    wholeBodyInterface *icub = new icubWholeBodyInterface("wbiTest", "icubSim");
    icub->addJoints(LocalIdList(RIGHT_ARM,0,1,2,3,4));
    icub->addJoints(LocalIdList(LEFT_ARM,0,1,2,3,4));
    icub->addJoints(LocalIdList(TORSO,0,1,2));
    icub->addFTsens(LocalId(RIGHT_LEG,1));

    if(!icub->init())
        return 0;
    Time::delay(0.5);
    
    int dof = icub->getDoFs();
    printf("Joint list: %s\n", icub->getJointList().toString().c_str());
    printf("Number of DoFs: %d\n", dof);
    
    Vector q(dof), dq(dof), d2q(dof);
    icub->getQ(q.data());
    printf("Q:   %s\n", q.toString(1).c_str());
    
    Vector refSpeed(dof, 10.0), qd = q;
    qd += 30.0;
    icub->setReferenceSpeed(refSpeed.data());
    icub->setPosRef(qd.data());
    int j = 0;
    for(int i=0; i<30; i++)
    {
        Time::delay(0.1);
        icub->getQ(q.data());
        icub->getDq(dq.data());
        icub->getD2q(d2q.data());
        printf("(Q, dq, d2q):   %.2f \t %.2f \t %.2f\n", q(j), dq(j), d2q(j));
    }
    
    qd -= 30.0;
    icub->setPosRef(qd.data());
    Time::delay(1.0);
    printf("Test finished. Press return to exit.");
    getchar();
    
    icub->close();
    printf("Main returning...\n");
    return 0;
}


