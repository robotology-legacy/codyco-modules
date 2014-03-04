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

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>

#include <wbiIcub/wholeBodyInterfaceIcub.h>
//#include <extern/eigen_unsupported/Eigen/src/Core/products/GeneralBlockPanelKernel.h>

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
        robotName = "icubSim";
    }

    // TEST WHOLE BODY INTERFACE
    wholeBodyInterface *icub = new icubWholeBodyInterface("wbiTest",robotName.c_str());
    //wholeBodyInterface *icub = new icubWholeBodyInterface("wbiTest","icubSim");
//      icub->addJoints(LocalIdList(RIGHT_ARM,0,1,2,3,4));
//      icub->addJoints(LocalIdList(LEFT_ARM,0,1,2,3,4));
//      icub->addJoints(LocalIdList(TORSO,0,1,2));
//     icub->addJoints(LocalIdList());
    icub->addJoints(ICUB_MAIN_JOINTS);
    //icub->addFTsens(LocalId(RIGHT_LEG,1));

    if(!icub->init())
        return 0;
    Time::delay(0.5);

    int dof = icub->getDoFs();
    printf("Joint list: %s\n", icub->getJointList().toString().c_str());
    printf("Number of DoFs: %d\n", dof);

    Vector q(dof), dq(dof), d2q(dof);
    icub->getEstimates(ESTIMATE_JOINT_POS, q.data());
    Vector qInit = q;
    Vector refSpeed(dof, CTRL_DEG2RAD*10.0), qd = q;
    
    Vector biasQ;
    biasQ.resize(dof, 0.3);
    qInit += 0.3 * qInit + biasQ;
    
//     qd += 15.0*CTRL_DEG2RAD;
    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
//     printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());
    icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
    //icub->setControlReference(qd.data());
    icub->setControlMode(CTRL_MODE_TORQUE);
//     icub->setControlMode(CTRL_MODE_POS);
    int j = 0;
    //Eigen::Matrix<double,6,Dynamic,RowMajor> jacob;
    Frame H_base_leftFoot, Ha, xBase;
    //Matrix4d            H_w2b;                  // rototranslation from world to base reference frame
    
    int LINK_ID_LEFT_FOOT;    
    const char *linkName;
    linkName = "l_sole";
    Ha.R = Rotation(0,0,1, 0,-1,0, 1,0,0);
    
    int linkId;
    
    Vector dxB, d2xB;
    // Assuming null base velocity and accelerations
    dxB.resize(6, 0);
    d2xB.resize(6, 0);
    
    Vector biasForce;
    biasForce.resize(dof+6,0);
    
    Matrix M(dof + 6, dof+6);
    
    //jacob.resize(6,dof+6); //13 because in this test we only have right and left arm plus torso
    icub->getLinkId(linkName, linkId);
    double grav[3];
    grav[0] = 0.0;
    grav[1] = 0.0;
    grav[2] = 9.8;

    Vector com(7,0.0);
    wbi::Frame world2base;
    world2base.identity();
    xBase=Frame::identity();
    
    Matrix Mj(25, 25);
    Vector subV;
    Vector torques;
    Vector invDynTau(dof);
    
    Vector zeroVel;
    zeroVel.resize(dof, 0);
    
    for(int i=0; true; i++)
    {
        icub->getEstimates(ESTIMATE_JOINT_POS, q.data());
        icub->getEstimates(ESTIMATE_JOINT_VEL, dq.data());
        icub->getEstimates(ESTIMATE_JOINT_ACC,d2q.data());
        if(icub->computeGeneralizedBiasForces(q.data(),xBase, zeroVel.data(), dxB.data(), grav, biasForce.data())){
          //fprintf(stderr,"GRAVITY VEC ok\n");
        }
        else{
          fprintf(stderr,"ERROR in computeGeneraliezdBiasForces\n");    
        }
        
        if(icub->computeMassMatrix(q.data(),xBase,M.data())){
            Mj = M.submatrix(6, dof + 5, 6, dof + 5);
        }
        else{
          fprintf(stderr,"ERROR in computeMassMatrix\n");    
        }

        if(icub->inverseDynamics(q.data(), xBase, dq.data(), dxB.data(), d2q.data(), d2xB.data(), grav, invDynTau.data())){
            fprintf(stderr,"inverseDynamics ok\n");
        }
        else{
            fprintf(stderr,"ERROR in inverseDynamics\n");
        }
        
        
        subV =  biasForce.subVector(6, biasForce.size()-1);
        torques = - 1 * Mj * (0.1 * (q - qInit) + 0.1* dq);
        printf("Error norm: %lf\n", yarp::math::norm(q - qInit));
//         torques = -1*Mj* (.1 * (q - qInit) + .1* dq);
        icub->setControlReference(torques.data());

       // Time::delay(0.01);
    }

    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());

//     qd -= CTRL_DEG2RAD*15.0;

     icub->setControlMode(CTRL_MODE_POS);
     icub->setControlReference(qInit.data());

    Time::delay(1.0);
    printf("Test finished. Press return to exit.");
    getchar();

    icub->close();
    delete icub;
    printf("Main returning...\n");
    return 0;
}


