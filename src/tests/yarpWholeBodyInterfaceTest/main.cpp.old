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
#include <kdl_format_io/urdf_import.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>

#include <wbiIcub/yarpWholeBodyInterface.h>
#include <wbiIcub/icubWholeBodySensors.h>
#include <wbi/wbi.h>

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
//    Vector refSpeed(dof, CTRL_DEG2RAD*10.0), qd = q;
//    qd += 15.0*CTRL_DEG2RAD;
    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
//    printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());
//    double_pendulum->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
    
    dq.zero();
    d2q.zero();
    //    double_pendulum->setControlReference(qd.data());
    //
    //    int j = 0;
    //
    //    for(int i=0; i<30; i++)
    //    {
    //        Vector com(7,0.0);
    //        wbi::Frame world2base;
    //        world2base.identity();
    //
    //        Time::delay(1);
    //        double_pendulum_sensor->readSensors(SENSOR_ENCODER, q.data());
    //        printf("(Q):   %.2f \n", CTRL_RAD2DEG*q(j));
    //
    //        yarp::sig::Matrix mass_matrix(6+dof,6+dof);
    //        mass_matrix.zero();
    //
    //        wbi::Frame id = wbi::Frame::identity();
    //        double_pendulum->computeMassMatrix(q.data(),id,mass_matrix.data());
    //
    //        std::cout << "Joint mass matrix: " << std::endl;
    //        std::cout << mass_matrix.submatrix(6,6+dof-1,6,6+dof-1).toString() << std::endl;
    //    }
    //
    //    printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
    //
    //    qd -= CTRL_DEG2RAD*15.0;
    //    double_pendulum->setControlReference(qd.data());
    
    assert(dof == 2);
    

    
    //parameters
    const double m1  = 1;
    const double m2  = 1;
    const double l1  = 0.5;
    const double l2  = 0.5;
    const double a1  = 1;
    const double Il1 = 1.0; //inertia like in the URDF model, w.r.t COM
    const double Il2 = 1.0; //inertia like in the URDF model, w.r.t COM
    
    double tau1 = 0;
    double tau2 = 0;
    
    double readTau1 = 0;
    double readTau2 = 0;
    
    double grav1 = 0;
    double grav2 = 0;
    
    double cH = 0;
    double cTerm1 = 0;
    double cTerm2 = 0;
    
    double m11 = 0;
    double m12 = 0;
    double m22 = 0;
    
    double q1 = q(0);
    double q2 = q(1);
    double dq1 = 0;
    double dq2 = 0;
    
    const double g = 9.8;
    
    
    dq(0) = dq1 = 55;
    dq(1) = dq2 = -65.3;
    
    
    grav1 = (m1 * l1 + m2 * a1) * g * cos(-q1) + m2 * l2 * g * cos(-q1 - q2);
    grav2 = m2 * l2 * g * cos(-q1 - q2);
    
    cH = -m2 * a1 * l2 * sin(-q2);
    cTerm1 = cH * -dq2 * -dq1 + cH * (-dq1 - dq2) * -dq2;
    cTerm2 = -cH * -dq1 * -dq1;
    
    m11 = Il1 + m1 * l1 * l1 + Il2 + m2 * (a1 * a1 + l2 * l2 + 2 * a1* l2 * cos(-q2));
    m12 = Il2 + m2 * (l2 * l2 + a1* l2 * cos(-q2));
    m22 = Il2 + m2 * l2 * l2;
    
    yarp::sig::Matrix M(6 + dof, 6 + dof);
//    M(0,0) = m11;
//    M(0,1) = M(1,0) = m12;
//    M(1,1) = m22;
    
    yarp::sig::Vector Cdq(6 + dof);
//    Cdq(0) = cTerm1;
//    Cdq(1) = cTerm2;
    
    yarp::sig::Vector grav(dof + 6);
//    grav(0) = grav1;
//    grav(1) = grav2;
    yarp::sig::Vector tempTau(dof + 6);
    yarp::sig::Vector zeroVec(dof);
    zeroVec.zero();
    
    Frame xBase = Frame::identity();
    //try the base to coicide with the origin of the upper link
    xBase.p[2] = 0;
    double baseVelocity[6] = {0, 0, 0, 0, 0, 0};
    double baseAcc[6] = {0, 0, 0, 0, 0, 0};
    double gravity[3] = {0, 0, -g};
    
    if (!double_pendulum->inverseDynamics(q.data(), xBase, zeroVec.data(), baseVelocity, zeroVec.data(), baseAcc, gravity, grav.data())) {
        fprintf(stderr, "Inverse Dynamics failed \n");
    }
    
    std::cout << "Obtained gravity " << grav.toString() << std::endl;
    
    if (grav1 != grav(6 + 0)) {
        fprintf(stderr, "Gravity is %lf but should be %lf\n", grav(6), grav1);
    }
    if(grav2 != grav(6 + 1))
    {
        fprintf(stderr, "Gravity is %lf but should be %lf\n", grav(7), grav2);
    }
    
    //Calculate gravity terms also with KDL
    KDL::Tree my_tree;

    

    kdl_format_io::treeFromUrdfFile(urdfFile,my_tree);
    KDL::Vector g_kdl(0,0,-g);
    KDL::CoDyCo::TreeIdSolver_RNE id_solver(my_tree);
    
    KDL::CoDyCo::TreeInertialParametersRegressor regres(my_tree);
    
    KDL::JntArray qq(2),dqq(2), ddqq(2), torques_kdl(2);
    SetToZero(dqq);
    SetToZero(ddqq);
    KDL::CoDyCo::Wrenches f(3,KDL::Wrench::Zero());
    qq(0) = q[0];
    qq(1) = q[1];
    
    KDL::Wrench f_base;
    KDL::Twist a,v;
    a = v = KDL::Twist::Zero();
    
    a.vel[2] = -g;
    
    assert(my_tree.getNrOfJoints() == 2);
    int ret = id_solver.CartToJnt(qq,dqq,ddqq,v,a,f,torques_kdl,f_base);
    
    if( ret == 0 ) {
        std::cout << "Obtained gravity with kdl" << torques_kdl.data[0] << " " << torques_kdl.data[1] << std::endl;
    } else {
        std::cout << "Gravity compensation with kdl failed " << ret << std::endl;
    }
    
    std::cout << "With q " << q.toString() << std::endl;

    VectorXd inertial_param = regres.getInertialParameters();
    
    std::cout << "Inertial parameters: " << inertial_param << std::endl;
    
    if(!double_pendulum->inverseDynamics(q.data(), xBase, dq.data(), baseVelocity, zeroVec.data(), baseAcc, gravity, tempTau.data())) {
            fprintf(stderr, "Inverse Dynamics failed \n");
    }
    Cdq = tempTau - grav;
    if (cTerm1 != Cdq(6 + 0)) {
        fprintf(stderr, "C term is %lf but should be %lf\n", Cdq(6), cTerm1);
    }
    if(cTerm2 != Cdq(6 + 1))
    {
        fprintf(stderr, "C Term is %lf but should be %lf\n", Cdq(7), cTerm2);
    }
    
    if(!double_pendulum->computeMassMatrix(q.data(), xBase, M.data())) {
        fprintf(stderr, "Mass Matrix failed \n");
    }
    if(m11 != M(6 + 0,6 + 0))
    {
        fprintf(stderr, "Mass matrix term 0,0 is %lf but should be %lf\n", M(6 + 0,6 + 0), m11);
    }
    if(m12 != M(6 + 0, 6 + 1))
    {
        fprintf(stderr, "Mass matrix term 0,1 is %lf but should be %lf\n", M(6 + 1,6 + 0), m12);
    }
    if(m12 != M(6 + 1, 6 + 0))
    {
        fprintf(stderr, "Mass matrix term 1,0 is %lf but should be %lf\n", M(6 + 0,6 + 1), m12);
    }
    if(m22 != M(6 + 1, 6 + 1))
    {
    fprintf(stderr, "Mass matrix term 1,1 is %lf but should be %lf\n",M(6 + 1,6 + 1), m22);
    }
    

    
    Time::delay(1.0);
    printf("Test finished..");
    
    double_pendulum_sensor->close();
    double_pendulum->close();
    
    delete double_pendulum_sensor;
    delete double_pendulum;
    
    printf("Main returning...\n");
    return 0;
}


