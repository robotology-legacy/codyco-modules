/**
* Copyright: 2010-2013 RobotCub Consortium
* Author: Silvio Traversaro, Serena Ivaldi
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/

//
// This test/example is based on the iDyn tutorial with the same name,
// to show the similarities in the API between iDyn and iDynTree (and
// for testing
//
// An example on using both iCubTree and iCubWholeBody to estimate the measurements of the FT sensors
// for all the arms and legs, exploiting the modeled dynamic and the inertial sensor
// measurements.
//
// Author: Silvio Traversaro - <silvio.traversaro@iit.it>
// Author: Serena Ivaldi

#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Time.h>
#include <yarp/os/Random.h>

#include <yarp/math/api.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iCub/iDynTree/iCubTree.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iDynTree;


void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=1.0)
{
    for( int i=0; i < (int)vec.size(); i++ ) {
        vec[i] =  coeff*M_PI*rng.uniform();
    }
}

#define degTOrad (M_PI/180.0)

int main()
{
    ////////////////////////////////////////////////////////////////////
    //// iDyn
    ////////////////////////////////////////////////////////////////////
    double tol = 1e-6;


    // declare an iCubTree
    iCubTree_version_tag version;
    version.head_version = 2;
    version.legs_version = 2;
    version.feet_ft = true;

    //Use the same serialization as used in IDYN_SERIALIZATION
    iCubTree icub_tree(version);

    // just priting some information to see how the class works
    cout<<endl
        <<"iCub has many DOF: "<<endl
        <<" - head      : "<<icub_tree.getNrOfDOFs("head")<<endl
        <<" - left arm  : "<<icub_tree.getNrOfDOFs("left_arm")<<endl
        <<" - right arm : "<<icub_tree.getNrOfDOFs("right_arm")<<endl
        <<" - torso     : "<<icub_tree.getNrOfDOFs("torso")<<endl
        <<" - left leg  : "<<icub_tree.getNrOfDOFs("left_leg")<<endl
        <<" - right leg : "<<icub_tree.getNrOfDOFs("right_leg")<<endl<<endl;


    // here we set the joints position, velocity and acceleration
    // for all the limbs!


    // initialize the head with the kinematic measures retrieved
    // by the inertial sensor on the head
    yarp::sig::Vector ddp0(3,0.0), w0, dw0;
    w0 = dw0 = ddp0;
    ddp0[2] = 9.8;
    yarp::sig::Vector q(icub_tree.getNrOfDOFs(),0.0);
    q.zero();
    yarp::sig::Vector q_arm(7,0.0);
    q_arm[0] = degTOrad*(-30);
    q_arm[1] = degTOrad*30;
    q_arm[3] = degTOrad*45;

    icub_tree.setInertialMeasure(0*w0,0*dw0,ddp0);
    icub_tree.setAng(q);
    icub_tree.setDAng(q);
    icub_tree.setD2Ang(q);

    icub_tree.setAng(q_arm,"right_arm");
    icub_tree.setAng(q_arm,"left_arm");

    std::cout << "q : " << icub_tree.getAng().toString() << std::endl;

    icub_tree.kinematicRNEA();
    icub_tree.dynamicRNEA();

    Vector left_arm_ft(6), right_arm_ft(6), left_leg_ft(6), right_leg_ft(6);
    icub_tree.getSensorMeasurement(0,left_arm_ft);
    icub_tree.getSensorMeasurement(1,right_arm_ft);

    cout<<endl
    <<"Estimate FT sensor measurements on upper body: "<<endl
    <<" left  : "<<left_arm_ft.toString()<<endl
    <<" right : "<<right_arm_ft.toString()<<endl
    <<endl;
    cout<<"Mass: " <<endl
        <<" left : " << norm(left_arm_ft.subVector(0,2)) << " / " << norm(ddp0) << " = "  << norm(left_arm_ft.subVector(0,2))/norm(ddp0) <<endl
        <<" right: " << norm(right_arm_ft.subVector(0,2)) << " / " << norm(ddp0) << " = " <<  norm(right_arm_ft.subVector(0,2))/norm(ddp0)<<endl;

    return EXIT_SUCCESS;
}



