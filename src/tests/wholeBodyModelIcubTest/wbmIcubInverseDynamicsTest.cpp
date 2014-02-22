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

#include <wbiIcub/wholeBodyInterfaceIcub.h>

#include <wbi/wbiUtil.h>

#include <stdio.h>
#include <math.h>
#include <string>
#include <cstdlib>

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

/**
 * Do a consistency test on a wholeBodyModel interface. Nothing of implementation specific, 
 * so eventually we could move this function to wbi to provided a consistency check for an arbitrary implementation
 * 
 */
bool checkInverseDynamicsAndMassMatrixConsistency(iWholeBodyModel * model_interface, const LocalIdList & possible_joints, double tol, bool verbose)
{
    int nr_of_possible_joints = 0;
    FOR_ALL_OF(itBp, itJ, possible_joints) {  
        model_interface->removeJoint(LocalId(itBp->first,*itJ));
        nr_of_possible_joints++;
    }
    
    //Select the random number of considered joints
    int nr_of_considered_joints = rand() % nr_of_possible_joints;
    int nr_of_activated_joints = 0;
    
    double threshold = ((double)nr_of_considered_joints)/((double)nr_of_possible_joints);
    
    //Select the random considered joints
    while(  nr_of_activated_joints < nr_of_considered_joints ) {
        FOR_ALL_OF(itBp, itJ, possible_joints) {  
            if( nr_of_activated_joints < nr_of_considered_joints && yarp::math::Rand::scalar() < threshold ) {
                if( model_interface->addJoint(LocalId(itBp->first,*itJ)) ) {
                    nr_of_activated_joints++;
                }
            }
        }
    }
    
    assert(nr_of_activated_joints == nr_of_considered_joints);
    assert(nr_of_considered_joints == model_interface->getJointList().size());
   
    //std::cout << "checkInverseDynamicsAndMassMatrixConsistency: nrOfPossibleJoints : " << nr_of_possible_joints << " nrOfConsiderJoints " << nr_of_considered_joints << " " << nr_of_activated_joints << std::endl;
    
    //Select the random input
    
    wbi::Frame xB(wbi::Rotation::RPY(Rand::scalar(),Rand::scalar(),Rand::scalar()));
    
    //wbi::Frame xB;//(wbi::Rotation::rotX(M_PI));
    
    xB.p[0] = Rand::scalar();
    xB.p[1] = Rand::scalar();
    xB.p[2] = Rand::scalar();
    
    yarp::sig::Vector dxB = 2*M_PI*yarp::math::Rand::vector(6);
    yarp::sig::Vector ddxB = 2*M_PI*yarp::math::Rand::vector(6);
    yarp::sig::Vector theta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);
    yarp::sig::Vector dtheta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);
    yarp::sig::Vector ddtheta = 2*M_PI*yarp::math::Rand::vector(nr_of_considered_joints);
    
    yarp::sig::Vector ddq = cat(ddxB,ddtheta);
    
    //Create the outputs
    yarp::sig::Vector generalized_torques(6+nr_of_considered_joints);
    yarp::sig::Vector generalized_torques_computed_with_mass_matrix(6+nr_of_considered_joints);
    yarp::sig::Vector generalized_bias_torques(6+nr_of_considered_joints);
    yarp::sig::Matrix mass_matrix(6+nr_of_considered_joints,6+nr_of_considered_joints);
    
    if( !model_interface->inverseDynamics(theta.data(),xB,dtheta.data(),dxB.data(),ddtheta.data(),ddxB.data(),generalized_torques.data()) ) { 
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: inverseDynamics failed" << std::endl; }
        return false; 
    }
    if( !model_interface->computeMassMatrix(theta.data(),xB,mass_matrix.data()) ) { 
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: computeMassMatrix failed" << std::endl; }
        return false; 
    }
    if( !model_interface->computeGeneralizedBiasForces(theta.data(),xB,dtheta.data(),dxB.data(),generalized_bias_torques.data()) ) {
        if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: computeGeneralizedBiasForces failed" << std::endl; }
        return false; 
    }
    
    /*
    std::cout << "Mass Matrix:             " << std::endl << mass_matrix.toString() << std::endl;
    std::cout << "ddq:                     " << std::endl << ddq.toString() << std::endl;
    std::cout << "M*ddq                    " << std::endl << (mass_matrix*ddq).toString() << std::endl;
    std::cout << "M*ddq with inv dyn       " << std::endl << (generalized_torques-generalized_bias_torques).toString() << std::endl;
    std::cout << "bias:                    " << std::endl << generalized_bias_torques.toString() << std::endl;
    */
    //std::cout << "invDyn:                  " << std::endl << generalized_torques.toString() << std::endl;
    
    generalized_torques_computed_with_mass_matrix = mass_matrix*ddq + generalized_bias_torques;
    //std::cout << "invDyn with mass matrix: " << std::endl << generalized_torques_computed_with_mass_matrix.toString() << std::endl;

    
    for(int i = 0; i < generalized_torques.size(); i++ ) {
        if( fabs(generalized_torques[i]-generalized_torques_computed_with_mass_matrix[i]) > tol ) { 
            if( verbose ) { std::cout << "checkInverseDynamicsAndMassMatrixConsistency: generalized torque " << i << " is different, failing" << std::endl; }
            return false;
        }
    }
    
    return true;
}

int main(int argc, char * argv[])
{
    Property options;
    options.fromCommand(argc,argv);
    
    int n_checks = 1000;
    if( options.check("n_checks") ) {
        n_checks = options.find("n_checks").asInt();
    }
  
    // TEST WHOLE BODY MODEL
    std::string localName = "wbiTest";
    std::string robotName = "icub";
    std::cout << "Creating icubWholeBodyModel with robotName " << robotName << " and localName " << localName << std::endl;
    iWholeBodyModel *icub = new icubWholeBodyModel(localName.c_str(),robotName.c_str());
    
    Rand::init();
    for(int i = 0; i < n_checks; i++ ) {
        if( i % 100 == 0 ) { std::cout << "wholeBodyModelIcub inverse dynamics : test " << i << std::endl; }
        if( ! checkInverseDynamicsAndMassMatrixConsistency(icub,ICUB_MAIN_DYNAMIC_JOINTS,TOL,true) ) {
            return EXIT_FAILURE;
        }
    }
    
    
    return EXIT_SUCCESS;
}

