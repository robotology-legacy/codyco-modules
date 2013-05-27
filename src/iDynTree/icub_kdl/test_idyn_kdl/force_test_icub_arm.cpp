#include <iostream>

#include <iCub/iDyn/iDyn.h>
#include <yarp/sig/all.h>
#include <yarp/math/api.h>
#include <yarp/os/Log.h>
#include <yarp/os/all.h>
#include <yarp/os/Time.h>

#include "../iDyn_KDL_conversion/iDyn2KDL.h"
#include "../iDyn_KDL_conversion/KDL2iDyn.h"
#include "iDyn_KDL_emulation.h"


#include <kdl/chainfksolver.hpp>
#include "custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <cassert>

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-10;
#define EQUALISH(x,y) norm(x-y) < delta


void printMatrix(string s,const Matrix &m)
{
	cout<<s<<endl;
	for(int i=0;i<m.rows();i++)
	{
		for(int j=0;j<m.cols();j++)
			cout << " " <<m(i,j);
		cout<<endl;
	}
}

int main()
{
	//Declaring a tipical iCub arm iDyn structure
	iCubArmNoTorsoDyn armNoTorsoDyn("right");
    iDynSensorArmNoTorso iCubArmSensor(&armNoTorsoDyn,DYNAMIC,iCub::skinDynLib::VERBOSE);
    int N = armNoTorsoDyn.getN();
    
    Vector q(N); q.zero();
    Vector dq(q); Vector ddq(q);
    

    //Assign a random state (but with no external forces) to the icub arm
    Random rng;
    rng.seed(yarp::os::Time::now());
    for(int i=0;i<N;i++) 
    {
            q[i] = 180*CTRL_DEG2RAD*rng.uniform();
            dq[i] = 180*CTRL_DEG2RAD*rng.uniform();
            ddq[i] = 180*CTRL_DEG2RAD*rng.uniform();
    }
   	Vector w0(3); Vector dw0(3); Vector ddp0(3);
   	//KDL RNE solver is not supporting moving base
	w0=dw0=0.0; 
	//But initial gravity is equivalent to an arbitrary base linear acceleration
	ddp0[0] = rng.uniform();
	ddp0[1] = rng.uniform();
	ddp0[2] = rng.uniform();
    
    
    Vector Fend(3); Fend.zero();
    Vector Muend(Fend);

	//Making sure that q, dq, ddq are inside the limits
    q = armNoTorsoDyn.setAng(q);
    dq = armNoTorsoDyn.setDAng(dq);
    ddq = armNoTorsoDyn.setD2Ang(ddq);

    //Calculating the Dynamics using iDyn, and getting wrenches and torques
    armNoTorsoDyn.prepareNewtonEuler(DYNAMIC);
    armNoTorsoDyn.computeNewtonEuler(w0,dw0,ddp0,Fend,Muend);
    Matrix F = armNoTorsoDyn.getForces();
    Matrix Mu = armNoTorsoDyn.getMoments();
    Vector Tau = armNoTorsoDyn.getTorques();
    
	cout << "Results of the dynamics calculation using iDyn" << endl;
    printMatrix("F",F);
    printMatrix("Mu",Mu);
    printVector("tau",Tau);
    
    //Calculating the sensor dynamics using iDyn, and getting the sensor wrench
    iCubArmSensor.computeSensorForceMoment();
    Vector Fsens = iCubArmSensor.getSensorForce();
    Vector Musens = iCubArmSensor.getSensorMoment(); 

    int sensor_link = iCubArmSensor.getSensorLink();
    
    /*
    //Converting the chains from iDyn to KDL
    KDL::Chain armTorsoKDL;
    idynSensorChain2kdlChain(*(armNoTorsoDyn.asChain()),iCubArmSensor,armTorsoKDL);
    KDL::Vector grav_kdl;
    idynVector2kdlVector(ddp0,grav_kdl);
    
    KDL::ChainIdSolver_RNE_IW neSolver = KDL::ChainIdSolver_RNE_IW(armTorsoKDL,grav_kdl);
    unsigned int nj = armTorsoKDL.getNrOfJoints();
    assert(nj == N);
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);

    //Coping the same state of iDyn chain to the KDL chain
    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    KDL::Wrenches f_ext(nj+1);
    KDL::Wrenches f_int(nj+1);
    
    //Executing a modified version of the KDL RNE solver, to expose internal wrenches
    //assert(neSolver.CartToJnt_and_internal_wrenches(jointpositions,jointvel,jointacc,f_ext,torques,f_int) >= 0);
    
    //Vector Tau_KDL = Vector(nj);
    Vector f_i = Vector(3);
    Vector mu_i = Vector(3);
    
    */
    
    //Simulate getForces,getMoments and getTorques calls using KDL, all the necessary conversion are done inside this functions
    Matrix F_KDL = idynChainGetForces_usingKDL(armNoTorsoDyn,ddp0);
    Matrix Mu_KDL = idynChainGetMoments_usingKDL(armNoTorsoDyn,ddp0);
    Vector Tau_KDL = idynChainGetTorques_usingKDL(armNoTorsoDyn,ddp0);
    
    
	//Simulate getForces,getMoments and getTorques calls using KDL, all the necessary conversion are done inside this functions
	//This version uses a different KDL chain model, specifically with two links and a fixed joint in the middle to simulate the sensor measurment
    Matrix F_KDL_sens = idynChainGetForces_usingKDL(armNoTorsoDyn,iCubArmSensor,ddp0);
    Matrix Mu_KDL_sens = idynChainGetMoments_usingKDL(armNoTorsoDyn,iCubArmSensor,ddp0);
    Vector Tau_KDL_sens = idynChainGetTorques_usingKDL(armNoTorsoDyn,iCubArmSensor,ddp0);
    
    //In this way, it is possible to simulate also getSensorForce and getSensorMoment calls
    Vector Fsens_KDL_sens = idynSensorGetSensorForce_usingKDL(armNoTorsoDyn,iCubArmSensor,ddp0);
    Vector Musens_KDL_sens = idynSensorGetSensorMoment_usingKDL(armNoTorsoDyn,iCubArmSensor,ddp0); 
    
    cout << "~~~~~~ Dynamical quantities calculated using KDL ~~~~~~" << endl;
    printVector("Tau_KDL",Tau_KDL);
    printVector("Tau_KDL_sens",Tau_KDL_sens);
    printMatrix("F_KDL_sens",F_KDL_sens);
    printMatrix("Mu_KDL_sens",Mu_KDL_sens);
        
    printVector("Fsens",Fsens);
    printVector("Musens",Musens);
    printVector("Fsens_KDL_sens",Fsens_KDL_sens);
    printVector("Musens_KDL_sens",Musens_KDL_sens);
    
    cout << "~~~~~~ Difference between sensor wrench calculated in iDyn and KDL ~~~~~~" << endl;
    cout << "Diff in norm Fsens " << norm(Fsens-Fsens_KDL_sens) << endl;
    cout << "Diff in norm Musens " << norm(Musens-Musens_KDL_sens) << endl;

	cout << "Assert equal results up to a precision of " << delta << endl;
    for(int l=0;l<F_KDL.cols();l++)
    {
            YARP_ASSERT(EQUALISH(F_KDL.getCol(l),F.getCol(l)));
            YARP_ASSERT(EQUALISH(F_KDL_sens.getCol(l),F.getCol(l)));
            YARP_ASSERT(EQUALISH(Mu_KDL.getCol(l),Mu.getCol(l)));
            YARP_ASSERT(EQUALISH(Mu_KDL_sens.getCol(l),Mu.getCol(l)));
    }
    return 0;
    
}


