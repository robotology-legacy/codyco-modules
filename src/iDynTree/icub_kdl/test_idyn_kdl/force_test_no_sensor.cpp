#include <iostream>

#include <iCub/iDyn/iDyn.h>
#include <yarp/sig/all.h>
#include <yarp/math/api.h>
#include <yarp/os/Log.h>
#include <yarp/os/all.h>
#include <yarp/os/Time.h>

#include "chain_conversion.h"

#include <kdl/chainfksolver.hpp>
#include "custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <cassert>
#include <cmath>

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-12;
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

class iDynRandomLimb : public iDynLimb
{
    public:
    iDynRandomLimb();
};

Random rng;

iDynRandomLimb::iDynRandomLimb()
{
    allocate(""); 
    for(int l=0;l < 3;l++) {   
        pushLink(new iDynLink(rng.uniform(),					rng.uniform(),		  rng.uniform(),		 rng.uniform(),				rng.uniform(),			rng.uniform(),			rng.uniform(),			rng.uniform(),			rng.uniform(),			rng.uniform(),	    rng.uniform(),        rng.uniform(),             rng.uniform(),            rng.uniform(),        0,  360.0*CTRL_DEG2RAD));
    }
}

int main()
{
    rng.seed(yarp::os::Time::now());
    // we create a iDyn::iCubArmDyn = arm+torso
    // if you are familiar with iKin, it is exactly the same limb type, with the same kinematic properties..
    // .. but it also has dynamics ^_^
	iCubArmDyn  armTorsoDyn("right");
    //iDynRandomLimb armTorsoDyn;

	// by default in iCubArmDyn the torso is blocked, as it is in its corresponding iKin limb, iCubArm;
    // we unblock it, so we can also use the torso links
    // note: releasing the links we have N==DOF
	armTorsoDyn.releaseLink(0);
	armTorsoDyn.releaseLink(1);
	armTorsoDyn.releaseLink(2);

	// we prepare the necessary variables for our computations
    // 1) the joint angles (pos, vel, acc) of the limbs: we can take this values from the encoders..
    int N = armTorsoDyn.getN();
    
    Vector q(N); q.zero();
    Vector dq(q); Vector ddq(q);
    
   
    for(int i=0;i<N;i++) 
    {
            q[i] = 180*CTRL_DEG2RAD*rng.uniform();
            dq[i] = 180*CTRL_DEG2RAD*rng.uniform();
            ddq[i] = 180*CTRL_DEG2RAD*rng.uniform();
    }
    
    // 2) the inertial information to initialize the kinematic phase
    // this information must be set at the base of the chain, where the base is intended to be the first link (first.. in the Denavit)
	Vector w0(3); Vector dw0(3); Vector ddp0(3);
	w0=dw0=ddp0=0.0; ddp0[2]=9.8;
    
    // 3) the end-effector external wrench (force/moment), to initialize the wrench phase
    //    the end-effector wrench is zero by default, if we are not touching anything..
    Vector Fend(3); Fend.zero();
    Vector Muend(Fend);

    // now we set the joints values
    // note: if the joint angles exceed the joint limits, their value is saturated to the min/max automatically
    q = armTorsoDyn.setAng(q);
    dq = armTorsoDyn.setDAng(dq);
    ddq = armTorsoDyn.setD2Ang(ddq);

    // we can start using iDyn :)
    armTorsoDyn.prepareNewtonEuler(DYNAMIC);

    // then we perform the computations
    // 1) Kinematic Phase
    // 2) Wrench Phase
    // the computeNewtonEuler() methoddo everything autonomously!
    armTorsoDyn.computeNewtonEuler(w0,dw0,ddp0,Fend,Muend);
	
    // then we can retrieve our results...
    // forces moments and torques
    Matrix F = armTorsoDyn.getForces();
    Matrix Mu = armTorsoDyn.getMoments();
    Vector Tau = armTorsoDyn.getTorques();
    
    printMatrix("F",F);
    printMatrix("Mu",Mu);
    printVector("Tau",Tau);
    

	Matrix F_KDL = idynChainGetForces_usingKDL( *(armTorsoDyn.asChain()),ddp0);
    Matrix Mu_KDL = idynChainGetMoments_usingKDL( *(armTorsoDyn.asChain()),ddp0);
    Vector Tau_KDL = idynChainGetTorques_usingKDL(*(armTorsoDyn.asChain()),ddp0);
    
    printMatrix("F_KDL",F_KDL);
    printMatrix("Mu_KDL",Mu_KDL);
    printVector("Tau_KDL",Tau_KDL);
    
    for(int l=0;l<F_KDL.cols();l++)
    {
			/*
			cout << " l = " << l << endl;
			cout << norm(F_KDL.getCol(l)) << endl;
			cout << norm(F.getCol(l)) << endl;
            */
            YARP_ASSERT(EQUALISH(F_KDL.getCol(l),F.getCol(l)));
            YARP_ASSERT(EQUALISH(Mu_KDL.getCol(l),Mu.getCol(l)));
    
    }
    
    for(int l=0;l<Tau_KDL.size();l++)
    {
        double a,b;
        a = Tau_KDL(l);
        b = Tau(l);
        YARP_ASSERT( abs(a - b) < delta);
    }
    
    return 0;
    
}


