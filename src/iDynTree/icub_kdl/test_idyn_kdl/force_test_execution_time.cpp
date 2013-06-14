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


double g = 9.8;

class ManyDofDyn : public iDynLimb
{
    public:
    ManyDofDyn();
};

ManyDofDyn::ManyDofDyn()
{
    allocate(""); 
    pushLink(new iDynLink(0.189,	 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(0.179,	-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.884,	  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.525,	-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.213,	  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
     pushLink(new iDynLink(0.189,	 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(0.179,	-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.884,	  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.525,	-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.213,	  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
     pushLink(new iDynLink(0.189,	 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(0.179,	-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.884,	  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.525,	-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.213,	  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.189,	 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(0.179,	-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.884,	  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.525,	-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.213,	  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));

}




int main()
{
    Vector Fend(3); Fend.zero();
    Vector Muend(Fend);

    Vector w0(3); Vector dw0(3); Vector ddp0(3);
    w0 = dw0=ddp0= 3.8; ddp0[1]=g;
    
    ManyDofDyn Chain_iDyn;

    int N = Chain_iDyn.getN();

    
    Vector q(N);
    Vector dq(N);
    Vector ddq(N);
    
    
    Random rng;
    rng.seed(yarp::os::Time::now());
    for(int i=0;i<N-1;i++) 
    {
            q[i] = 0*CTRL_DEG2RAD*rng.uniform();
            dq[i] = 0*CTRL_DEG2RAD*rng.uniform();
            ddq[i] = 0*CTRL_DEG2RAD*rng.uniform();
    }
    
    
    for(int i=N-1;i<N;i++) 
    {
            q[i] = 0;
            dq[i] = 0;
            ddq[i] = 1;
    }
    
    int nj=N;
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);
    
    
    int N_TRIALS = 10000;
    double time_idyn = 0.0;
    double time_kdl = 0.0;
    double tic = 0.0;
    double toc = 0.0;
    
    
    Chain_iDyn.prepareNewtonEuler(DYNAMIC);
        
    KDL::Chain kdlChain;
    
    idynChain2kdlChain(Chain_iDyn,kdlChain);
    

    /* 
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);
    */ 
    
    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    KDL::Wrenches f_ext(nj);
    KDL::Wrenches f_int(nj);
    
    KDL::Vector grav_kdl;
    idynVector2kdlVector(Chain_iDyn.getH0().submatrix(0,2,0,2).transposed()*ddp0,grav_kdl);
    
    KDL::ChainIdSolver_RNE_IW neSolver = KDL::ChainIdSolver_RNE_IW(kdlChain,-grav_kdl);

    for(int trial=0; trial < N_TRIALS; trial++ ) {
    
        q = Chain_iDyn.setAng(q);
        dq = Chain_iDyn.setDAng(dq);
        ddq = Chain_iDyn.setD2Ang(ddq);
        
        for(unsigned int i=0;i<nj;i++){
            jointpositions(i)=q[i];
            jointvel(i) = dq[i];
            jointacc(i) = ddq[i];
        }
        tic = yarp::os::Time::now();
        Chain_iDyn.computeNewtonEuler(w0,dw0,ddp0,Fend,Muend);
        toc = yarp::os::Time::now();
        Matrix F = Chain_iDyn.getForces();
        time_idyn += (toc-tic);
        
        tic = yarp::os::Time::now();
        //YARP_ASSERT( neSolver.CartToJnt_and_internal_wrenches(jointpositions,jointvel,jointacc,f_ext,torques,f_int) >= 0);
        toc = yarp::os::Time::now();
        time_kdl += (toc-tic);
    }
        
        
    cout << "Total time for KDL:" << time_kdl/N_TRIALS << endl;
    cout << "Total time for iDyn:" << time_idyn/N_TRIALS << endl;
    
}



