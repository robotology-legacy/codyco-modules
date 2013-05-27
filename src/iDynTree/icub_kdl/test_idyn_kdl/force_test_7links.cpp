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


//Defining the link
double a0 = 1.0;
double a1 = 2.0;
double a2 = 4.0;
double a3 = 8.0;
double a4 = 16.0;
double d0 = 1.0;
double d1 = 2.0;
double d2 = 4.0;
double d3 = 8.0;
double d4 = 16.0;
double alpha0 = 1;
double alpha1 = 2;
double alpha2 = 3;
double alpha3 = 4;
double alpha4 = 5;

double m0 = 0.1;
double m1 = 0.2;
double m2 = 0.3;
double m3 = 0.4;
double m4 = 2.0;
double lc0 = -a0/2;
double lc1 = -a1/2;
double lc2 = -a2/2;
double lc3 = -a3/2;
double lc4 = -a4/2;
double Izz0 = 0.2;
double Izz1 = 0.3;
double Izz2 = 0.4;
double Izz3 = 0.5;
double Izz4 = 0.6;

double g = 9.8;

class L5PMDyn : public iDynLimb
{
    public:
    L5PMDyn();
};

L5PMDyn::L5PMDyn()
{
    allocate(""); 
    /*  
    pushLink(new iDynLink(m0,					lc0,		  0,		 0,				0,			0,			0,			0,			0,			Izz0,	    a0,        d0,            alpha0,            0.0,        0,  360.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(m1,					lc1,		  0,		 0,				0,			0,			0,			0,			0,			Izz1,	    a1,        d1,            alpha1,            0.0,        0,  360.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(m2,					lc2,		  0,		 0,				0,			0,			0,			0,			0,			Izz2,	    a2,        d2,            alpha2,            0.0,        0,  360.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(m3,					lc3,		  0,		 0,				0,			0,			0,			0,			0,			Izz3,	    a3,        d3,            alpha3,            0.0,        0,  360.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(m4,					lc4,		  0,		 0,				0,			0,			0,			0,			0,			Izz4,	    a4,        d4,            alpha4,            0.0,        0,  360.0*CTRL_DEG2RAD));
    */
    pushLink(new iDynLink(0.189,	 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(0.179,	-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.884,	  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.525,	-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.213,	  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
}

class L5PMSensorLink : public SensorLinkNewtonEuler
{
public:

	/**
    * Constructor: the sensor parameters are automatically set with "right" or "left" choice
	* @param _type a string "left"/"right" 
	* @param _mode the analysis mode (STATIC/DYNAMIC)
	* @param verb flag for verbosity
    */
	L5PMSensorLink(const std::string &_type, const NewEulMode _mode = DYNAMIC, unsigned int verb = iCub::skinDynLib::NO_VERBOSE);

};

L5PMSensorLink::L5PMSensorLink(const string &_type, const NewEulMode _mode, unsigned int verb)
:SensorLinkNewtonEuler(_mode,verb)
{
    /*
	//now setting inertia, mass and COM specifically for each link
	H.resize(4,4); COM.resize(4,4); I.resize(3,3);
		H.eye();
        H(0,0) = 0;
        H(0,1) = 1;
        H(1,0) = 1;
        H(1,1) = 0;
        
        H(0,3) = -a2/2; //The sensor is in the middle of the link
		H(3,3) = 1;
        
        COM.eye(); COM(0,3) = -a2/4;
		I.zero(); 
        I(2,2) = Izz2/2;
        m = m2/2;
        */
    
		H.zero(); H(0,0) = 1.0; H(2,1) = -1.0; H(1,2) = 1.0; 
        
        //H.eye();
        /*
        H(0,0) = 0.0;
        H(0,1) = 1.0;
        H(1,0) = 1.0;
        H(1,1) = 0.0;
        H(2,2) = 1.0;
        */
        //H(1,3) = -0.08428; H(3,3) = 1.0;
        H(0,3) = -a2/2; //The sensor is in the middle of the link
		H(3,3) = 1;
        
        
        //COM.eye(); COM(0,3) = -1.5906019e-04; COM(1,3) =   8.2873258e-05; COM(2,3) =  2.9882773e-02;
        COM.eye(); COM(0,3) = -a2/4;
        //I.zero(); I(0,0) = 4.08e-04; I(0,1) = I(1,0) = -1.08e-6; I(0,2) = I(2,0) = -2.29e-6;
		//I(1,1) = 3.80e-04; I(1,2) = I(2,1) =  3.57e-6; I(2,2) = 2.60e-4;
        I.zero(); 
        I(2,2) = Izz2/2;
		m = m2/2;
        //m = 7.29e-01;  
    
    R = H.submatrix(0,2,0,2); r = H.subcol(0,3,3); r_proj = r*R;
	RC = COM.submatrix(0,2,0,2); rc = COM.subcol(0,3,3); rc_proj = rc*R;

	//then the sensor information
	info.clear(); info = "mail pegua1@gmail.com";
}

class iDynInvSensorL5PM : public iDynInvSensor
{
public:
	iDynInvSensorL5PM(iDynChain *_c, const std::string _type, const NewEulMode _mode = DYNAMIC, unsigned int verb = iCub::skinDynLib::NO_VERBOSE);
};

iDynInvSensorL5PM::iDynInvSensorL5PM(iDynChain *_c, const string _type, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c,_type,_mode,verb)
{
	// FT sensor is in position 5 in the kinematic chain in both arms
	lSens = 2;

    // set the sensor properly
    sens = new L5PMSensorLink(_type,mode,verbose);
}

int main()
{
    Vector Fend(3); Fend.zero();
    Vector Muend(Fend);

    Vector w0(3); Vector dw0(3); Vector ddp0(3);
    w0 = dw0=ddp0=0.0; ddp0[1]=g;

    int N = 7;
    
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
    
     
     
    
    
    L5PMDyn threeChain;
    
    
    iDynInvSensorL5PM threeChainSensor(threeChain.asChain(),"papare",DYNAMIC,iCub::skinDynLib::VERBOSE);
    int sensor_link = threeChainSensor.getSensorLink();
    
    q = threeChain.setAng(q);
    dq = threeChain.setDAng(dq);
    ddq = threeChain.setD2Ang(ddq);
    
    int nj=N;
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);

    
    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    threeChain.prepareNewtonEuler(DYNAMIC);
    threeChain.computeNewtonEuler(w0,dw0,ddp0,Fend,Muend);
    threeChainSensor.computeSensorForceMoment();


    // then we can retrieve our results...
    // forces moments and torques
    Matrix F = threeChain.getForces();
    Matrix Mu = threeChain.getMoments();
    Vector Tau = threeChain.getTorques();
    
    Matrix F_KDL = idynChainGetForces_usingKDL(threeChain,ddp0);
    Matrix Mu_KDL = idynChainGetMoments_usingKDL(threeChain,ddp0);
    Vector Tau_KDL = idynChainGetTorques_usingKDL(threeChain,ddp0);
    
    Matrix F_KDL_sens = idynChainGetForces_usingKDL(threeChain,threeChainSensor,ddp0);
    Matrix Mu_KDL_sens = idynChainGetMoments_usingKDL(threeChain,threeChainSensor,ddp0);
    Vector Tau_KDL_sens = idynChainGetTorques_usingKDL(threeChain,threeChainSensor,ddp0);
    
    Matrix p_idyn(3,N),p_kdl_no_sens(3,N),p_kdl_sens(3,N+1);
    
    for(int l=0;l<N;l++) 
    {
        p_idyn.setCol(l,threeChain.getH(l).subcol(0,3,3));
    }
    KDL::Chain threeChainKDL;
    
    std::vector<std::string> dummy;
    std::string dummy_str = "";
    
    for(int l=0;l<N;l++) {
        Vector p_kdl;
        idynChain2kdlChain(*(threeChain.asChain()),threeChainKDL,dummy,dummy,dummy_str,dummy_str,l+1);
        KDL::ChainFkSolverPos_recursive posSolver = KDL::ChainFkSolverPos_recursive(threeChainKDL);
        KDL::Frame cartpos;
        KDL::JntArray jpos;
        jpos.resize(l+1);
        for(int p=0;p<jpos.rows();p++) jpos(p) = jointpositions(p);
        posSolver.JntToCart(jpos,cartpos);
        to_iDyn(cartpos.p,p_kdl);
        p_kdl_no_sens.setCol(l,p_kdl);
    }
        KDL::Chain threeChainKDLsens;
    for(int l=0;l<N+1;l++) {
        Vector p_kdl;
        idynSensorChain2kdlChain(*(threeChain.asChain()),threeChainSensor,threeChainKDLsens,dummy,dummy,dummy_str,dummy_str,l+1);
        KDL::ChainFkSolverPos_recursive posSolver = KDL::ChainFkSolverPos_recursive(threeChainKDLsens);
        KDL::Frame cartpos;
        KDL::JntArray jpos;
        if( l <= sensor_link ) { 
            jpos.resize(l+1);
            for(int p=0;p<jpos.rows();p++) jpos(p) = jointpositions(p);
        } 
        //if( l >= sensor_link ) 
        else {
            jpos.resize(l);
            for(int p=0;p<jpos.rows();p++) jpos(p) = jointpositions(p);
        }
        cout << "l " << l << " nrJoints " << threeChainKDLsens.getNrOfJoints() <<  " nrsegments " << threeChainKDLsens.getNrOfSegments() <<" jpos dim " << jpos.rows() << endl;
        assert(posSolver.JntToCart(jpos,cartpos) >= 0);
        to_iDyn(cartpos.p,p_kdl);
        p_kdl_sens.setCol(l,p_kdl);
    }
    printMatrix("p_idyn",p_idyn);
    printMatrix("p_kdl_no_sens",p_kdl_no_sens);
    printMatrix("p_kdl_sens",p_kdl_sens);
    
    
    
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`KDL Chain~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~``" << endl << threeChainKDL << endl;
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~KDL Chain sens~~~~~~~~~~~~~~~~~~~~~~~~~~~~`" << endl << threeChainKDLsens << endl;
    

    //printKDLchain("threeChainKDLsens",threeChainKDLsens);

    printMatrix("F",F);
    printMatrix("F_KDL",F_KDL);
    printMatrix("F_KDL_sens",F_KDL_sens);
    printMatrix("Mu",Mu);
    printMatrix("Mu_KDL",Mu_KDL);
    printMatrix("Mu_KDL_sens",Mu_KDL_sens);
    printVector("Tau",Tau);
    printVector("Tau_KDL",Tau_KDL);
    printVector("Tau_KDL_sens",Tau_KDL_sens);
    
    for(int l=0;l<F_KDL.cols();l++)
    {
            YARP_ASSERT(EQUALISH(F_KDL.getCol(l),F.getCol(l)));
            YARP_ASSERT(EQUALISH(F_KDL_sens.getCol(l),F.getCol(l)));
            YARP_ASSERT(EQUALISH(Mu_KDL.getCol(l),Mu.getCol(l)));
            YARP_ASSERT(EQUALISH(Mu_KDL_sens.getCol(l),Mu.getCol(l)));
    }
    
    for(int l=0;l<Tau.size();l++) {
        YARP_ASSERT(abs(Tau(l)-Tau_KDL(l))<delta);
        YARP_ASSERT(abs(Tau(l)-Tau_KDL_sens(l))<delta);
    }
}
