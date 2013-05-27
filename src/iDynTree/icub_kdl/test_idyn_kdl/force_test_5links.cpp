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
#include <kdl/kinfam_io.hpp>

#include <cassert>

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;


double delta = 1e-10;
#define EQUALISH(x,y) norm(x-y) < delta


//Defining the link
double a0 = 1.0;
double a1 = 1.0;
double a2 = 1.0;
double a3 = 1.0;
double a4 = 1.0;
double d0 = 0.0;
double d1 = 0.0;
double d2 = 0.0;
double d3 = 0.0;
double d4 = 0.0;
double alpha0 = 0;
double alpha1 = 0;
double alpha2 = 0;
double alpha3 = 0;
double alpha4 = 0;

double m0 = 1;
double m1 = 1;
double m2 = 1;
double m3 = 1;
double m4 = 1;
double lc0 = -a0/2;
double lc1 = -a1/2;
double lc2 = -a2/2;
double lc3 = -a3/2;
double lc4 = -a4/2;
double Izz0 = 0.0;
double Izz1 = 0.0;
double Izz2 = 0.0;
double Izz3 = 0.0;
double Izz4 = 0.0;

double g = 9.8;

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

class L5PMDyn : public iDynLimb
{
    public:
    L5PMDyn();
};

L5PMDyn::L5PMDyn()
{
    allocate("");   
    pushLink(new iDynLink(m0,					lc0,		  0,		 0,				0,			0,			0,			0,			0,			Izz0,	    a0,        d0,            alpha0,            0.0,        0,  360.0*CTRL_DEG2RAD)); 
    pushLink(new iDynLink(m1,					lc1,		  0,		 0,				0,			0,			0,			0,			0,			Izz1,	    a1,        d1,            alpha1,            0.0,        0,  360.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(m2,					lc2,		  0,		 0,				0,			0,			0,			0,			0,			Izz2,	    a2,        d2,            alpha2,            0.0,        0,  360.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(m3,					lc3,		  0,		 0,				0,			0,			0,			0,			0,			Izz3,	    a3,        d3,            alpha3,            0.0,        0,  360.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(m4,					lc4,		  0,		 0,				0,			0,			0,			0,			0,			Izz4,	    a4,        d4,            alpha4,            0.0,        0,  360.0*CTRL_DEG2RAD));

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
	//now setting inertia, mass and COM specifically for each link
	H.resize(4,4); COM.resize(4,4); I.resize(3,3);
		H.eye();
        /*
        H(0,0) = 0;
        H(0,1) = 1;
        H(1,0) = 1;
        H(1,1) = 0;
        H(2,2) = -1;
        */
        H(2,2) = 1;
        
        H(0,3) = -a2/2; //The sensor is in the middle of the link
		H(3,3) = 1;
        
        COM.eye(); COM(0,3) = a2/4;
		I.zero(); 
        I(2,2) = Izz2/2;
        m = m2/2;
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

    int N = 5;
    
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

    for(int l=0;l<N;l++) {
        Vector p_kdl;
        idynChain2kdlChain(*(threeChain.asChain()),threeChainKDL,l+1);
        KDL::ChainFkSolverPos_recursive posSolver = KDL::ChainFkSolverPos_recursive(threeChainKDL);
        KDL::Frame cartpos;
        KDL::JntArray jpos;
        jpos.resize(l+1);
        for(int p=0;p<jpos.rows();p++) jpos(p) = jointpositions(p);
        posSolver.JntToCart(jpos,cartpos);
        kdlVector2idynVector(cartpos.p,p_kdl);
        p_kdl_no_sens.setCol(l,p_kdl);
    }
        KDL::Chain threeChainKDLsens;
    for(int l=0;l<N+1;l++) {
        Vector p_kdl;
        idynSensorChain2kdlChain(*(threeChain.asChain()),threeChainSensor,threeChainKDLsens,l+1);
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
        kdlVector2idynVector(cartpos.p,p_kdl);
        p_kdl_sens.setCol(l,p_kdl);
    }
    printMatrix("p_idyn",p_idyn);
    printMatrix("p_kdl_no_sens",p_kdl_no_sens);
    printMatrix("p_kdl_sens",p_kdl_sens);
    
    
    
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`KDL Chain~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~``" << endl << threeChainKDL << endl;
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~KDL Chain sens~~~~~~~~~~~~~~~~~~~~~~~~~~~~`" << endl << threeChainKDLsens << endl;
    

    //printKDLchain("threeChainKDL",threeChainKDL);
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
