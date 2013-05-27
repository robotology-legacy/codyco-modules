#include <iostream>

#include <iCub/iDyn/iDyn.h>
#include <yarp/sig/all.h>
#include <yarp/math/api.h>
#include <yarp/os/Log.h>
#include <yarp/os/all.h>
#include <yarp/os/Time.h>

#include "../iDyn_KDL_conversion/iDyn2KDL.h"
#include "../iDyn_KDL_conversion/KDL2iDyn.h"
#include "../icub_model/idyn2kdl_icub.h"

#include "iDyn_KDL_emulation.h"


#include <kdl/chainfksolver.hpp>
#include "custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <kdl_codyco/treecomsolver.hpp>
#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>

#include <kdl_urdf/kdl_export.hpp>
#include <kdl_urdf/kdl_import.hpp>


#include <cassert>

#define EQUALISH(x,y) norm(x-y) < delta

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-5;



KDL::JntArray clean_vec(KDL::JntArray & in)
{
    KDL::JntArray out = in;
    for(int j =0; j < out.rows(); j++) {
        out(j) = out(j) < 1e-15 ? 0 : out(j); 
    }
    return out;
} 


int main(int argc, char * argv[])
{
    iCub::iDyn::version_tag icub_type;
    
    icub_type.head_version = 2;
    if( argc == 1 ) {
        icub_type.legs_version = 1;    
    } else {
        icub_type.legs_version = atoi(argv[1]);
    }
    
    
    //Creating iDyn iCub
    iCubWholeBody icub_idyn(icub_type);
    
    

    KDL::Tree icub_kdl;
    KDL::Tree icub_kdl_urdf;
    
    int N_TRIALS = 1000;
    int N = 32;
    
    double time_kdl = 0.0;
    double time_idyn = 0.0;
    double time_kdl_urdf = 0.0;
    double tic = 0.0;
    double toc = 0.0;
    
    
    //Creating KDL iCub
    bool ret = toKDL(icub_idyn,icub_kdl);
    assert(ret);
    
    //Dumping URDF file
    ret = kdl_export::treeToFile("com_test_icub.xml",icub_kdl,"test_icub");
    assert(ret);
    
    //Creating KDL iCub from URDF
    ret = kdl_import::treeFromFile("com_test_icub.xml",icub_kdl_urdf);
    assert(ret); 
    
    //Creating solver
    KDL::TreeIdSolver_RNE tree_solver(icub_kdl);
    KDL::TreeIdSolver_RNE tree_solver_urdf(icub_kdl_urdf,KDL::Vector::Zero(),tree_solver.getSerialization());

    int ns = icub_kdl.getNrOfSegments();
    int nj = icub_kdl.getNrOfJoints();
    
    assert(ns == icub_kdl_urdf.getNrOfSegments());
    assert(nj == icub_kdl_urdf.getNrOfJoints());
    
    Random rng;
    rng.seed(yarp::os::Time::now());
    
    KDL::JntArray q(nj), dq(nj), ddq(nj);
    KDL::Wrenches f_ext(ns);
    KDL::Twist a0,v0;
    
    KDL::JntArray torques(nj), torques_urdf(nj);
    KDL::Wrench f0,f0_urdf;
    
    for(int trial=0; trial < N_TRIALS; trial++ ) {
        KDL::Vector COM_kdl, COM_kdl_urdf;
        Vector COM_kdl_yarp(3), COM_kdl_urdf_yarp(3);
        
        for(int i=0;i<nj; i++ ) {
            q(i) = 2*M_PI*rng.uniform();
            dq(i) = 2*M_PI*rng.uniform();
            ddq(i) = 2*M_PI*rng.uniform();
        }
        
        for(int j=0; j < ns; j++ ) {
            f_ext[j] = KDL::Wrench::Zero();
        }
        
        a0 = KDL::Twist(KDL::Vector(1,0,0),KDL::Vector::Zero());
        v0 = KDL::Twist::Zero();
        
        //Compute inverse dynamics with KDL trought URDF
        tic = yarp::os::Time::now();
        ret = tree_solver_urdf.CartToJnt(q,dq,ddq,v0,a0,f_ext,torques_urdf,f0_urdf);
        toc = yarp::os::Time::now();
        assert(ret == 0);
        time_kdl_urdf += (toc-tic);
        
        //Compute inverse dynamics with KDL
        tic = yarp::os::Time::now();
        int ret;
        ret = tree_solver.CartToJnt(q,dq,ddq,v0,a0,f_ext,torques,f0);
        toc = yarp::os::Time::now();
        assert(ret == 0);
        time_kdl += (toc-tic);
        
        cout << "KDL dyn: " << endl;
        cout << f0 << endl;
        cout << clean_vec(torques) << endl;

        cout << "KDL urdf dyn: " << endl;
        cout << f0_urdf << endl;
        cout << clean_vec(torques_urdf) << endl;
        
    

        cout << "force error " <<  (f0-f0_urdf).force.Norm() << endl;
        cout << "torque error " << (f0-f0_urdf).torque.Norm()  << endl;
        assert((f0-f0_urdf).force.Norm() < delta);
        assert((f0-f0_urdf).torque.Norm() < delta);
    }
    
    
    cout << "Total time for KDL:" << time_kdl/N_TRIALS << endl;
    cout << "Total time for KDL (URDF):" << time_kdl_urdf/N_TRIALS << endl;
 

    
    
}


