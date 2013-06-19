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
#include <kdl_codyco/treecomserialsolver.hpp>
#include <kdl_codyco/utils.hpp>

#include <kdl_urdf/kdl_export.hpp>
#include <kdl_urdf/kdl_import.hpp>


#include <cassert>

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-5;
#define EQUALISH(x,y) norm(x-y) < delta



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
    double time_kdl_treegraph = 0.0;
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
    KDL::CoDyCo::TreeCOMSerialSolver com_solver(icub_kdl);
    KDL::CoDyCo::TreeCOMSerialSolver com_solver_urdf(icub_kdl_urdf,com_solver.getSerialization());
    KDL::CoDyCo::TreeCOMSolver com_solver_graph(icub_kdl_urdf,com_solver.getSerialization());

    Random rng;
    rng.seed(yarp::os::Time::now());
    
    Vector q(N);
    KDL::JntArray q_kdl(N);
    
    for(int trial=0; trial < N_TRIALS; trial++ ) {
        KDL::Vector COM_kdl, COM_kdl_urdf, COM_kdl_treegraph;
        Vector COM_kdl_yarp(3), COM_kdl_urdf_yarp(3);
        Vector COM_kdl_treegraph_yarp(3);

        for(int i=0;i<N;i++) 
        {
            q[i] = 0.0*CTRL_DEG2RAD*360*rng.uniform();
        }
        
        q = icub_idyn.setAllPositions(q);
        
        for(int i=0;i<N; i++ ) {
            q_kdl(i) = q[i];
        }
        
        //Compute COM with iDyn
        tic = yarp::os::Time::now();
        icub_idyn.computeCOM();
        toc = yarp::os::Time::now();
        time_idyn += (toc-tic);
        
        
        //Compute COM with KDL
        tic = yarp::os::Time::now();
        int ret;
        ret = com_solver.JntToCOM(q_kdl,COM_kdl);
        toc = yarp::os::Time::now();
        assert(ret == 0);
        time_kdl += (toc-tic);
        
        //Compute COM with KDL trought URDF
        tic = yarp::os::Time::now();
        ret = com_solver_urdf.JntToCOM(q_kdl,COM_kdl_urdf);
        toc = yarp::os::Time::now();
        assert(ret == 0);
        time_kdl_urdf += (toc-tic);
        
        //Compute COM with KDL and TreeGraph
        tic = yarp::os::Time::now();
        ret = com_solver_graph.JntToCOM(q_kdl,COM_kdl_treegraph);
        toc = yarp::os::Time::now();
        assert(ret == 0);
        time_kdl_treegraph += (toc-tic);
        
        //Compare: 
        cout << "iDyn mass: " << endl;
        cout << icub_idyn.whole_mass << endl;
        cout << "iDyn COM: " << icub_idyn.whole_COM.toString() << endl;
        
        cout << "KDL mass: " << endl;
        cout << KDL::CoDyCo::computeMass(icub_kdl) << endl;
        cout << "KDL COM: " << endl;
        cout << COM_kdl << endl;
        
        cout << "KDL urdf mass: " << endl;
        cout << KDL::CoDyCo::computeMass(icub_kdl_urdf) << endl;
        cout << "KDL urdf COM: " << endl;
        cout << COM_kdl_urdf << endl;
        
        cout << "KDL TreeGraph mass: " << endl;
        cout << KDL::CoDyCo::computeMass(icub_kdl_urdf) << endl;
        cout << "KDL TreeGraph COM: " << endl;
        cout << COM_kdl_treegraph << endl;
        
        bool retb;
        retb = to_iDyn(COM_kdl,COM_kdl_yarp);
        assert(retb);
        retb = to_iDyn(COM_kdl_urdf,COM_kdl_urdf_yarp);
        assert(retb);
        retb = to_iDyn(COM_kdl_treegraph,COM_kdl_treegraph_yarp);
        assert(retb);
        
        
        assert(EQUALISH(icub_idyn.whole_COM,COM_kdl_yarp));
        assert(EQUALISH(icub_idyn.whole_COM,COM_kdl_urdf_yarp));        
        assert(EQUALISH(icub_idyn.whole_COM,COM_kdl_treegraph_yarp));


    }
    
    
    cout << "Total time for KDL:" << time_kdl/N_TRIALS << endl;
    cout << "Total time for KDL (URDF):" << time_kdl_urdf/N_TRIALS << endl;
    cout << "Total time for KDL (TreeGraph):" << time_kdl_treegraph/N_TRIALS << endl;
    cout << "Total time for iDyn:" << time_idyn/N_TRIALS << endl;
    cout << "Improvement factor KDL:" << time_idyn/time_kdl << endl;
    cout << "Improvement factor KDL (URDF):" << time_idyn/time_kdl_urdf << endl;
    cout << "Improvement factor KDL (TreeGraph):" << time_idyn/time_kdl_treegraph << endl;

    

    
    
}


