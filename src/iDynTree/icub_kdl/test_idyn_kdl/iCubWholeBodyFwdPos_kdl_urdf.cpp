#include <iostream>

#include <iCub/iDyn/iDyn.h>

#include <iCub/iKin/iKinFwd.h>


#include <yarp/sig/all.h>
#include <yarp/math/api.h>
#include <yarp/os/Log.h>
#include <yarp/os/all.h>
#include <yarp/os/Time.h>

#include "../iDyn_KDL_conversion/iDyn2KDL.h"
#include "../iDyn_KDL_conversion/KDL2iDyn.h"
#include "../icub_model/idyn2kdl_icub.h"

#include "iDyn_KDL_emulation.h"

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>


#include <kdl_codyco/treecomsolver.hpp>
#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/treefksolverpos_iterative.hpp>

#include <kdl_urdf/kdl_export.hpp>
#include <kdl_urdf/kdl_import.hpp>


#include <cassert>

#define EQUALISH(x,y) norm(x-y) < delta

using namespace std;
using namespace iCub::iDyn;
using namespace iCub::iKin;

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-10;



KDL::JntArray clean_vec(KDL::JntArray & in)
{
    KDL::JntArray out = in;
    for(int j =0; j < out.rows(); j++) {
        out(j) = out(j) < 1e-15 ? 0 : out(j); 
    }
    return out;
} 

void compare(KDL::CoDyCo::TreeFkSolverPos_iterative & tree_solver, KDL::CoDyCo::TreeFkSolverPos_iterative & tree_solver_urdf,
             KDL::JntArray q, std::string link_name)
{
    KDL::Frame pos_kdl, pos_kdl_urdf;
    int ret;
    //Compute forward position with KDL
    ret = tree_solver.JntToCart(q,pos_kdl,link_name);
    assert(ret == 0);
    
    //Compute forward position with KDL trought URDF
    ret = tree_solver_urdf.JntToCart(q,pos_kdl_urdf,link_name);
    assert(ret == 0);
    
    cout << "KDL " << link_name << endl;
    cout << pos_kdl << endl;
    
    cout << "KDL urdf " << link_name << endl;
    cout << pos_kdl_urdf << endl;
}

void compare(KDL::CoDyCo::TreeFkSolverPos_iterative & tree_solver, KDL::CoDyCo::TreeFkSolverPos_iterative & tree_solver_urdf,
             KDL::JntArray q, std::string link_name_parent, std::string link_name_child)
{
    KDL::Frame pos_kdl, pos_kdl_urdf;
    int ret;
    //Compute forward position with KDL
    ret = tree_solver.setBaseLink(link_name_parent);
    assert(ret == 0);
    ret = tree_solver.JntToCart(q,pos_kdl,link_name_child);
    assert(ret == 0);
    ret = tree_solver.setBaseLink("root_link");
    assert(ret == 0);
    
    //Compute forward position with KDL trought URDF
    ret = tree_solver_urdf.setBaseLink(link_name_parent);
    assert(ret == 0);
    ret = tree_solver_urdf.JntToCart(q,pos_kdl_urdf,link_name_child);
    assert(ret == 0);
    ret = tree_solver_urdf.setBaseLink("root_link");
    assert(ret == 0);
    
    cout << "KDL " << link_name_parent << " " << link_name_child << endl;
    cout << pos_kdl << endl;
    
    cout << "KDL urdf " << link_name_parent << " " << link_name_child  << endl;
    cout << pos_kdl_urdf << endl;
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
    
    int N_TRIALS = 1;
  
    double time_kdl = 0.0;
    double time_idyn = 0.0;
    double time_kdl_urdf = 0.0;
    double time_kdl_old = 0.0;
    double tic = 0.0;
    double toc = 0.0;
    
    
    //Creating KDL iCub
    bool ret = toKDL(icub_idyn,icub_kdl,true);
    assert(ret);
    
    //Dumping URDF file
    ret = kdl_export::treeToFile("com_test_icub.xml",icub_kdl,"test_icub");
    assert(ret);
    
    //Creating KDL iCub from URDF
    ret = kdl_import::treeFromFile("com_test_icub.xml",icub_kdl_urdf);
    assert(ret); 
    
    int ns = icub_kdl.getNrOfSegments();
    int nj = icub_kdl.getNrOfJoints();
    
    
    //Creating solver
    KDL::CoDyCo::TreeFkSolverPos_iterative tree_solver(icub_kdl);
    //std::cout << "Default serialization " << std::endl;
    //std::cout << tree_solver.getTreeGraph().getSerialization().toString() << std::endl;
    KDL::CoDyCo::TreeFkSolverPos_iterative tree_solver_urdf(icub_kdl_urdf,"",tree_solver.getTreeGraph().getSerialization());
    assert(tree_solver.getTreeGraph().getSerialization().is_consistent(icub_kdl));
    
    assert(tree_solver.getTreeGraph().getSerialization().getNrOfJoints() == nj);
    assert(tree_solver.getTreeGraph().getSerialization().getNrOfSegments() == ns);
    std::cout << "Original TreeGraph:" << std::endl;
    std::cout << tree_solver.getTreeGraph().toString() << std::endl;
    std::cout << "Serialization " << std::endl;
    std::cout << tree_solver.getTreeGraph().getSerialization().toString() << std::endl;
    
    std::cout << "Converted to URDF TreeGraph:" << std::endl;
    std::cout << tree_solver_urdf.getTreeGraph().toString() << std::endl;
    
    KDL::TreeFkSolverPos_recursive tree_solver_old(icub_kdl);


    assert(ns == (int)icub_kdl_urdf.getNrOfSegments());
    assert(nj == (int)icub_kdl_urdf.getNrOfJoints());
    
    Random rng;
    rng.seed(yarp::os::Time::now());
    
    KDL::JntArray q(nj), dq(nj), ddq(nj);
    KDL::Wrenches f_ext(ns);
    KDL::Frame pos_kdl, pos_kdl_old, pos_kdl_urdf, pos_idyn, pos_idyn_body, pos_ikin;
    
    Vector q_idyn(nj);
    
    for(int trial=0; trial < N_TRIALS; trial++ ) {
        
        for(int i=0;i<nj;i++) 
        {
            q_idyn[i] = 1.0*CTRL_DEG2RAD*360*rng.uniform();
        }
        
        q_idyn = icub_idyn.setAllPositions(q_idyn);
        
        for(int i=0;i<nj; i++ ) {
            q(i) = q_idyn(i);
        }
        
        for(int j=0; j < ns; j++ ) {
            f_ext[j] = KDL::Wrench::Zero();
        }
        
        //Compute forward position with iDyn
        iCubArmDyn icub_arm("right");
        Vector q_rarm(10);
        q_rarm[0] = q_idyn[12];
        q_rarm[1] = q_idyn[13];
        q_rarm[2] = q_idyn[14];
        q_rarm[3] = q_idyn[22];
        q_rarm[4] = q_idyn[23];
        q_rarm[5] = q_idyn[24];
        q_rarm[6] = q_idyn[25];
        q_rarm[7] = q_idyn[26];
        q_rarm[8] = q_idyn[27];
        q_rarm[9] = q_idyn[28];
        
        std::cout << "Used q" << q_rarm.toString() << std::endl;
        
        icub_arm.setAng(q_rarm);
        tic = yarp::os::Time::now();
        idynMatrix2kdlFrame(icub_arm.getH(),pos_idyn);
        toc = yarp::os::Time::now();
        
        //compute forward position with iKin
        iCubArm icub_arm_ikin("right");
        icub_arm_ikin.setAng(q_rarm);
        tic = yarp::os::Time::now();
        idynMatrix2kdlFrame(icub_arm.getH(),pos_ikin);
        toc = yarp::os::Time::now();
        
        //Compute forward position with whole body iDyn
        Matrix H_torso, H_rarm, H_right, H_up;
        H_up = icub_idyn.lowerTorso->HUp;
        H_torso = icub_idyn.lowerTorso->up->getH();
        H_right = icub_idyn.upperTorso->HRight;
        H_rarm = icub_idyn.upperTorso->right->getH();
        
        idynMatrix2kdlFrame(H_up*H_torso*H_right*H_rarm,pos_idyn_body);
        
        //Compute forward position with KDL (old solver) 
        tic = yarp::os::Time::now();
        int ret;
        ret = tree_solver_old.JntToCart(q,pos_kdl_old,"r_gripper");
        toc = yarp::os::Time::now();
        cout << " ret " << ret << endl;
        assert(ret == 0);
        time_kdl_old += (toc-tic);
        
        //Compute forward position with KDL
        tic = yarp::os::Time::now();
        ret = tree_solver.JntToCart(q,pos_kdl,"r_gripper");
        toc = yarp::os::Time::now();
        cout << " ret " << ret << endl;
        assert(ret == 0);
        time_kdl += (toc-tic);
        
        
        //Compute forward position with KDL trought URDF
        tic = yarp::os::Time::now();
        ret = tree_solver_urdf.JntToCart(q,pos_kdl_urdf,"r_gripper");
        toc = yarp::os::Time::now();
        cout << "ret " << ret << endl;
        assert(ret == 0);
        time_kdl_urdf += (toc-tic);
        
        //Compare: 
        cout << "iDyn r_gripper pos" << endl;
        cout << pos_idyn.p << endl;
        
        cout << "iKin r_gripper pos" << endl;
        cout << pos_ikin.p << endl;
        
        
        cout << "iDyn Body r_gripper pos" << endl;
        cout << pos_idyn_body.p << endl;
    
    
        cout << "KDL old solver r_gripper pos" << endl;
        cout << pos_kdl_old.p << endl;
    
        cout << "KDL r_gripper pos" << endl;
        cout << pos_kdl.p << endl;
    
        cout << "KDL urdf r_gripper pos" << endl;
        cout << pos_kdl_urdf.p << endl;
        
        //Compute forward position with KDL (old solver) 
        ret = tree_solver_old.JntToCart(q,pos_kdl_old,"torso");
        assert(ret == 0);
        
        //Compute forward position with KDL
        ret = tree_solver.JntToCart(q,pos_kdl,"torso");
        assert(ret == 0);
        
        //Compute forward position with KDL trought URDF
        ret = tree_solver_urdf.JntToCart(q,pos_kdl_urdf,"torso");
        assert(ret == 0);
        
        
        cout << "KDL old solver torso" << endl;
        cout << pos_kdl_old.p << endl;
    
        cout << "KDL torso pos" << endl;
        cout << pos_kdl.p << endl;
    
        cout << "KDL urdf torso pos" << endl;
        cout << pos_kdl_urdf.p << endl;
        
        //Compute forward position with KDL (old solver) 
        ret = tree_solver_old.JntToCart(q,pos_kdl_old,"root_link");
        assert(ret == 0);
        
        //Compute forward position with KDL
        ret = tree_solver.JntToCart(q,pos_kdl,"root_link");
        assert(ret == 0);
        
        //Compute forward position with KDL trought URDF
        ret = tree_solver_urdf.JntToCart(q,pos_kdl_urdf,"root_link");
        assert(ret == 0);
        
        cout << "KDL old solver root_link" << endl;
        cout << pos_kdl_old.p << endl;
    
        cout << "KDL torso root link" << endl;
        cout << pos_kdl.p << endl;
    
        cout << "KDL urdf torso root link" << endl;
        cout << pos_kdl_urdf.p << endl;
        
        
        compare(tree_solver,tree_solver_urdf,q,"torso_yaw");
        compare(tree_solver,tree_solver_urdf,q,"torso_roll");
        compare(tree_solver,tree_solver_urdf,q,"torso_pitch");
        compare(tree_solver,tree_solver_urdf,q,"r_shoulder_pitch");
        compare(tree_solver,tree_solver_urdf,q,"r_shoulder_roll");
        
        compare(tree_solver,tree_solver_urdf,q,"l_shoulder_pitch");
        compare(tree_solver,tree_solver_urdf,q,"l_shoulder_roll");
        
        
        
        compare(tree_solver,tree_solver_urdf,q,"torso_pitch","r_shoulder_pitch");
        compare(tree_solver,tree_solver_urdf,q,"r_shoulder_pitch","r_shoulder_roll");
        compare(tree_solver,tree_solver_urdf,q,"r_shoulder_roll","r_arm_ft_sensor");
        
        compare(tree_solver,tree_solver_urdf,q,"l_gripper");
        compare(tree_solver,tree_solver_urdf,q,"r_gripper");
        compare(tree_solver,tree_solver_urdf,q,"eyes_tilt");



    }
    
    
    cout << "Total time for KDL:" << time_kdl/N_TRIALS << endl;
    cout << "Total time for KDL (URDF):" << time_kdl_urdf/N_TRIALS << endl;

    
}


