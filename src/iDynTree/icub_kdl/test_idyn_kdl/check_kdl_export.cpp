#include <iostream>

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include <kdl_urdf/kdl_export.hpp>
#include <kdl_urdf/kdl_import.hpp>


#include <cassert>

#define EQUALISH(x,y) norm(x-y) < delta

using namespace std;
using namespace KDL;


double delta = 1e-10;



KDL::JntArray clean_vec(KDL::JntArray & in)
{
    KDL::JntArray out = in;
    for(int j =0; j < (int)out.rows(); j++) {
        out(j) = out(j) < 1e-15 ? 0 : out(j); 
    }
    return out;
} 

KDL::Tree SimpleThreeLinkRobot()
{
    Tree stlr("fake_root");
    stlr.addSegment(Segment("real_root",Joint("fake_joint",Joint::None),Frame::Identity()),"fake_root");
    stlr.addSegment(Segment("first_link",Joint("first_joint",Joint::RotZ),Frame::DH(1.0,M_PI/2,0.0,0.0)),"real_root");
    stlr.addSegment(Segment("first_link_fixed",Joint("first_link_fixed_joint",Joint::None),Frame::Identity()),"first_link");
    stlr.addSegment(Segment("second_link",Joint("second_joint",Joint::RotZ),Frame::DH(1.0,M_PI/2,0.0,0.0)),"first_link");
    stlr.addSegment(Segment("second_link_fixed",Joint("second_link_fixed_joint",Joint::None),Frame::Identity()),"second_link");
    stlr.addSegment(Segment("third_link",Joint("third_joint",Joint::RotZ),Frame::DH(1.0,M_PI/2,0.0,0.0)),"first_link");
    stlr.addSegment(Segment("end_effector",Joint("end_effector_joint",Joint::None),Frame::Identity()),"third_link");

    
    return stlr;
}
int main(int argc, char * argv[])
{
    int N_TRIALS = 1;
    
    KDL::Tree original = SimpleThreeLinkRobot();
    KDL::Tree urdf; 
    
    //Dumping URDF file
    int ret = kdl_export::treeToFile("three_link_robot.xml",original,"three_link_robot");
    assert(ret);
    
    //Creating KDL::Tree from URDF
    ret = kdl_import::treeFromFile("three_link_robot.xml",urdf);
    assert(ret); 
    
    //Creating solver
    KDL::TreeFkSolverPos_recursive tree_solver(original);
    KDL::TreeFkSolverPos_recursive tree_solver_urdf(urdf);

    unsigned int ns = original.getNrOfSegments();
    unsigned int nj = original.getNrOfJoints();
    
    assert(ns == urdf.getNrOfSegments());
    assert(nj == urdf.getNrOfJoints());
    
    KDL::JntArray q(nj);
    KDL::Frame pos_original, pos_urdf;
    KDL::Frame fl_original, fl_urdf;
    KDL::Frame sl_original, sl_urdf;
    
    for(int trial=0; trial < N_TRIALS; trial++ ) {
        
        //
        q(0) = 0.6*M_PI/2;
        q(1) = 0.4*M_PI/2;
        q(2) = 0.5*M_PI/2;
            
        //Compute forward position with KDL (original model) 
        ret = tree_solver.JntToCart(q,fl_original,"first_link_fixed");
        assert(ret == 0);       
        ret = tree_solver.JntToCart(q,sl_original,"second_link_fixed");
        assert(ret == 0);
        ret = tree_solver.JntToCart(q,pos_original,"end_effector");
        assert(ret == 0);
                
        //Compute forward position with KDL (urdf model)
        ret = tree_solver_urdf.JntToCart(q,fl_urdf,"first_link_fixed");
        assert(ret == 0);
        ret = tree_solver_urdf.JntToCart(q,sl_urdf,"second_link_fixed");
        assert(ret == 0);
        ret = tree_solver_urdf.JntToCart(q,pos_urdf,"end_effector");
        assert(ret == 0);
        
        //Compare
        cout << "KDL original model" << endl;
        cout << fl_original << endl;
        cout << sl_original << endl;
        cout << pos_original << endl;
    
        cout << "KDL URDF model" << endl;
        cout << fl_urdf << endl;
        cout << sl_urdf << endl;
        cout << pos_urdf << endl;
    
  }
    
}


