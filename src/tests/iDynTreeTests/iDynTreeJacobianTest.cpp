
#include <iCub/iDynTree/iCubTree.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/frames.hpp>

#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

#include <yarp/math/api.h>
#include <yarp/os/Log.h>

#include <iCub/ctrl/math.h>


using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace iCub::iDynTree;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;


using namespace std;

void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=1.0)
{
    for( int i=0; i < (int)vec.size(); i++ ) {
        vec[i] =  coeff*M_PI*rng.uniform();
    }
}

Matrix adjoint_twist(Matrix H)
{
    Matrix R = H.submatrix(0,2,0,2);
    Vector r = H.submatrix(0,2,0,3).getCol(3);
    //std::cout << "~~~~~~~ " << std::endl << " r " << r.toString() << std::endl;
    Matrix adj(6,6);
    adj.zero();
    adj.setSubmatrix(R,0,0);
    adj.setSubmatrix(R,3,3);
    adj.setSubmatrix(crossProductMatrix(r)*R,0,3);
    return adj;
}

Matrix SE3Inv(const Matrix &H)
{    
    if ((H.rows()!=4) || (H.cols()!=4))
    {

        return Matrix(0,0);
    }

    Vector p(4);
    p[0]=H(0,3);
    p[1]=H(1,3);
    p[2]=H(2,3);
    p[3]=1.0;

    Matrix invH=H.transposed();
    p=invH*p;
        
    invH(0,3)=-p[0];
    invH(1,3)=-p[1];
    invH(2,3)=-p[2];
    invH(3,0)=invH(3,1)=invH(3,2)=0.0;

    return invH;
}



void iDyn_print_velocity_acceleration(const yarp::sig::Vector & lin_vel, const yarp::sig::Vector & ang_vel, const yarp::sig::Vector & lin_acc, const yarp::sig::Vector & ang_acc, std::string link_name)
{
    Vector v(6), a(6);
    v.setSubvector(0,lin_vel);
    v.setSubvector(3,ang_vel);

    a.setSubvector(0,lin_acc);
    a.setSubvector(3,ang_acc);

    std::cout <<"Velocity of the " << link_name << endl
    << v.toString() << endl
    <<"Acceleration of the " << link_name << endl
    << a.toString() << endl;
    return;
}

void iDyn_compose_velocity_acceleration(const yarp::sig::Vector & lin_vel, const yarp::sig::Vector & ang_vel, const yarp::sig::Vector & lin_acc, const yarp::sig::Vector & ang_acc, Vector & v, Vector & a)
{
    v = Vector(6);
    a = Vector(6);
    v.setSubvector(0,lin_vel);
    v.setSubvector(3,ang_vel);

    a.setSubvector(0,lin_acc);
    a.setSubvector(3,ang_acc);
    return;
}


void iDynTree_print_velocity_acceleration(DynTree & icub_idyntree, const std::string link_name)
{    
    std::cout <<"Velocity of the " << link_name << endl;
    cout << icub_idyntree.getVel(icub_idyntree.getLinkIndex((link_name))).toString() << endl;
    cout <<"Acceleration of " << link_name << endl;
    cout << icub_idyntree.getAcc(icub_idyntree.getLinkIndex(link_name)).toString() << endl;
}

void set_random_q_dq_ddq(yarp::os::Random & rng, iCubTree & icub_tree)
{
    double pos_c = 0.0,vel_c = 0.0,acc_c =0.0;
    
    yarp::sig::Matrix H_w2b(4,4);
    H_w2b.eye();
    
    pos_c = 2.0;
    vel_c = 1.0;
    acc_c = 4.0;
    
    H_w2b[0][3] = M_PI*rng.uniform();
    H_w2b[1][3] = M_PI*rng.uniform();
    H_w2b[2][3] = M_PI*rng.uniform();
    
  
    
    YARP_ASSERT(icub_tree.setWorldBasePose(H_w2b));

    std::cout << "iDynTree Jacobian test world pose " << icub_tree.getWorldBasePose().toString();
    
    
    Vector q(icub_tree.getNrOfDOFs());            
    set_random_vector(q,rng,pos_c);
    icub_tree.setAng(q);
    
    Vector dq(icub_tree.getNrOfDOFs());            
    set_random_vector(dq,rng,vel_c);
    dq[1] = 1000.0;
    icub_tree.setDAng(dq);

    Vector ddq(icub_tree.getNrOfDOFs());            
    set_random_vector(ddq,rng,acc_c);
    icub_tree.setD2Ang(ddq);
    
    return;
}

int main()
{
    
    double tol = 1e-5;
    
    //Initializing the random number generator
    yarp::os::Random rng;
    rng.seed(yarp::os::Time::now());
    
    ////////////////////////////////////////////////////////////////////
    //// iCubTree 
    ////////////////////////////////////////////////////////////////////    
    //Similarly in iDynTree a iCubTree_version_tag structure is defined
    iCubTree_version_tag icub_idyntree_version;
    
    icub_idyntree_version.head_version = 2;
    icub_idyntree_version.legs_version = 2;
    icub_idyntree_version.feet_ft = true;
    
    //The iCubTree is istantiated
    //note that the serialization used is the one used in iDyn, while the 
    //default one is the one used in skinDynLib
    iCubTree icub_idyntree(icub_idyntree_version,IDYN_SERIALIZATION);

    Vector w0(3,0.0);
    Vector dw0(3,0.0);
    Vector ddp0(3,0.0);
    ddp0[2] = 9.8;
    
    //The setInertialMeasure impose the velocity in the imu, like in iCubWholeBody
    icub_idyntree.setInertialMeasure(w0,dw0,ddp0);
    
    //We fill the robot state with random values, for testing
    //in reality this should be filled with value read from the robot 
    //NB: the serialization of q, dq, ddq is defined in the constructor of iCubTree
    set_random_q_dq_ddq(rng,icub_idyntree);
       
    //std::cout << "Joint position" << std::endl;
    //std::cout << icub_idyntree.getAng().toString() << std::endl;
    
    
    //We then perform kinematic propagation
    icub_idyntree.kinematicRNEA();
       
    
    ////////////////////////////////////////////////////////////////////
    // Checking JACOBIANS
    ////////////////////////////////////////////////////////////////////
    
    //In iDynTree there are two type of Jacobians:
    //* The absolute jacobian, that has 6+NrOfDOFs columns and maps 
    //  the velocity of the floating base and the velocity of the joints to 
    //  the velocity of a link (check getJacobian documentation ) 
    //
    //* The relative jacobian, that has NrOfDOFs columsn and maps the 
    //  the velocity of the joints to the difference of the velocity of 
    //  two links (check getRelativeJacobian documentation)
    
    Matrix rel_jacobian;
    Matrix abs_jacobian;
    
    //We can calculate then the velocity of the right hand using the 
    //kinematic propagation or using the jacobians
    Vector v_rhand;
    Vector v_rhand_rel_jac;
    Vector v_rhand_abs_jac;
    
    //We get the index for the right hand and the left hand
    int r_hand_index = icub_idyntree.getLinkIndex("r_gripper");
    int l_hand_index = icub_idyntree.getLinkIndex("l_gripper");
    
    //By default the returned velocity is expressed in global reference frame (but with local reference point)
    //but it is possible to specify, via the local flag, to express them in local cordinates
    v_rhand = icub_idyntree.getVel(r_hand_index);
    
    //By default the absloute jacobian is expressed in global reference frame
    //but it is possible to specify, via the local flag, to express them in local cordinates
    icub_idyntree.getJacobian(r_hand_index,abs_jacobian);

    v_rhand_abs_jac = abs_jacobian*icub_idyntree.getDQ_fb();
    
    //The relative jacobian is instead by default expressed in local coordinates
    //In this example we calculate the Jacobian between the two hands
    //icub_idyntree.getRelativeJacobian(r_hand_index,l_hand_index,rel_jacobian);
    
    //v_rhand_rel_jac = rel_jacobian*icub_idyntree.getDAng() + adjoint_twist(icub_idyntree.getPosition(r_hand_index,l_hand_index))*icub_idyntree.getVel(l_hand_index,true);
    
    std::cout << "Comparison between velocities" << std::endl 
              << "Real one          " << v_rhand.toString() << std::endl
              //<< "Relative jacobian " << v_rhand_rel_jac.toString() << std::endl
              << "Absolute jacobian " << v_rhand_abs_jac.toString() << std::endl
              //<< "Difference in norm " << norm(v_rhand-v_rhand_rel_jac) << std::endl
              << "Difference in norm " << norm(v_rhand-v_rhand_abs_jac) << std::endl;

              
    //if( norm(v_rhand-v_rhand_rel_jac) > tol ) { return EXIT_FAILURE; }
    if( norm(v_rhand-v_rhand_abs_jac) > tol ) { return EXIT_FAILURE; }
    
              
    //For testing, it is also possible to check that the absolute velocity is computed correctly
    yarp::sig::Vector abs_v_rhand, abs_v_rhand_abs_jac;
    icub_idyntree.getJacobian(r_hand_index,abs_jacobian);
    abs_v_rhand = icub_idyntree.getVel(r_hand_index);
    abs_v_rhand_abs_jac = abs_jacobian*icub_idyntree.getDQ_fb();
         
    std::cout << "Comparison between velocities expressed in world frame" << std::endl 
              << "Real one           " << abs_v_rhand.toString() << std::endl
              << "Absolute jacobian  " << abs_v_rhand_abs_jac.toString() << std::endl
              << "Difference in norm " << norm(abs_v_rhand-abs_v_rhand_abs_jac) << std::endl;
          
              
    if( norm(abs_v_rhand-abs_v_rhand_abs_jac) > tol ) { return EXIT_FAILURE; }
    
    //We can also add a testing on com DJdq (only on "linear" part for now)
    Matrix com_jacobian_6d, com_jacobian;
    icub_idyntree.getCOMJacobian(com_jacobian_6d);
    
    com_jacobian = com_jacobian_6d.submatrix(0,2,0,com_jacobian_6d.cols()-1);
    yarp::sig::Vector v_com, v_com_jacobian;
    v_com = icub_idyntree.getVelCOM();
    YARP_ASSERT( com_jacobian.cols() == icub_idyntree.getNrOfDOFs()+6);
    v_com_jacobian = com_jacobian*icub_idyntree.getDQ_fb();
    
    std::cout << "Comparison between com velocities" << std::endl
              << "Real one     " << v_com.toString() << std::endl
              << "Jacobian one " << v_com_jacobian.toString() << std::endl;
    
    if( norm(v_com-v_com_jacobian) > tol ) { return EXIT_FAILURE; }           
              
    //To compare real com acceleration and the one calculated with the jacobian
    //We need to compute kinematicRNEA using root link as the kinematic source
    std::string kinematic_base_link_name = "root_link";
    
    iCub::iDynTree::iCubTree waist_imu_icub(icub_idyntree_version,IDYN_SERIALIZATION,0,kinematic_base_link_name);
    yarp::sig::Vector a_com, a_com_jacobian;
    a_com = icub_idyntree.getAccCOM();
    
    
    
    //World of waist_imu_icub is the same of icub_idyntree: root_link
    yarp::sig::Vector six_zeros(6,0.0);
    yarp::sig::Vector dof_zeros(icub_idyntree.getNrOfDOFs(),0.0);
    waist_imu_icub.setKinematicBaseVelAcc(icub_idyntree.getVel(icub_idyntree.getLinkIndex(kinematic_base_link_name)),
                                          six_zeros);
    
    waist_imu_icub.setAng(icub_idyntree.getAng());
    waist_imu_icub.setDAng(icub_idyntree.getDAng());
    waist_imu_icub.setD2Ang(dof_zeros);
    YARP_ASSERT(waist_imu_icub.kinematicRNEA());
    
    YARP_ASSERT(icub_idyntree.getLinkIndex(kinematic_base_link_name) == waist_imu_icub.getLinkIndex(kinematic_base_link_name)); 
    
    a_com_jacobian = com_jacobian*icub_idyntree.getD2Q_fb() + waist_imu_icub.getAccCOM(); 

     
    std::cout << "Comparison between com accelerations" << std::endl
              << "Real one     " << a_com.toString() << std::endl
              << "Jacobian one " << a_com_jacobian.toString() << std::endl;
    
    if( norm(a_com-a_com_jacobian) > tol ) { 
        std::cerr << "iDynTreeJacobianTest failed: Consistency error between com accelerations " << std::endl;
        return EXIT_FAILURE; 
    }           
    
    //Test the new getCentroidalMomentum
    yarp::sig::Vector centroidal_momentum = icub_idyntree.getCentroidalMomentum();
    
    yarp::sig::Matrix centroidal_momentum_jacobian;
    
    icub_idyntree.getCentroidalMomentumJacobian(centroidal_momentum_jacobian);
    
    yarp::sig::Vector centroidal_momentum_with_jac = centroidal_momentum_jacobian*icub_idyntree.getDQ_fb();
   
    std::cout << "Comparison between centroidal momentums" << std::endl
              << "Real one     " << centroidal_momentum.toString() << std::endl
              << "Jacobian one " << centroidal_momentum_with_jac.toString() << std::endl;
    
    if( norm(centroidal_momentum-centroidal_momentum_with_jac) > tol ) { 
        std::cerr << "iDynTreeJacobianTest failed: Consistency error between centroidal momentums " << std::endl;
        return EXIT_FAILURE; 
    }           
    
    
    return 0;
    
}
