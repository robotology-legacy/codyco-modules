
#include "icubfixed.h"

#include <map>
#include <vector>
#include <iostream>


typedef  Eigen::Displacementd::AdjointMatrix  AdjointMatrix;

AdjointMatrix getAdjacencyMatrix(const Eigen::Twistd& tw)
{
    
    AdjointMatrix dAdj;
    
    dAdj << 0    ,-tw(2), tw(1), 0    , 0    , 0    ,
            tw(2), 0    ,-tw(0), 0    , 0    , 0    ,
           -tw(1), tw(0), 0    , 0    , 0    , 0    ,
            0    ,-tw(5), tw(4), 0    ,-tw(2), tw(1),
            tw(5), 0    ,-tw(3), tw(2), 0    ,-tw(0),
           -tw(4), tw(3), 0    , -tw(1), tw(0), 0    ;
    
    return dAdj;
    
}

Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d& vec)
{
    
    Eigen::Matrix3d mat;
    
    mat << 0     ,-vec(2), vec(1),
           vec(2), 0     ,-vec(0),
          -vec(1), vec(0), 0     ;
    
    return mat;
    
}

class RevoluteJoint
{
public:
    //=================== Variables ====================//
    int                          parent;
    int                          child;
    int                          internaldof;
    int                          dof;
    Eigen::Displacementd         H_seg_joint;
    Eigen::Matrix<double,6,1>    jacobian;
    Eigen::Matrix<double,6,1>    djacobian;
    Eigen::Vector3d              axis;

    RevoluteJoint(int _parent, int _child, int _internaldof, int _dof, const Eigen::Displacementd& _H_seg_joint, const Eigen::Vector3d& _axis)
    : parent(_parent)
    , child(_child)
    , internaldof(_internaldof)
    , dof(_dof)
    , H_seg_joint(_H_seg_joint)
    , jacobian(Eigen::Matrix<double,6,1>::Zero())
    , djacobian(Eigen::Matrix<double,6,1>::Zero())
    , axis(_axis)
    {
        jacobian << axis[0], axis[1], axis[2], 0, 0, 0 ;
    };

    Eigen::Displacementd get_H_joint_child(double q_dof)
    {
        return Eigen::Displacementd(0,0,0,cos(q_dof/2.), axis[0]*sin(q_dof/2.),axis[1]*sin(q_dof/2.),axis[2]*sin(q_dof/2.));
    }

    Eigen::Twistd get_twist(double dq_dof)
    {
        return Eigen::Twistd(axis[0]*dq_dof,axis[1]*dq_dof,axis[2]*dq_dof,0,0,0);
    }

    AdjointMatrix getdAdjointInverse(double q_dof, double dq_dof)
    {
        
        AdjointMatrix  iAdj = get_H_joint_child( q_dof ).inverse().adjoint();
        
        Eigen::Twistd  tw  = - iAdj * get_twist( dq_dof );
        
        return iAdj * getAdjacencyMatrix(tw);
        
    }


};

struct icubfixed::Pimpl
{
public:
    //=============== General Variables ================//
    int                                      nbSeg;
    Eigen::VectorXd                          actuatedDofs;
    Eigen::VectorXd                          lowerLimits;
    Eigen::VectorXd                          upperLimits;
    Eigen::VectorXd                          q;
    Eigen::VectorXd                          dq;
    Eigen::Displacementd                     Hroot;
    Eigen::Twistd                            Troot;
    Eigen::VectorXd                          alldq;
    bool                                     isFreeFlyer;
    int                                      nbDofs;
    Eigen::Matrix<double,6,1>                gravity_dtwist;

    //=============== Dynamic Variables ================//
    Eigen::MatrixXd                          M;
    Eigen::MatrixXd                          Minv;
    Eigen::MatrixXd                          B;
    Eigen::VectorXd                          n;
    Eigen::VectorXd                          l;
    Eigen::VectorXd                          g;

    //================= CoM Variables ==================//
    double                                   total_mass;
    Eigen::Vector3d                          comPosition;
    Eigen::Vector3d                          comVelocity;
    Eigen::Vector3d                          comJdotQdot;
    Eigen::Matrix<double,3,Eigen::Dynamic>   comJacobian;
    Eigen::Matrix<double,3,Eigen::Dynamic>   comJacobianDot;

    //=============== Segments Variables ===============//
    std::vector< double >                                   segMass;
    std::vector< Eigen::Vector3d >                          segCoM;
    std::vector< Eigen::Matrix<double,6,6> >                segMassMatrix;
    std::vector< Eigen::Vector3d >                          segMomentsOfInertia;
    std::vector< Eigen::Rotation3d >                        segInertiaAxes;
    std::vector< Eigen::Displacementd >                     segPosition;
    std::vector< Eigen::Twistd >                            segVelocity;
    std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJacobian;
    std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJdot;
    std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJointJacobian;
    std::vector< Eigen::Twistd >                            segJdotQdot;

    //================ Other Variables =================//
    std::map< std::string, int >             segIndexFromName;
    std::vector< std::string >               segNameFromIndex;
    bool                                     isUpToDate;

    //================= Update Status ==================//
    bool                           M_isUpToDate;
    bool                           Minv_isUpToDate;
    bool                           n_isUpToDate;
    bool                           g_isUpToDate;
    bool                           comPosition_isUpToDate;
    bool                           comVelocity_isUpToDate;
    bool                           comJdotQdot_isUpToDate;
    bool                           comJacobian_isUpToDate;
    bool                           comJacobianDot_isUpToDate;
    std::vector< bool >            segPosition_isUpToDate;
    std::vector< bool >            segVelocity_isUpToDate;
    std::vector< bool >            segJacobian_isUpToDate;
    std::vector< bool >            segJdot_isUpToDate;
    std::vector< bool >            segJdotQdot_isUpToDate;
    std::vector< bool >            segNonLinearEffects_isUpToDate;

    //============== Recursive Variables ===============//
    std::vector< std::vector< RevoluteJoint* > >  segJoints;
    std::vector< Eigen::Matrix<double,6,6> >      segNonLinearEffects;
    Eigen::Matrix<double,6,Eigen::Dynamic>        Jroot;
    Eigen::Matrix<double,6,Eigen::Dynamic>        dJroot;

    Pimpl()
    : nbSeg(33)
    , actuatedDofs(32)
    , lowerLimits(32)
    , upperLimits(32)
    , q(32)
    , dq(32)
    , Hroot(0,0,0)
    , Troot(0,0,0,0,0,0)
    , M(32,32)
    , Minv(32,32)
    , B(32,32)
    , n(32)
    , l(32)
    , g(32)
    , total_mass(0)
    , comPosition(0,0,0)
    , comVelocity(0,0,0)
    , comJdotQdot(0,0,0)
    , comJacobian(3,32)
    , comJacobianDot(3,32)
    , segMass(33, double(0))
    , segCoM(33, Eigen::Vector3d(0,0,0))
    , segMassMatrix(33, Eigen::Matrix<double,6,6>())
    , segMomentsOfInertia(33, Eigen::Vector3d(0,0,0))
    , segInertiaAxes(33, Eigen::Rotation3d())
    , segPosition(33, Eigen::Displacementd(0,0,0))
    , segVelocity(33, Eigen::Twistd(0,0,0,0,0,0))
    , segJacobian(33, Eigen::Matrix<double,6,Eigen::Dynamic>(6,32))
    , segJdot(33, Eigen::Matrix<double,6,Eigen::Dynamic>(6,32))
    , segJointJacobian(33, Eigen::Matrix<double,6,Eigen::Dynamic>(6,32))
    , segJdotQdot(33, Eigen::Twistd(0,0,0,0,0,0))
    , segIndexFromName()
    , segNameFromIndex(33, std::string(""))
    , isUpToDate(false)
    , alldq(32)
    , isFreeFlyer(false)
    , nbDofs(32)
    , M_isUpToDate(false)
    , Minv_isUpToDate(false)
    , n_isUpToDate(false)
    , g_isUpToDate(false)
    , comPosition_isUpToDate(false)
    , comVelocity_isUpToDate(false)
    , comJdotQdot_isUpToDate(false)
    , comJacobian_isUpToDate(false)
    , comJacobianDot_isUpToDate(false)
    , segPosition_isUpToDate(33, bool(false))
    , segVelocity_isUpToDate(33, bool(false))
    , segJacobian_isUpToDate(33, bool(false))
    , segJdot_isUpToDate(33, bool(false))
    , segJdotQdot_isUpToDate(33, bool(false))
    , gravity_dtwist(Eigen::Matrix<double,6,1>::Zero())
    , segJoints(33)
    , Jroot(6,32)
    , dJroot(6,32)
    , segNonLinearEffects(33, Eigen::Matrix<double,6,6>::Zero())
    , segNonLinearEffects_isUpToDate(33)
    {
        //================== Register all Joint for the kinematic structure ==================//
        segJoints[0].push_back( new RevoluteJoint(0, 1, 0, 0, Eigen::Displacementd(0.0,-0.0681,-0.1199,0.707106781185,-0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        segJoints[0].push_back( new RevoluteJoint(0, 7, 6, 6, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,-0.5,0.500000000003), Eigen::Vector3d(0,0,1)) );
        segJoints[0].push_back( new RevoluteJoint(0, 27, 26, 26, Eigen::Displacementd(0.0,0.0681,-0.1199,0.707106781185,-0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        
        segJoints[1].push_back( new RevoluteJoint(1, 2, 1, 1, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,-0.5,-0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[2].push_back( new RevoluteJoint(2, 3, 2, 2, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,-0.5,-0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[3].push_back( new RevoluteJoint(3, 4, 3, 3, Eigen::Displacementd(0.0,0.0,-0.2236,0.499999999997,0.5,-0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[4].push_back( new RevoluteJoint(4, 5, 4, 4, Eigen::Displacementd(-0.0,-0.213,0.0,0.0,-0.707106781185,-0.707106781188,7.45058059692e-09), Eigen::Vector3d(0,0,1)) );
        
        segJoints[5].push_back( new RevoluteJoint(5, 6, 5, 5, Eigen::Displacementd(0.0,0.0,0.0,0.707106781185,-0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        
        
        segJoints[7].push_back( new RevoluteJoint(7, 8, 7, 7, Eigen::Displacementd(0.032,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        
        segJoints[8].push_back( new RevoluteJoint(8, 9, 8, 8, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,-0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[9].push_back( new RevoluteJoint(9, 17, 16, 16, Eigen::Displacementd(0.0060472293,0.0225685672,-0.1433,0.430459334574,0.430459334576,-0.5609855268,-0.560985526797), Eigen::Vector3d(0,0,1)) );
        segJoints[9].push_back( new RevoluteJoint(9, 10, 9, 9, Eigen::Displacementd(-0.0060472293,0.0225685672,-0.1433,0.430459334574,-0.430459334576,-0.5609855268,0.560985526797), Eigen::Vector3d(0,0,1)) );
        segJoints[9].push_back( new RevoluteJoint(9, 24, 23, 23, Eigen::Displacementd(0.0,-0.00231,-0.1933,0.499999999997,-0.5,0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[10].push_back( new RevoluteJoint(10, 11, 10, 10, Eigen::Displacementd(0.0,0.0,0.10774,0.499999999997,-0.5,-0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[11].push_back( new RevoluteJoint(11, 12, 11, 11, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,-0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[12].push_back( new RevoluteJoint(12, 13, 12, 12, Eigen::Displacementd(0.0,0.0,0.15228,0.560985526795,-0.560985526797,-0.430459334579,0.430459334577), Eigen::Vector3d(0,0,1)) );
        
        segJoints[13].push_back( new RevoluteJoint(13, 14, 13, 13, Eigen::Displacementd(-0.015,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        
        segJoints[14].push_back( new RevoluteJoint(14, 15, 14, 14, Eigen::Displacementd(0.0,0.0,0.1373,0.499999999997,0.5,-0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[15].push_back( new RevoluteJoint(15, 16, 15, 15, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        
        segJoints[17].push_back( new RevoluteJoint(17, 18, 17, 17, Eigen::Displacementd(0.0,0.0,-0.10774,0.499999999997,0.5,-0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[18].push_back( new RevoluteJoint(18, 19, 18, 18, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,-0.5,0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[19].push_back( new RevoluteJoint(19, 20, 19, 19, Eigen::Displacementd(0.0,0.0,-0.15228,0.430459334574,-0.430459334576,0.5609855268,-0.560985526797), Eigen::Vector3d(0,0,1)) );
        
        segJoints[20].push_back( new RevoluteJoint(20, 21, 20, 20, Eigen::Displacementd(0.015,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        
        segJoints[21].push_back( new RevoluteJoint(21, 22, 21, 21, Eigen::Displacementd(0.0,0.0,-0.1373,0.499999999997,0.5,-0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[22].push_back( new RevoluteJoint(22, 23, 22, 22, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        
        segJoints[24].push_back( new RevoluteJoint(24, 25, 24, 24, Eigen::Displacementd(0.0,0.033,0.0,0.499999999997,0.5,0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[25].push_back( new RevoluteJoint(25, 26, 25, 25, Eigen::Displacementd(0.0,0.0,0.001,0.499999999997,-0.5,0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        
        segJoints[27].push_back( new RevoluteJoint(27, 28, 27, 27, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[28].push_back( new RevoluteJoint(28, 29, 28, 28, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[29].push_back( new RevoluteJoint(29, 30, 29, 29, Eigen::Displacementd(0.0,0.0,0.2236,0.499999999997,-0.5,0.500000000003,-0.5), Eigen::Vector3d(0,0,1)) );
        
        segJoints[30].push_back( new RevoluteJoint(30, 31, 30, 30, Eigen::Displacementd(-0.0,-0.213,0.0,0.0,-0.707106781185,-0.707106781188,7.45058059692e-09), Eigen::Vector3d(0,0,1)) );
        
        segJoints[31].push_back( new RevoluteJoint(31, 32, 31, 31, Eigen::Displacementd(0.0,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0), Eigen::Vector3d(0,0,1)) );
        
        
        Jroot.setZero();
        dJroot.setZero();
        //================== Register all constant data other than kinematic structure ==================//
        segIndexFromName["icub.waist"] = 0;
        segNameFromIndex[0]     = "icub.waist";
        segMass[0]             = 0.20297;
        segCoM[0]              = Eigen::Vector3d(0.006, 0.0, -0.1);
        segMomentsOfInertia[0] = Eigen::Vector3d(0.000258841296496, 0.000290758328996, 0.000106643820833);
        segInertiaAxes[0]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[0]      << 0.0022885412965, 0.0, 0.000121782, 0.0, 0.020297, 0.0, 0.0, 0.002327765249, 0.0, -0.020297, 0.0, -0.00121782, 0.000121782, 0.0, 0.000113950740833, 0.0, 0.00121782, 0.0, 0.0, -0.020297, 0.0, 0.20297, 0.0, 0.0, 0.020297, 0.0, 0.00121782, 0.0, 0.20297, 0.0, 0.0, -0.00121782, 0.0, 0.0, 0.0, 0.20297;
        
        segIndexFromName["icub.l_hip_1"] = 1;
        segNameFromIndex[1]     = "icub.l_hip_1";
        segMass[1]             = 0.32708;
        segCoM[1]              = Eigen::Vector3d(0.0, 0.0, 0.0375);
        segMomentsOfInertia[1] = Eigen::Vector3d(0.000122682256667, 0.000122682256667, 0.00023615176);
        segInertiaAxes[1]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[1]      << 0.000582638506667, 0.0, 0.0, 0.0, -0.0122655, 0.0, 0.0, 0.000582638506667, 0.0, 0.0122655, 0.0, 0.0, 0.0, 0.0, 0.00023615176, 0.0, 0.0, 0.0, 0.0, 0.0122655, 0.0, 0.32708, 0.0, 0.0, -0.0122655, 0.0, 0.0, 0.0, 0.32708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.32708;
        
        segIndexFromName["icub.l_hip_2"] = 2;
        segNameFromIndex[2]     = "icub.l_hip_2";
        segMass[2]             = 1.5304;
        segCoM[2]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[2] = Eigen::Vector3d(0.0010850536, 0.0010850536, 0.0007353572);
        segInertiaAxes[2]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[2]      << 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0007353572, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304;
        
        segIndexFromName["icub.l_thigh"] = 3;
        segNameFromIndex[3]     = "icub.l_thigh";
        segMass[3]             = 2.32246;
        segCoM[3]              = Eigen::Vector3d(-0.0, 0.0, -0.1501968774);
        segMomentsOfInertia[3] = Eigen::Vector3d(0.0139763503018, 0.00147239522875, 0.0137814870406);
        segInertiaAxes[3]      = Eigen::Rotation3d(0.499999999997, 0.5, 0.500000000003, 0.5);
        segMassMatrix[3]      << 0.0661740990267, -9.94457796952e-16, -6.28177535894e-14, -1.38740911706e-28, 0.348826239886, -4.30061988307e-33, -9.94457796952e-16, 0.0663689622879, 1.38821792552e-18, -0.348826239886, -1.84987882274e-28, 1.38740911706e-28, -6.28177535894e-14, 1.38821792552e-18, 0.00147239522875, 6.50317069587e-33, -6.34363897162e-29, -7.80240048226e-48, 9.07190041004e-30, -0.348826239886, 6.50317069587e-33, 2.32246, 4.169918745e-27, 2.05734924082e-27, 0.348826239886, 2.27783586383e-29, -6.34363897162e-29, 3.23117426779e-27, 2.32246, 6.04868577449e-23, -1.10778927987e-33, 5.69952004022e-29, -7.80240048226e-48, 1.61558713389e-27, 6.04868577735e-23, 2.32246;
        
        segIndexFromName["icub.l_shank"] = 4;
        segNameFromIndex[4]     = "icub.l_shank";
        segMass[4]             = 0.95262;
        segCoM[4]              = Eigen::Vector3d(0.0, -0.1065, -0.0);
        segMomentsOfInertia[4] = Eigen::Vector3d(0.00383792736375, 0.0004726185975, 0.00383792736375);
        segInertiaAxes[4]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[4]      << 0.0146427815587, 0.0, 0.0, 0.0, 0.0, -0.10145403, 0.0, 0.0004726185975, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0146427815587, 0.10145403, 0.0, 0.0, 0.0, 0.0, 0.10145403, 0.95262, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.95262, 0.0, -0.10145403, 0.0, 0.0, 0.0, 0.0, 0.95262;
        
        segIndexFromName["icub.l_ankle_1"] = 5;
        segNameFromIndex[5]     = "icub.l_ankle_1";
        segMass[5]             = 0.14801;
        segCoM[5]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[5] = Eigen::Vector3d(7.1165058125e-05, 7.1165058125e-05, 4.442150125e-05);
        segInertiaAxes[5]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[5]      << 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.442150125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801;
        
        segIndexFromName["icub.l_foot"] = 6;
        segNameFromIndex[6]     = "icub.l_foot";
        segMass[6]             = 0.6747;
        segCoM[6]              = Eigen::Vector3d(-0.0047312139, 0.0, 0.0151082333);
        segMomentsOfInertia[6] = Eigen::Vector3d(0.000811937097733, 0.000336068314542, 0.000731741708191);
        segInertiaAxes[6]      = Eigen::Rotation3d(0.537182077292, 0.459821069367, 0.45982106937, 0.537182077294);
        segMassMatrix[6]      << 0.000876332562725, -4.57348952538e-16, 0.000108533100316, -6.00890142649e-30, -0.0101935250075, -5.73927123427e-31, -4.57348952538e-16, 0.000981045996233, 3.07767379075e-16, 0.0101935250075, 3.69778549322e-31, 0.00319215001833, 0.000108533100316, 3.07767379075e-16, 0.000360586358508, -3.41737009332e-30, -0.00319215001833, -4.45852782126e-31, -1.26772412659e-29, 0.0101935250075, -2.26027138273e-30, 0.6747, -1.62798920394e-17, -6.30870139115e-18, -0.0101935250075, -5.76238239361e-31, -0.00319215001833, -1.627989204e-17, 0.6747, 1.45977331394e-18, -2.85808003747e-31, 0.00319215001833, -2.94859874486e-31, -3.12114700404e-17, 1.45977331384e-18, 0.6747;
        
        segIndexFromName["icub.lap_belt_1"] = 7;
        segNameFromIndex[7]     = "icub.lap_belt_1";
        segMass[7]             = 3.623;
        segCoM[7]              = Eigen::Vector3d(-0.0383, -0.0173, 0.0);
        segMomentsOfInertia[7] = Eigen::Vector3d(0.0142217845833, 0.0105504779167, 0.00606792116667);
        segInertiaAxes[7]      = Eigen::Rotation3d(0.923879532511, 0.0, 0.0, -0.382683432366);
        segMassMatrix[7]      << 0.01347045892, -0.0042362169033, 0.0, 0.0, 0.0, -0.0626779, -0.0042362169033, 0.01770067372, 0.0, 0.0, 0.0, 0.1387609, 0.0, 0.0, 0.0124667913067, 0.0626779, -0.1387609, 0.0, 0.0, 0.0, 0.0626779, 3.623, -1.29670579829e-16, 0.0, 0.0, 0.0, -0.1387609, 1.56341953272e-16, 3.623, 0.0, -0.0626779, 0.1387609, 0.0, 0.0, 0.0, 3.623;
        
        segIndexFromName["icub.lap_belt_2"] = 8;
        segNameFromIndex[8]     = "icub.lap_belt_2";
        segMass[8]             = 0.91179;
        segCoM[8]              = Eigen::Vector3d(0.0, 0.0, 0.008);
        segMomentsOfInertia[8] = Eigen::Vector3d(0.00093397689, 0.00093397689, 0.000438115095);
        segInertiaAxes[8]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[8]      << 0.00099233145, 0.0, 0.0, 0.0, -0.00729432, 0.0, 0.0, 0.00099233145, 0.0, 0.00729432, 0.0, 0.0, 0.0, 0.0, 0.000438115095, 0.0, 0.0, 0.0, 0.0, 0.00729432, 0.0, 0.91179, 0.0, 0.0, -0.00729432, 0.0, 0.0, 0.0, 0.91179, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.91179;
        
        segIndexFromName["icub.chest"] = 9;
        segNameFromIndex[9]     = "icub.chest";
        segMass[9]             = 4.12925;
        segCoM[9]              = Eigen::Vector3d(0.0, 0.0, -0.1094482037);
        segMomentsOfInertia[9] = Eigen::Vector3d(0.0101058081992, 0.0116382113674, 0.00890101366667);
        segInertiaAxes[9]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[9]      << 0.059569719398, 0.0, 0.0, 0.0, 0.451938995128, 0.0, 0.0, 0.0611021225662, 0.0, -0.451938995128, 0.0, 0.0, 0.0, 0.0, 0.00890101366667, 0.0, 0.0, 0.0, 0.0, -0.451938995128, 0.0, 4.12925, 0.0, 0.0, 0.451938995128, 0.0, 0.0, 0.0, 4.12925, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.12925;
        
        segIndexFromName["icub.l_shoulder_1"] = 10;
        segNameFromIndex[10]     = "icub.l_shoulder_1";
        segMass[10]             = 0.48278;
        segCoM[10]              = Eigen::Vector3d(0.0, 0.0, 0.07224);
        segMomentsOfInertia[10] = Eigen::Vector3d(0.000120855926667, 0.000120855926667, 0.00023197579);
        segInertiaAxes[10]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[10]      << 0.0026403001316, 0.0, 0.0, 0.0, -0.0348760272, 0.0, 0.0, 0.0026403001316, 0.0, 0.0348760272, 0.0, 0.0, 0.0, 0.0, 0.00023197579, 0.0, 0.0, 0.0, 0.0, 0.0348760272, 0.0, 0.48278, 0.0, 0.0, -0.0348760272, 0.0, 0.0, 0.0, 0.48278, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48278;
        
        segIndexFromName["icub.l_shoulder_2"] = 11;
        segNameFromIndex[11]     = "icub.l_shoulder_2";
        segMass[11]             = 0.20779;
        segCoM[11]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[11] = Eigen::Vector3d(0.000107029165833, 0.000107029165833, 9.35055e-05);
        segInertiaAxes[11]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[11]      << 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.35055e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779;
        
        segIndexFromName["icub.l_arm"] = 12;
        segNameFromIndex[12]     = "icub.l_arm";
        segMass[12]             = 1.1584;
        segCoM[12]              = Eigen::Vector3d(0.0, 0.0, 0.078);
        segMomentsOfInertia[12] = Eigen::Vector3d(0.0025450048, 0.0025450048, 0.0003915392);
        segInertiaAxes[12]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[12]      << 0.0095927104, 0.0, 0.0, 0.0, -0.0903552, 0.0, 0.0, 0.0095927104, 0.0, 0.0903552, 0.0, 0.0, 0.0, 0.0, 0.0003915392, 0.0, 0.0, 0.0, 0.0, 0.0903552, 0.0, 1.1584, 0.0, 0.0, -0.0903552, 0.0, 0.0, 0.0, 1.1584, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1584;
        
        segIndexFromName["icub.l_elbow_1"] = 13;
        segNameFromIndex[13]     = "icub.l_elbow_1";
        segMass[13]             = 0.050798;
        segCoM[13]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[13] = Eigen::Vector3d(2.03192e-06, 2.03192e-06, 2.03192e-06);
        segInertiaAxes[13]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[13]      << 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798;
        
        segIndexFromName["icub.l_forearm"] = 14;
        segNameFromIndex[14]     = "icub.l_forearm";
        segMass[14]             = 0.48774;
        segCoM[14]              = Eigen::Vector3d(0.0, 0.0, 0.07);
        segMomentsOfInertia[14] = Eigen::Vector3d(0.000845416, 0.000845416, 9.7548e-05);
        segInertiaAxes[14]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[14]      << 0.003235342, 0.0, 0.0, 0.0, -0.0341418, 0.0, 0.0, 0.003235342, 0.0, 0.0341418, 0.0, 0.0, 0.0, 0.0, 9.7548e-05, 0.0, 0.0, 0.0, 0.0, 0.0341418, 0.0, 0.48774, 0.0, 0.0, -0.0341418, 0.0, 0.0, 0.0, 0.48774, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48774;
        
        segIndexFromName["icub.l_wrist_1"] = 15;
        segNameFromIndex[15]     = "icub.l_wrist_1";
        segMass[15]             = 0.05;
        segCoM[15]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[15] = Eigen::Vector3d(7.91666666667e-06, 7.91666666667e-06, 2.5e-06);
        segInertiaAxes[15]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[15]      << 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05;
        
        segIndexFromName["icub.l_hand"] = 16;
        segNameFromIndex[16]     = "icub.l_hand";
        segMass[16]             = 0.19099;
        segCoM[16]              = Eigen::Vector3d(0.0345, 0.0, 0.0);
        segMomentsOfInertia[16] = Eigen::Vector3d(7.64119158333e-05, 8.49428025e-05, 0.000143019678333);
        segInertiaAxes[16]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[16]      << 7.64119158333e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00031226865, 0.0, 0.0, 0.0, -0.006589155, 0.0, 0.0, 0.000370345525833, 0.0, 0.006589155, 0.0, 0.0, 0.0, 0.0, 0.19099, 0.0, 0.0, 0.0, 0.0, 0.006589155, 0.0, 0.19099, 0.0, 0.0, -0.006589155, 0.0, 0.0, 0.0, 0.19099;
        
        segIndexFromName["icub.r_shoulder_1"] = 17;
        segNameFromIndex[17]     = "icub.r_shoulder_1";
        segMass[17]             = 0.48278;
        segCoM[17]              = Eigen::Vector3d(0.0, 0.0, -0.07224);
        segMomentsOfInertia[17] = Eigen::Vector3d(0.000120855926667, 0.000120855926667, 0.00023197579);
        segInertiaAxes[17]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[17]      << 0.0026403001316, 0.0, 0.0, 0.0, 0.0348760272, 0.0, 0.0, 0.0026403001316, 0.0, -0.0348760272, 0.0, 0.0, 0.0, 0.0, 0.00023197579, 0.0, 0.0, 0.0, 0.0, -0.0348760272, 0.0, 0.48278, 0.0, 0.0, 0.0348760272, 0.0, 0.0, 0.0, 0.48278, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48278;
        
        segIndexFromName["icub.r_shoulder_2"] = 18;
        segNameFromIndex[18]     = "icub.r_shoulder_2";
        segMass[18]             = 0.20779;
        segCoM[18]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[18] = Eigen::Vector3d(0.000107029165833, 0.000107029165833, 9.35055e-05);
        segInertiaAxes[18]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[18]      << 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.35055e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779;
        
        segIndexFromName["icub.r_arm"] = 19;
        segNameFromIndex[19]     = "icub.r_arm";
        segMass[19]             = 1.1584;
        segCoM[19]              = Eigen::Vector3d(0.0, 0.0, -0.078);
        segMomentsOfInertia[19] = Eigen::Vector3d(0.0025450048, 0.0025450048, 0.0003915392);
        segInertiaAxes[19]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[19]      << 0.0095927104, 0.0, 0.0, 0.0, 0.0903552, 0.0, 0.0, 0.0095927104, 0.0, -0.0903552, 0.0, 0.0, 0.0, 0.0, 0.0003915392, 0.0, 0.0, 0.0, 0.0, -0.0903552, 0.0, 1.1584, 0.0, 0.0, 0.0903552, 0.0, 0.0, 0.0, 1.1584, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1584;
        
        segIndexFromName["icub.r_elbow_1"] = 20;
        segNameFromIndex[20]     = "icub.r_elbow_1";
        segMass[20]             = 0.050798;
        segCoM[20]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[20] = Eigen::Vector3d(2.03192e-06, 2.03192e-06, 2.03192e-06);
        segInertiaAxes[20]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[20]      << 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798;
        
        segIndexFromName["icub.r_forearm"] = 21;
        segNameFromIndex[21]     = "icub.r_forearm";
        segMass[21]             = 0.48774;
        segCoM[21]              = Eigen::Vector3d(0.0, 0.0, -0.07);
        segMomentsOfInertia[21] = Eigen::Vector3d(0.000845416, 0.000845416, 9.7548e-05);
        segInertiaAxes[21]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[21]      << 0.003235342, 0.0, 0.0, 0.0, 0.0341418, 0.0, 0.0, 0.003235342, 0.0, -0.0341418, 0.0, 0.0, 0.0, 0.0, 9.7548e-05, 0.0, 0.0, 0.0, 0.0, -0.0341418, 0.0, 0.48774, 0.0, 0.0, 0.0341418, 0.0, 0.0, 0.0, 0.48774, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48774;
        
        segIndexFromName["icub.r_wrist_1"] = 22;
        segNameFromIndex[22]     = "icub.r_wrist_1";
        segMass[22]             = 0.05;
        segCoM[22]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[22] = Eigen::Vector3d(7.91666666667e-06, 7.91666666667e-06, 2.5e-06);
        segInertiaAxes[22]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[22]      << 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05;
        
        segIndexFromName["icub.r_hand"] = 23;
        segNameFromIndex[23]     = "icub.r_hand";
        segMass[23]             = 0.19099;
        segCoM[23]              = Eigen::Vector3d(-0.0345, 0.0, 0.0);
        segMomentsOfInertia[23] = Eigen::Vector3d(7.64119158333e-05, 8.49428025e-05, 0.000143019678333);
        segInertiaAxes[23]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[23]      << 7.64119158333e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00031226865, 0.0, 0.0, 0.0, 0.006589155, 0.0, 0.0, 0.000370345525833, 0.0, -0.006589155, 0.0, 0.0, 0.0, 0.0, 0.19099, 0.0, 0.0, 0.0, 0.0, -0.006589155, 0.0, 0.19099, 0.0, 0.0, 0.006589155, 0.0, 0.0, 0.0, 0.19099;
        
        segIndexFromName["icub.neck_1"] = 24;
        segNameFromIndex[24]     = "icub.neck_1";
        segMass[24]             = 0.28252;
        segCoM[24]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[24] = Eigen::Vector3d(0.000155480173333, 0.000155480173333, 3.17835e-05);
        segInertiaAxes[24]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[24]      << 0.000155480173333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000155480173333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.17835e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.28252, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.28252, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.28252;
        
        segIndexFromName["icub.neck_2"] = 25;
        segNameFromIndex[25]     = "icub.neck_2";
        segMass[25]             = 0.1;
        segCoM[25]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[25] = Eigen::Vector3d(1.47e-05, 1.47e-05, 1.125e-05);
        segInertiaAxes[25]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[25]      << 1.47e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.47e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1;
        
        segIndexFromName["icub.head"] = 26;
        segNameFromIndex[26]     = "icub.head";
        segMass[26]             = 0.78;
        segCoM[26]              = Eigen::Vector3d(0.0, 0.0, 0.0825);
        segMomentsOfInertia[26] = Eigen::Vector3d(0.0019968, 0.0019968, 0.0019968);
        segInertiaAxes[26]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[26]      << 0.007305675, 0.0, 0.0, 0.0, -0.06435, 0.0, 0.0, 0.007305675, 0.0, 0.06435, 0.0, 0.0, 0.0, 0.0, 0.0019968, 0.0, 0.0, 0.0, 0.0, 0.06435, 0.0, 0.78, 0.0, 0.0, -0.06435, 0.0, 0.0, 0.0, 0.78, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.78;
        
        segIndexFromName["icub.r_hip_1"] = 27;
        segNameFromIndex[27]     = "icub.r_hip_1";
        segMass[27]             = 0.32708;
        segCoM[27]              = Eigen::Vector3d(0.0, 0.0, -0.0375);
        segMomentsOfInertia[27] = Eigen::Vector3d(0.000122682256667, 0.000122682256667, 0.00023615176);
        segInertiaAxes[27]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[27]      << 0.000582638506667, 0.0, 0.0, 0.0, 0.0122655, 0.0, 0.0, 0.000582638506667, 0.0, -0.0122655, 0.0, 0.0, 0.0, 0.0, 0.00023615176, 0.0, 0.0, 0.0, 0.0, -0.0122655, 0.0, 0.32708, 0.0, 0.0, 0.0122655, 0.0, 0.0, 0.0, 0.32708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.32708;
        
        segIndexFromName["icub.r_hip_2"] = 28;
        segNameFromIndex[28]     = "icub.r_hip_2";
        segMass[28]             = 1.5304;
        segCoM[28]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[28] = Eigen::Vector3d(0.0010850536, 0.0010850536, 0.0007353572);
        segInertiaAxes[28]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[28]      << 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0007353572, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304;
        
        segIndexFromName["icub.r_thigh"] = 29;
        segNameFromIndex[29]     = "icub.r_thigh";
        segMass[29]             = 2.32246;
        segCoM[29]              = Eigen::Vector3d(0.0, 0.0, 0.1501968774);
        segMomentsOfInertia[29] = Eigen::Vector3d(0.0139763503018, 0.00147239522875, 0.0137814870406);
        segInertiaAxes[29]      = Eigen::Rotation3d(0.499999999997, 0.5, 0.500000000003, 0.5);
        segMassMatrix[29]      << 0.0661740990267, -9.94457796952e-16, -6.28177535894e-14, 1.38740911706e-28, -0.348826239886, 4.30061988307e-33, -9.94457796952e-16, 0.0663689622879, 1.38821792552e-18, 0.348826239886, 1.84987882274e-28, -1.38740911706e-28, -6.28177535894e-14, 1.38821792552e-18, 0.00147239522875, -6.50317069587e-33, 6.34363897162e-29, 7.80240048226e-48, -9.07190041004e-30, 0.348826239886, -6.50317069587e-33, 2.32246, 4.169918745e-27, 2.05734924082e-27, -0.348826239886, -2.27783586383e-29, 6.34363897162e-29, 3.23117426779e-27, 2.32246, 6.04868577449e-23, 1.10778927987e-33, -5.69952004022e-29, 7.80240048226e-48, 1.61558713389e-27, 6.04868577735e-23, 2.32246;
        
        segIndexFromName["icub.r_shank"] = 30;
        segNameFromIndex[30]     = "icub.r_shank";
        segMass[30]             = 0.95262;
        segCoM[30]              = Eigen::Vector3d(0.0, -0.1065, -0.0);
        segMomentsOfInertia[30] = Eigen::Vector3d(0.00383792736375, 0.0004726185975, 0.00383792736375);
        segInertiaAxes[30]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[30]      << 0.0146427815587, 0.0, 0.0, 0.0, 0.0, -0.10145403, 0.0, 0.0004726185975, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0146427815587, 0.10145403, 0.0, 0.0, 0.0, 0.0, 0.10145403, 0.95262, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.95262, 0.0, -0.10145403, 0.0, 0.0, 0.0, 0.0, 0.95262;
        
        segIndexFromName["icub.r_ankle_1"] = 31;
        segNameFromIndex[31]     = "icub.r_ankle_1";
        segMass[31]             = 0.14801;
        segCoM[31]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[31] = Eigen::Vector3d(7.1165058125e-05, 7.1165058125e-05, 4.442150125e-05);
        segInertiaAxes[31]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[31]      << 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.442150125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801;
        
        segIndexFromName["icub.r_foot"] = 32;
        segNameFromIndex[32]     = "icub.r_foot";
        segMass[32]             = 0.6747;
        segCoM[32]              = Eigen::Vector3d(-0.0047312139, 0.0, -0.0151082333);
        segMomentsOfInertia[32] = Eigen::Vector3d(0.000811937097733, 0.000336068314542, 0.000731741708191);
        segInertiaAxes[32]      = Eigen::Rotation3d(0.459821069364, 0.537182077294, 0.537182077297, 0.459821069367);
        segMassMatrix[32]      << 0.000876332562724, -4.57369809339e-16, -0.00010853310032, 2.92125053965e-30, 0.0101935250075, -3.01215443302e-31, -4.57369809339e-16, 0.000981045996233, -3.07751754741e-16, -0.0101935250075, 8.32001735975e-32, 0.00319215001833, -0.00010853310032, -3.07751754741e-16, 0.000360586358509, -2.46364958486e-30, -0.00319215001833, 3.65926689434e-33, -2.86578375725e-31, -0.0101935250075, 5.11526993229e-31, 0.6747, -3.59031715592e-17, 1.38303539628e-17, 0.0101935250075, 3.42969604496e-30, -0.00319215001833, -3.59031715591e-17, 0.6747, 2.44386452964e-17, 2.20326385638e-31, 0.00319215001833, -6.54816181092e-33, 3.82181265801e-18, 2.44386452964e-17, 0.6747;
        
        total_mass        = 27.197066;
        actuatedDofs      = Eigen::VectorXd::Constant(nbDofs, 1);
        lowerLimits      << -0.767944870878, -2.07694180987, -1.37881010908, -2.18166156499, -0.733038285838, -0.418879020479, -0.383972435439, -0.680678408278, -1.02974425868, -1.65806278939, 0.0, -0.645771823238, 0.0959931088597, -0.872664625997, -1.1344640138, -0.436332312999, -1.65806278939, 0.0, -0.645771823238, 0.0959931088597, -0.872664625997, -1.1344640138, -0.436332312999, -0.698131700798, -1.2217304764, -0.959931088597, -0.767944870878, -2.07694180987, -1.37881010908, -2.18166156499, -0.733038285838, -0.418879020479;
        upperLimits      << 2.30383461263, 0.296705972839, 1.37881010908, 0.401425727959, 0.366519142919, 0.418879020479, 1.46607657168, 0.680678408278, 1.02974425868, 0.0872664625997, 2.80648943721, 1.74532925199, 1.85004900711, 0.872664625997, 0.174532925199, 0.436332312999, 0.0872664625997, 2.80648943721, 1.74532925199, 1.85004900711, 0.872664625997, 0.174532925199, 0.436332312999, 0.523598775598, 1.0471975512, 0.959931088597, 2.30383461263, 0.296705972839, 1.37881010908, 0.401425727959, 0.366519142919, 0.418879020479;
        gravity_dtwist   << 0, 0 ,0, 0, 0, -9.80665;
        B.setZero();
        B.diagonal()      = Eigen::VectorXd::Constant(nbDofs, 0.001);
        outdateModel();
        
    };

    ~Pimpl()
    {
        
    }

    int nbSegments()
    {
        return nbSeg;
    }

    const Eigen::VectorXd& getActuatedDofs()
    {
        return actuatedDofs;
    }

    const Eigen::VectorXd& getJointLowerLimits()
    {
        return lowerLimits;
    }

    const Eigen::VectorXd& getJointUpperLimits()
    {
        return upperLimits;
    }

    const Eigen::VectorXd& getJointPositions()
    {
        return q;
    }

    const Eigen::VectorXd& getJointVelocities()
    {
        return dq;
    }

    const Eigen::Displacementd& getFreeFlyerPosition()
    {
        return Hroot;
    }

    const Eigen::Twistd& getFreeFlyerVelocity()
    {
        return Troot;
    }

    const Eigen::MatrixXd& getInertiaMatrix()
    {
        if (!M_isUpToDate)
        {
            updateModel();
            M.setZero();
            for (int i=0; i<33; i++)
            {
                M.noalias() += getSegmentJacobian(i).transpose() * getSegmentMassMatrix(i) * getSegmentJacobian(i);
            }
        
            M_isUpToDate = true;
        }
        return M;
    }

    const Eigen::MatrixXd& getInertiaMatrixInverse()
    {
        if (!Minv_isUpToDate)
        {
            updateModel();
            SelfAdjointEigenSolver<MatrixXd> es( getInertiaMatrix() );
            const Eigen::VectorXd invdiag = es.eigenvalues().array().inverse();
            const Eigen::MatrixXd& R      = es.eigenvectors();
            Minv = R * invdiag.asDiagonal() * R.transpose();
        
            Minv_isUpToDate = true;
        }
        return Minv;
    }

    const Eigen::MatrixXd& getDampingMatrix()
    {
        return B;
    }

    const Eigen::VectorXd& getNonLinearTerms()
    {
        if (!n_isUpToDate)
        {
            updateModel();
            n.setZero();
            for (int i=0; i<33; i++)
            {
                n.noalias() += getSegmentJacobian(i).transpose() * ( getSegmentMassMatrix(i) * getSegmentJdotQdot(i) + getSegmentNonLinearEffectsMatrix(i) * getSegmentVelocity(i) ) ;
            }
        
            n_isUpToDate = true;
        }
        return n;
    }

    const Eigen::VectorXd& getLinearTerms()
    {
        return l;
    }

    const Eigen::VectorXd& getGravityTerms()
    {
        if (!g_isUpToDate)
        {
            updateModel();
            g.setZero();
            for (int i=0; i<33; i++)
            {
                g -= ( getSegmentJacobian(i).transpose() * getSegmentMassMatrix(i) * getSegmentPosition(i).inverse().adjoint() ) * gravity_dtwist;
            }
        
            g_isUpToDate = true;
        }
        return g;
    }

    double getMass()
    {
        return total_mass;
    }

    const Eigen::Vector3d& getCoMPosition()
    {
        if (!comPosition_isUpToDate)
        {
            updateModel();
            comPosition.setZero();
            for (int i=0; i<33; i++)
            {
                comPosition.noalias() += (getSegmentPosition(i).getTranslation() + getSegmentPosition(i).getRotation() * getSegmentCoM(i)) * (getSegmentMass(i)/total_mass); 
            }
        
            comPosition_isUpToDate = true;
        }
        return comPosition;
    }

    const Eigen::Vector3d& getCoMVelocity()
    {
        if (!comVelocity_isUpToDate)
        {
            updateModel();
            comVelocity = getCoMJacobian() * alldq;
        
            comVelocity_isUpToDate = true;
        }
        return comVelocity;
    }

    const Eigen::Vector3d& getCoMJdotQdot()
    {
        if (!comJdotQdot_isUpToDate)
        {
            updateModel();
            comJdotQdot = getCoMJacobianDot() * alldq;
        
            comJdotQdot_isUpToDate = true;
        }
        return comJdotQdot;
    }

    const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobian()
    {
        if (!comJacobian_isUpToDate)
        {
            updateModel();
            comJacobian.setZero();
            for (int i=0; i<33; i++)
            {
                Eigen::Matrix<double,6,32> tmpJ = Eigen::Displacementd(getSegmentCoM(i), getSegmentPosition(i).getRotation().inverse()).inverse().adjoint() * getSegmentJacobian(i);
                comJacobian.noalias()              += ( tmpJ.bottomRows<3>() ) * (getSegmentMass(i)/ total_mass);
            }
        
            comJacobian_isUpToDate = true;
        }
        return comJacobian;
    }

    const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobianDot()
    {
        if (!comJacobianDot_isUpToDate)
        {
            updateModel();
            comJacobianDot.setZero();
            for (int i=0; i<33; i++)
            {
                Eigen::Twistd T_comR_seg_comR       = getSegmentVelocity(i);
                T_comR_seg_comR.getLinearVelocity().setZero();
                AdjointMatrix Ad_comR_seg           = Eigen::Displacementd(getSegmentCoM(i), getSegmentPosition(i).getRotation().inverse()).inverse().adjoint();
                AdjointMatrix dAd_comR_seg          = Ad_comR_seg * getAdjacencyMatrix( T_comR_seg_comR );
                Eigen::Matrix<double,6,32> tmpJ = Ad_comR_seg * getSegmentJdot(i)  +  dAd_comR_seg * getSegmentJacobian(i);
                comJacobianDot.noalias()           += ( tmpJ.bottomRows<3>() ) * (getSegmentMass(i)/ total_mass);
            }
        
            comJacobianDot_isUpToDate = true;
        }
        return comJacobianDot;
    }

    double getSegmentMass(int index)
    {
        return segMass[index];
    }

    const Eigen::Vector3d& getSegmentCoM(int index)
    {
        return segCoM[index];
    }

    const Eigen::Matrix<double,6,6>& getSegmentMassMatrix(int index)
    {
        return segMassMatrix[index];
    }

    const Eigen::Vector3d& getSegmentMomentsOfInertia(int index)
    {
        return segMomentsOfInertia[index];
    }

    const Eigen::Rotation3d& getSegmentInertiaAxes(int index)
    {
        return segInertiaAxes[index];
    }

    const Eigen::Displacementd& getSegmentPosition(int index)
    {
        if (!segPosition_isUpToDate[index])
        {
            updateModel();
            
        
            segPosition_isUpToDate[index] = true;
        }
        return segPosition[index];
    }

    const Eigen::Twistd& getSegmentVelocity(int index)
    {
        if (!segVelocity_isUpToDate[index])
        {
            updateModel();
            
        
            segVelocity_isUpToDate[index] = true;
        }
        return segVelocity[index];
    }

    const Eigen::Matrix<double,6,Eigen::Dynamic>& getSegmentJacobian(int index)
    {
        if (!segJacobian_isUpToDate[index])
        {
            updateModel();
            
        
            segJacobian_isUpToDate[index] = true;
        }
        return segJacobian[index];
    }

    const Eigen::Matrix<double,6,Eigen::Dynamic>& getSegmentJdot(int index)
    {
        if (!segJdot_isUpToDate[index])
        {
            updateModel();
            
        
            segJdot_isUpToDate[index] = true;
        }
        return segJdot[index];
    }

    const Eigen::Matrix<double,6,Eigen::Dynamic>& getJointJacobian(int index)
    {
        segJointJacobian[index] = getSegmentJacobian(index);
        return segJointJacobian[index];
    }

    const Eigen::Twistd& getSegmentJdotQdot(int index)
    {
        if (!segJdotQdot_isUpToDate[index])
        {
            updateModel();
            
        
            segJdotQdot_isUpToDate[index] = true;
        }
        return segJdotQdot[index];
    }

    void doSetJointPositions(const Eigen::VectorXd& _q)
    {
        q = _q;
        outdateModel();
    }

    void doSetJointVelocities(const Eigen::VectorXd& _dq)
    {
        dq    = _dq;
        alldq =  dq;
        outdateModel();
    }

    void doSetFreeFlyerPosition(const Eigen::Displacementd& _Hroot)
    {
        Hroot = _Hroot;
        outdateModel();
    }

    void doSetFreeFlyerVelocity(const Eigen::Twistd& _Troot)
    {
        Troot = _Troot;
        outdateModel();
    }

    int doGetSegmentIndex(const std::string& name)
    {
        return segIndexFromName[name];
    }

    const std::string& doGetSegmentName(int index)
    {
        return segNameFromIndex[index];
    }

    void outdateModel()
    {
        isUpToDate                     = false;
        M_isUpToDate                   = false;
        Minv_isUpToDate                = false;
        n_isUpToDate                   = false;
        g_isUpToDate                   = false;
        comPosition_isUpToDate         = false;
        comVelocity_isUpToDate         = false;
        comJdotQdot_isUpToDate         = false;
        comJacobian_isUpToDate         = false;
        comJacobianDot_isUpToDate      = false;
        for (int i=0; i<33; i++)
        {
            segPosition_isUpToDate[i]  = false;
            segVelocity_isUpToDate[i]  = false;
            segJacobian_isUpToDate[i]  = false;
            segJdot_isUpToDate[i]      = false;
            segJdotQdot_isUpToDate[i]  = false;
        }
    }

    void updateModel()
    {
        if (!isUpToDate)
        {
            recursiveUpdateModel(0, Hroot, Troot, Jroot, dJroot); //0 because this index must be the root link of the robot
            isUpToDate = true;
        }
    }

    void recursiveUpdateModel(int segIdx, const Eigen::Displacementd& H_0_seg, const Eigen::Twistd& T_s_0_s, const Eigen::Matrix<double,6,Eigen::Dynamic>& J_s_0_s, const Eigen::Matrix<double,6,Eigen::Dynamic>& dJ_s_0_s)
    {
        
        segPosition[segIdx] = H_0_seg;
        segVelocity[segIdx] = T_s_0_s;
        segJacobian[segIdx] = J_s_0_s;
        segJdot[segIdx]     = dJ_s_0_s;
        
        segJdotQdot[segIdx] = dJ_s_0_s * alldq;
        
        Eigen::Matrix3d wx                  = getSkewSymmetricMatrix( T_s_0_s.getAngularVelocity() );
        Eigen::Matrix3d rx                  = getSkewSymmetricMatrix( segCoM[segIdx] );
        Eigen::Matrix<double, 6, 6 > _segN  = Eigen::Matrix<double, 6, 6 >::Zero();
        _segN.topLeftCorner<3,3>()          = wx;
        _segN.bottomRightCorner<3,3>()      = wx;
        _segN.topRightCorner<3,3>()         = rx * wx - wx * rx;
        
        segNonLinearEffects[segIdx]         = _segN * segMassMatrix[segIdx];
        
        // register update status
        segPosition_isUpToDate[segIdx]         = true;
        segVelocity_isUpToDate[segIdx]         = true;
        segJacobian_isUpToDate[segIdx]         = true;
        segJdot_isUpToDate[segIdx]             = true;
        segJdotQdot_isUpToDate[segIdx]         = true;
        segNonLinearEffects_isUpToDate[segIdx] = true;
        
        
        // -----> recursive computation of children
        for (int jidx=0; jidx < segJoints[segIdx].size(); jidx++)
        {
            int cdof         = segJoints[segIdx][jidx]->dof;
            int cinternaldof = segJoints[segIdx][jidx]->internaldof;
        
            double q_dof  =  q[cinternaldof];
            double dq_dof = dq[cinternaldof];
        
        
            // DATA //
            Eigen::Displacementd  H_seg_child   =  segJoints[segIdx][jidx]->H_seg_joint * segJoints[segIdx][jidx]->get_H_joint_child( q_dof ) ;
            AdjointMatrix         Ad_child_seg  =  H_seg_child.inverse().adjoint();
            AdjointMatrix         Ad_joint_seg  =  segJoints[segIdx][jidx]->H_seg_joint.inverse().adjoint();
            Eigen::Twistd         T_c_j_c       =  segJoints[segIdx][jidx]->get_twist( dq_dof );
            AdjointMatrix         dAd_child_seg =  segJoints[segIdx][jidx]->getdAdjointInverse( q_dof, dq_dof ) * Ad_joint_seg;
        
        
            //                    H_0_child    = H_0_parent * H_parent_child
            //                                 = H_0_parent * H_parent_joint * H_joint_child(q_i)
            Eigen::Displacementd  H_0_child    = H_0_seg * H_seg_child;
        
            // T_child/0(child)    =  T_child/parent(child) + T_parent/0(child)
            //                     =  T_child/parent(child) + Ad_child_parent * T_parent/0(parent)
            //                     =  T_child/joint(child) + T_joint/parent(child)[=0 because same link] + Ad_child_parent * T_parent/0(parent)
            //                     =  T_child/joint(child)(dq_i) + Ad_child_parent * T_parent/0(parent)
            Eigen::Twistd  T_c_0_c =  T_c_j_c  +  Ad_child_seg * T_s_0_s;
        
            // J_child/0(child)  =  (... same reasoning as for T_child/0(child) ...)
            //                   =  [ 0 0 0 ... J_child/joint(child) ... 0 0 ]  +  Ad_child_parent * J_parent/0(parent)
            Eigen::Matrix<double,6,Eigen::Dynamic> J_c_0_c  = Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6, nbDofs);
            J_c_0_c.col(cdof)    = segJoints[segIdx][jidx]->jacobian;
            J_c_0_c.noalias()   += Ad_child_seg * J_s_0_s;
            
            // dJ_child/0(child)  =  (... same reasoning as for T_child/0(child) ...)
            //                    =  [ 0 0 0 ... dJ_child/joint(child) ... 0 0 ]  +  dAd_child_parent * J_parent/0(parent) + Ad_child_parent * dJ_parent/0(parent)
            Eigen::Matrix<double,6,Eigen::Dynamic> dJ_c_0_c = Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6, nbDofs);
            dJ_c_0_c.col(cdof)    = segJoints[segIdx][jidx]->djacobian;
            dJ_c_0_c.noalias()   += dAd_child_seg * J_s_0_s + Ad_child_seg * dJ_s_0_s;
        
        
            recursiveUpdateModel(segJoints[segIdx][jidx]->child, H_0_child, T_c_0_c, J_c_0_c, dJ_c_0_c);
        }
        
    }

    const Eigen::Matrix<double,6,6>& getSegmentNonLinearEffectsMatrix(int index)
    {
        if (!segNonLinearEffects_isUpToDate[index])
        {
            updateModel();
            
        
            segNonLinearEffects_isUpToDate[index] = true;
        }
        return segNonLinearEffects[index];
    }


};

icubfixed::icubfixed(const std::string& robotName)
: orcisir::ISIRModel(robotName, 32, false)
, pimpl( new Pimpl() )
{
    
};

icubfixed::~icubfixed()
{
    
}

int icubfixed::nbSegments() const
{
    return pimpl->nbSegments();
}

const Eigen::VectorXd& icubfixed::getActuatedDofs() const
{
    return pimpl->getActuatedDofs();
}

const Eigen::VectorXd& icubfixed::getJointLowerLimits() const
{
    return pimpl->getJointLowerLimits();
}

const Eigen::VectorXd& icubfixed::getJointUpperLimits() const
{
    return pimpl->getJointUpperLimits();
}

const Eigen::VectorXd& icubfixed::getJointPositions() const
{
    return pimpl->getJointPositions();
}

const Eigen::VectorXd& icubfixed::getJointVelocities() const
{
    return pimpl->getJointVelocities();
}

const Eigen::Displacementd& icubfixed::getFreeFlyerPosition() const
{
    return pimpl->getFreeFlyerPosition();
}

const Eigen::Twistd& icubfixed::getFreeFlyerVelocity() const
{
    return pimpl->getFreeFlyerVelocity();
}

const Eigen::MatrixXd& icubfixed::getInertiaMatrix() const
{
    return pimpl->getInertiaMatrix();
}

const Eigen::MatrixXd& icubfixed::getInertiaMatrixInverse() const
{
    return pimpl->getInertiaMatrixInverse();
}

const Eigen::MatrixXd& icubfixed::getDampingMatrix() const
{
    return pimpl->getDampingMatrix();
}

const Eigen::VectorXd& icubfixed::getNonLinearTerms() const
{
    return pimpl->getNonLinearTerms();
}

const Eigen::VectorXd& icubfixed::getLinearTerms() const
{
    return pimpl->getLinearTerms();
}

const Eigen::VectorXd& icubfixed::getGravityTerms() const
{
    return pimpl->getGravityTerms();
}

double icubfixed::getMass() const
{
    return pimpl->getMass();
}

const Eigen::Vector3d& icubfixed::getCoMPosition() const
{
    return pimpl->getCoMPosition();
}

const Eigen::Vector3d& icubfixed::getCoMVelocity() const
{
    return pimpl->getCoMVelocity();
}

const Eigen::Vector3d& icubfixed::getCoMJdotQdot() const
{
    return pimpl->getCoMJdotQdot();
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& icubfixed::getCoMJacobian() const
{
    return pimpl->getCoMJacobian();
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& icubfixed::getCoMJacobianDot() const
{
    return pimpl->getCoMJacobianDot();
}

double icubfixed::getSegmentMass(int index) const
{
    return pimpl->getSegmentMass(index);
}

const Eigen::Vector3d& icubfixed::getSegmentCoM(int index) const
{
    return pimpl->getSegmentCoM(index);
}

const Eigen::Matrix<double,6,6>& icubfixed::getSegmentMassMatrix(int index) const
{
    return pimpl->getSegmentMassMatrix(index);
}

const Eigen::Vector3d& icubfixed::getSegmentMomentsOfInertia(int index) const
{
    return pimpl->getSegmentMomentsOfInertia(index);
}

const Eigen::Rotation3d& icubfixed::getSegmentInertiaAxes(int index) const
{
    return pimpl->getSegmentInertiaAxes(index);
}

const Eigen::Displacementd& icubfixed::getSegmentPosition(int index) const
{
    return pimpl->getSegmentPosition(index);
}

const Eigen::Twistd& icubfixed::getSegmentVelocity(int index) const
{
    return pimpl->getSegmentVelocity(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& icubfixed::getSegmentJacobian(int index) const
{
    return pimpl->getSegmentJacobian(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& icubfixed::getSegmentJdot(int index) const
{
    return pimpl->getSegmentJdot(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& icubfixed::getJointJacobian(int index) const
{
    return pimpl->getJointJacobian(index);
}

const Eigen::Twistd& icubfixed::getSegmentJdotQdot(int index) const
{
    return pimpl->getSegmentJdotQdot(index);
}

void icubfixed::doSetJointPositions(const Eigen::VectorXd& q)
{
    pimpl->doSetJointPositions(q);
    invalidateReducedProblemMatrices();
}

void icubfixed::doSetJointVelocities(const Eigen::VectorXd& dq)
{
    pimpl->doSetJointVelocities(dq);
    invalidateReducedProblemMatrices();
}

void icubfixed::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
    pimpl->doSetFreeFlyerPosition(Hroot);
    invalidateReducedProblemMatrices();
}

void icubfixed::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
    pimpl->doSetFreeFlyerVelocity(Troot);
    invalidateReducedProblemMatrices();
}

int icubfixed::doGetSegmentIndex(const std::string& name) const
{
    return pimpl->doGetSegmentIndex(name);
}

const std::string& icubfixed::doGetSegmentName(int index) const
{
    return pimpl->doGetSegmentName(index);
}

void icubfixed::printAllData() const
{
    std::cout<<"nbSeg:\n";
    std::cout<<nbSegments()<<"\n";
    
    std::cout<<"actuatedDofs:\n";
    std::cout<<getActuatedDofs()<<"\n";
    
    std::cout<<"lowerLimits:\n";
    std::cout<<getJointLowerLimits()<<"\n";
    
    std::cout<<"upperLimits:\n";
    std::cout<<getJointUpperLimits()<<"\n";
    
    std::cout<<"q:\n";
    std::cout<<getJointPositions()<<"\n";
    
    std::cout<<"dq:\n";
    std::cout<<getJointVelocities()<<"\n";
    
    std::cout<<"Hroot:\n";
    std::cout<<getFreeFlyerPosition()<<"\n";
    
    std::cout<<"Troot:\n";
    std::cout<<getFreeFlyerVelocity()<<"\n";
    
    std::cout<<"total_mass:\n";
    std::cout<<getMass()<<"\n";
    
    std::cout<<"comPosition:\n";
    std::cout<<getCoMPosition()<<"\n";
    
    std::cout<<"comVelocity:\n";
    std::cout<<getCoMVelocity()<<"\n";
    
    std::cout<<"comJdotQdot:\n";
    std::cout<<getCoMJdotQdot()<<"\n";
    
    std::cout<<"comJacobian:\n";
    std::cout<<getCoMJacobian()<<"\n";
    
    std::cout<<"comJacobianDot:\n";
    std::cout<<getCoMJacobianDot()<<"\n";
    
    std::cout<<"M:\n";
    std::cout<<getInertiaMatrix()<<"\n";
    
    std::cout<<"Minv:\n";
    std::cout<<getInertiaMatrixInverse()<<"\n";
    
    std::cout<<"B:\n";
    std::cout<<getDampingMatrix()<<"\n";
    
    std::cout<<"n:\n";
    std::cout<<getNonLinearTerms()<<"\n";
    
    std::cout<<"l:\n";
    std::cout<<getLinearTerms()<<"\n";
    
    std::cout<<"g:\n";
    std::cout<<getGravityTerms()<<"\n";
    
    
    for (int idx=0; idx<nbSegments(); idx++)
    {
        std::cout<<"segMass "<<idx<<":\n";
        std::cout<<getSegmentMass(idx)<<"\n";
    
        std::cout<<"segCoM "<<idx<<":\n";
        std::cout<<getSegmentCoM(idx)<<"\n";
    
        std::cout<<"segMassMatrix "<<idx<<":\n";
        std::cout<<getSegmentMassMatrix(idx)<<"\n";
    
        std::cout<<"segMomentsOfInertia "<<idx<<":\n";
        std::cout<<getSegmentMomentsOfInertia(idx)<<"\n";
    
        std::cout<<"segInertiaAxes "<<idx<<":\n";
        std::cout<<getSegmentInertiaAxes(idx)<<"\n";
    
        std::cout<<"segPosition "<<idx<<":\n";
        std::cout<<getSegmentPosition(idx)<<"\n";
    
        std::cout<<"segVelocity "<<idx<<":\n";
        std::cout<<getSegmentVelocity(idx)<<"\n";
    
        std::cout<<"segJacobian "<<idx<<":\n";
        std::cout<<getSegmentJacobian(idx)<<"\n";
    
        std::cout<<"segJdot "<<idx<<":\n";
        std::cout<<getSegmentJdot(idx)<<"\n";
    
        std::cout<<"segJointJacobian "<<idx<<":\n";
        std::cout<<getJointJacobian(idx)<<"\n";
    
        std::cout<<"segJdotQdot "<<idx<<":\n";
        std::cout<<getSegmentJdotQdot(idx)<<"\n";
    
    }
    
}





