
#include "iCubModel.h"

#include <map>
#include <vector>
#include <iostream>

typedef  Eigen::Displacementd::AdjointMatrix  AdjointMatrix;

//================================= Joint structure =================================//
class iCubModel_Joint
{
public:
    const int             parent;
    const int             child;
    const int             dof;
    const Eigen::Displacementd H_seg_joint;
    Eigen::Matrix<double,6,1> jacobian;
    Eigen::Matrix<double,6,1> djacobian;

     iCubModel_Joint(int _parent, int _child, int _dof, const Eigen::Displacementd& _H_seg_joint)
    :parent(_parent)
    ,child(_child)
    ,dof(_dof)
    ,H_seg_joint(_H_seg_joint)
    ,jacobian(Eigen::Matrix<double,6,1>::Zero())
    ,djacobian(Eigen::Matrix<double,6,1>::Zero())
    {
        
    }

    virtual Eigen::Displacementd get_H_joint_child(double q_dof) = 0;
    virtual Eigen::Twistd get_twist(double dq_dof) = 0;
    AdjointMatrix get_inv_dAdjoint(double q_dof, double dq_dof)
    {
        
        AdjointMatrix  iAdj = get_H_joint_child( q_dof ).inverse().adjoint();
        
        Eigen::Twistd  tw  = - iAdj * get_twist( dq_dof );
        
        AdjointMatrix idAdj;
        
        idAdj << 0    ,-tw(2), tw(1), 0    , 0    , 0    ,
                 tw(2), 0    ,-tw(0), 0    , 0    , 0    ,
                -tw(1), tw(0), 0    , 0    , 0    , 0    ,
                 0    ,-tw(5), tw(4), 0    ,-tw(2), tw(1),
                 tw(5), 0    ,-tw(3), tw(2), 0    ,-tw(0),
                -tw(4), tw(3), 0    , -tw(1), tw(0), 0    ;
        
        return iAdj * idAdj;
        
    }

};

class iCubModel_JointRx: public iCubModel_Joint
{
public:
     iCubModel_JointRx(int _parent, int _child, int _dof, const Eigen::Displacementd& _H_seg_joint)
    :iCubModel_Joint(_parent, _child, _dof, _H_seg_joint)
    {
        jacobian << 1,0,0,0,0,0;
    }

    virtual Eigen::Displacementd get_H_joint_child(double q_dof)
    {
        return Eigen::Displacementd(0,0,0,cos(q_dof/2.), sin(q_dof/2.),0,0);
    }

    virtual Eigen::Twistd get_twist(double dq_dof)
    {
        return Eigen::Twistd(dq_dof,0,0,0,0,0);
    }

};

class iCubModel_JointRy: public iCubModel_Joint
{
public:
     iCubModel_JointRy(int _parent, int _child, int _dof, const Eigen::Displacementd& _H_seg_joint)
    :iCubModel_Joint(_parent, _child, _dof, _H_seg_joint)
    {
        jacobian << 0,1,0,0,0,0;
    }

    virtual Eigen::Displacementd get_H_joint_child(double q_dof)
    {
        return Eigen::Displacementd(0,0,0,cos(q_dof/2.), 0, sin(q_dof/2.),0);
    }

    virtual Eigen::Twistd get_twist(double dq_dof)
    {
        return Eigen::Twistd(0,dq_dof,0,0,0,0);
    }

};

class iCubModel_JointRz: public iCubModel_Joint
{
public:
     iCubModel_JointRz(int _parent, int _child, int _dof, const Eigen::Displacementd& _H_seg_joint)
    :iCubModel_Joint(_parent, _child, _dof, _H_seg_joint)
    {
        jacobian << 0,0,1,0,0,0;
    }

    virtual Eigen::Displacementd get_H_joint_child(double q_dof)
    {
        return Eigen::Displacementd(0,0,0,cos(q_dof/2.), 0,0, sin(q_dof/2.));
    }

    virtual Eigen::Twistd get_twist(double dq_dof)
    {
        return Eigen::Twistd(0,0,dq_dof,0,0,0);
    }

};



//================================= Pimpl structure =================================//
struct iCubModel::Pimpl
{
public:
    int                                                     nbSegments;
    Eigen::VectorXd                                         actuatedDofs;
    Eigen::VectorXd                                         lowerLimits;
    Eigen::VectorXd                                         upperLimits;
    Eigen::VectorXd                                         q;
    Eigen::VectorXd                                         dq;
    Eigen::Displacementd                                    Hroot;
    Eigen::Twistd                                           Troot;
    Eigen::MatrixXd                                         M;
    Eigen::MatrixXd                                         Minv;
    Eigen::MatrixXd                                         B;
    Eigen::VectorXd                                         n;
    Eigen::VectorXd                                         l;
    Eigen::VectorXd                                         g;
    double                                                  total_mass;
    Eigen::Vector3d                                         comPosition;
    Eigen::Vector3d                                         comVelocity;
    Eigen::Vector3d                                         comJdotQdot;
    Eigen::Matrix<double,3,Eigen::Dynamic>                  comJacobian;
    Eigen::Matrix<double,3,Eigen::Dynamic>                  comJacobianDot;
    std::vector< Eigen::Displacementd >                     segPosition;
    std::vector< Eigen::Twistd >                            segVelocity;
    std::vector< double >                                   segMass;
    std::vector< Eigen::Vector3d >                          segCoM;
    std::vector< Eigen::Matrix<double,6,6> >                segMassMatrix;
    std::vector< Eigen::Vector3d >                          segMomentsOfInertia;
    std::vector< Eigen::Rotation3d >                        segInertiaAxes;
    std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJacobian;
    std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJdot;
    std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJointJacobian;
    std::vector< Eigen::Twistd >                            segJdotQdot;
    std::map< std::string, int >                            segIndexFromName;
    std::vector< std::string >                              segNameFromIndex;
    std::vector< std::vector< iCubModel_Joint* > >          segJoints;
    Eigen::Matrix<double,6,Eigen::Dynamic>                  Jroot;
    Eigen::Matrix<double,6,Eigen::Dynamic>                  dJroot;
    bool                                                    modelIsUpToDate;
    int                                                     nbDofs;

     Pimpl()
    :nbSegments(33)
    ,actuatedDofs(32)
    ,lowerLimits(32)
    ,upperLimits(32)
    ,q(32)
    ,dq(32)
    ,Hroot(0,0,0)
    ,Troot(0,0,0,0,0,0)
    ,M(32,32)
    ,Minv(32,32)
    ,B(32,32)
    ,n(32)
    ,l(32)
    ,g(32)
    ,total_mass(0)
    ,comPosition(0,0,0)
    ,comVelocity(0,0,0)
    ,comJdotQdot(0,0,0)
    ,comJacobian(3,32)
    ,comJacobianDot(3,32)
    ,segPosition(33, Eigen::Displacementd(0,0,0))
    ,segVelocity(33, Eigen::Twistd(0,0,0,0,0,0))
    ,segMass(33, 0)
    ,segCoM(33, Eigen::Vector3d(0,0,0))
    ,segMassMatrix(33, Eigen::Matrix<double,6,6>::Zero())
    ,segMomentsOfInertia(33, Eigen::Vector3d(0,0,0))
    ,segInertiaAxes(33, Eigen::Rotation3d(1,0,0,0))
    ,segJacobian(33, Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6,32))
    ,segJdot(33, Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6,32))
    ,segJointJacobian(33, Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6,32))
    ,segJdotQdot(33, Eigen::Twistd(0,0,0,0,0,0))
    ,segNameFromIndex(33)
    ,segJoints(33)
    ,Jroot(Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6,32))
    ,dJroot(Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6,32))
    ,modelIsUpToDate(false)
    ,nbDofs(32)
    {
        initContantVariables();
        outdateModel();
    }

    void initContantVariables()
    {
        segIndexFromName["waist"] = 0;
        segNameFromIndex[0]     = "waist";
        segMass[0]             = 0.20297;
        segCoM[0]              = Eigen::Vector3d(0.006, 0.0, -0.1);
        segMomentsOfInertia[0] = Eigen::Vector3d(0.000258841296496, 0.000290758328996, 0.000106643820833);
        segInertiaAxes[0]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[0]      << 0.0022885412965, 0.0, 0.000121782, 0.0, 0.020297, 0.0, 0.0, 0.002327765249, 0.0, -0.020297, 0.0, -0.00121782, 0.000121782, 0.0, 0.000113950740833, 0.0, 0.00121782, 0.0, 0.0, -0.020297, 0.0, 0.20297, 0.0, 0.0, 0.020297, 0.0, 0.00121782, 0.0, 0.20297, 0.0, 0.0, -0.00121782, 0.0, 0.0, 0.0, 0.20297;
        segJoints[0].push_back( new iCubModel_JointRz(0, 1, 0, Eigen::Displacementd(0.0,-0.0681,-0.1199,0.707106781185,-0.707106781188,0.0,0.0)));
        segJoints[0].push_back( new iCubModel_JointRz(0, 7, 6, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,-0.5,0.500000000003)));
        segJoints[0].push_back( new iCubModel_JointRz(0, 27, 26, Eigen::Displacementd(0.0,0.0681,-0.1199,0.707106781185,-0.707106781188,0.0,0.0)));
        
        segIndexFromName["l_hip_1"] = 1;
        segNameFromIndex[1]     = "l_hip_1";
        segMass[1]             = 0.32708;
        segCoM[1]              = Eigen::Vector3d(0.0, 0.0, 0.0375);
        segMomentsOfInertia[1] = Eigen::Vector3d(0.000122682256667, 0.000122682256667, 0.00023615176);
        segInertiaAxes[1]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[1]      << 0.000582638506667, 0.0, 0.0, 0.0, -0.0122655, 0.0, 0.0, 0.000582638506667, 0.0, 0.0122655, 0.0, 0.0, 0.0, 0.0, 0.00023615176, 0.0, 0.0, 0.0, 0.0, 0.0122655, 0.0, 0.32708, 0.0, 0.0, -0.0122655, 0.0, 0.0, 0.0, 0.32708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.32708;
        segJoints[1].push_back( new iCubModel_JointRz(1, 2, 1, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,-0.5,-0.500000000003,0.5)));
        
        segIndexFromName["l_hip_2"] = 2;
        segNameFromIndex[2]     = "l_hip_2";
        segMass[2]             = 1.5304;
        segCoM[2]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[2] = Eigen::Vector3d(0.0010850536, 0.0010850536, 0.0007353572);
        segInertiaAxes[2]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[2]      << 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0007353572, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304;
        segJoints[2].push_back( new iCubModel_JointRz(2, 3, 2, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,-0.5,-0.500000000003,0.5)));
        
        segIndexFromName["l_thigh"] = 3;
        segNameFromIndex[3]     = "l_thigh";
        segMass[3]             = 2.32246;
        segCoM[3]              = Eigen::Vector3d(-0.0, 0.0, -0.1501968774);
        segMomentsOfInertia[3] = Eigen::Vector3d(0.0139763503018, 0.00147239522875, 0.0137814870406);
        segInertiaAxes[3]      = Eigen::Rotation3d(0.499999999997, 0.5, 0.500000000003, 0.5);
        segMassMatrix[3]      << 0.0661740990267, -9.94457796952e-16, -6.28177535894e-14, -1.38740911706e-28, 0.348826239886, -4.30061988307e-33, -9.94457796952e-16, 0.0663689622879, 1.38821792552e-18, -0.348826239886, -1.84987882274e-28, 1.38740911706e-28, -6.28177535894e-14, 1.38821792552e-18, 0.00147239522875, 6.50317069587e-33, -6.34363897162e-29, -7.80240048226e-48, 9.07190041004e-30, -0.348826239886, 6.50317069587e-33, 2.32246, 4.169918745e-27, 2.05734924082e-27, 0.348826239886, 2.27783586383e-29, -6.34363897162e-29, 3.23117426779e-27, 2.32246, 6.04868577449e-23, -1.10778927987e-33, 5.69952004022e-29, -7.80240048226e-48, 1.61558713389e-27, 6.04868577735e-23, 2.32246;
        segJoints[3].push_back( new iCubModel_JointRz(3, 4, 3, Eigen::Displacementd(0.0,0.0,-0.2236,0.499999999997,0.5,-0.500000000003,-0.5)));
        
        segIndexFromName["l_shank"] = 4;
        segNameFromIndex[4]     = "l_shank";
        segMass[4]             = 0.95262;
        segCoM[4]              = Eigen::Vector3d(0.0, -0.1065, -0.0);
        segMomentsOfInertia[4] = Eigen::Vector3d(0.00383792736375, 0.0004726185975, 0.00383792736375);
        segInertiaAxes[4]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[4]      << 0.0146427815587, 0.0, 0.0, 0.0, 0.0, -0.10145403, 0.0, 0.0004726185975, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0146427815587, 0.10145403, 0.0, 0.0, 0.0, 0.0, 0.10145403, 0.95262, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.95262, 0.0, -0.10145403, 0.0, 0.0, 0.0, 0.0, 0.95262;
        segJoints[4].push_back( new iCubModel_JointRz(4, 5, 4, Eigen::Displacementd(-0.0,-0.213,0.0,0.0,-0.707106781185,-0.707106781188,7.45058059692e-09)));
        
        segIndexFromName["l_ankle_1"] = 5;
        segNameFromIndex[5]     = "l_ankle_1";
        segMass[5]             = 0.14801;
        segCoM[5]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[5] = Eigen::Vector3d(7.1165058125e-05, 7.1165058125e-05, 4.442150125e-05);
        segInertiaAxes[5]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[5]      << 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.442150125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801;
        segJoints[5].push_back( new iCubModel_JointRz(5, 6, 5, Eigen::Displacementd(0.0,0.0,0.0,0.707106781185,-0.707106781188,0.0,0.0)));
        
        segIndexFromName["l_foot"] = 6;
        segNameFromIndex[6]     = "l_foot";
        segMass[6]             = 0.6747;
        segCoM[6]              = Eigen::Vector3d(-0.0047312139, 0.0, 0.0151082333);
        segMomentsOfInertia[6] = Eigen::Vector3d(0.000811937097733, 0.000336068314542, 0.000731741708191);
        segInertiaAxes[6]      = Eigen::Rotation3d(0.537182077292, 0.459821069367, 0.45982106937, 0.537182077294);
        segMassMatrix[6]      << 0.000876332562725, -4.57348952538e-16, 0.000108533100316, -6.00890142649e-30, -0.0101935250075, -5.73927123427e-31, -4.57348952538e-16, 0.000981045996233, 3.07767379075e-16, 0.0101935250075, 3.69778549322e-31, 0.00319215001833, 0.000108533100316, 3.07767379075e-16, 0.000360586358508, -3.41737009332e-30, -0.00319215001833, -4.45852782126e-31, -1.26772412659e-29, 0.0101935250075, -2.26027138273e-30, 0.6747, -1.62798920394e-17, -6.30870139115e-18, -0.0101935250075, -5.76238239361e-31, -0.00319215001833, -1.627989204e-17, 0.6747, 1.45977331394e-18, -2.85808003747e-31, 0.00319215001833, -2.94859874486e-31, -3.12114700404e-17, 1.45977331384e-18, 0.6747;
        
        segIndexFromName["lap_belt_1"] = 7;
        segNameFromIndex[7]     = "lap_belt_1";
        segMass[7]             = 3.623;
        segCoM[7]              = Eigen::Vector3d(-0.0383, -0.0173, 0.0);
        segMomentsOfInertia[7] = Eigen::Vector3d(0.0142217845833, 0.0105504779167, 0.00606792116667);
        segInertiaAxes[7]      = Eigen::Rotation3d(0.923879532511, 0.0, 0.0, -0.382683432366);
        segMassMatrix[7]      << 0.01347045892, -0.0042362169033, 0.0, 0.0, 0.0, -0.0626779, -0.0042362169033, 0.01770067372, 0.0, 0.0, 0.0, 0.1387609, 0.0, 0.0, 0.0124667913067, 0.0626779, -0.1387609, 0.0, 0.0, 0.0, 0.0626779, 3.623, -1.29670579829e-16, 0.0, 0.0, 0.0, -0.1387609, 1.56341953272e-16, 3.623, 0.0, -0.0626779, 0.1387609, 0.0, 0.0, 0.0, 3.623;
        segJoints[7].push_back( new iCubModel_JointRz(7, 8, 7, Eigen::Displacementd(0.032,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0)));
        
        segIndexFromName["lap_belt_2"] = 8;
        segNameFromIndex[8]     = "lap_belt_2";
        segMass[8]             = 0.91179;
        segCoM[8]              = Eigen::Vector3d(0.0, 0.0, 0.008);
        segMomentsOfInertia[8] = Eigen::Vector3d(0.00093397689, 0.00093397689, 0.000438115095);
        segInertiaAxes[8]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[8]      << 0.00099233145, 0.0, 0.0, 0.0, -0.00729432, 0.0, 0.0, 0.00099233145, 0.0, 0.00729432, 0.0, 0.0, 0.0, 0.0, 0.000438115095, 0.0, 0.0, 0.0, 0.0, 0.00729432, 0.0, 0.91179, 0.0, 0.0, -0.00729432, 0.0, 0.0, 0.0, 0.91179, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.91179;
        segJoints[8].push_back( new iCubModel_JointRz(8, 9, 8, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,-0.500000000003,-0.5)));
        
        segIndexFromName["chest"] = 9;
        segNameFromIndex[9]     = "chest";
        segMass[9]             = 4.12925;
        segCoM[9]              = Eigen::Vector3d(0.0, 0.0, -0.1094482037);
        segMomentsOfInertia[9] = Eigen::Vector3d(0.0101058081992, 0.0116382113674, 0.00890101366667);
        segInertiaAxes[9]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[9]      << 0.059569719398, 0.0, 0.0, 0.0, 0.451938995128, 0.0, 0.0, 0.0611021225662, 0.0, -0.451938995128, 0.0, 0.0, 0.0, 0.0, 0.00890101366667, 0.0, 0.0, 0.0, 0.0, -0.451938995128, 0.0, 4.12925, 0.0, 0.0, 0.451938995128, 0.0, 0.0, 0.0, 4.12925, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.12925;
        segJoints[9].push_back( new iCubModel_JointRz(9, 10, 9, Eigen::Displacementd(0.0060472293,0.0225685672,-0.1433,0.430459334574,0.430459334576,-0.5609855268,-0.560985526797)));
        segJoints[9].push_back( new iCubModel_JointRz(9, 17, 16, Eigen::Displacementd(-0.0060472293,0.0225685672,-0.1433,0.430459334574,-0.430459334576,-0.5609855268,0.560985526797)));
        segJoints[9].push_back( new iCubModel_JointRz(9, 24, 23, Eigen::Displacementd(0.0,-0.00231,-0.1933,0.499999999997,-0.5,0.500000000003,-0.5)));
        
        segIndexFromName["r_shoulder_1"] = 10;
        segNameFromIndex[10]     = "r_shoulder_1";
        segMass[10]             = 0.48278;
        segCoM[10]              = Eigen::Vector3d(0.0, 0.0, -0.07224);
        segMomentsOfInertia[10] = Eigen::Vector3d(0.000120855926667, 0.000120855926667, 0.00023197579);
        segInertiaAxes[10]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[10]      << 0.0026403001316, 0.0, 0.0, 0.0, 0.0348760272, 0.0, 0.0, 0.0026403001316, 0.0, -0.0348760272, 0.0, 0.0, 0.0, 0.0, 0.00023197579, 0.0, 0.0, 0.0, 0.0, -0.0348760272, 0.0, 0.48278, 0.0, 0.0, 0.0348760272, 0.0, 0.0, 0.0, 0.48278, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48278;
        segJoints[10].push_back( new iCubModel_JointRz(10, 11, 10, Eigen::Displacementd(0.0,0.0,-0.10774,0.499999999997,0.5,-0.500000000003,-0.5)));
        
        segIndexFromName["r_shoulder_2"] = 11;
        segNameFromIndex[11]     = "r_shoulder_2";
        segMass[11]             = 0.20779;
        segCoM[11]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[11] = Eigen::Vector3d(0.000107029165833, 0.000107029165833, 9.35055e-05);
        segInertiaAxes[11]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[11]      << 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.35055e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779;
        segJoints[11].push_back( new iCubModel_JointRz(11, 12, 11, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,-0.5,0.500000000003,-0.5)));
        
        segIndexFromName["r_arm"] = 12;
        segNameFromIndex[12]     = "r_arm";
        segMass[12]             = 1.1584;
        segCoM[12]              = Eigen::Vector3d(0.0, 0.0, -0.078);
        segMomentsOfInertia[12] = Eigen::Vector3d(0.0025450048, 0.0025450048, 0.0003915392);
        segInertiaAxes[12]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[12]      << 0.0095927104, 0.0, 0.0, 0.0, 0.0903552, 0.0, 0.0, 0.0095927104, 0.0, -0.0903552, 0.0, 0.0, 0.0, 0.0, 0.0003915392, 0.0, 0.0, 0.0, 0.0, -0.0903552, 0.0, 1.1584, 0.0, 0.0, 0.0903552, 0.0, 0.0, 0.0, 1.1584, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1584;
        segJoints[12].push_back( new iCubModel_JointRz(12, 13, 12, Eigen::Displacementd(0.0,0.0,-0.15228,0.430459334574,-0.430459334576,0.5609855268,-0.560985526797)));
        
        segIndexFromName["r_elbow_1"] = 13;
        segNameFromIndex[13]     = "r_elbow_1";
        segMass[13]             = 0.050798;
        segCoM[13]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[13] = Eigen::Vector3d(2.03192e-06, 2.03192e-06, 2.03192e-06);
        segInertiaAxes[13]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[13]      << 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798;
        segJoints[13].push_back( new iCubModel_JointRz(13, 14, 13, Eigen::Displacementd(0.015,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0)));
        
        segIndexFromName["r_forearm"] = 14;
        segNameFromIndex[14]     = "r_forearm";
        segMass[14]             = 0.48774;
        segCoM[14]              = Eigen::Vector3d(0.0, 0.0, -0.07);
        segMomentsOfInertia[14] = Eigen::Vector3d(0.000845416, 0.000845416, 9.7548e-05);
        segInertiaAxes[14]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[14]      << 0.003235342, 0.0, 0.0, 0.0, 0.0341418, 0.0, 0.0, 0.003235342, 0.0, -0.0341418, 0.0, 0.0, 0.0, 0.0, 9.7548e-05, 0.0, 0.0, 0.0, 0.0, -0.0341418, 0.0, 0.48774, 0.0, 0.0, 0.0341418, 0.0, 0.0, 0.0, 0.48774, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48774;
        segJoints[14].push_back( new iCubModel_JointRz(14, 15, 14, Eigen::Displacementd(0.0,0.0,-0.1373,0.499999999997,0.5,-0.500000000003,-0.5)));
        
        segIndexFromName["r_wrist_1"] = 15;
        segNameFromIndex[15]     = "r_wrist_1";
        segMass[15]             = 0.05;
        segCoM[15]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[15] = Eigen::Vector3d(7.91666666667e-06, 7.91666666667e-06, 2.5e-06);
        segInertiaAxes[15]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[15]      << 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05;
        segJoints[15].push_back( new iCubModel_JointRz(15, 16, 15, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5)));
        
        segIndexFromName["r_hand"] = 16;
        segNameFromIndex[16]     = "r_hand";
        segMass[16]             = 0.19099;
        segCoM[16]              = Eigen::Vector3d(-0.0345, 0.0, 0.0);
        segMomentsOfInertia[16] = Eigen::Vector3d(7.64119158333e-05, 8.49428025e-05, 0.000143019678333);
        segInertiaAxes[16]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[16]      << 7.64119158333e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00031226865, 0.0, 0.0, 0.0, 0.006589155, 0.0, 0.0, 0.000370345525833, 0.0, -0.006589155, 0.0, 0.0, 0.0, 0.0, 0.19099, 0.0, 0.0, 0.0, 0.0, -0.006589155, 0.0, 0.19099, 0.0, 0.0, 0.006589155, 0.0, 0.0, 0.0, 0.19099;
        
        segIndexFromName["l_shoulder_1"] = 17;
        segNameFromIndex[17]     = "l_shoulder_1";
        segMass[17]             = 0.48278;
        segCoM[17]              = Eigen::Vector3d(0.0, 0.0, 0.07224);
        segMomentsOfInertia[17] = Eigen::Vector3d(0.000120855926667, 0.000120855926667, 0.00023197579);
        segInertiaAxes[17]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[17]      << 0.0026403001316, 0.0, 0.0, 0.0, -0.0348760272, 0.0, 0.0, 0.0026403001316, 0.0, 0.0348760272, 0.0, 0.0, 0.0, 0.0, 0.00023197579, 0.0, 0.0, 0.0, 0.0, 0.0348760272, 0.0, 0.48278, 0.0, 0.0, -0.0348760272, 0.0, 0.0, 0.0, 0.48278, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48278;
        segJoints[17].push_back( new iCubModel_JointRz(17, 18, 17, Eigen::Displacementd(0.0,0.0,0.10774,0.499999999997,-0.5,-0.500000000003,0.5)));
        
        segIndexFromName["l_shoulder_2"] = 18;
        segNameFromIndex[18]     = "l_shoulder_2";
        segMass[18]             = 0.20779;
        segCoM[18]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[18] = Eigen::Vector3d(0.000107029165833, 0.000107029165833, 9.35055e-05);
        segInertiaAxes[18]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[18]      << 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000107029165833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.35055e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20779;
        segJoints[18].push_back( new iCubModel_JointRz(18, 19, 18, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,-0.500000000003,-0.5)));
        
        segIndexFromName["l_arm"] = 19;
        segNameFromIndex[19]     = "l_arm";
        segMass[19]             = 1.1584;
        segCoM[19]              = Eigen::Vector3d(0.0, 0.0, 0.078);
        segMomentsOfInertia[19] = Eigen::Vector3d(0.0025450048, 0.0025450048, 0.0003915392);
        segInertiaAxes[19]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[19]      << 0.0095927104, 0.0, 0.0, 0.0, -0.0903552, 0.0, 0.0, 0.0095927104, 0.0, 0.0903552, 0.0, 0.0, 0.0, 0.0, 0.0003915392, 0.0, 0.0, 0.0, 0.0, 0.0903552, 0.0, 1.1584, 0.0, 0.0, -0.0903552, 0.0, 0.0, 0.0, 1.1584, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1584;
        segJoints[19].push_back( new iCubModel_JointRz(19, 20, 19, Eigen::Displacementd(0.0,0.0,0.15228,0.560985526795,-0.560985526797,-0.430459334579,0.430459334577)));
        
        segIndexFromName["l_elbow_1"] = 20;
        segNameFromIndex[20]     = "l_elbow_1";
        segMass[20]             = 0.050798;
        segCoM[20]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[20] = Eigen::Vector3d(2.03192e-06, 2.03192e-06, 2.03192e-06);
        segInertiaAxes[20]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[20]      << 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.03192e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050798;
        segJoints[20].push_back( new iCubModel_JointRz(20, 21, 20, Eigen::Displacementd(-0.015,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0)));
        
        segIndexFromName["l_forearm"] = 21;
        segNameFromIndex[21]     = "l_forearm";
        segMass[21]             = 0.48774;
        segCoM[21]              = Eigen::Vector3d(0.0, 0.0, 0.07);
        segMomentsOfInertia[21] = Eigen::Vector3d(0.000845416, 0.000845416, 9.7548e-05);
        segInertiaAxes[21]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[21]      << 0.003235342, 0.0, 0.0, 0.0, -0.0341418, 0.0, 0.0, 0.003235342, 0.0, 0.0341418, 0.0, 0.0, 0.0, 0.0, 9.7548e-05, 0.0, 0.0, 0.0, 0.0, 0.0341418, 0.0, 0.48774, 0.0, 0.0, -0.0341418, 0.0, 0.0, 0.0, 0.48774, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48774;
        segJoints[21].push_back( new iCubModel_JointRz(21, 22, 21, Eigen::Displacementd(0.0,0.0,0.1373,0.499999999997,0.5,-0.500000000003,-0.5)));
        
        segIndexFromName["l_wrist_1"] = 22;
        segNameFromIndex[22]     = "l_wrist_1";
        segMass[22]             = 0.05;
        segCoM[22]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[22] = Eigen::Vector3d(7.91666666667e-06, 7.91666666667e-06, 2.5e-06);
        segInertiaAxes[22]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[22]      << 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.91666666667e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05;
        segJoints[22].push_back( new iCubModel_JointRz(22, 23, 22, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5)));
        
        segIndexFromName["l_hand"] = 23;
        segNameFromIndex[23]     = "l_hand";
        segMass[23]             = 0.19099;
        segCoM[23]              = Eigen::Vector3d(0.0345, 0.0, 0.0);
        segMomentsOfInertia[23] = Eigen::Vector3d(7.64119158333e-05, 8.49428025e-05, 0.000143019678333);
        segInertiaAxes[23]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[23]      << 7.64119158333e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00031226865, 0.0, 0.0, 0.0, -0.006589155, 0.0, 0.0, 0.000370345525833, 0.0, 0.006589155, 0.0, 0.0, 0.0, 0.0, 0.19099, 0.0, 0.0, 0.0, 0.0, 0.006589155, 0.0, 0.19099, 0.0, 0.0, -0.006589155, 0.0, 0.0, 0.0, 0.19099;
        
        segIndexFromName["neck_1"] = 24;
        segNameFromIndex[24]     = "neck_1";
        segMass[24]             = 0.28252;
        segCoM[24]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[24] = Eigen::Vector3d(0.000155480173333, 0.000155480173333, 3.17835e-05);
        segInertiaAxes[24]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[24]      << 0.000155480173333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000155480173333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.17835e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.28252, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.28252, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.28252;
        segJoints[24].push_back( new iCubModel_JointRz(24, 25, 24, Eigen::Displacementd(0.0,0.033,0.0,0.499999999997,0.5,0.500000000003,0.5)));
        
        segIndexFromName["neck_2"] = 25;
        segNameFromIndex[25]     = "neck_2";
        segMass[25]             = 0.1;
        segCoM[25]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[25] = Eigen::Vector3d(1.47e-05, 1.47e-05, 1.125e-05);
        segInertiaAxes[25]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[25]      << 1.47e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.47e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1;
        segJoints[25].push_back( new iCubModel_JointRz(25, 26, 25, Eigen::Displacementd(0.0,0.0,0.001,0.499999999997,-0.5,0.500000000003,-0.5)));
        
        segIndexFromName["head"] = 26;
        segNameFromIndex[26]     = "head";
        segMass[26]             = 0.78;
        segCoM[26]              = Eigen::Vector3d(0.0, 0.0, 0.0825);
        segMomentsOfInertia[26] = Eigen::Vector3d(0.0019968, 0.0019968, 0.0019968);
        segInertiaAxes[26]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[26]      << 0.007305675, 0.0, 0.0, 0.0, -0.06435, 0.0, 0.0, 0.007305675, 0.0, 0.06435, 0.0, 0.0, 0.0, 0.0, 0.0019968, 0.0, 0.0, 0.0, 0.0, 0.06435, 0.0, 0.78, 0.0, 0.0, -0.06435, 0.0, 0.0, 0.0, 0.78, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.78;
        
        segIndexFromName["r_hip_1"] = 27;
        segNameFromIndex[27]     = "r_hip_1";
        segMass[27]             = 0.32708;
        segCoM[27]              = Eigen::Vector3d(0.0, 0.0, -0.0375);
        segMomentsOfInertia[27] = Eigen::Vector3d(0.000122682256667, 0.000122682256667, 0.00023615176);
        segInertiaAxes[27]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[27]      << 0.000582638506667, 0.0, 0.0, 0.0, 0.0122655, 0.0, 0.0, 0.000582638506667, 0.0, -0.0122655, 0.0, 0.0, 0.0, 0.0, 0.00023615176, 0.0, 0.0, 0.0, 0.0, -0.0122655, 0.0, 0.32708, 0.0, 0.0, 0.0122655, 0.0, 0.0, 0.0, 0.32708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.32708;
        segJoints[27].push_back( new iCubModel_JointRz(27, 28, 27, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5)));
        
        segIndexFromName["r_hip_2"] = 28;
        segNameFromIndex[28]     = "r_hip_2";
        segMass[28]             = 1.5304;
        segCoM[28]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[28] = Eigen::Vector3d(0.0010850536, 0.0010850536, 0.0007353572);
        segInertiaAxes[28]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[28]      << 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0010850536, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0007353572, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5304;
        segJoints[28].push_back( new iCubModel_JointRz(28, 29, 28, Eigen::Displacementd(0.0,0.0,0.0,0.499999999997,0.5,0.500000000003,0.5)));
        
        segIndexFromName["r_thigh"] = 29;
        segNameFromIndex[29]     = "r_thigh";
        segMass[29]             = 2.32246;
        segCoM[29]              = Eigen::Vector3d(0.0, 0.0, 0.1501968774);
        segMomentsOfInertia[29] = Eigen::Vector3d(0.0139763503018, 0.00147239522875, 0.0137814870406);
        segInertiaAxes[29]      = Eigen::Rotation3d(0.499999999997, 0.5, 0.500000000003, 0.5);
        segMassMatrix[29]      << 0.0661740990267, -9.94457796952e-16, -6.28177535894e-14, 1.38740911706e-28, -0.348826239886, 4.30061988307e-33, -9.94457796952e-16, 0.0663689622879, 1.38821792552e-18, 0.348826239886, 1.84987882274e-28, -1.38740911706e-28, -6.28177535894e-14, 1.38821792552e-18, 0.00147239522875, -6.50317069587e-33, 6.34363897162e-29, 7.80240048226e-48, -9.07190041004e-30, 0.348826239886, -6.50317069587e-33, 2.32246, 4.169918745e-27, 2.05734924082e-27, -0.348826239886, -2.27783586383e-29, 6.34363897162e-29, 3.23117426779e-27, 2.32246, 6.04868577449e-23, 1.10778927987e-33, -5.69952004022e-29, 7.80240048226e-48, 1.61558713389e-27, 6.04868577735e-23, 2.32246;
        segJoints[29].push_back( new iCubModel_JointRz(29, 30, 29, Eigen::Displacementd(0.0,0.0,0.2236,0.499999999997,-0.5,0.500000000003,-0.5)));
        
        segIndexFromName["r_shank"] = 30;
        segNameFromIndex[30]     = "r_shank";
        segMass[30]             = 0.95262;
        segCoM[30]              = Eigen::Vector3d(0.0, -0.1065, -0.0);
        segMomentsOfInertia[30] = Eigen::Vector3d(0.00383792736375, 0.0004726185975, 0.00383792736375);
        segInertiaAxes[30]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[30]      << 0.0146427815587, 0.0, 0.0, 0.0, 0.0, -0.10145403, 0.0, 0.0004726185975, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0146427815587, 0.10145403, 0.0, 0.0, 0.0, 0.0, 0.10145403, 0.95262, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.95262, 0.0, -0.10145403, 0.0, 0.0, 0.0, 0.0, 0.95262;
        segJoints[30].push_back( new iCubModel_JointRz(30, 31, 30, Eigen::Displacementd(-0.0,-0.213,0.0,0.0,-0.707106781185,-0.707106781188,7.45058059692e-09)));
        
        segIndexFromName["r_ankle_1"] = 31;
        segNameFromIndex[31]     = "r_ankle_1";
        segMass[31]             = 0.14801;
        segCoM[31]              = Eigen::Vector3d(0.0, 0.0, 0.0);
        segMomentsOfInertia[31] = Eigen::Vector3d(7.1165058125e-05, 7.1165058125e-05, 4.442150125e-05);
        segInertiaAxes[31]      = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        segMassMatrix[31]      << 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.1165058125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.442150125e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.14801;
        segJoints[31].push_back( new iCubModel_JointRz(31, 32, 31, Eigen::Displacementd(0.0,0.0,0.0,0.707106781185,0.707106781188,0.0,0.0)));
        
        segIndexFromName["r_foot"] = 32;
        segNameFromIndex[32]     = "r_foot";
        segMass[32]             = 0.6747;
        segCoM[32]              = Eigen::Vector3d(-0.0047312139, 0.0, -0.0151082333);
        segMomentsOfInertia[32] = Eigen::Vector3d(0.000811937097733, 0.000336068314542, 0.000731741708191);
        segInertiaAxes[32]      = Eigen::Rotation3d(0.459821069364, 0.537182077294, 0.537182077297, 0.459821069367);
        segMassMatrix[32]      << 0.000876332562724, -4.57369809339e-16, -0.00010853310032, 2.92125053965e-30, 0.0101935250075, -3.01215443302e-31, -4.57369809339e-16, 0.000981045996233, -3.07751754741e-16, -0.0101935250075, 8.32001735975e-32, 0.00319215001833, -0.00010853310032, -3.07751754741e-16, 0.000360586358509, -2.46364958486e-30, -0.00319215001833, 3.65926689434e-33, -2.86578375725e-31, -0.0101935250075, 5.11526993229e-31, 0.6747, -3.59031715592e-17, 1.38303539628e-17, 0.0101935250075, 3.42969604496e-30, -0.00319215001833, -3.59031715591e-17, 0.6747, 2.44386452964e-17, 2.20326385638e-31, 0.00319215001833, -6.54816181092e-33, 3.82181265801e-18, 2.44386452964e-17, 0.6747;
        
        
    }

    void outdateModel()
    {
        modelIsUpToDate = false;
    }

    void updateModel()
    {
        
        if (!modelIsUpToDate)
        {
            recursiveUpdateModel(0, Hroot, Troot, Jroot, dJroot); //0 because this index must be the root link of the robot
            
            M.setZero(); //Should be written in one line;
            for (int i=0; i < nbSegments; i++)
            {
                M += segJacobian[i].transpose() * segMassMatrix[i] * segJacobian[i];
                Minv = M.inverse();
            }
            
            modelIsUpToDate = true;
        }
        
    }

    void recursiveUpdateModel(int segIdx, const Eigen::Displacementd& H_0_seg, const Eigen::Twistd& T_s_0_s, const Eigen::Matrix<double,6,Eigen::Dynamic>& J_s_0_s, const Eigen::Matrix<double,6,Eigen::Dynamic>& dJ_s_0_s)
    {
        
        segPosition[segIdx] = H_0_seg;
        segVelocity[segIdx] = T_s_0_s;
        segJacobian[segIdx] = J_s_0_s;
        segJdot[segIdx]     = dJ_s_0_s;
        
        segJdotQdot[segIdx] = dJ_s_0_s * T_s_0_s;
        
        
        for (int jidx=0; jidx < segJoints[segIdx].size(); jidx++)
        {
            int cdof      = segJoints[segIdx][jidx]->dof;
            double q_dof  = q[cdof];
            double dq_dof = dq[cdof];
        
            // DATA //
            Eigen::Displacementd  H_seg_child   =  segJoints[segIdx][jidx]->H_seg_joint * segJoints[segIdx][jidx]->get_H_joint_child( q_dof ) ;
            AdjointMatrix         Ad_child_seg  =  H_seg_child.inverse().adjoint();
            AdjointMatrix         Ad_joint_seg  =  segJoints[segIdx][jidx]->H_seg_joint.inverse().adjoint();
            Eigen::Twistd         T_c_j_c       =  segJoints[segIdx][jidx]->get_twist( dq_dof );
        
            AdjointMatrix         dAd_child_seg = segJoints[segIdx][jidx]->get_inv_dAdjoint( q_dof, dq_dof ) * Ad_joint_seg;
        
        
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
            J_c_0_c.col(cdof) = segJoints[segIdx][jidx]->jacobian;
            J_c_0_c += Ad_child_seg * J_s_0_s;
            
            // dJ_child/0(child)  =  (... same reasoning as for T_child/0(child) ...)
            //                    =  [ 0 0 0 ... dJ_child/joint(child) ... 0 0 ]  +  dAd_child_parent * J_parent/0(parent) + Ad_child_parent * dJ_parent/0(parent)
            Eigen::Matrix<double,6,Eigen::Dynamic> dJ_c_0_c = Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6, nbDofs);
            dJ_c_0_c.col(cdof) = segJoints[segIdx][jidx]->djacobian;
            dJ_c_0_c += dAd_child_seg * J_s_0_s + Ad_child_seg * dJ_s_0_s;
        
        
            recursiveUpdateModel(segJoints[segIdx][jidx]->child, H_0_child, T_c_0_c, J_c_0_c, dJ_c_0_c);
        }
        
    }

};



//=================================  Class methods  =================================//
 iCubModel::iCubModel(const std::string& robotName)
:orcisir::ISIRModel(robotName, 32, false)
,pimpl( new Pimpl() )
{
    
}

 iCubModel::~iCubModel()
{
    
}

int iCubModel::nbSegments() const
{
    pimpl->updateModel();
    return pimpl->nbSegments;
}

const Eigen::VectorXd& iCubModel::getActuatedDofs() const
{
    pimpl->updateModel();
    return pimpl->actuatedDofs;
}

const Eigen::VectorXd& iCubModel::getJointLowerLimits() const
{
    pimpl->updateModel();
    return pimpl->lowerLimits;
}

const Eigen::VectorXd& iCubModel::getJointUpperLimits() const
{
    pimpl->updateModel();
    return pimpl->upperLimits;
}

const Eigen::VectorXd& iCubModel::getJointPositions() const
{
    pimpl->updateModel();
    return pimpl->q;
}

const Eigen::VectorXd& iCubModel::getJointVelocities() const
{
    pimpl->updateModel();
    return pimpl->dq;
}

const Eigen::Displacementd& iCubModel::getFreeFlyerPosition() const
{
    pimpl->updateModel();
    return pimpl->Hroot;
}

const Eigen::Twistd& iCubModel::getFreeFlyerVelocity() const
{
    pimpl->updateModel();
    return pimpl->Troot;
}

const Eigen::MatrixXd& iCubModel::getInertiaMatrix() const
{
    pimpl->updateModel();
    return pimpl->M;
}

const Eigen::MatrixXd& iCubModel::getInertiaMatrixInverse() const
{
    pimpl->updateModel();
    return pimpl->Minv;
}

const Eigen::MatrixXd& iCubModel::getDampingMatrix() const
{
    pimpl->updateModel();
    return pimpl->B;
}

const Eigen::VectorXd& iCubModel::getNonLinearTerms() const
{
    pimpl->updateModel();
    return pimpl->n;
}

const Eigen::VectorXd& iCubModel::getLinearTerms() const
{
    pimpl->updateModel();
    return pimpl->l;
}

const Eigen::VectorXd& iCubModel::getGravityTerms() const
{
    pimpl->updateModel();
    return pimpl->g;
}

double iCubModel::getMass() const
{
    pimpl->updateModel();
    return pimpl->total_mass;
}

const Eigen::Vector3d& iCubModel::getCoMPosition() const
{
    pimpl->updateModel();
    return pimpl->comPosition;
}

const Eigen::Vector3d& iCubModel::getCoMVelocity() const
{
    pimpl->updateModel();
    return pimpl->comVelocity;
}

const Eigen::Vector3d& iCubModel::getCoMJdotQdot() const
{
    pimpl->updateModel();
    return pimpl->comJdotQdot;
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& iCubModel::getCoMJacobian() const
{
    pimpl->updateModel();
    return pimpl->comJacobian;
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& iCubModel::getCoMJacobianDot() const
{
    pimpl->updateModel();
    return pimpl->comJacobianDot;
}

const Eigen::Displacementd& iCubModel::getSegmentPosition(int index) const
{
    pimpl->updateModel();
    return pimpl->segPosition[index];
}

const Eigen::Twistd& iCubModel::getSegmentVelocity(int index) const
{
    pimpl->updateModel();
    return pimpl->segVelocity[index];
}

double iCubModel::getSegmentMass(int index) const
{
    pimpl->updateModel();
    return pimpl->segMass[index];
}

const Eigen::Vector3d& iCubModel::getSegmentCoM(int index) const
{
    pimpl->updateModel();
    return pimpl->segCoM[index];
}

const Eigen::Matrix<double,6,6>& iCubModel::getSegmentMassMatrix(int index) const
{
    pimpl->updateModel();
    return pimpl->segMassMatrix[index];
}

const Eigen::Vector3d& iCubModel::getSegmentMomentsOfInertia(int index) const
{
    pimpl->updateModel();
    return pimpl->segMomentsOfInertia[index];
}

const Eigen::Rotation3d& iCubModel::getSegmentInertiaAxes(int index) const
{
    pimpl->updateModel();
    return pimpl->segInertiaAxes[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& iCubModel::getSegmentJacobian(int index) const
{
    pimpl->updateModel();
    return pimpl->segJacobian[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& iCubModel::getSegmentJdot(int index) const
{
    pimpl->updateModel();
    return pimpl->segJdot[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& iCubModel::getJointJacobian(int index) const
{
    pimpl->updateModel();
    return pimpl->segJointJacobian[index];
}

const Eigen::Twistd& iCubModel::getSegmentJdotQdot(int index) const
{
    pimpl->updateModel();
    return pimpl->segJdotQdot[index];
}

void iCubModel::doSetJointPositions(const Eigen::VectorXd& q)
{
    pimpl->q = q;
    pimpl->outdateModel();
}

void iCubModel::doSetJointVelocities(const Eigen::VectorXd& dq)
{
    pimpl->dq = dq;
    pimpl->outdateModel();
}

void iCubModel::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
    pimpl->Hroot = Hroot;
    pimpl->outdateModel();
}

void iCubModel::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
    pimpl->Troot = Troot;
    pimpl->outdateModel();
}

int iCubModel::doGetSegmentIndex(const std::string& name) const
{
    pimpl->updateModel();
    return pimpl->segIndexFromName[name];
}

const std::string& iCubModel::doGetSegmentName(int index) const
{
    pimpl->updateModel();
    return pimpl->segNameFromIndex[index];
}

void iCubModel::printAllData()
{
    std::cout<<"nbSegments:\n";
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
        std::cout<<"segPosition "<<idx<<":\n";
        std::cout<<getSegmentPosition(idx)<<"\n";
    
        std::cout<<"segVelocity "<<idx<<":\n";
        std::cout<<getSegmentVelocity(idx)<<"\n";
    
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


