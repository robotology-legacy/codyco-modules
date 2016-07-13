#include <IK.h>
#include "UtilityFunctions.h"
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWholeBodyStates.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <iCub/iDynTree/DynTree.h>
#include <iDynTree/Estimation/simpleLeggedOdometryKDL.h>

#include <cmath>

//using namespace qpOASES;
//using namespace RigidBodyDynamics;
using namespace Eigen;
typedef Eigen::Matrix<double,-1,-1,Eigen::RowMajor> MatrixXd_row;

void getEulerAngles(Eigen::Matrix3d R, Eigen::Vector3d& angles, std::string order)
{
    if(order.compare("123") == 0)
    {
        angles[0] = atan2(R(1,2),R(2,2));
        angles[1] = -asin(R(0,2));
        angles[2] = atan2(R(0,1),R(0,0));
    } else if(order.compare("321") == 0)
    {
        angles[0] = atan2(-R(1,0),R(0,0));
        angles[1] = asin(R(2,0));
        angles[2] = atan2(-R(2,1),R(2,2));
    }
}


//by Kevin
Eigen::MatrixXd CalcOrientationEulerXYZ (const Eigen::VectorXd &input,
                                         std::string order) {
    Eigen::Matrix3d R1 = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d R2 = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d R3 = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

    if(order.compare("123")==0)
    {
        R1 = rotx(input[0]);
        R2 = roty(input[1]);
        R3 = rotz(input[2]);
        result = R1*R2*R3;
    }
    else if(order.compare("321")==0)
    {
        R1 = rotx(input[2]);
        R2 = roty(input[1]);
        R3 = rotz(input[0]);
        result = R3*R2*R1;
    } else
    {
        std::cout << "Order " << order << " not implemented yet!" << std::endl;
        abort();
    }

    return result;
}


Eigen::Vector3d CalcAngularVelocityfromMatrix (const Eigen::Matrix3d &RotMat) {


    float tol = 1e-12;

    Eigen::Vector3d l = Eigen::Vector3d (RotMat(2,1) - RotMat(1,2), RotMat(0,2) - RotMat(2,0), RotMat(1,0) - RotMat(0,1));
    if(l.norm() > tol)
    {
        double preFactor = atan2(l.norm(),(RotMat.trace() - 1.0))/l.norm();
        return preFactor*l;
    }
    else if((RotMat(0,0)>0 && RotMat(1,1)>0 && RotMat(2,2) > 0) || l.norm() < tol)
    {
        return Eigen::Vector3d::Zero();
    }
    else
    {
        return Eigen::Vector3d (M_PI/2*(RotMat(0,0) + 1.0),M_PI/2*(RotMat(1,1) + 1.0),M_PI/2*(RotMat(2,2) + 1.0));
    }
}

//FIXME: parameter for switching fixed foot is temporary
bool IKinematics (yarpWbi::yarpWholeBodyModel* wbm,
                  yarpWbi::yarpWholeBodyStates* wbs,
                  floatingBaseOdometry * odometry,
                  const Eigen::VectorXd &Qinit,
                  const std::vector<unsigned int>& body_id,
                  const std::vector<Eigen::Vector3d>& target_pos,
                  const std::vector<Eigen::Matrix3d>& target_orientation,
                  std::vector<Eigen::Vector3d>& body_point,
                  Eigen::VectorXd &Qres,
                  bool switch_fixed,
                  double step_tol,
                  double lambda,
                  unsigned int max_iter)
{
    assert (Qinit.size() == wbm->getDoFs());
    assert (body_id.size() == target_pos.size());
    assert (body_id.size() == body_point.size());
    assert (body_id.size() == target_orientation.size());

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6 * body_id.size(), wbm->getDoFs()+6);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(6 * body_id.size());

    Qres = Qinit;

//FIXME: parameter for switching fixed foot switch_fixed, the fixed link update should happen only once at the beginning
    odometry->update(Qres.data(), switch_fixed);

    for (unsigned int ik_iter = 0; ik_iter < max_iter; ik_iter++) {
//        UpdateKinematicsCustom (model, &Qres, NULL, NULL);

        for (unsigned int k = 0; k < body_id.size(); k++) {
            // Initializing Jacobian matrix to 6 x DOFS
            MatrixXd_row G (Eigen::MatrixXd::Zero(6, wbm->getDoFs() + 6));
            wbi::Frame base_H_world;
            // Update odometry and compute world_H_floatingbase
            //!!!!: parameter for switching fixed foot switch_fixed, the fixed link update should NOT happen inside the IK iterations
            odometry->update(Qres.data(), false);
            wbi::Frame world_H_floatingbase;
            odometry->get_world_H_floatingbase(world_H_floatingbase);
            wbm->computeJacobian(Qres.data(), world_H_floatingbase, body_id[k], G.data(), body_point[k].data());
//            CalcPointJacobian6D (model, Qres, body_id[k], body_point[k], G, false);

            // Calculate coordinates of a point in the root reference frame
            Eigen::VectorXd point_pose(7);
            wbm->forwardKinematics(Qres.data(), world_H_floatingbase, body_id[k], point_pose.data(), body_point[k].data());
            Eigen::Vector3d point_base = point_pose.head(3);
//            Eigen::Vector3d point_base = CalcBodyToBaseCoordinates (model, Qres, body_id[k], body_point[k], false);

            // Calculate orientation of a given body as 3x3 matrix
            Eigen::VectorXd orientationAxisAngle(4);
            orientationAxisAngle = point_pose.tail(4);
            Eigen::AngleAxis<double> aa(orientationAxisAngle(3), Eigen::Vector3d(orientationAxisAngle(0),
                                                                                 orientationAxisAngle(1),
                                                                                 orientationAxisAngle(2)));
            Eigen::Matrix3d R = aa.toRotationMatrix();
//            Eigen::Matrix3d R = CalcBodyWorldOrientation(model, Qres, body_id[k], false);

            // Error in orientation as the corresponding angular velocity of the error rotation matrix
            Eigen::Vector3d ort_rates = Eigen::Vector3d::Zero();

            //TODO: Thoroughly check the following equation!!! Possible source of error
            //Fixed by Yue
            // original formulation for RBDL
            // ort_rates = R.transpose()*CalcAngularVelocityfromMatrix(R*target_orientation[k].transpose());
            if(!target_orientation[k].isZero(0))
                ort_rates = R*CalcAngularVelocityfromMatrix(target_orientation[k]*R.transpose());

            for (unsigned int i = 0; i < 6; i++) {
                for (unsigned int j = 0; j < wbm->getDoFs()+6; j++) {
                    unsigned int row = k * 6 + i;
                    J(row, j) = G (i,j);
                }
            }
            //NOTE: With RBDL first, we would have ort_rates and then (target_pos - point_base)
            for (unsigned int i = 0; i < 3; i++) {
                e[k * 6 + i] = target_pos[k][i] - point_base[i];
                e[k * 6 + i + 3] = ort_rates[i];
            }
        }

        // abort if we are getting "close"
        if (e.norm() < step_tol) {
//            std::cerr << "Reached target close enough after " << ik_iter << " steps" << std::endl;
            return true;
        }

        double wn = lambda;

        // "task space" from puppeteer
        Eigen::MatrixXd Ek = Eigen::MatrixXd::Zero (e.size(), e.size());

        for (size_t ei = 0; ei < e.size(); ei ++) {
            Ek(ei,ei) = e[ei] * e[ei] * 0.5 + wn;
        }

        Eigen::MatrixXd JJT_Ek_wnI = J * J.transpose() + Ek;

        Eigen::VectorXd delta_theta = J.transpose() * JJT_Ek_wnI.colPivHouseholderQr().solve (e);
//        std::cerr << "Size of Qres: " << Qres.size() << " while size of delta_theta " << delta_theta.size() << std::endl;
        //TODO: Here, size of Qres is 15 while size delta is 21
        Qres = Qres + delta_theta.tail(15);
        if (delta_theta.norm() < step_tol) {
//            std::cerr << "Reached target close enough with small delta_theta after " << ik_iter << " steps" << std::endl;
            return true;
        }
    }
    return false;
}
