#include <iostream>
#include <boost/concept_check.hpp>
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWholeBodyStates.h>

//#include <qpOASES.hpp>
#include <rbdl/rbdl.h>

//using namespace qpOASES;
//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;

void getEulerAngles(Eigen::Matrix3d R, Eigen::Vector3d& angles, std::string order);
Eigen::MatrixXd CalcOrientationEulerXYZ (const Eigen::VectorXd &input, std::string order);
Eigen::Vector3d CalcAngularVelocityfromMatrix (const Eigen::Matrix3d &RotMat);

bool IKinematics (wbi::iWholeBodyModel* wbm,
                  wbi::iWholeBodyStates* wbs,
                  const Eigen::VectorXd &Qinit,
                  const std::vector<unsigned int>& body_id,
                  const std::vector<Eigen::Vector3d>& target_pos,
                  const std::vector<Eigen::Matrix3d>& target_orientation,
                  std::vector<Eigen::Vector3d>& body_point,
                  Eigen::VectorXd &Qres,
                  double step_tol = 1e-8,
                  double lambda = 0.001,
                  unsigned int max_iter = 100);
