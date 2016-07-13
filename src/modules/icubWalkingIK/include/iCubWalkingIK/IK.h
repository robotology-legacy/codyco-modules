#include <iostream>
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWholeBodyStates.h>
#include "floatingBaseOdometry.h"

void getEulerAngles(Eigen::Matrix3d R, Eigen::Vector3d& angles, std::string order);
Eigen::MatrixXd CalcOrientationEulerXYZ (const Eigen::VectorXd &input, std::string order);
Eigen::Vector3d CalcAngularVelocityfromMatrix (const Eigen::Matrix3d &RotMat);
//FIXME: parameter for switching fixed foot
bool IKinematics (yarpWbi::yarpWholeBodyModel* wbm,
                  yarpWbi::yarpWholeBodyStates* wbs,
                  floatingBaseOdometry* odometry,
                  const Eigen::VectorXd &Qinit,
                  const std::vector<unsigned int>& body_id,
                  const std::vector<Eigen::Vector3d>& target_pos,
                  const std::vector<Eigen::Matrix3d>& target_orientation,
                  std::vector<Eigen::Vector3d>& body_point,
                  Eigen::VectorXd &Qres,
                  bool switch_fixed = false,
                  double step_tol = 1e-8,
                  double lambda = 0.001,
                  unsigned int max_iter = 100);
