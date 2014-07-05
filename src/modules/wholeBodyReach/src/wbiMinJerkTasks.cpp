/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <wholeBodyReach/wbiMinJerkTasks.h>

using namespace wholeBodyReach;
using namespace std;
using namespace wbi;
using namespace Eigen;

MinJerkPDLinkPoseTask::MinJerkPDLinkPoseTask(string taskName, string linkName, wholeBodyInterface* robot)
: WbiEqualityTask(taskName, 6, robot),
  WbiPDTask(6, DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
  MinJerkTask(3),   // the trajectory generator is 3d because it works only for the linear part
  _linkName(linkName)
{
    if(!(_initSuccessfull = _robot->getLinkId(linkName.c_str(), _linkId)))
        printf("[MinJerkPDLinkPoseTask] Error while trying to get id of link %s\n", linkName.c_str());
}

bool MinJerkPDLinkPoseTask::update(RobotState& state)
{
    assert(_initSuccessfull);
    // compute stuff
    bool res = _robot->computeH(state.qJ.data(), state.xBase, _linkId, _H);
    res = res && _robot->computeJacobian(state.qJ.data(), state.xBase, _linkId, _J.data());
    res = res && _robot->computeDJdq(state.qJ.data(), state.xBase, state.dqJ.data(), state.vBase.data(), _linkId, _dJdq.data());
    _v = _J*state.dq;
    // convert homogeneous matrix to axis-angle notation
    _x(0) = _H.p[0]; _x(1) = _H.p[1]; _x(2) = _H.p[2];
    _H.R.getAxisAngle(_x.data()+3);
    
    // update reference trajectory
    _trajGen.computeNextValues(_positionDesired);
    _dvStar.head<3>() = _trajGen.getAcc()   + _Kd.head<3>().cwiseProduct(_trajGen.getVel()-_v.head<3>())
                                            + _Kp.head<3>().cwiseProduct(_trajGen.getPos()-_x.head<3>());
    computeOrientationError(_H.R, _Hdesired.R, _orientationError);
    _dvStar.tail<3>() = _Kp.tail<3>().cwiseProduct(_orientationError) - _Kd.tail<3>().cwiseProduct(_v.tail<3>());
    
    // update equality matrix and equality vectory
    _A_eq = _J;
    _a_eq = _dvStar - _dJdq;
    
    return res;
}


/*********************************************************************************************************/
/******************************************* MinJerkPDMomentumTask ***************************************/
/*********************************************************************************************************/

MinJerkPDMomentumTask::MinJerkPDMomentumTask(std::string taskName, wbi::wholeBodyInterface* robot)
:   WbiEqualityTask(taskName, 6, robot),
    WbiPDTask(6, DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
    MinJerkTask(3)   // the trajectory generator is 3d because it works only for the linear part
{}

bool MinJerkPDMomentumTask::update(RobotState& state)
{
    bool res = true;
    // compute stuff
    // update equality matrix and equality vectory
    return res;
}


/*********************************************************************************************************/
/******************************************* MinJerkPDPostureTask ****************************************/
/*********************************************************************************************************/

MinJerkPDPostureTask::MinJerkPDPostureTask(std::string taskName, wbi::wholeBodyInterface* robot)
:   WbiEqualityTask(taskName, robot->getDoFs(), robot),
    WbiPDTask(robot->getDoFs(), DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
    MinJerkTask(robot->getDoFs())
{}

bool MinJerkPDPostureTask::update(RobotState& state)
{
    bool res = true;
    // compute stuff
    // update equality matrix and equality vectory
    return res;
}


/*********************************************************************************************************/
/******************************************* ContactConstraint *******************************************/
/*********************************************************************************************************/

ContactConstraint::ContactConstraint(std::string name, std::string linkName, wbi::wholeBodyInterface* robot)
:   WbiEqualityTask(name,6,robot),
    WbiInequalityTask(name,6,robot),
    _linkName(linkName)
{}

bool ContactConstraint::update(RobotState& state)
{
    bool res = true;
    // compute stuff
    // update equality matrix and equality vectory
    return res;
}

/*********************************************************************************************************/
/******************************************* JointLimitTask **********************************************/
/*********************************************************************************************************/

JointLimitTask::JointLimitTask(std::string taskName, wbi::wholeBodyInterface* robot)
:   WbiInequalityTask(taskName, robot->getDoFs(), robot),
    WbiPDTask(robot->getDoFs())
{
    _qMin.resize(_m);
    _qMax.resize(_m);
    _dqMin.resize(_m);
    _dqMax.resize(_m);
    _ddqMin.resize(_m);
    _ddqMax.resize(_m);
}

bool JointLimitTask::update(RobotState& state)
{
    bool res = true;
    // compute stuff
    // update equality matrix and equality vectory
    return res;
}

bool JointLimitTask::setPositionLimits(VectorConst qMin, VectorConst qMax)
{
    if( !checkVectorSize(qMin) || !checkVectorSize(qMax))
        return false;
    _qMin = qMin;
    _qMax = qMax;
    return true;
}

bool JointLimitTask::setVelocityLimits(VectorConst dqMin, VectorConst dqMax)
{
    if( !checkVectorSize(dqMin) || !checkVectorSize(dqMax))
        return false;
    _dqMin = dqMin;
    _dqMax = dqMax;
    return true;
}

bool JointLimitTask::setAccelerationLimits(VectorConst ddqMin, VectorConst ddqMax)
{
    if( !checkVectorSize(ddqMin) || !checkVectorSize(ddqMax))
        return false;
    _ddqMin = ddqMin;
    _ddqMax = ddqMax;
    return true;
}

/*********************************************************************************************************/
/********************************************** MinJerkTask **********************************************/
/*********************************************************************************************************/

void MinJerkTask::linkParameterTrajectoryDuration(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_trajDur = paramId;
    paramHelper->linkParam(paramId, &_trajDuration);
}

void MinJerkTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id==_paramId_trajDur)
    {
        _trajGen.setTrajectoryDuration(_trajDuration);
    }
}


/*********************************************************************************************************/
/******************************************* UTILITIES ***************************************************/
/*********************************************************************************************************/

void wholeBodyReach::compute6DError(const wbi::Frame& H, const wbi::Frame& H_des, Eigen::VectorRef res)
{
    assert(res.size()>=6);
    Vector4d aa;
    Rotation Re = H.R; // Re = R_des * R.transposed();
    Re.setToInverse();
    H_des.R.rotateInPlace(Re);
    
    Re.getAxisAngle(aa.data());
    res[0] = H_des.p[0]-H.p[0];
    res[1] = H_des.p[1]-H.p[1];
    res[2] = H_des.p[2]-H.p[2];
    res[3] = aa[3] * aa[0];
    res[4] = aa[3] * aa[1];
    res[5] = aa[3] * aa[2];
}

void wholeBodyReach::computeOrientationError(const wbi::Rotation& R, const wbi::Rotation& R_des, Eigen::VectorRef res)
{
    assert(res.size()>=3);
    Vector4d aa;
    Rotation Re = R; // Re = R_des * R.transposed();
    Re.setToInverse();
    R_des.rotateInPlace(Re);
    Re.getAxisAngle(aa.data());
    res[0] = aa[3] * aa[0];
    res[1] = aa[3] * aa[1];
    res[2] = aa[3] * aa[2];
}
