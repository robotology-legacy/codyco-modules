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
#include <wholeBodyReach/Logger.h>

using namespace wholeBodyReach;
using namespace std;
using namespace wbi;
using namespace Eigen;

MinJerkPDLinkPoseTask::MinJerkPDLinkPoseTask(string taskName, string linkName, double sampleTime, wholeBodyInterface* robot)
: WbiAbstractTask(taskName, 6, robot),
  WbiEqualityTask(6, robot->getDoFs()+6),
  WbiPDTask(6, DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
  MinJerkTask(3, sampleTime),   // the trajectory generator is 3d because it works only for the linear part
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
    
    // copy data into Eigen vector
    _pose(0) = _H.p[0]; _pose(1) = _H.p[1]; _pose(2) = _H.p[2];
    _H.R.axisAngle(_pose.data()+3);
    
    // update reference trajectory
    _trajGen.computeNextValues(_poseDes.head<3>());
    _dvStar.head<3>() = _trajGen.getAcc()   + _Kd.head<3>().cwiseProduct(_trajGen.getVel()-_v.head<3>())
                                            + _Kp.head<3>().cwiseProduct(_trajGen.getPos()-_pose.head<3>());
    computeOrientationError(_H.R, _Hdes.R, _orientationError);
    _dvStar.tail<3>() = _Kp.tail<3>().cwiseProduct(_orientationError) - _Kd.tail<3>().cwiseProduct(_v.tail<3>());
    
    // update equality matrix and equality vectory
    _A_eq = _J;
    _a_eq = _dvStar - _dJdq;
    
    return res;
}

void MinJerkPDLinkPoseTask::init(RobotState& state)
{
    bool res = _robot->computeH(state.qJ.data(), state.xBase, _linkId, _H);
    assert(res);
    _pose(0) = _H.p[0]; _pose(1) = _H.p[1]; _pose(2) = _H.p[2];
    _trajGen.init(_pose.head<3>());
}

void MinJerkPDLinkPoseTask::linkParameterPoseDes(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_poseDes = paramId;
    paramHelper->linkParam(paramId, _poseDes.data());
    paramHelper->registerParamValueChangedCallback(paramId, this);
}

void MinJerkPDLinkPoseTask::linkParameterPose(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_pose = paramId;
    paramHelper->linkParam(paramId, _pose.data());
}

void MinJerkPDLinkPoseTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id==_paramId_poseDes)
    {
        _Hdes.p[0] = _poseDes[0];
        _Hdes.p[1] = _poseDes[1];
        _Hdes.p[2] = _poseDes[2];
        // convert from axis/angle to rotation matrix
        _Hdes.R.axisAngle(_poseDes.data()+3);
        return;
    }
    MinJerkTask::parameterUpdated(pp);
    WbiPDTask::parameterUpdated(pp);
}



/*********************************************************************************************************/
/******************************************* MinJerkPDMomentumTask ***************************************/
/*********************************************************************************************************/

MinJerkPDMomentumTask::MinJerkPDMomentumTask(std::string taskName, double sampleTime, wbi::wholeBodyInterface* robot)
:   WbiAbstractTask(taskName, 6, robot),
    WbiEqualityTask(6, robot->getDoFs()+6),
    WbiPDTask(6, DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
    MinJerkTask(3, sampleTime)   // the trajectory generator is 3d because it works only for the linear part
{
    _robotMass = -1.0;
}

bool MinJerkPDMomentumTask::update(RobotState& state)
{
    bool res = true;
    // the first time compute the total mass of the robot
    if(_robotMass<0.0)
    {
        int n = _robot->getDoFs();
        MatrixRXd M(n+6, n+6);
        res = res && _robot->computeMassMatrix(state.qJ.data(), state.xBase, M.data());
        _robotMass = M(0,0);
        cout<<"Robot mass is "<<_robotMass<<endl;
    }
    res = res && _robot->computeH(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _H);
    res = res && _robot->computeCentroidalMomentum(state.qJ.data(), state.xBase, state.dqJ.data(),
                                                   state.vBase.data(), _momentum.data());
    res = res && _robot->computeJacobian(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _A_eq.data());
    _v = _A_eq.topRows<3>()*state.dq;  // compute CoM velocity
    
    // copy data into Eigen vector
    _com(0) = _H.p[0]; _com(1) = _H.p[1]; _com(2) = _H.p[2];
    
    // update reference trajectory
    _trajGen.computeNextValues(_comDes);
    _a_eq.head<3>() = _robotMass * ( -state.g + _trajGen.getAcc()
                                     + _Kd.head<3>().cwiseProduct(_trajGen.getVel()-_v)
                                     + _Kp.head<3>().cwiseProduct(_trajGen.getPos()-_com) );
    _a_eq.tail<3>() = - _Kd.tail<3>().cwiseProduct(_momentum.tail<3>());
    
    getLogger().sendMsg("Momentum: Kp*e    = "+toString(_Kp.head<3>().cwiseProduct(_trajGen.getPos()-_com),2), MSG_STREAM_INFO);
    getLogger().sendMsg("Momentum: Kd*de   = "+toString(_Kd.head<3>().cwiseProduct(_trajGen.getVel()-_v),2),   MSG_STREAM_INFO);
    getLogger().sendMsg("Momentum: ddx_ref = "+toString(_trajGen.getAcc(),2), MSG_STREAM_INFO);
//    cout<<"state.g "<< state.g.transpose() << endl;
//    cout<<"_Kp = "<< _Kp.transpose() << endl;
//    cout<<"_Kd = "<< _Kd.transpose() << endl;
//    cout<<"_trajGen.getAcc() = "<< _trajGen.getAcc().transpose() << endl;
//    cout<<"_trajGen.getVel() = "<< _trajGen.getVel().transpose() << endl;
//    cout<<"_trajGen.getPos() = "<< _trajGen.getPos().transpose() << endl;
//    cout<<"_v = "<< _v.transpose() << endl;
//    cout<<"_com = "<< _com.transpose() << endl;
//    cout<<"_a_eq = "<< _a_eq.transpose() << endl;
    
    return res;
}

void MinJerkPDMomentumTask::init(RobotState& state)
{
    bool res = _robot->computeH(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _H);
    assert(res);
    _com(0) = _H.p[0]; _com(1) = _H.p[1]; _com(2) = _H.p[2];
    _trajGen.init(_com);
}

void MinJerkPDMomentumTask::linkParameterComDes(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_comDes = paramId;
    paramHelper->linkParam(paramId, _comDes.data());
}

void MinJerkPDMomentumTask::linkParameterCom(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_com = paramId;
    paramHelper->linkParam(paramId, _com.data());
}


/*********************************************************************************************************/
/******************************************* MinJerkPDPostureTask ****************************************/
/*********************************************************************************************************/

MinJerkPDPostureTask::MinJerkPDPostureTask(std::string taskName, double sampleTime, wbi::wholeBodyInterface* robot)
:   WbiAbstractTask(taskName, robot->getDoFs(), robot),
    WbiEqualityTask(robot->getDoFs(), robot->getDoFs()),
    WbiPDTask(robot->getDoFs(), DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
    MinJerkTask(robot->getDoFs(), sampleTime),
    _paramId_qDes(-1)
{
    _qDes.setZero(robot->getDoFs());
}

bool MinJerkPDPostureTask::update(RobotState& state)
{
    _trajGen.computeNextValues(_qDes);  // the trajectory generator uses deg (not rad)
    _a_eq  = WBR_DEG2RAD * (_trajGen.getAcc()
                            + _Kd.cwiseProduct(_trajGen.getVel() - WBR_RAD2DEG*state.dqJ)
                            + _Kp.cwiseProduct(_trajGen.getPos() - WBR_RAD2DEG*state.qJ));
    return true;
}

void MinJerkPDPostureTask::init(RobotState& state)
{
    _trajGen.init(WBR_RAD2DEG*state.qJ);
#ifdef DEBUG_MINJERKPDPOSTURETASK
    for(int i=0; i<10; i++)
    {
        _trajGen.computeNextValues(_qDes);  // the trajectory generator uses deg (not rad)
        cout<<"*** Time "<< i*_trajGen.getSampleTime() << endl;
        cout<<"  Posture acc "<<_trajGen.getAcc().transpose()<<endl;;
        cout<<"  Posture vel "<<_trajGen.getVel().transpose()<<endl;
        cout<<"  Posture pos "<<_trajGen.getPos().transpose()<<endl;
//        _a_eq += WBR_DEG2RAD * _Kd.cwiseProduct(_trajGen.getVel() - WBR_RAD2DEG*state.dqJ);
//        _a_eq += WBR_DEG2RAD * _Kp.cwiseProduct(_trajGen.getPos() - WBR_RAD2DEG*state.qJ);
    }
    _trajGen.init(WBR_RAD2DEG*state.qJ);
#endif
}


void MinJerkPDPostureTask::linkParameterPostureDes(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_qDes = paramId;
    paramHelper->linkParam(paramId, _qDes.data());
}


/*********************************************************************************************************/
/******************************************* ContactConstraint *******************************************/
/*********************************************************************************************************/

ContactConstraint::ContactConstraint(std::string name, std::string linkName, wbi::wholeBodyInterface* robot)
:   WbiAbstractTask(name, 6, robot),
    WbiEqualityTask(6, robot->getDoFs()+6),
    WbiInequalityTask(6,robot->getDoFs()+6),
    _linkName(linkName)
{
    _X.setIdentity(6, 6);
    if(!robot->getLinkId(linkName.c_str(), _linkId))
        cout<<"Error while trying to get the ID of link "<<linkName<<endl;
}

bool ContactConstraint::update(RobotState& state)
{
    bool res = true;
    // update equality matrix and equality vectory
    res = res && _robot->computeJacobian(state.qJ.data(), state.xBase, _linkId, _A_eq.data());
    res = res && _robot->computeDJdq(state.qJ.data(), state.xBase, state.dqJ.data(),
                                     state.vBase.data(), _linkId, _a_eq.data());
    _a_eq *= -1.0;  // _a_eq = -dJ*dq
    
    // compute force-momentum mapping matrix _X
    res = res && _robot->computeH(state.qJ.data(), state.xBase, _linkId, _H);
    res = res && _robot->computeH(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _Hcom);
    _p_com[0] = _H.p[0] - _Hcom.p[0];
    _p_com[1] = _H.p[1] - _Hcom.p[1];
    _p_com[2] = _H.p[2] - _Hcom.p[2];
    _X.bottomLeftCorner<3,3>() = crossProductMatrix(_p_com);
    
    return res;
}

/*********************************************************************************************************/
/******************************************* JointLimitTask **********************************************/
/*********************************************************************************************************/

JointLimitTask::JointLimitTask(std::string taskName, wbi::wholeBodyInterface* robot)
:   WbiAbstractTask(taskName, 6, robot),
    WbiInequalityTask(robot->getDoFs()+6, robot->getDoFs()+6)
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
    paramHelper->registerParamValueChangedCallback(paramId, this);
}

void MinJerkTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id==_paramId_trajDur)
    {
        cout<<"Set trajectory duration to "<<_trajDuration<<endl;
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

Eigen::MatrixR3d wholeBodyReach::crossProductMatrix(Eigen::VectorConst v)
{
    //  0 -z +y
    // +z  0 -x
    // -y +x  0
    MatrixR3d S;
    S(0,0) = S(1,1) = S(2,2) = 0.0;
    S(1,0) = v(2);
    S(2,0) = -v(1);
    S(2,1) = v(0);
    S(0,1) = -v(2);
    S(0,2) = v(1);
    S(1,2) = -v(0);
    return S;
}
