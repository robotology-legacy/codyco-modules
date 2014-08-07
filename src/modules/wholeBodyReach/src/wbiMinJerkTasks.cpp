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
#include <Eigen/Geometry>

// TEMP
#include <yarp/os/Time.h>
using namespace yarp::math;
// END TEMP

using namespace wholeBodyReach;
using namespace std;
using namespace wbi;
using namespace Eigen;
using namespace paramHelp;

MinJerkPDLinkPoseTask::MinJerkPDLinkPoseTask(string taskName, string linkName, double sampleTime, wholeBodyInterface* robot)
: WbiAbstractTask(taskName, 6, robot),
  WbiEqualityTask(6, robot->getDoFs()+6),
  WbiPDTask(6, DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
  MinJerkTask(3, sampleTime),   // the trajectory generator is 3d because it works only for the linear part
  _linkName(linkName)
{
    _J.setZero(6, robot->getDoFs()+6);
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
    _H.R.getAxisAngle(_pose.data()+3);
//    getLogger().sendMsg(_name+" H:\n"+_H.toString(2)+"\nPose: "+toString(_pose,2), MSG_STREAM_INFO);
    
    // update reference trajectory
    _trajGen.computeNextValues(_poseDes.head<3>());
    _posRef = _trajGen.getPos();
    
    // compute PD
    _dvStar.head<3>() = _trajGen.getAcc()   + _Kd.head<3>().cwiseProduct(_trajGen.getVel() - _v.head<3>())
                                            + _Kp.head<3>().cwiseProduct(_posRef - _pose.head<3>());
    computeOrientationError(_H.R, _Hdes.R, _orientationError);
//    _dvStar.tail<3>() = _Kp.tail<3>().cwiseProduct(_orientationError) - _Kd.tail<3>().cwiseProduct(_v.tail<3>());
    _dvStar.tail<3>() = - _Kd.tail<3>().cwiseProduct(_v.tail<3>());
    
    // TEMP
//    _dvStar.tail<3>().setZero();
    // END TEMP
    
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
//    cout<<_name<<" H:\n"<<_H.toString()<<endl;
}

void MinJerkPDLinkPoseTask::linkParameterPoseDes(ParamHelperServer* paramHelper, int paramId)
{
    _paramId_poseDes = paramId;
    paramHelper->linkParam(paramId, _poseDes.data());
    paramHelper->registerParamValueChangedCallback(paramId, this);
    parameterUpdated(paramHelper->getParamProxy(paramId));
}

void MinJerkPDLinkPoseTask::linkParameterPose(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _pose.data());
}

void MinJerkPDLinkPoseTask::linkParameterPosRef(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _posRef.data());
}

void MinJerkPDLinkPoseTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id==_paramId_poseDes)
    {
        _Hdes.p[0] = _poseDes[0];
        _Hdes.p[1] = _poseDes[1];
        _Hdes.p[2] = _poseDes[2];
        // convert from axis/angle to rotation matrix
        _Hdes.R = Rotation::axisAngle(_poseDes.data()+3);
//        cout<<_name<<" H des:\n"<<_Hdes.toString()<<endl;
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
    WbiPDTask(6, false),
    MinJerkTask(3, sampleTime),   // the trajectory generator is 3d because it works only for the linear part
    _sampleTime(sampleTime)
{
    _robotMass = -1.0;
}

bool MinJerkPDMomentumTask::update(RobotState& state)
{
    assert(_A_eq.rows()==6 && _A_eq.cols()==state.qJ.size()+6);
    assert(_a_eq.size()==6);
    
    bool res = true;
    res = res && _robot->computeH(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _H);
    res = res && _robot->computeCentroidalMomentum(state.qJ.data(), state.xBase, state.dqJ.data(),
                                                   state.vBase.data(), _momentum.data());
    _v = _momentum.head<3>()/_robotMass;    // compute CoM velocity
    _momentumIntegral += _momentum*_sampleTime;
    
//    res = res && _robot->computeJacobian(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _A_eq.data());
//    _v = _A_eq.topRows<3>()*state.dq;  // compute CoM velocity
    
    // copy data into Eigen vector
    _com(0) = _H.p[0]; _com(1) = _H.p[1]; _com(2) = _H.p[2];
    
    // update reference trajectory
    _trajGen.computeNextValues(_comDes);
    _comRef = _trajGen.getPos();
    _a_eq.head<3>() = _robotMass * ( -state.g + _trajGen.getAcc()
                                     + _Kd.head<3>().cwiseProduct(_trajGen.getVel() - _v)
                                     + _Kp.head<3>().cwiseProduct(_comRef - _com) );
    _a_eq.tail<3>() = - _Kp.tail<3>().cwiseProduct(_momentumIntegral.tail<3>())
                      - _Kd.tail<3>().cwiseProduct(_momentum.tail<3>());
    
//    getLogger().sendMsg("Momentum: Kp*e    = "+toString(_Kp.head<3>().cwiseProduct(_trajGen.getPos()-_com),2), MSG_STREAM_INFO);
//    getLogger().sendMsg("Momentum: Kd*de   = "+toString(_Kd.head<3>().cwiseProduct(_trajGen.getVel()-_v),2),   MSG_STREAM_INFO);
//    getLogger().sendMsg("Momentum: ddx_ref = "+toString(_trajGen.getAcc(),2), MSG_STREAM_INFO);
//    cout<<"state.g "<< state.g.transpose() << endl;
//    cout<<"_Kp = "<< _Kp.transpose() << endl;
//    cout<<"_Kd = "<< _Kd.transpose() << endl;
//    cout<<"_trajGen.getAcc() = "<< _trajGen.getAcc().transpose() << endl;
//    cout<<"_trajGen.getVel() = "<< _trajGen.getVel().transpose() << endl;
//    cout<<"_trajGen.getPos() = "<< _trajGen.getPos().transpose() << endl;
    
//    cout<<"_com = "<< _com.transpose() << endl;
//    cout<<"_a_eq = "<< _a_eq.transpose() << endl;
    
    return res;
}

void MinJerkPDMomentumTask::init(RobotState& state)
{
    // initialize trajectory generator
    bool res = _robot->computeH(state.qJ.data(), state.xBase, iWholeBodyModel::COM_LINK_ID, _H);
    _com(0) = _H.p[0]; _com(1) = _H.p[1]; _com(2) = _H.p[2];
    _trajGen.init(_com);
    
    /// TEMP
    _comDes = _com;
    
    // compute robot's mass
    int n = _robot->getDoFs();
    MatrixRXd M(n+6, n+6);
    res = res && _robot->computeMassMatrix(state.qJ.data(), state.xBase, M.data());
    _robotMass = M(0,0);
    cout<<"The robot's mass is "<<_robotMass<<" kg."<<endl;
    assert(res);
    
    // reset momentum integral
    _momentumIntegral.setZero();
    
}

void MinJerkPDMomentumTask::linkParameterComDes(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _comDes.data());
}

void MinJerkPDMomentumTask::linkParameterCom(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _com.data());
}

void MinJerkPDMomentumTask::linkParameterComRef(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _comRef.data());
}

void MinJerkPDMomentumTask::linkParameterComVel(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _v.data());
}

void MinJerkPDMomentumTask::linkParameterMomentum(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _momentum.data());
}

void MinJerkPDMomentumTask::linkParameterMomentumIntegral(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _momentumIntegral.data());
}

/*********************************************************************************************************/
/******************************************* MinJerkPDPostureTask ****************************************/
/*********************************************************************************************************/

MinJerkPDPostureTask::MinJerkPDPostureTask(std::string taskName, double sampleTime, wbi::wholeBodyInterface* robot)
:   WbiAbstractTask(taskName, robot->getDoFs(), robot),
    WbiEqualityTask(robot->getDoFs(), robot->getDoFs()),
    WbiPDTask(robot->getDoFs(), DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS),
    MinJerkTask(robot->getDoFs(), sampleTime),
    _sampleTime(sampleTime)
{
    _Ki.setZero(robot->getDoFs());
    _qDes.setZero(robot->getDoFs());
    _qRef.setZero(robot->getDoFs());
    _qErrorIntegral.setZero(robot->getDoFs());
}

bool MinJerkPDPostureTask::update(RobotState& state)
{
    _trajGen.computeNextValues(_qDes);  // the trajectory generator uses deg (not rad)
    _qRef           = _trajGen.getPos();
    _qErrorIntegral += _sampleTime * _Ki.cwiseProduct(_qRef - WBR_RAD2DEG*state.qJ);

//    getLogger().sendMsg("qErrInt: "+jointToString(_qErrorIntegral,1),MSG_STREAM_INFO);
    
    _a_eq  = WBR_DEG2RAD * ( _trajGen.getAcc() +
                             _Kd.cwiseProduct(_trajGen.getVel() - WBR_RAD2DEG*state.dqJ) +
                             _Kp.cwiseProduct(_trajGen.getPos() - WBR_RAD2DEG*state.qJ) +
                             _qErrorIntegral);
    return true;
}

void MinJerkPDPostureTask::init(RobotState& state)
{
    _trajGen.init(WBR_RAD2DEG*state.qJ);
    _qErrorIntegral.setZero();
    
    // TEMP
    _qDes = WBR_RAD2DEG*state.qJ;
    
#define DEBUG_MINJERKPDPOSTURETASK
#ifdef  DEBUG_MINJERKPDPOSTURETASK
//    cout<<"  Posture initial pos "<<_trajGen.getPos().transpose()<<endl;
    for(int i=0; i<1; i++)
    {
        _trajGen.computeNextValues(_qDes);  // the trajectory generator uses deg (not rad)
        _a_eq  = WBR_DEG2RAD * ( _trajGen.getAcc() +
                                _Kd.cwiseProduct(_trajGen.getVel() - WBR_RAD2DEG*state.dqJ) +
                                _Kp.cwiseProduct(_trajGen.getPos() - WBR_RAD2DEG*state.qJ) +
                                _qErrorIntegral);
        
//        cout<<"*** Time "<< i*_trajGen.getSampleTime() << endl;
//        cout<<"  ddqDes              "<<_trajGen.getAcc().norm()<<endl;;
//        cout<<"  dqDes               "<<_trajGen.getVel().norm()<<endl;
        cout<<"  dq                  "<<WBR_RAD2DEG*state.dqJ.norm()<<endl;
//        cout<<"  Posture pos         "<<_trajGen.getPos().transpose()<<endl;
        cout<<"  Kd*(dqDes - dqJ)    "<<(_Kd.cwiseProduct(_trajGen.getVel() - WBR_RAD2DEG*state.dqJ)).norm()<<endl;
//        cout<<"  Kp*(qDes - qJ)      "<<(_Kp.cwiseProduct(_trajGen.getPos() - WBR_RAD2DEG*state.qJ)).norm()<<endl;
        cout<<"  _a_eq (°/s^2)       "<<WBR_RAD2DEG*_a_eq.norm()<<endl;
    }
    _trajGen.init(WBR_RAD2DEG*state.qJ);
#endif
}

void MinJerkPDPostureTask::linkParameterPostureDes(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _qDes.data());
}

void MinJerkPDPostureTask::linkParameterPostureRef(ParamHelperServer* paramHelper, int paramId)
{
    paramHelper->linkParam(paramId, _qRef.data());
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
    parameterUpdated(paramHelper->getParamProxy(paramId));
}

void MinJerkTask::parameterUpdated(const ParamProxyInterface *pp)
{
    if(pp->id==_paramId_trajDur)
    {
        cout<<"MinJerkTask set trajectory duration to "<<_trajDuration<<endl;
        _trajGen.setTrajectoryDuration(_trajDuration);
    }
}

