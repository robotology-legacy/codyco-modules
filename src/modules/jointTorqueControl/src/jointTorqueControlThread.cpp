/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Daniele Pucci
 * email:  daniele.pucci@iit.it
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

#include <jointTorqueControl/jointTorqueControlThread.h>
#include <jointTorqueControl/jointTorqueControlConstants.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <wbiIcub/wbiIcubUtil.h>

using namespace jointTorqueControl;
using namespace wbiIcub;


jointTorqueControlThread::jointTorqueControlThread(int period, string _name, string _robotName, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    : RateThread(period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi), sendCommands(SEND_COMMANDS_NONACTIVE),
    monitoredJointId(0)
{
    mustStop = false;
    status = CONTROL_OFF;
    printCountdown = 0;
    gravityCompOn = 0;
}

//*************************************************************************************************************************
bool jointTorqueControlThread::threadInit()
{
    // link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_AJ,		            activeJoints.data()));    // constant size
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KT,		            kt.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVP,	            kvp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVN,	            kvn.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCP,	            kcp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCN,	            kcn.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KI,		            ki.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP,		            kp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KS,		            ks.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KD,		            kd.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_DES,		        qDes.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_GRAV_COMP_ON,		&gravityCompOn));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COULOMB_VEL_THR,	coulombVelThr.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VMAX,	            Vmax.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SENDCMD,            &sendCommands));
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MONITORED_JOINT,    &monitoredJointName));

    // link controller input streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_OFFSET,	        tauOffset.data()));
	
    // link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VM,		            motorVoltage.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU,	            tau.data()));
	
	//link monitored variables
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_MEAS,	        &monitor.tauMeas));
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD_MONITOR,	    &monitor.tauDes));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD_PLUS_PI,	    &monitor.tadDesPlusPI));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL,	        &monitor.dq));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL_SIGN,	    &monitor.dqSign));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_DESIRED,	    &monitor.pwmDes));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_MEASURED,       &monitor.pwmMeas));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FEEDFORWARD,	&monitor.pwmFF));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FEEDBACK,	    &monitor.pwmFB));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_TORQUE_FF,	    &monitor.pwmTorqueFF));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FRICTION_FF,	&monitor.pwmFrictionFF));

    // Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_AJ,                 this));
	YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_SENDCMD,            this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_MONITORED_JOINT,    this));

    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

    tau 			= VectorNd::Constant(0.0); 
	etau 			= VectorNd::Constant(0.0); 
	tauD 			= VectorNd::Constant(0.0); 
    tauOffset 		= VectorNd::Constant(0.0); 
	//tauM 			= VectorNd::Constant(0.0); 
	integralState 	= VectorNd::Constant(0.0); 
	motorVoltage	= VectorNd::Constant(0.0);
 //   pwmMeas	        = VectorNd::Constant(0.0);
	//dq              = VectorNd::Constant(0.0);
    dqSign          = VectorNd::Constant(0.0);

    ///< thread constants
    zeroN.setZero();
    zero6[0] = zero6[1] = zero6[2] = zero6[3] = zero6[4] = zero6[5] = 0.0;
    ddxB[0] = ddxB[1] = ddxB[3] = ddxB[4] = ddxB[5] = 0.0;
    ddxB[2] = 9.81;     ///< gravity acceleration

    activeJointsOld = activeJoints;

    if(!updateJointToMonitor())
        printf("Specified monitored joint name was not recognized: %s\n", monitoredJointName.c_str());

    robot->setEstimationParameter(ESTIMATE_JOINT_VEL, ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE, &JOINT_VEL_ESTIMATION_WINDOW);

    if(!readRobotStatus(true))
    {
        printf("Error while initializing the controller: it was not possible to read the robot status!\n");
        return false;
    }
    
    return true;
}

//*************************************************************************************************************************
void jointTorqueControlThread::run()
{
	paramHelper->lock();
    paramHelper->readStreamParams();
	
	if(status == CONTROL_ON)
    {
		double currentTime  = yarp::os::Time::now();
		double dt           = currentTime - oldTime;
		
		readRobotStatus(false);

		for (int i=0; i < N_DOF; i++)
        {
            dqSign(i)       = fabs(dq(i))>coulombVelThr(i) ? sign(dq(i)) : pow(dq(i)/coulombVelThr(i),3);
            tauD(i)         = tauOffset(i) + ks(i)*(qDes(i)-q(i)) - kd(i)*dq(i) + gravityCompOn*tauGrav(i+6);

			if (activeJoints(i) == 1) 
            {
				etau(i) 			= tauM(i) - tauD(i);
				integralState(i) 	= saturation(integralState(i) + ki(i)*dt*etau(i), TORQUE_INTEGRAL_SATURATION, -TORQUE_INTEGRAL_SATURATION) ;
				tau(i) 				= tauD(i) - kp(i)*etau(i) - integralState(i);

                if(dq(i)>0)
                    motorVoltage(i) = kt(i)*tau(i) + kvp(i)*dq(i) + kcp(i)*dqSign(i);
                else
                    motorVoltage(i) = kt(i)*tau(i) + kvn(i)*dq(i) + kcn(i)*dqSign(i);
					
				if (sendCommands == SEND_COMMANDS_ACTIVE)
					robot->setControlReference(&motorVoltage(i), i);
			}
		}

		oldTime = currentTime;
    }

    prepareMonitorData();
    
    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool jointTorqueControlThread::readRobotStatus(bool blockingRead)
{
    // read joint angles and torques
    bool res =   robot->getEstimates(ESTIMATE_JOINT_VEL,    dq.data(),      -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_POS,    q.data(),       -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_TORQUE, tauM.data(),    -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_MOTOR_PWM,    pwmMeas.data(), -1.0, blockingRead);
    res = res && robot->inverseDynamics(q.data(), Frame(), zeroN.data(), zero6, zeroN.data(), ddxB, tauGrav.data());

    // convert angles from rad to deg
    q   *= CTRL_RAD2DEG;
    dq  *= CTRL_RAD2DEG;
    return res;
}

//*************************************************************************************************************************
void jointTorqueControlThread::startSending()
{
    resetIntegralState(-1);
	setControlModePWMOnJoints(sendCommands == SEND_COMMANDS_ACTIVE);
    status = CONTROL_ON;       //sets thread status to ON
    oldTime = yarp::os::Time::now();
    printf("Activating the torque control.\n"); 
}

//*************************************************************************************************************************
void jointTorqueControlThread::stopSending()
{
    status = CONTROL_OFF;
    setControlModePWMOnJoints(false);
    printf("Deactivating the torque control.\n");
}

//*************************************************************************************************************************
void jointTorqueControlThread::resetIntegralState(int j)
{
    if(j>=0)
    {
        integralState(j) = 0.0;
        return;
    }

	for (int i=0; i < N_DOF; i++)
		if (activeJoints(i) == 1) 
			integralState(i) = 0.0; 
}

//*************************************************************************************************************************
void jointTorqueControlThread::setControlModePWMOnJoints(bool torqueActive)
{
	for (int i=0; i < N_DOF; i++)
	{
		if (activeJoints(i) == 1 && torqueActive) 
			robot->setControlMode(CTRL_MODE_MOTOR_PWM, 0, i);
		else
			robot->setControlMode(CTRL_MODE_POS, 0, i);
	}
}

//*************************************************************************************************************************
void jointTorqueControlThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    case PARAM_ID_AJ:
        activeJointsChanged();
        break;
	case PARAM_ID_SENDCMD:
		//setControlModePWMOnJoints(sendCommands == SEND_COMMANDS_ACTIVE);
		break;
    case PARAM_ID_MONITORED_JOINT:
        if(!updateJointToMonitor())
            printf("Specified monitored joint name was not recognized: %s\n", monitoredJointName.c_str());
        break;
    default:
        printf("A callback is registered but not managed for the parameter %s\n", pd->name.c_str());
    }
}

//*************************************************************************************************************************
void jointTorqueControlThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:  startSending(); break;
    case COMMAND_ID_STOP:   stopSending();  break;
    default:    printf("A callback is registered but not managed for the command %s\n", cd.name.c_str());
    }
}

//*************************************************************************************************************************
bool jointTorqueControlThread::activeJointsChanged()
{
    for(int i=0; i<N_DOF; i++)
    {
        if(activeJoints(i)==1 && activeJointsOld(i)==0)         // joint has been activated
        {
            resetIntegralState(i);
            if(sendCommands == SEND_COMMANDS_ACTIVE)
                robot->setControlMode(CTRL_MODE_MOTOR_PWM, 0, i);
        }
        else if(activeJoints(i)==0 && activeJointsOld(i)==1)    // joint has been deactivated
        {
            robot->setControlMode(CTRL_MODE_POS, 0, i);
        }
    }
    activeJointsOld = activeJoints;
    return true;
}

//*************************************************************************************************************************
void jointTorqueControlThread::prepareMonitorData()
{
    int j = monitoredJointId;
    monitor.dq              = dq(j);
    monitor.dqSign          = dqSign(j);
    monitor.tauMeas         = tauM(j);
    monitor.tauDes          = tauD(j);
    monitor.tadDesPlusPI    = tau(j);
    monitor.pwmDes          = motorVoltage(j);
    monitor.pwmMeas         = pwmMeas(j);
    monitor.pwmTorqueFF     = kt(j)*tauD(j);
    monitor.pwmFrictionFF   = dq(j)>0 ? kvp(j)*dq(j) + kcp(j)*dqSign(j) : kvn(j)*dq(j) + kcn(j)*dqSign(j);
    monitor.pwmFF           = monitor.pwmTorqueFF + monitor.pwmFrictionFF;
    monitor.pwmFB           = -kt(j)*(kp(j)*etau(j) + integralState(j));
}

//*************************************************************************************************************************
bool jointTorqueControlThread::updateJointToMonitor()
{
    LocalId lid = globalToLocalIcubId(monitoredJointName);
    if(lid.bodyPart==iCub::skinDynLib::BODY_PART_UNKNOWN)
        return false;
    monitoredJointId = robot->getJointList().localToGlobalId(lid);
    return true;
}

//*************************************************************************************************************************
int jointTorqueControlThread::convertGlobalToLocalJointId(const Bottle &b)
{
    if(b.size()==0)
        return -1;

    LocalId lid;
    if(b.get(0).isString())
    {
        string jointName = b.get(0).asString();
        lid = globalToLocalIcubId(jointName);
    }
    else if(b.get(0).isInt())
    {
        int jointId = b.get(0).asInt();
        lid = globalToLocalIcubId(jointId);
    }
    
    if(lid.bodyPart==iCub::skinDynLib::BODY_PART_UNKNOWN)
        return -1;

    int jid = robot->getJointList().localToGlobalId(lid);
    return jid;
}

//*************************************************************************************************************************
void jointTorqueControlThread::sendMsg(const string &s, MsgType type)
{
    if(printCountdown==0 && type>=MSG_DEBUG)
        printf("[jointTorqueControlThread] %s\n", s.c_str());
}
