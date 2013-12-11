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

using namespace jointTorqueControl;

// #warning remove this function
// double getTimeFunction(double a, double w){
// 	static double t0 = yarp::os::Time::now();
// 	double t = yarp::os::Time::now();
// 	return a*sin(w*(t-t0));
// }
// 
// 

jointTorqueControlThread::jointTorqueControlThread(int period, string _name, string _robotName, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    : RateThread(period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi), sendCommands(SEND_COMMANDS_NONACTIVE),
    monitoredJoint(0)
{
    mustStop = false;
    status = CONTROL_OFF;
    
	tau 			= VectorNd::Constant(0.0); 
	etau 			= VectorNd::Constant(0.0); 
	tauD 			= VectorNd::Constant(0.0); 
	tauM 			= VectorNd::Constant(0.0); 
	integralState 	= VectorNd::Constant(0.0); 
	motorVoltage	= VectorNd::Constant(0.0);
	
}

//*************************************************************************************************************************
bool jointTorqueControlThread::threadInit()
{
    // link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_AJ,		activeJoints.data()));    // constant size
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KT,		kt.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVP,	kvp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVN,	kvn.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCP,	kcp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCN,	kcn.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KI,		ki.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP,		kp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KS,		ks.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VMAX,	Vmax.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SENDCMD,&sendCommands));
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MONITORED_JOINT,&monitoredJoint));
	
    // link controller input streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD,	tauD.data()));
	
    // link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VM,		motorVoltage.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU,	tau.data()));
	
	//link monitored variables
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_V_OUT,	&monitoredVoltage));
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_OUT,	&monitoredTau));
	 
    // Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_AJ,     this));
	YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_SENDCMD,this));

    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));
    
    return true;
}

//*************************************************************************************************************************
void jointTorqueControlThread::run()
{
	//NOTE: Cannata says: "Never use a mutex in a (real time) control loop".... what should I do?
	paramHelper->lock();
    paramHelper->readStreamParams();
	
	if(status == CONTROL_ON)
    {
		double currentTime = yarp::os::Time::now();
		double	dt	=  (currentTime - oldTime);
		
		readRobotStatus(false);

		for (int i=0; i < N_DOF; i++)
        {
			if (activeJoints(i) == 1) 
            {
// #warning remove the tauD line
// 				tauD(i)				= getTimeFunction(2,1);
// #warning remove the dq line
// 				dq(i)				= getTimeFunction(10,0.5);
				
				etau(i) 			= tauM(i) - tauD(i);
				integralState(i) 	= saturation(integralState(i) + dt*etau(i), 1000, -1000) ;
				tau(i) 				= tauD(i) - kp(i)*etau(i) - ki(i)*integralState(i);
				motorVoltage(i) 	= kt(i)*tau(i) + (kvp(i)*stepFunction(dq(i)) + kvn(i)*stepFunction(-dq(i)))*dq(i) + (kcp(i)*stepFunction(dq(i)) + kcn(i)*stepFunction(-dq(i)))*tanh(ks(i)*dq(i));
				printf("Err = %lf\tIntErr = %lf\ttau = %lf\ttauD = %lf\tV = %lf\tdq = %lf\tdt=%lf\n", etau(i), integralState(i), tau(i), tauD(i), motorVoltage(i),dq(i),dt);
					
				if (sendCommands == SEND_COMMANDS_ACTIVE)
					robot->setControlReference(&motorVoltage(i), i);
				else                
					printf("Not sending commands");
			}
		}
		oldTime = currentTime;
    }
    
    if (monitoredJoint >= 0 && monitoredJoint < N_DOF) {
		monitoredTau = tau(monitoredJoint);
		monitoredVoltage = motorVoltage(monitoredJoint);
	}
    
    paramHelper->sendStreamParams();
    paramHelper->unlock();
}

//*************************************************************************************************************************
void jointTorqueControlThread::startSending()
{
    status = CONTROL_ON;       //sets thread status to ON
    oldTime = yarp::os::Time::now();
}
//*************************************************************************************************************************
void jointTorqueControlThread::stopSending()
{
    status = CONTROL_OFF;
}

//*************************************************************************************************************************
void jointTorqueControlThread::threadRelease()
{

}

//*************************************************************************************************************************
void jointTorqueControlThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    case PARAM_ID_AJ:
		printf("%s",toString(activeJoints).c_str());
        resetIntegralStates();
		setControlModePWMOnJoints(sendCommands == SEND_COMMANDS_ACTIVE);
        break;
	case PARAM_ID_SENDCMD:
		setControlModePWMOnJoints(sendCommands == SEND_COMMANDS_ACTIVE);
		break;
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void jointTorqueControlThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
		setControlModePWMOnJoints(sendCommands == SEND_COMMANDS_ACTIVE);
        startSending();
        sendMsg("Activating the torque control.", MSG_INFO); break;
    case COMMAND_ID_STOP:
		setControlModePWMOnJoints(false);
        stopSending();
        sendMsg("Deactivating the torque control.", MSG_INFO); break;
    default:
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
string jointTorqueControlThread::readParamsFile(ifstream& fp)
{
    string lineStr;
    getline(fp,lineStr);
    return(lineStr);
}

//*************************************************************************************************************************
string jointTorqueControlThread::get_env_var( string const & key ) 
{
    char * val;
    val = getenv( key.c_str() );
    std::string retval = "";
    if (val != NULL) 
        retval = val;
    return retval;
}

//*************************************************************************************************************************
void jointTorqueControlThread::sendMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[jointTorqueControlThread] %s\n", s.c_str());
}

//*************************************************************************************************************************
void jointTorqueControlThread::fromListToVector(Bottle * pointerToList, VectorNd& vector) 
{
	for (int i=0; i < pointerToList->size(); i++)
    {
        vector(i) = pointerToList->get(i).asDouble();
    }
}

//*************************************************************************************************************************
double jointTorqueControlThread::stepFunction(double x) 
{
	if ( x >= 0)
		return 1;
	else
		return 0;
}

//*************************************************************************************************************************
bool jointTorqueControlThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res = robot->getEstimates(ESTIMATE_JOINT_VEL, dq.data(), -1.0, blockingRead);
//     res = res && robot->getEstimates(ESTIMATE_JOINT_TORQUE, tauM.data(), -1.0, blockingRead);
	tauM = tau;
    return res;
}

//*************************************************************************************************************************
void jointTorqueControlThread::resetIntegralStates()
{
	for (int i=0; i < N_DOF; i++)
	{
		if (activeJoints(i) == 1) 
		{
			integralState(i) = 0; 
		}
	}
}

//*************************************************************************************************************************
void jointTorqueControlThread::setControlModePWMOnJoints(bool torqueActive)
{
	for (int i=0; i < N_DOF; i++)
	{
		if (activeJoints(i) == 1 && torqueActive) 
		{
			robot->setControlMode(CTRL_MODE_MOTOR_PWM, 0, i);
			printf("Activating PWM control on joint %d\n", i);
		}
		else {
			robot->setControlMode(CTRL_MODE_POS, 0, i);
 			printf("Deactivating PWM control on joint %d\n", i);
		}
	}
}

double jointTorqueControlThread::saturation(double x, double xMax, double xMin)
{
// 	if (x > xMax)
// 		return xMax;
// 	if (x < xMin)
// 		return xMin;
	return x;
}


