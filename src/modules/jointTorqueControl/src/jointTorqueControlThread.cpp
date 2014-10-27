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
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <Eigen/LU>

using namespace jointTorqueControl;
using namespace yarpWbi;

// #define IMPEDANCE_CONTROL
//#define GAZEBO_SIM

jointTorqueControlThread::jointTorqueControlThread(int period, string _name, string _robotName, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
: RateThread(period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi), sendCommands(SEND_COMMANDS_NONACTIVE),
monitoredJointId(0)
{
    mustStop = false;
    status = CONTROL_OFF;
    printCountdown = 0;
    gravityCompOn = 0;
    frictionCompensationFactor.setZero();
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
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP,                 kp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KI,		            ki.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KD,                 kd.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KSTIFF,		        kStiff.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KDAMP,		        kDamp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_DES,		        qDes.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_GRAV_COMP_ON,		&gravityCompOn));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COULOMB_VEL_THR,	coulombVelThr.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VMAX,	            Vmax.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SENDCMD,            &sendCommands));
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MONITORED_JOINT,    &monitoredJointName));

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_FRICTION_COMPENSATION,  frictionCompensationFactor.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TORQUE_FILT_CUT_FREQ,   &torqueFiltCutFreq));

    // link controller input streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_OFFSET,	        tauOffset.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_SIN_AMPL,	    tauSinAmpl.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_SIN_FREQ,	    tauSinFreq.data()));

#ifdef INV_DYN_CONTROL
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_SIN_AMPL,         qSinAmpl.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_SIN_FREQ,         qSinFreq.data()));
#endif

    // link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VM,		            motorVoltage.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU,	            tauM.data()));

	//link monitored variables
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_MEAS,	        &monitor.tauMeas));
	YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD_MONITOR,	    &monitor.tauDes));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_MEAS1,          &monitor.tauMeas1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD_MONITOR1,      &monitor.tauDes1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_MEAS2,          &monitor.tauMeas2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD_MONITOR2,      &monitor.tauDes2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD_PLUS_PI,	    &monitor.tadDesPlusPI));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU_ERR,	        &monitor.tauErr));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q,	                &monitor.q));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_DES_MONITOR,	    &monitor.qDes));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL,	        &monitor.dq));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL_SIGN,	    &monitor.dqSign));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_DESIRED,	    &monitor.pwmDes));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_MEASURED,       &monitor.pwmMeas));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FEEDFORWARD,	&monitor.pwmFF));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FEEDBACK,	    &monitor.pwmFB));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_TORQUE_FF,	    &monitor.pwmTorqueFF));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FRICTION_FF,	&monitor.pwmFrictionFF));

    // Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_AJ,                     this));
	YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_SENDCMD,                this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_MONITORED_JOINT,        this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TORQUE_FILT_CUT_FREQ,   this));

    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

    tau 			= VectorNd::Constant(0.0);
	etau 			= VectorNd::Constant(0.0);
	tauD 			= VectorNd::Constant(0.0);
    tauOffset 		= VectorNd::Constant(0.0);
	integralState 	= VectorNd::Constant(0.0);
	motorVoltage	= VectorNd::Constant(0.0);
    dqSign          = VectorNd::Constant(0.0);

    ///< thread constants
    zeroN.setZero();
    zero6[0] = zero6[1] = zero6[2] = zero6[3] = zero6[4] = zero6[5] = 0.0;
    ddxB[0] = ddxB[1] = ddxB[3] = ddxB[4] = ddxB[5] = 0.0;
    ddxB[2] = 9.81;     ///< gravity acceleration

    activeJointsOld = activeJoints;

    if(!updateJointToMonitor())
        printf("Specified monitored joint name was not recognized: %s\n", monitoredJointName.c_str());

    robot->setEstimationParameter(ESTIMATE_JOINT_VEL,    ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE, &JOINT_VEL_ESTIMATION_WINDOW);
    robot->setEstimationParameter(ESTIMATE_JOINT_TORQUE, ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ, &torqueFiltCutFreq);

    if(!readRobotStatus(true))
    {
        printf("Error while initializing the controller: it was not possible to read the robot status!\n");
        return false;
    }

    qDes = q;


#ifdef GAZEBO_SIM

    leftShoulderVelocityCouplingMatrix = Matrix3d::Identity();
    leftShoulderTorqueCouplingMatrix = Matrix3d::Identity();

    rightShoulderTorqueCouplingMatrix = leftShoulderTorqueCouplingMatrix;
    rightShoulderVelocityCouplingMatrix = leftShoulderVelocityCouplingMatrix;

    torsoVelocityCouplingMatrix = Matrix3d::Identity();


    torsoTorqueCouplingMatrix = torsoVelocityCouplingMatrix;
#else
    leftShoulderVelocityCouplingMatrix = Matrix3d::Zero();
    leftShoulderVelocityCouplingMatrix(0,0) =  -1.0;
    leftShoulderVelocityCouplingMatrix(0,1) =   0.0;
    leftShoulderVelocityCouplingMatrix(0,2) =   0.0;

    leftShoulderVelocityCouplingMatrix(1,0) =   TRANSMISSION_RATIO_SHOULDER;
    leftShoulderVelocityCouplingMatrix(1,1) =  -TRANSMISSION_RATIO_SHOULDER;
    leftShoulderVelocityCouplingMatrix(1,2) =   0.0;

    leftShoulderVelocityCouplingMatrix(2,0) =   TRANSMISSION_RATIO_SHOULDER;
    leftShoulderVelocityCouplingMatrix(2,1) =  -TRANSMISSION_RATIO_SHOULDER;
    leftShoulderVelocityCouplingMatrix(2,2) =  -TRANSMISSION_RATIO_SHOULDER;


    leftShoulderTorqueCouplingMatrix = Matrix3d::Zero();
    Matrix3d leftShoulderVelocityCouplingMatrixTranspose = leftShoulderVelocityCouplingMatrix.transpose();
    leftShoulderTorqueCouplingMatrix = leftShoulderVelocityCouplingMatrixTranspose.inverse().eval();

    rightShoulderVelocityCouplingMatrix = Matrix3d::Zero();
    rightShoulderVelocityCouplingMatrix(0,0) =  1.0;
    rightShoulderVelocityCouplingMatrix(0,1) =  0.0;
    rightShoulderVelocityCouplingMatrix(0,2) =  0.0;

    rightShoulderVelocityCouplingMatrix(1,0) = -TRANSMISSION_RATIO_SHOULDER;
    rightShoulderVelocityCouplingMatrix(1,1) =  TRANSMISSION_RATIO_SHOULDER;
    rightShoulderVelocityCouplingMatrix(1,2) =  0.0;

    rightShoulderVelocityCouplingMatrix(2,0) = -TRANSMISSION_RATIO_SHOULDER;
    rightShoulderVelocityCouplingMatrix(2,1) =  TRANSMISSION_RATIO_SHOULDER;
    rightShoulderVelocityCouplingMatrix(2,2) =  TRANSMISSION_RATIO_SHOULDER;


    rightShoulderTorqueCouplingMatrix = Matrix3d::Zero();
    Matrix3d rightShoulderVelocityCouplingMatrixTranspose = rightShoulderVelocityCouplingMatrix.transpose();
    rightShoulderTorqueCouplingMatrix = rightShoulderVelocityCouplingMatrixTranspose.inverse().eval();


    Matrix3d torsoVelocityCouplingMatrixInverse = Matrix3d::Zero();

    torsoVelocityCouplingMatrixInverse(0,0) =  0.5;
    torsoVelocityCouplingMatrixInverse(0,1) = -0.5;
    torsoVelocityCouplingMatrixInverse(0,2) =  0.0;

    torsoVelocityCouplingMatrixInverse(1,0) =  0.5;
    torsoVelocityCouplingMatrixInverse(1,1) =  0.5;
    torsoVelocityCouplingMatrixInverse(1,2) =  0.0;

    torsoVelocityCouplingMatrixInverse(2,0) =  0.5*PULLEY_RADIUS_ROLL_MOTOR/PULLEY_RADIUS_ROLL_JOINT;
    torsoVelocityCouplingMatrixInverse(2,1) =  0.5*PULLEY_RADIUS_ROLL_MOTOR/PULLEY_RADIUS_ROLL_JOINT;
    torsoVelocityCouplingMatrixInverse(2,2) =      PULLEY_RADIUS_ROLL_MOTOR/PULLEY_RADIUS_ROLL_JOINT;

    torsoTorqueCouplingMatrix   = torsoVelocityCouplingMatrixInverse.transpose();
    torsoVelocityCouplingMatrix = torsoVelocityCouplingMatrixInverse.inverse().eval();

#endif

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

        wbi::wbiIdList jointList = robot->getJointList();

		for (int i=0; i < N_DOF; i++)
        {

            dqSign(i) = fabs(dq(i))>coulombVelThr(i) ? sign(dq(i)) : pow(dq(i)/coulombVelThr(i),3);
#ifdef IMPEDANCE_CONTROL

            double error = qDes(i)-q(i);
            double dumpingAndGrav =  gravityCompOn*tauGrav(i+6);
            tauD(i)         = kStiff(i)*(qDes(i)-q(i)) - kDamp(i)*dq(i) + gravityCompOn*tauGrav(i+6);

#else /* IMPEDANCE_CONTROL */

#ifdef INV_DYN_CONTROL
            tauD(i) = tauInvDyn(i+6);
#else
            tauD(i) = tauOffset(i) + tauSinAmpl(i)*sin(2*M_PI*tauSinFreq(i)*currentTime);
#endif

#endif /* IMPEDANCE_CONTROL */
            if (activeJoints(i) == 1)
            {
                etau(i)             = tauM(i) - tauD(i);
                Detau(i)            = (etau(i) - etauPrevious(i))/dt;
//                 cout << "etau(i) " << etau(i) << " etauPrevious(i) " << etauPrevious(i) <<  " Detau(i) " << Detau(i) << "\n";

                etauPrevious(i)     = etau(i);
                integralState(i)    = saturation(integralState(i) + ki(i)*dt*etau(i), TORQUE_INTEGRAL_SATURATION, -TORQUE_INTEGRAL_SATURATION) ;
                tau(i)              = tauD(i) - kp(i)*etau(i) - integralState(i) -kd(i)*Detau(i);

                wbi::LocalId lid = jointList.globalToLocalId(i);
                tauMotor    = tau(i);
                dqMotor     = dq(i);
                dqSignMotor = dqSign(i);

                int torsoYawGID    = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 0));
                int torsoRollGID   = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 1));
                int torsoPitchGID  = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 2));
                if (lid.bodyPart == iCub::skinDynLib::TORSO)
                {
//                     cout << "In TORSO handeler \n";
                    if (i == torsoYawGID)
                    {
                        tauMotor = torsoTorqueCouplingMatrix(0,0)  * tau(i) +  torsoTorqueCouplingMatrix(0,1)  * tau(i+1) + torsoTorqueCouplingMatrix(0,2)  * tau(i+2);
                        dqMotor  = torsoVelocityCouplingMatrix(0,0) * dq(i) +  torsoVelocityCouplingMatrix(0,1) * dq(i+1) + torsoVelocityCouplingMatrix(0,2) * dq(i+2);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
//                         cout << "Motor 1 \n";
                    }
                    if (i == torsoRollGID)
                    {
                        tauMotor = torsoTorqueCouplingMatrix(1,0)  * tau(i-1) +  torsoTorqueCouplingMatrix(1,1)  * tau(i) + torsoTorqueCouplingMatrix(1,2)  * tau(i+1);
                        dqMotor  = torsoVelocityCouplingMatrix(1,0) * dq(i-1) +  torsoVelocityCouplingMatrix(1,1) * dq(i) + torsoVelocityCouplingMatrix(1,2) * dq(i+1);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
//                         cout << "Motor 2 \n";
                    }
                    if (i == torsoPitchGID)
                    {
                        tauMotor = torsoTorqueCouplingMatrix(2,0)  * tau(i-2) +  torsoTorqueCouplingMatrix(2,1)  * tau(i-1) + torsoTorqueCouplingMatrix(2,2)  * tau(i);
                        dqMotor  = torsoVelocityCouplingMatrix(2,0) * dq(i-2) +  torsoVelocityCouplingMatrix(2,1) * dq(i-1) + torsoVelocityCouplingMatrix(2,2) * dq(i);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
//                         cout << "Motor 3 \n";
                    }
                }
                if (lid.bodyPart == iCub::skinDynLib::LEFT_ARM) {
                    int leftShoulderPitchGID = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 0));
                    int leftShoulderRollGID  = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 1));
                    int leftShoulderYawGID   = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 2));
                    if (i == leftShoulderPitchGID)
                    {
//                         cout << "In Left shoulder pitch handeler \n";
                        tauMotor = leftShoulderTorqueCouplingMatrix(0,0)  * tau(i) +  leftShoulderTorqueCouplingMatrix(0,1)  * tau(i+1) + leftShoulderTorqueCouplingMatrix(0,2)  * tau(i+2);
                        dqMotor  = leftShoulderVelocityCouplingMatrix(0,0) * dq(i) +  leftShoulderVelocityCouplingMatrix(0,1) * dq(i+1) + leftShoulderVelocityCouplingMatrix(0,2) * dq(i+2);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
                    }
                    if (i == leftShoulderRollGID)
                    {
//                         cout << "In Left shoulder roll handeler \n";
                        tauMotor = leftShoulderTorqueCouplingMatrix(1,0)  * tau(i-1) +  leftShoulderTorqueCouplingMatrix(1,1)  * tau(i) + leftShoulderTorqueCouplingMatrix(1,2)  * tau(i+1);
                        dqMotor  = leftShoulderVelocityCouplingMatrix(1,0) * dq(i-1) +  leftShoulderVelocityCouplingMatrix(1,1) * dq(i) + leftShoulderVelocityCouplingMatrix(1,2) * dq(i+1);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
                    }
                    if (i == leftShoulderYawGID)
                    {
//                         cout << "In Left shoulder yaw handeler \n";
                        tauMotor = leftShoulderTorqueCouplingMatrix(2,0)  * tau(i-2) +  leftShoulderTorqueCouplingMatrix(2,1)  * tau(i-1) + leftShoulderTorqueCouplingMatrix(2,2)  * tau(i);
                        dqMotor  = leftShoulderVelocityCouplingMatrix(2,0) * dq(i-2) +  leftShoulderVelocityCouplingMatrix(2,1) * dq(i-1) + leftShoulderVelocityCouplingMatrix(2,2) * dq(i);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
                    }
                }
                if (lid.bodyPart == iCub::skinDynLib::RIGHT_ARM) {
                    int rightShoulderPitchGID = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 0));
                    int rightShoulderRollGID  = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 1));
                    int rightShoulderYawGID   = jointList.localToGlobalId(wbi::LocalId(lid.bodyPart, 2));
//                     cout << "In RIGHT SHOULDER handeler \n";
                    if (i == rightShoulderPitchGID)
                    {
//                         cout << "In Right shoulder pitch handeler \n";
                        tauMotor = rightShoulderTorqueCouplingMatrix(0,0)  * tau(i) +  rightShoulderTorqueCouplingMatrix(0,1)  * tau(i+1) + rightShoulderTorqueCouplingMatrix(0,2)  * tau(i+2);
                        dqMotor  = rightShoulderVelocityCouplingMatrix(0,0) * dq(i) +  rightShoulderVelocityCouplingMatrix(0,1) * dq(i+1) + rightShoulderVelocityCouplingMatrix(0,2) * dq(i+2);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
                    }
                    if (i == rightShoulderRollGID)
                    {
//                         cout << "In Right shoulder roll handeler \n";
                        tauMotor = rightShoulderTorqueCouplingMatrix(1,0)  * tau(i-1) +   rightShoulderTorqueCouplingMatrix(1,1) * tau(i) + rightShoulderTorqueCouplingMatrix(1,2)  * tau(i+1);
                        dqMotor  = rightShoulderVelocityCouplingMatrix(1,0) * dq(i-1) +  rightShoulderVelocityCouplingMatrix(1,1) * dq(i) + rightShoulderVelocityCouplingMatrix(1,2) * dq(i+1);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
                    }
                    if (i == rightShoulderYawGID)
                    {
//                         cout << "In Right shoulder yaw handeler \n";
                        tauMotor = rightShoulderTorqueCouplingMatrix(2,0)  * tau(i-2) +  rightShoulderTorqueCouplingMatrix(2,1)  * tau(i-1) + rightShoulderTorqueCouplingMatrix(2,2)  * tau(i);
                        dqMotor  = rightShoulderVelocityCouplingMatrix(2,0) * dq(i-2) +  rightShoulderVelocityCouplingMatrix(2,1) * dq(i-1) + rightShoulderVelocityCouplingMatrix(2,2) * dq(i);
                        dqSignMotor = fabs(dqMotor)>coulombVelThr(i) ? sign(dqMotor) : pow(dqMotor/coulombVelThr(i),3);
                    }
                }

                //These three lines should be moved into a callback /setter of frictionCompensationFactor
                if (frictionCompensationFactor(i) < 0 || frictionCompensationFactor(i) > 1) {
                    frictionCompensationFactor(i) = 0; //Safest thing: do not compensate for friction
                }

                if(dqMotor>0)
                    motorVoltage(i) = kt(i)*tauMotor + frictionCompensationFactor(i) * kvp(i)*dqMotor + frictionCompensationFactor(i) * kcp(i)*dqSignMotor;
                else
                    motorVoltage(i) = kt(i)*tauMotor + frictionCompensationFactor(i) * kvn(i)*dqMotor + frictionCompensationFactor(i) * kcn(i)*dqSignMotor;
                Voltage = motorVoltage(i);
#ifndef GAZEBO_SIM
                if (lid.bodyPart == iCub::skinDynLib::TORSO)
                {
//                     cout << "In torso handler \n";
                    if (i == torsoYawGID)
                    {
                        Voltage = motorVoltage(i) - motorVoltage(i+1);
                    }
                    if (i == torsoRollGID)
                    {
                        Voltage = motorVoltage(i-1) + motorVoltage(i);
                    }
                }
#endif
                if (sendCommands == SEND_COMMANDS_ACTIVE)
                {
//                     cout << "Sending PWM" << "\n";
                    robot->setControlReference(&Voltage, i);
                    //robot->setControlParam(wbi::CTRL_PARAM_OFFSET, &motorVoltage(i), i);
                }
//                 cout << "desiredJointTorque"  << i << " = " << tauD(i) << "\n \n";
//                 cout << "JointTorque = " << tau(i) << "\n";
//                 cout << "tauMotor" << i << tauMotor << "\n";
//                 cout << "kt = " << kt(i) << "\n";
//                 cout << "ks = " << ks(i) << "\n";
//                 cout << "error = " << error << "\n";
//                 cout << "Motor" << i << "\n";
//                 cout << "tauMotor" << i << " = " << tauMotor << "\n";
//                 cout << "dqMotor = " << dqMotor << "\n";
//                 cout << "motorVoltageRead" << i << " = " <<  pwmMeas(i);
//                 cout << " motorVoltageSet" << i << "  = " << (int) motorVoltage(i) << "\n \n";
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

#ifdef INV_DYN_CONTROL
    // q   =  A*cos(2*pi*f*t)
    // dq  = -2*pi*f*A**sin(2*pi*f*t)
    // ddq = -(2*pi*f)^2 *A*cos(2*pi*f*t)
    double t = Time::now()-initialTime;
    VectorNd two_pi_f = 2*M_PI*qSinFreq;
    VectorNd two_pi_f_t = two_pi_f * t;
    VectorNd A_cos_2pi_f_t = qSinAmpl * two_pi_f_t.cos();
    qDes   = qOffset + A_cos_2pi_f_t;
    dqDes  = -two_pi_f * qSinAmpl * two_pi_f_t.sin();
    ddqDes = -two_pi_f.square() * A_cos_2pi_f_t;

    ddqInvDyn = activeJoints.cast<double>() * (ddqDes + kStiff*(qDes-q) + kDamp*(dqDes-dq));
//    ddqInvDyn = activeJoints.cast<double>() * (kStiff*(qDes-q));
    res = res && robot->inverseDynamics(q.data(), Frame(), dq.data(), zero6, ddqInvDyn.data(), ddxB, zero6, tauInvDyn.data());

//    printf("compute inv dyn\n");
    sendMsg("q         "+toString(q(monitoredJointId)));
    sendMsg("qDes      "+toString(qDes(monitoredJointId)));
    sendMsg("dqDes     "+toString(dqDes(monitoredJointId)));
    sendMsg("ddqDes    "+toString(ddqDes(monitoredJointId)));
    sendMsg("ddqInvDyn "+toString(ddqInvDyn(monitoredJointId)));
    sendMsg("tauInvDyn "+toString(tauInvDyn(monitoredJointId+6)));

#endif

    res = res && robot->inverseDynamics(q.data(), Frame(), zeroN.data(), zero6, zeroN.data(), ddxB, zero6, tauGrav.data());
//    tauInvDyn.tail<N_DOF>() = activeJoints.cast<double>() * (ks*(qDes-q));
//    tauInvDyn += tauGrav;

    // convert angles from rad to deg
    q    *= CTRL_RAD2DEG;
    dq   *= CTRL_RAD2DEG;
    qDes *= CTRL_RAD2DEG;

    return res;
}

//*************************************************************************************************************************
void jointTorqueControlThread::startSending()
{
    if (status != CONTROL_ON) {
        resetIntegralState(-1);
        readRobotStatus(false);
        etau             = tauM - tauD;
        etauPrevious     = etau;
#ifdef IMPEDANCE_CONTROL
        //reset desired positions for impedance control
        qDes = q;
#endif

#ifdef INV_DYN_CONTROL
        qOffset = q*CTRL_DEG2RAD;
        initialTime = Time::now();
#endif
        status = CONTROL_ON;       //sets thread status to ON
        setControlModePWMOnJoints(sendCommands == SEND_COMMANDS_ACTIVE);
        oldTime = yarp::os::Time::now();
        printf("Activating the torque control.\n");
    } else
        printf("Joint torque control already on.\n");
}

//*************************************************************************************************************************
void jointTorqueControlThread::stopSending()
{
    if (status != CONTROL_OFF) {
        status = CONTROL_OFF;
        setControlModePWMOnJoints(false);
        printf("Deactivating the torque control.\n");
    } else
        printf("Joint torque control already off.\n");
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
    double zeroValue = 0;
	for (int i=0; i < N_DOF; i++)
	{
		if (activeJoints(i) == 1 && torqueActive && status==CONTROL_ON) {
			robot->setControlMode(CTRL_MODE_MOTOR_PWM, NULL, i);
//             cout << "PWM active for joint " << i << " \n";
			//robot->setControlMode(CTRL_MODE_TORQUE, NULL, i);
            //robot->setControlParam(wbi::CTRL_PARAM_KP, &zeroValue);
        }
		else
        {
			robot->setControlMode(CTRL_MODE_POS, NULL, i);
//             cout << "Postion active \n";
//             cout << "torqueActive = " << torqueActive  << " status = " <<  status <<"\n";
        }
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
        case PARAM_ID_TORQUE_FILT_CUT_FREQ:
            robot->setEstimationParameter(ESTIMATE_JOINT_TORQUE, ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ, &torqueFiltCutFreq);
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
    double zeroValue = 0;
    for(int i=0; i<N_DOF; i++)
    {
        if(activeJoints(i)==1 && activeJointsOld(i)==0)         // joint has been activated
        {
            resetIntegralState(i);
            if(status==CONTROL_ON && sendCommands == SEND_COMMANDS_ACTIVE) {
                robot->setControlMode(CTRL_MODE_MOTOR_PWM, NULL, i);
                //robot->setControlMode(CTRL_MODE_TORQUE, NULL, i);
                //robot->setControlParam(wbi::CTRL_PARAM_KP, &zeroValue); //why I didn't used the function "setControlModePWMOnJoints"?
            }
        }
        else if(activeJoints(i)==0 && activeJointsOld(i)==1)    // joint has been deactivated
        {
            robot->setControlMode(CTRL_MODE_POS, NULL, i);
        }
    }
    activeJointsOld = activeJoints;
    return true;
}

//*************************************************************************************************************************
void jointTorqueControlThread::prepareMonitorData()
{
    int j = monitoredJointId;
//     cout << j;
    monitor.tauMeas         = tauM(j);
    monitor.tauDes          = tauD(j);
    monitor.tauMeas1        = Detau(j);
    monitor.tauDes1         = tauD(j+1);
    monitor.tauMeas2        = tauM(j+3);
    monitor.tauDes2         = tauD(j+3);
    monitor.tadDesPlusPI    = tau(j);
    double NormSquareTorqueError = 0;
    for(int i = 0; i < N_DOF; i++)
    {
        if(activeJoints(i) == 1 )
        {
            NormSquareTorqueError = NormSquareTorqueError + etau(i)*etau(i);
        }
    }
    monitor.tauErr          = sqrt(NormSquareTorqueError);
    //cout << "|tauErr| = " << monitor.tauErr << "\n";
    monitor.q               = q(j);
    monitor.qDes            = qDes(j);
    monitor.dq              = dq(j);
    monitor.dqSign          = dqSign(j);

    monitor.pwmDes          = motorVoltage(j);
    monitor.pwmMeas         = pwmMeas(j);
    monitor.pwmTorqueFF     = kt(j)*tauD(j);
    monitor.pwmFrictionFF   = dq(j)>0 ? frictionCompensationFactor(j) *kvp(j)*dq(j) + frictionCompensationFactor(j) *kcp(j)*dqSign(j) : frictionCompensationFactor(j) *kvn(j)*dq(j) + frictionCompensationFactor(j) *kcn(j)*dqSign(j);
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
        string jointName = b.get(0).asString().c_str();
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
