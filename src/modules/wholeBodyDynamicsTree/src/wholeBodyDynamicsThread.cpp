/* 
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#include <wholeBodyDynamicsTree/wholeBodyDynamicsThread.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>


using namespace yarp::math;
using namespace wbiIcub;

//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name, string _robotName, int _period, wholeBodySensors *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), robot(_wbi)
{
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::threadInit()
{

    // read robot status (to be done before initializing trajectory generators)
    if(!readRobotStatus(true))
        return false;

    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    readRobotStatus();                      // read encoders, compute positions and Jacobians
    if(status==LOCOMOTION_ON)
    {
        updateReferenceTrajectories();      // compute desired velocities for all tasks
        solver->solve(dqDes, qDegE);        // compute desired joint velocities

        if(areDesiredJointVelTooLarge())    // check desired joint velocities are not too large
        {
            preStopOperations();            // stop the controller
            cout<<"\n************ ERROR: CONTROLLER STOPPED BECAUSE DESIRED JOINT VELOCITIES ARE TOO LARGE: "<<toString(dqDes.transpose(),2)<<endl;
        }
        else
            robot->setControlReference(dqDes.data()); // send velocities to the joint motors
            
        sendMsg("Solver time: "+toString(solver->solverTime)+"; iterations: "+toString(solver->solverIterations)+"; blocked joints: "+toString(solver->getBlockedJointList()), MSG_INFO);
        sendMsg("dqDes: "+toString(1e3*dqDes.transpose(), 1), MSG_DEBUG);
    }

    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res =   robot->getEstimates(ESTIMATE_JOINT_POS,    qRad.data(),    -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_VEL,    dqJ.data(),     -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_FORCE_TORQUE, ftSens.data(),  -1.0, blockingRead);
}


//*************************************************************************************************************************
void wholeBodyDynamicsThread::threadRelease()
{
    if(trajGenCom)      delete trajGenCom;
    if(trajGenFoot)     delete trajGenFoot;
    if(trajGenPosture)  delet

}