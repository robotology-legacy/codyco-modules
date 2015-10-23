
/*
 * Copyright (C) 2013 CODYCO Project
 * Author: Serena Ivaldi, Joseph Salini 
 * email:  serena.ivaldi@isir.upmc.fr, joseph.salini@isir.upmc.fr
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 
A simple test of integration between ORCISIR and YARP.

This is an integration example between icubSim and the iCub model with fixed base (icubfixed.h) used by ORCISIR, ISIR-XDE etcetera.
  
 \section tested_os_sec Tested OS
 Linux
 
 \author Serena Ivaldi, Joseph Salini
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <modHelp/modHelp.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <iostream>
#include <Eigen/Eigen>
// the model used to test ORCISIR
#include "icubfixed.h"
// these are to include ORCISIR
#include "orcisir/ISIRController.h"
#include "orcisir/Tasks/ISIREasyTaskManager.h" 

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace modHelp;
using namespace iCub::ctrl;
using namespace std;





Vector evalVel(const Vector &x, AWLinEstimator  *linEst)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return linEst->estimate(el);  
}

Vector evalAcc(const Vector &x, AWQuadEstimator *quadEstLow)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return quadEstUp->estimate(el);
}

void JointsToAll(Vector &q_all, 
    Vector &encoders_torso, Vector &encoders_head, Vector &encoders_arm_left,
    Vector &encoders_arm_right, Vector &encoders_leg_left, Vector &encoders_leg_right)
{
    if(q_all.size()!=33) q_all.resize(33,0.0);

    q_all[0] = 0;                       //icub.waist (the base moving in the 6DOF space)
    q_all[1] = encoders_leg_left[0];    //icub.l_hip_1
    q_all[2] = encoders_leg_left[1];    //icub.l_hip_2
    q_all[3] = encoders_leg_left[2];    //icub.l_thigh
    q_all[4] = encoders_leg_left[3];    //icub.l_shank
    q_all[5] = encoders_leg_left[4];    //icub.l_ankle_1
    q_all[6] = encoders_leg_left[5];    //icub.l_foot
    q_all[7] = encoders_torso[0];       //icub.lap_belt_1
    q_all[8] = encoders_torso[1];       //icub.lap_belt_2
    q_all[9] = encoders_torso[2];       //icub.chest
    q_all[10]= encoders_arm_left[0];    //icub.l_shoulder_1
    q_all[11]= encoders_arm_left[1];    //icub.l_shoulder_2
    q_all[12]= encoders_arm_left[2];    //icub.l_arm
    q_all[13]= encoders_arm_left[3];    //icub.l_elbow_1
    q_all[14]= encoders_arm_left[4];    //icub.l_forearm
    q_all[15]= encoders_arm_left[5];    //icub.l_wrist_1
    q_all[16]= encoders_arm_left[6];    //icub.l_hand
    q_all[17]= encoders_arm_right[0];   //icub.r_shoulder_1
    q_all[18]= encoders_arm_right[1];   //icub.r_shoulder_2
    q_all[19]= encoders_arm_right[2];   //icub.r_arm
    q_all[20]= encoders_arm_right[3];   //icub.r_elbow_1
    q_all[21]= encoders_arm_right[4];   //icub.r_forearm
    q_all[22]= encoders_arm_right[5];   //icub.r_wrist_1
    q_all[23]= encoders_arm_right[6];   //icub.r_hand
    q_all[24]= encoders_head[0];        //icub.neck_1
    q_all[25]= encoders_head[1];        //icub.neck_2
    q_all[26]= encoders_head[2];        //icub.head
    q_all[27]= encoders_leg_right[0];   //icub.r_hip_1
    q_all[28]= encoders_leg_right[1];   //icub.r_hip_2
    q_all[29]= encoders_leg_right[2];   //icub.r_thigh
    q_all[30]= encoders_leg_right[3];   //icub.r_shank
    q_all[31]= encoders_leg_right[4];   //icub.r_ankle_1
    q_all[32]= encoders_leg_right[5];   //icub.r_foot

}

void filterVelAcc_All(Vector &q_all, Vector &dq_all, Vector &ddq_all, 
    AWLinEstimator  *velEst, AWQuadEstimator *accEst)
{
    dq_all =  evalVel(q_all,velEst);
    ddq_all = evalAcc(q_all,accEst);
}




//===============================
//===============================

//        MAIN

//===============================
//===============================

int main(int argc, char** argv)
{
    YARP_REGISTER_DEVICES(icubmod)

    // first YARP
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }

    // stuff related to connection with iCubSim
    string robot="icubSim";
    string name="test_orcisir";


    //================================
    //================================

    //        DATA VARIABLES

    //================================
    //================================


    // drivers properties
    Property optionsRA, optionsLA,optionsTO, optionsHE, optionsLL, optionsRL; 
    Property optionsCartLA;
    // drivers icub parts
    PolyDriver *ddRA, *ddLA;
    PolyDriver *ddTO, *ddHE;
    PolyDriver *ddRL, *ddLL;
    // drivers cartesian
    PolyDriver *ddCartLA;
    // motor interfaces
    IPositionControl *iposRA, *iposLA,*ipos;
    IPositionControl *iposRL, *iposLL;
    IPositionControl *iposTO, *iposH;
    IVelocityControl *ivelRA, *ivelLA,*ivel;
    IVelocityControl *ivelRL, *ivelLL;
    IVelocityControl *ivelTO, *ivelH;
    IEncoders *iencRA, *iencLA;
    IEncoders *iencRL, *iencLL;
    IEncoders *iencTO, *iencHE;
    ICartesianControl *icrtLA;
    //
    int startup_context_id_LA;
    //
    Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;
    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;

    Vector q_all(33,0.0);
    dq_all = ddq_all = q_all;
    AWLinEstimator  *velEst = new AWLinEstimator(16,1.0); 
    AWQuadEstimator *accEst = new AWQuadEstimator(25,1.0);

    // parameters for ORC
    bool useReducedProblem = false;     //true does not work yet...
    bool useMultiLevel     = false;

    // vectors for ORC
    VectorXd q;  
    VectorXd dq;
    VectorXd ddq;
    VectorXd tau; 
    double dt = 0.01;   
   

    //================================
    //================================

    //      CONNECT TO ICUB

    //================================
    //================================


    // opening drivers
    
    // right arm
    optionsRA.put("device","remote_controlboard");
    optionsRA.put("local",string("/"+name+"/right_arm").c_str());
    optionsRA.put("remote",string("/"+robot+"/right_arm").c_str());
    // left arm
    optionsLA.put("device","remote_controlboard");
    optionsLA.put("local",string("/"+name+"/left_arm").c_str());
    optionsLA.put("remote",string("/"+robot+"/left_arm").c_str());
    // right leg
    optionsRL.put("device","remote_controlboard");
    optionsRL.put("local",string("/"+name+"/right_leg").c_str());
    optionsRL.put("remote",string("/"+robot+"/right_leg").c_str());
    // left leg
    optionsLL.put("device","remote_controlboard");
    optionsLL.put("local",string("/"+name+"/left_leg").c_str());
    optionsLL.put("remote",string("/"+robot+"/left_leg").c_str());
    // torso
    optionsTO.put("device","remote_controlboard");
    optionsTO.put("local",string("/"+name+"/torso").c_str());
    optionsTO.put("remote",string("/"+robot+"/torso").c_str());
    // head
    optionsHE.put("device","remote_controlboard");
    optionsHE.put("local",string("/"+name+"/head").c_str());
    optionsHE.put("remote",string("/"+robot+"/head").c_str());
    //

    // torso
    //....................................................
    if(!createDriver(ddTO, optionsTO))
    {
        cout<<"Error: unable to create driver for torso"<<endl;
        close();
        return false;
    }
    if(!ddT->view(iencTO) || !ddT->view(iposTO) || !ddT->view(ivelTO))
    {
        cout<<"Problems acquiring interfaces of torso"<<endl;
        close();
        return false;
    }
    // init torso
    int torsoAxes;
    iencTO->getAxes(&torsoAxes);
    torso.resize(torsoAxes,0.0);
    
    // head
    //....................................................
    if(!createDriver(ddHE,optionsHE))
    {
        cout<<"Error: unable to create driver of head"<<endl;
        close();
        return false;
    }
    if(!ddH->view(iencHE) || !ddH->view(iposHE) )
    {
        cout<<"Error: problems acquiring interfaces of head"<<endl;
        close();
        return false;
    }
    
    // init head
    int headAxes;
    iencHE->getAxes(&headAxes);
    head.resize(headAxes,0.0);
    
    // right arm
    //....................................................
    if(!createDriver(ddRA,optionsRA))
    {
        cout<<"Problems connecting to the remote driver of right_arm"<<endl;
        close();
        return false;
    }
    if(!ddRA->view(iencRA) || !ddRA->view(iposRA) )
    {
        cout<<"Problems acquiring interfaces of right_arm"<<endl;
        close();
        return false;
    }
    
    // left arm
    //....................................................
    
    if(!createDriver(ddLA,optionsLA))
    {
        cout<<"Problems connecting to the remote driver of left_arm"<<endl;
        close();
        return false;
    }
    if(!ddLA->view(iencLA) || !ddLA->view(iposLA) )
    {
        cout<<"Problems acquiring interfaces of left_arm"<<endl;
        close();
        return false;
    }
    
    // right leg
    //....................................................
    if(!createDriver(ddRL,optionsRL))
    {
        cout<<"Problems connecting to the remote driver of right_leg"<<endl;
        close();
        return false;
    }
    if(!ddRL->view(iencRL) || !ddRL->view(iposRL) )
    {
        cout<<"Problems acquiring interfaces of right_leg"<<endl;
        close();
        return false;
    }
    
    // left leg
    //....................................................
    if(!createDriver(ddLL,optionsLL))
    {
        cout<<"Problems connecting to the remote driver of left_leg"<<endl;
        close();
        return false;
    }
    if(!ddLL->view(iencLL) || !ddLL->view(iposLL) )
    {
        cout<<"Problems acquiring interfaces of left_leg"<<endl;
        close();
        return false;
    }

    
    cout<<"\n\nAll iCub parts where reached successfully."<<endl;
    cout<<"\n\nNow configuring the task."<<endl;
    Time::delay(2);



    //================================
    //================================

    //           ORCISIR

    //================================
    //================================



    //---------------------------------------------------------------------------------
    // now stuff related to the initialization of ORCISIR and its model

    //SET CONTROLLER PARAMETERS
    cout<<"SET PARAMETERS\n";
    string internalSolver("qld");  //or "quadprog" "qld"


    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    cout<<"INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER\n";
    iCubModel                       model("iCubModel_fixedBase");  // name is not important, not used
    orcisir::ISIRController         ctrl("myCtrl", model, internalSolver, useReducedProblem, useMultiLevel);
    orcisir::ISIREasyTaskManager    taskManager(ctrl, model);

    // in iCubModel we have N joints = 32 + 1 DOF for to anchor the base
    q      = Eigen::VectorXd(iCubModel.nbInternalDofs());  
    dq     = Eigen::VectorXd(iCubModel.nbInternalDofs());
    tau    = Eigen::VectorXd(iCubModel.nbInternalDofs()); 
    tau=0.0;
    dt = 0.01;

    //CREATE SOME TASKS
    cout<<"CREATE SOME TASKS\n";
    int refTaskIdx      = taskManager.createFullStateTask("refTask",       0, 0.0001, "INTERNAL");
    taskManager.setTaskStiffnessAndDamping(refTaskIdx,   9, 6);    //set Kp and Kd

    // task for left hand of icub
    // the segment hosting the frame must be indicated (it is defined in icubfixed.cpp)
    // frameTask is a Cartesian task
    int frameTaskIdx    = taskManager.createPositionFrameTask("frameTask", 0, 1,  "icub.l_hand", Displacementd(), "XYZ");
    taskManager.setTaskStiffnessAndDamping(frameTaskIdx, 9, 6);

    // if I want to move in this position (fixed) i can use this to update the frame task
    taskManager.updateFrameTask(frameTaskIdx, Displacementd(-0.3, 0.3, 0.2), Twistd(0,0,0,0,0,0), Twistd());

    // if I want to update the frame task to make it track a desired trajectory, I have to update it during the control loop
    //taskManager.updateFrameTask(frameTaskIdx, Displacementd(-0.3, 0.3, 0.2), Twistd(0,0,0,0,0,0), Twistd());



    //---------------------------------------------------------------------------------
    // init simulation

    // take q and qdot from icub simulator (it has its own dynamics)
    iencLA->getEncoders(encoders_arm_left.data());
    iencRA->getEncoders(encoders_arm_right.data());
    iencHE->getEncoders(encoders_head.data());
    iencLL->getEncoders(encoders_leg_left.data());
    iencRL->getEncoders(encoders_leg_right.data());
    iencTO->getEncoders(encoders_torso.data());
 
    
    // conversion q: from icub parts to a big q vector for icub model
    JointsToAll(q_all, 
    encoders_torso, encoders_head, encoders_arm_left,
    encoders_arm_right, encoders_leg_left, encoders_leg_right);

    // filter to estimate qdot
    filterVelAcc_All(q_all,dq_all, ddq_all, velEst, accEst);

    //convert from yarp to eigen
    // look eigenToYarpVector
    yarpToEigenVector(q_all, q);
    yarpToEigenVector(dq_all,dq);
    yarpToEigenVector(ddq_all,ddq);

    //set q_all inside icub model
    model.setJointPositions(q); // set joints values on icub model fixed
    model.setJointVelocities(dq); // set also velocities


    //---------------------------------------------------------------------------------
    // now simulation with ORCISIR and its model and connection with iCubSim

    //SIMULATE
    cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {

        // is it necessary? yes...
        Time::delay(dt);

        cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";
        ctrl.computeOutput(tau);    //compute tau
        cout<<"tau: "<<tau.transpose()<<"\n";

        VectorXd ddq = model.getAccelerationVariable().getValue();
        cout<<"ddq: "<<ddq.transpose()<<"\n";
        //VectorXd ddq = 
        //  model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() 
        //    - model.getGravityTerms() );

        // conversion to positions and velocities
         dq += ddq * dt;
         q  += dq  * dt;

         // FINALLY SEND COMMANDS TO ICUBSIM



        // take q and qdot from icub simulator (it has its own dynamics)
        Ienc_LA->getEncoders(encoders_arm_left.data());
        Ienc_RA->getEncoders(encoders_arm_right.data());
        Ienc_HE->getEncoders(encoders_head.data());
        Ienc_LL->getEncoders(encoders_leg_left.data());
        Ienc_RL->getEncoders(encoders_leg_right.data());
        Ienc_TO->getEncoders(encoders_torso.data());
        // conversion q: from icub parts to a big q vector for icub model
        JointsToAll(q_all, 
        encoders_torso, encoders_head, encoders_arm_left,
        encoders_arm_right, encoders_leg_left, encoders_leg_right);

        // filter to estimate qdot
        filterVelAcc_All(q_all,dq_all, ddq_all, velEst, accEst);

        //convert from yarp to eigen
        // look eigenToYarpVector
        yarpToEigenVector(q_all, q);
        yarpToEigenVector(dq_all,dq);
        yarpToEigenVector(ddq_all,ddq);

        //set q_all inside icub model
        model.setJointPositions(q); // set joints values on icub model fixed
        model.setJointVelocities(dq); // set also velocities








        // now send to icubsim
        // init icubsim right hand with the initial point of the model
        eigenToYarpVector(q, xRA_des);
        icrtRA->goToPosition(xRA_des);

        cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";

    }

    // end of simulation
    q = model.getJointPositions();
    icrtRA->getPose(xRA,oRA);

    cout<<"FINAL RESULT"<<endl
        <<"model   = "<<q<<endl
        <<"icubsim = "<<xRA.toString()<<endl;


    //---------------------------------------------------------------------------------
    // closing iCubSim and YARP stuff

    cout<<"Closing drivers"<<endl;

    if(using_cartesian_arm_right)
    {
       icrtRA->stopControl(); 
       icrtRA->restoreContext(startup_context_id_RA);
       deleteDriver(ddCartRA);
    }
    deleteDriver(ddRA);

    cout<<"ISIRControllerYARPTest is over"<<endl;

    return 0;
}
