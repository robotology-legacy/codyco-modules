/*
 * Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Jorhabib Eljaik Gomez
 * email: jorhabib.eljaik@iit.it
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
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

#ifndef _ROBOT_STATE_CPP_
#define _ROBOT_STATE_CPP_

#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>

#include <Eigen/Core>                               // import most common Eigen types
#include <Eigen/SVD>
#include <iostream>
#include <stdio.h>

#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include "constants.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

using namespace iCub::ctrl;
using namespace iCub::iDyn;

using namespace std;
using namespace Eigen;

using namespace wbi;
using namespace wbiIcub;
using namespace locomotion;
using namespace paramHelp;

VectorXd dotq;
Vector qrad, xpose;
JacobianMatrix jacob;

class robotStatus{
private:
//        static wholeBodyInterface *wbInterface;
    static int creationCounter;
    int actJnts;
    bool blocking;
    int period;
    string moduleName;
    string robotName;
    Port rpcPort;		// a port to handle rpc messages


    // Copied straight from LocomotionThread member variables
    // Member variables
    int                 printCountdown;         // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    int                 LINK_ID_RIGHT_FOOT, LINK_ID_LEFT_FOOT;
    int                 footLinkId;             // id of the controlled (swinging) foot link
    int                 comLinkId;              // id of the COM
    int                 _n;                     // current number of active joints
    int                 _k;                     // current number of constraints
    Frame               xBase;                  // position/orientation of the floating base
    Vector              aa_w2b;                 // world to base rotation in angle/axis notation
    JacobianMatrix      Jcom_6xN;               // Jacobian of the center of mass (6 x N, where N=n+6)
    JacobianMatrix      JfootR;                 // Jacobian of the right foot
    JacobianMatrix      JfootL;                 // Jacobian of the left foot
    MatrixY             S;                      // matrix selecting the active joints
    VectorXd            dq, dqJ;                // joint velocities (size of vectors: n+6, n, 6)
    //VectorXd            qMin, qMax;             // lower and upper joint bounds
    VectorXd            dqDes;
    MatrixXd            Jcb;                    // first 6 columns of the constraint Jacobian
    JacobiSVD<MatrixXd> svdJcb;                 // singular value decomposition of Jcb
    VectorXd            ftSens;                 // ankle force/torque sensor readings (order is: left, right)
    Frame               H_base_leftFoot;        // rototranslation from robot base to left foot (i.e. world)
    Frame               Ha;                     // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]

    // Module parameters
    Vector              kp_com;
    Vector              kp_foot;
    Vector              kp_posture;
    double              tt_com, tt_foot, tt_posture;    // trajectory times of min jerk trajectory generators
    VectorNi            activeJoints;                   // vector of bool indicating which joints are used (1 used, 0 blocked)

    // Input streaming parameters
    int                 supportPhase;
    Vector              xd_com, xd_foot;        // desired positions (use yarp vector because minJerkTrajGen wants yarp vector)
    Vector              qd;                     // desired joint posture (for all ICUB_DOFS joints)
    Matrix4d             H_w2b;                  // rotation matrix from world to base reference frame

    // Output streaming parameters
    Vector              xr_com, xr_foot, qr;        // reference positions (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector              dxr_com, dxr_foot, dqr;     // reference velocities (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector              x_com, x_foot, qDeg, qRad;  // measured positions (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector              x_pose;
    Vector              dxc_com, dxc_foot, dqc;     // commanded velocities (use yarp vector because minJerkTrajGen gives yarp vector)


    // HAVE PROBLEMS COMPILING WHEN DECLARING THESE VARIABLES
    // Eigen vectors mapping Yarp vectors
    Map<Vector2d>       dxc_comE;               // commanded velocity of the COM
    Map<Vector6d>       dxc_footE;
    Map<VectorXd>       dqcE;
    Map<VectorXd>       qDegE;
    Map<VectorXd>       dqDesMap;

    // Trajectory generators
    minJerkTrajGen      *trajGenCom, *trajGenFoot, *trajGenPosture;


public:
    static int         *i;
    wholeBodyInterface *wbInterface;

    robotStatus():dxc_comE(0), dxc_footE(0), dqcE(0,0), qDegE(0,0), dqDesMap(0,0){     //This way of initializing dxc_comE, ecc is necessary for using MAP from Eigen.
        creationCounter++;
        fprintf(stderr,"ROBOTSTATUS instantiated %d times\n ",creationCounter);
        wbInterface = NULL;
//        paramHelper = NULL;
        /**** NEED TO PASS THESE VALUES FROM mdlStart ****/
//        moduleName  = "robotSate";
//        robotName   = "icubGazeboSim";
    }
    ~robotStatus(){
        if(i!=NULL){
            creationCounter--;
            fprintf(stderr,"wbInterface in destructor: %p \n",wbInterface);
            if(wbInterface->close()){
                delete wbInterface;
                fprintf(stderr,"wbInterface has been closed and deleted correctly. %d to go \n",creationCounter);
                wbInterface = NULL;
            }
            else{
                fprintf(stderr,"ERROR: wbInterface couldn't close correctly");
            }

//            delete paramHelper;
//            paramHelper = NULL;

            i           = NULL;
        }
    }
    void setRobotName(string rn){
        robotName = rn;
    }
    void setmoduleName(string mn){
        moduleName = mn;
    }

     // **************************************************************************************************
     int getCounter()
     {
         return creationCounter;
     }
     // **************************************************************************************************
     int decreaseCounter()
     {
        creationCounter--;
        return creationCounter;
     }

     // **************************************************************************************************
     bool robotConfig()
     {
 //        fprintf(stderr,"Configuring...\n");
         if(i!=NULL){
             wbInterface = (wholeBodyInterface *) i;
 //            fprintf(stderr,"Copying wholeBodyInterface POINTER!\n");
         }
         else{
             /************************ PARAMHELPER ******************************/
 //            paramHelper = new ParamHelperServer(locomotionParamDescr, PARAM_ID_SIZE, locomotionCommandDescr, COMMAND_ID_SIZE);
 //            paramHelper->linkParam(PARAM_ID_MODULE_NAME, &moduleName);
 //            paramHelper->linkParam(PARAM_ID_CTRL_PERIOD, &period);
 //            paramHelper->linkParam(PARAM_ID_ROBOT_NAME, &robotName);

 //            Bottle initMsg;
 //            paramHelper->initializeParams(rf, initMsg);
 //            printBottle(initMsg);

 //            if(!paramHelper->init(moduleName)){ fprintf(stderr, "Error while initializing parameter helper. Closing module.\n"); return false; }

             /*********** WHOLE BODY INTERFACE **********/
             wbInterface = new icubWholeBodyInterface(moduleName.c_str(),robotName.c_str());
             fprintf(stderr,"new wbInterface created ...\n");
             i = (int *) wbInterface;
             fprintf(stderr,"icubWholeBodyInterface has been created %p \n", wbInterface);
             wbInterface->addJoints(ICUB_MAIN_JOINTS);

             if(!wbInterface->init()){
                 fprintf(stderr,"Error initializing whole Body Interface!\n");
                 return false;
             }
             else{
                 fprintf(stderr,"Whole Body Interface Correctly Initialized, yayyy!!!!\n");
             }

             // SET CONTROL MODE
             fprintf(stderr,"About to set Control Mode\n");
             setCtrlMode(CTRL_MODE_POS);
         }

         // Initializing private variables This must be done regardless of the new creation of wbInterface
         actJnts = wbInterface->getJointList().size();
         qRad.resize(actJnts,0.0);
         dqJ.resize(actJnts);

         return true;
     }
     // **************************************************************************************************
     int getLinkId(const char *linkName, int &linkId)
     {
         if(linkName != "com"){
 //            fprintf(stderr,"linkName is NOT com\n");
             wbInterface->getLinkId(linkName, linkId);
             return linkId;
         }
         else{
 //            fprintf(stderr,"linkName is com\n");
             comLinkId = iWholeBodyModel::COM_LINK_ID;
             linkId = comLinkId;
             return linkId;
         }
     }
     // **************************************************************************************************
     int getLinkId(const char *linkName) //This is especifically for the COM
     {
         comLinkId = iWholeBodyModel::COM_LINK_ID;
         return comLinkId;
     }
//      **************************************************************************************************
     //    This method should be called only once in mdlStart() and reflect most of what has been done in LocomotionThread::threadInit()
     bool robotInit(int btype, int link){
         getLinkId("r_sole",LINK_ID_RIGHT_FOOT);
         getLinkId("l_sole",LINK_ID_LEFT_FOOT);
         getLinkId("com");

         // I must count the nonzero entries of activeJoints before calling numberOfJointsChanged (to know _n)
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ACTIVE_JOINTS,       activeJoints.data()));
         // I must know the support phase before calling numberOfConstraintsChanged (to know the number of constraints)
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SUPPORT_PHASE,       &supportPhase));
 //        numberOfJointsChanged();
 //        numberOfConstraintsChanged();

         const char *linkName;
         int default_size, linkID;

         // resize all Yarp vectors
         if(btype == 2 || btype == 3){
             //Interpreting link for either forwardKinematics or Jacobian.
             switch (link)
             {
             case 0:
                 linkName = "r_sole";
                 default_size = DEFAULT_XDES_FOOT.size();
                 break;
             case 1:
                 linkName = "l_sole";
                 default_size = DEFAULT_XDES_FOOT.size();
                 break;
             case 2:
                 linkName = "com";
                 default_size = DEFAULT_XDES_COM.size();
                 break;
             }
             getLinkId(linkName,linkID);
             x_pose.resize(default_size,0.0);                //output of forward kinematics and jacobian
         }

         x_com.resize(DEFAULT_XDES_COM.size(), 0.0);         // measured pos
         x_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);       // measured pos

         xd_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // desired pos
         xd_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);      // desired pos
         qd.resize(ICUB_DOFS, 0.0);                          // desired pos (all joints)

         xr_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // reference pos
         xr_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);      // reference pos
         qr.resize(ICUB_DOFS, 0.0);                          // reference pos

         dxr_com.resize(DEFAULT_XDES_COM.size(), 0.0);       // reference vel
         dxr_foot.resize(6, 0.0);                            // reference vel
         dqr.resize(ICUB_DOFS, 0.0);                         // reference vel

         dxc_com.resize(DEFAULT_XDES_COM.size(), 0.0);       // commanded vel
         dxc_foot.resize(6, 0.0);                            // commanded vel

         kp_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // proportional gain
         kp_foot.resize(6, 0.0);                             // proportional gain
         kp_posture.resize(ICUB_DOFS, 0.0);                  // proportional gain

         JfootR.resize(NoChange,ICUB_DOFS+6);

 ////        // map Yarp vectors to Eigen vectors
 //        new (&dxc_comE)     Map<Vector2d>(dxc_com.data());
 //        new (&dxc_footE)    Map<Vector6d>(dxc_foot.data());

 //        // link module rpc parameters to member variables
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP_COM,              kp_com.data()));    // constant size
 //        fprintf(stderr,"kp_com VALUE IS: %s \n", kp_com.toString().c_str());
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP_FOOT,             kp_foot.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP_POSTURE,          kp_posture.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_COM,       &tt_com));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_FOOT,      &tt_foot));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_POSTURE,   &tt_posture));
 ////        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PINV_DAMP,           &(solver->pinvDamp)));
 ////        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MAX,               solver->qMax.data()));
 ////        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MIN,               solver->qMin.data()));
 ////        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JNT_LIM_MIN_DIST,    &(solver->safetyThreshold)));
 //        // link module input streaming parameters to member variables
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XDES_COM,            xd_com.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XDES_FOOT,           xd_foot.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QDES,                qd.data()));        // constant size
 //    #ifndef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_H_W2B,               H_w2b.data()));
 //    #endif
 ////        // link module output streaming parameters to member variables
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XREF_COM,            xr_com.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XREF_FOOT,           xr_foot.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QREF,                qr.data()));        // constant size
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_X_COM,               x_com.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_X_FOOT,              x_foot.data()));
 //        YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q,                   qDeg.data()));      // variable size

         Ha.R = Rotation(0,0,1, 0,-1,0, 1,0,0);   // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]

 //        normalizeFootOrientation();

         return true;

     }
//      **************************************************************************************************
//     void normalizeFootOrientation()
//     {
//         double axisNorm = sqrt(xd_foot[3]*xd_foot[3] + xd_foot[4]*xd_foot[4] + xd_foot[5]*xd_foot[5]);
//         xd_foot[3] /= axisNorm;
//         xd_foot[4] /= axisNorm;
//         xd_foot[5] /= axisNorm;
//     }
     // **************************************************************************************************
     bool world2baseRototranslation(){
 //        fprintf(stderr,"About to compute H_base_leftFoot: \n");
         wbInterface->computeH(qRad.data(), Frame(), LINK_ID_LEFT_FOOT, H_base_leftFoot);
         H_base_leftFoot = H_base_leftFoot*Ha;
         H_base_leftFoot.setToInverse().get4x4Matrix(H_w2b.data());
 //        fprintf(stderr,"H_base_leftFoot: %s \n",H_base_leftFoot.toString().c_str());
         xBase.set4x4Matrix(H_w2b.data());
         // select which foot to control (when in double support, select the right foot)
//         footLinkId = supportPhase==SUPPORT_RIGHT ? LINK_ID_LEFT_FOOT : LINK_ID_RIGHT_FOOT;
         return true;

     }

//     // **************************************************************************************************
//     int getDOF(){
//         return wbInterface->getJointList().size();
//     }
     // **************************************************************************************************
     bool robotJntAngles(bool blockingRead)
     {
 //        fprintf(stderr,"Robot Joint Angles requested\n");
         return wbInterface->getEstimates(ESTIMATE_JOINT_POS, qRad.data(),-1.0, blockingRead);
     }
     // **************************************************************************************************
     bool robotJntVelocities(bool blockingRead)
     {
 //        fprintf(stderr,"Now we're getting velocities \n");
         return wbInterface->getEstimates(ESTIMATE_JOINT_VEL, dqJ.data(),-1.0, blockingRead);
     }
     // ***************************************************************************************************
     Vector forwardKinematics(int &linkId)
     {
         // THIS FUNCTION SHOULD ACTUALLY TAKE AS INPUT THE ID OF THE DESIRED LINK
         if(robotJntAngles(false)){
             if(world2baseRototranslation())
             {
                 footLinkId = linkId;
     //            bool ans = wbInterface->forwardKinematics(qRad.data(), xBase.data(), footLinkId, x_foot.data());
     //            fprintf(stderr,"fwd kinematics will be computed with footLinkId: %d and x_pose: %s", footLinkId, x_pose.toString().c_str());
		  
                 bool ans = wbInterface->forwardKinematics(qRad.data(), xBase, footLinkId, x_pose.data());
                 // Debugging is over here
                 if(ans){
     //                fprintf(stderr,"Forward Kinematics computed \n");
     //                fprintf(stderr,"pose: %s \n", x_pose.toString().c_str());
    //                 return x_pose;
                     return x_pose;
                 }
                 else
                 {
                     x_pose.zero();
                     return x_pose;
                 }
             }
             else{
                 fprintf(stderr,"ERROR computing world 2 base rototranslation!!!!!\n");
             }
         }
         else{
             fprintf(stderr,"ERROR: There has been an error acquiring robot joint angles \n");
         }
     }
//     // ***************************************************************************************************
//     void numberOfJointsChanged()
//     {
//         LocalId lid;
//         LocalIdList currentActiveJoints = wbInterface->getJointList();
//         for(int i=0; i<activeJoints.size(); i++)
//         {
//             lid = ICUB_MAIN_JOINTS.globalToLocalId(i);
//             if(currentActiveJoints.containsId(lid))
//             {
//                 if(activeJoints[i]==0)
//                     wbInterface->removeJoint(lid);
//             }
//             else
//             {
//                 if(activeJoints[i]==1)
//                     wbInterface->addJoint(lid);
//             }
//         }

//         _n = wbInterface->getJointList().size();
//         Jcom_6xN.resize(NoChange, _n+6);
//         JfootR.resize(NoChange, _n+6);
//         JfootL.resize(NoChange, _n+6);

//         qRad.resize(_n, 0.0);                               // measured pos
//         qDeg.resize(_n, 0.0);                               // measured pos
//         new (&qDegE) Map<VectorXd>(qDeg.data(), _n);        // measured pos (Eigen vector)
//         dq.resize(_n+6);                                    // measured vel (base + joints)
//         dqJ.resize(_n);                                     // measured vel (joints only)
//         dqc.resize(_n, 0.0);                                // commanded vel (Yarp vector)
//         new (&dqcE) Map<VectorXd>(dqc.data(), _n);          // commanded vel (Eigen vector)
//         dqDes.resize(_n);                                   // desired joint vel commanded to the motors
//         kp_posture.resize(_n, 0.0);                         // proportional gain (rpc input parameter)

//         updateSelectionMatrix();
//     }
//     // ***************************************************************************************************

//     void updateSelectionMatrix()
//     {
//         S.resize(_n, ICUB_DOFS);
//         S.zero();
//         int j=0;
//         for(int i=0; i<ICUB_DOFS; i++)
//         {
//             if(activeJoints[i] != 0.0)
//             {
//                 S(j,i) = 1.0;
//                 j++;
//             }
//         }
//     }
//     // ***************************************************************************************************
//     void numberOfConstraintsChanged()
//     {
//         _k = supportPhase==SUPPORT_DOUBLE ? 12 : 6;     // current number of constraints
//     }

     // ***************************************************************************************************
    JacobianMatrix jacobian(int &linkId)
    {
//        fprintf(stderr,"About to compute Jacobian for link %d \n", linkId);
        if(robotJntAngles(false)) {
            if(world2baseRototranslation()) {
	      
		//BEGIN ONLY FOR DEBUGGING
// 		qrad.zero();
// 		qrad(3) = CTRL_DEG2RAD*(-30);
// 		qrad(4) = CTRL_DEG2RAD*(30);
// 		qrad(6) = CTRL_DEG2RAD*(45);
// 		
// 		qrad(7)  = qrad(3);
// 		qrad(8)  = qrad(4);
// 		qrad(10) = qrad(6);
// 		
// 		wbi::Rotation R(-1,	0,	0,
// 				 0,	-1,	0,
// 				 0,	0,	1);
// 		
// 		double p[3];
// 		p[0] = 0; p[1] = -0.0681; p[2] = 0.5975;
// 		
// 		Frame xBase(R, p);
		//END ONLY FOR DEBUGGING
		
// 		fprintf(stderr,"xBase: %s \n",xBase.toString().c_str());
// 		fprintf(stderr,"qRad:  %s \n", qRad.toString().c_str());
		
                bool ans = wbInterface->computeJacobian(qRad.data(), xBase, -1, JfootR.data());
                if(ans)
                {
                    return JfootR;
                }
                else
                {
                    JfootR.setZero();
                    return JfootR;
                }
            }
            else{
	      fprintf(stderr,"There was an error computing world to base rototranslation \n");
	    }
        }
        else {
            fprintf(stderr,"There was an error getting robot joint angles to compute Jacobians \n");
        }
    }
     // ***************************************************************************************************
     Vector getEncoders()
     {
 //        fprintf(stderr,"qrad before returning from getEncoders: %s \n ",qRad.toString().c_str());
         return qRad;
     }
     // ***************************************************************************************************
     VectorXd getJntVelocities()
     {
         return dqJ;
     }
     // ***************************************************************************************************
	 bool setCtrlMode(ControlMode ctrl_mode)
     {
         if(wbInterface->setControlMode(ctrl_mode)){
             return true;
         }
         else{
             cout<<"ERROR: Velocity control mode could not be set"<<endl;
             return false;
         }

     }
     // ***************************************************************************************************

     void setdqDes(Vector dqD)
     {
         int n = dqD.length();
         new (&dqDesMap)    Map<VectorXd>(dqD.data(),_n);
 //        for(int i=0; i<dqD.length(); i++){
 //            dqDes[i] = dqD[i];
 //        }
 //        cout<<"Now printing dqDesMap: "<<endl;
 //        for(int i=0; i<dqDesMap.SizeAtCompileTime; i++)
 //            fprintf(stderr,"%f \n",dqDesMap(i));

         if(wbInterface->setControlReference(dqDesMap.data()))
         return;
     }
     // ***************************************************************************************************
     void preStop()
     {
         Vector zrs = zeros(ICUB_DOFS);
         new (&dqDesMap)    Map<VectorXd>(zrs.data(),_n);
 //        setCtrlMode(CTRL_MODE_POS);
         if(wbInterface->setControlReference(dqDesMap.data()))
             cout<<"Velocity set to ZERO before stopping"<<endl;
     }
     // ***************************************************************************************************
    void doNothing()
    {
        fprintf(stderr,"Doing nothing...\n");
    }
};



class counterClass
{
private:
    static int count;
public:
    counterClass();
    int getCount();
};
counterClass::counterClass()
{
    count++;
}
int counterClass::getCount()
{
    return count;
}

int counterClass::count = 0;


int robotStatus::creationCounter = 0;

int *robotStatus::i = NULL;
//wholeBodyInterface *robotStatus::wbInterface = new icubWholeBodyInterface("robotState","icubSim");


#endif


