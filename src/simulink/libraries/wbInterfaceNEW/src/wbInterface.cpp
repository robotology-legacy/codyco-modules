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

#include "wbInterface.h"

// MASK PARAMETERS --------------------------------------
#define NPARAMS 3                                  		// Number of input parameters
#define BLOCK_TYPE_IDX 0                                  	// Index number for first input parameter
#define BLOCK_TYPE_PARAM ssGetSFcnParam(S,BLOCK_TYPE_IDX) 	// Get first input parameter from mask
#define STRING_PARAM_IDX 1
#define LOCAL_PARAM_IDX 2
// END MASK PARAMETERS -----------------------------------

#define VERBOSE   0
#define DEBUGGING 0
#define TIMING    0
#define NEWCODE	  1
#define ICUB_FIXED

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

using namespace std;
using namespace Eigen;

using namespace wbi;
using namespace wbiIcub;



//-----------------------------------------------------------------------------------------------------------------------//
// START ROBOTSTATUS IMPLEMENTATION -------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------//
//These variables must be initialized in this .cpp file and in this particular way for being static member variables of a class
int  robotStatus::creationCounter = 0;
int *robotStatus::tmpContainer 	  = NULL;
int  counterClass::count 	  = 0;


robotStatus::robotStatus() {
    creationCounter++;
    fprintf(stderr,"robotStatus::robotStatus >> ROBOTSTATUS instantiated %d times\n ",creationCounter);
    wbInterface = NULL;
}
//=========================================================================================================================
robotStatus::~robotStatus() {
    if(tmpContainer!=NULL) {
        creationCounter--;
        fprintf(stderr,"robotStatus::~robotStatus >> wbInterface in destructor: %p \n",wbInterface);
        if(wbInterface->close()) {
            delete wbInterface;
            fprintf(stderr,"robotStatus::~robotStatus >> wbInterface has been closed and deleted correctly. %d to go \n",creationCounter);
            wbInterface = NULL;
        }
        else {
            fprintf(stderr,"robotStatus::~robotStatus >> ERROR: wbInterface couldn't close correctly");
        }

        tmpContainer = NULL;
    }
}
//=========================================================================================================================
void robotStatus::setRobotName(string rn) {
    robotName = rn;
}
//=========================================================================================================================
void robotStatus::setmoduleName(string mn) {
    moduleName = mn;
}
//=========================================================================================================================
int robotStatus::decreaseCounter() {
    creationCounter--;
    return creationCounter;
}
//=========================================================================================================================
void robotStatus::resetCounter() {
    creationCounter = 0;
}
//=========================================================================================================================
bool robotStatus::robotConfig() {
    if(VERBOSE) fprintf(stderr,"robotStatus::robotConfig >> Configuring...\n");
    if(tmpContainer!=NULL) {
        wbInterface = (wholeBodyInterface *) tmpContainer;
        if(DEBUGGING) fprintf(stderr,"robotStatus::robotConfig >> Copying wholeBodyInterface POINTER!\n");
    }
    else {
        //---------------- CREATION WHOLE BODY INTERFACE ---------------------/
        wbInterface = new icubWholeBodyInterface(moduleName.c_str(),robotName.c_str());
        if(DEBUGGING) fprintf(stderr,"robotStatus::robotConfig >> new wbInterface created ...\n");
        tmpContainer = (int *) wbInterface;
        if(DEBUGGING) fprintf(stderr,"robotStatus::robotConfig >> icubWholeBodyInterface has been created %p \n", wbInterface);
        //---------------- CONFIGURATION WHOLE BODY INTERFACE ----------------/
        // Add main iCub joints
        wbInterface->addJoints(ICUB_MAIN_JOINTS);
        // Initializing whole body interface
        if(!wbInterface->init()) {
            fprintf(stderr,"ERROR [robotStatus::robotConfig] Initializing Whole Body Interface!\n");
            return false;
        }
        else {
            fprintf(stderr,"robotStatus::robotConfig >> Whole Body Interface correctly initialized, yayyy!!!!\n");
        }

        // Put robot in position mode so that in won't fall at startup assuming it's balanced in its startup position
        if (VERBOSE) fprintf(stderr,"robotStatus::robotConfig >> About to set control mode\n");
        setCtrlMode(CTRL_MODE_POS);
    }

    // Initializing private variables. This must be done regardless of the new creation of wbInterface
    actJnts = wbInterface->getJointList().size();
    qRad.resize(actJnts,0.0);
    dqJ.resize(actJnts);

    ddqJ.resize(actJnts,0.0);
    tauJ.resize(actJnts,0.0);
    tauJ_out.resize(actJnts,0.0);


    return true;

}
//=========================================================================================================================
bool robotStatus::robotInit(int btype, int link) {

    // To initialize x_pose which will be used to compute either
    //forward kinematics or jacobians, it is necessary to know for what
    //it is desired. So far, we have it only for right and left leg and
    //center of mass
    /** There might be additional block types that should be considered here. Check out*/
    if(btype == 2 || btype == 3) {
        const char *linkName;
        int default_size = 0;
        int linkID = 0;
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
        // Output of forward kinematics and jacobian
        x_pose.resize(default_size,0.0);
    }

    // This variable JfootR must be changed with a more appropriate name
    JfootR.resize(NoChange,ICUB_DOFS+6);

    // dot{J}dot{q}
    dJdq.resize(6,0);

    /** \todo dot{xB} We will assume null velocity of the base for now since the estimate hasn't been implemented yet */
    dxB.resize(6,0);

    /** \todo ddot{xB} Assuming null acceleration for now since its estimation hasn't been implemented yet */
    ddxB.resize(6,0);

    /** \todo ity vector is assumed constant and oriented to the ground but this should vary according to the world reference frame */
    grav.resize(3,1);
    grav[0] = grav[1] = 0;
    grav[2] = -9.81;
    if(DEBUGGING) fprintf(stderr,"robotStatus::robotInit: reached here after setting gravity!\n");

    // Generalized bias forces term.
    hterm.resize(6+ICUB_DOFS,0);



    // Should the mass matrix be resized here??? In the future if the number of DOFS or limbs for which the interface will
    // be used are input parameters, all variables could be resized here and by default leave ICUB_DOFS.

    // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
    Ha.R = Rotation(0,0,1, 0,-1,0, 1,0,0);

    return true;
}
//=========================================================================================================================
void robotStatus::getLinkId(const char *linkName, int &lid) {
    char comlink[] = "com";
    if(strcmp(linkName,comlink) != 0) { // !=0 means that they're different
        wbInterface->getLinkId(linkName, lid);
    }
    else {
        lid = wbi::iWholeBodyModel::COM_LINK_ID;
    }
}
//=========================================================================================================================
//---------------- This is especifically for the COM ----------------
int robotStatus::getLinkId(const char *linkName) {
    comLinkId = iWholeBodyModel::COM_LINK_ID;
    return comLinkId;
}
//=========================================================================================================================
bool robotStatus::world2baseRototranslation(double *q) {
    /** \todo This method should take as input the link you wanna use for to define the world reference frame. Right now it's hard coded to be the left foot. */
#ifndef ICUB_FIXED
    int LINK_ID_LEFT_FOOT;
    getLinkId("l_sole",LINK_ID_LEFT_FOOT);
    wbInterface->computeH(q, Frame(), LINK_ID_LEFT_FOOT, H_base_leftFoot);
    H_base_leftFoot = H_base_leftFoot*Ha;
#else
    int LINK_ROOT;
    getLinkId("root_link",LINK_ROOT);
    wbInterface->computeH(q, Frame(), LINK_ROOT, H_base_leftFoot);
    H_base_leftFoot = H_base_leftFoot;
#endif
    H_base_leftFoot.setToInverse().get4x4Matrix(H_w2b.data());
    if(DEBUGGING) fprintf(stderr,"robotStatus::world2baseRototranslation >> H_base_leftFoot: %s \n",H_base_leftFoot.toString().c_str());
    if(DEBUGGING) fprintf(stderr,"robotStatus::world2baseRototranslation >> qRad           : %s \n",qRad.toString().c_str());
    xBase.set4x4Matrix(H_w2b.data());
    return true;
}
//=========================================================================================================================
bool robotStatus::robotJntAngles(bool blockingRead) {
    if(DEBUGGING) fprintf(stderr,"robotStatus::robotJntAngles >> About to estimate joint positions\n");
    return wbInterface->getEstimates(ESTIMATE_JOINT_POS, qRad.data(),-1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntVelocities(bool blockingRead) {
    return wbInterface->getEstimates(ESTIMATE_JOINT_VEL, dqJ.data(),-1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntAccelerations(bool blockingRead) {
    return wbInterface->getEstimates(ESTIMATE_JOINT_ACC, ddqJ.data(),-1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntTorques(bool blockingRead) {
    return wbInterface->getEstimates(ESTIMATE_JOINT_TORQUE, tauJ.data(),-1.0, blockingRead);
}
//=========================================================================================================================
Vector robotStatus::forwardKinematics(int &linkId) {
    if(robotJntAngles(false)) {
        if(world2baseRototranslation(qRad.data()))
        {
            footLinkId = linkId;
            if(DEBUGGING) fprintf(stderr,"robotStatus::forwardKinematics >> Forward kinematics will be computed with footLinkId: %d and x_pose: %s \n", footLinkId, x_pose.toString().c_str());

            bool ans = wbInterface->forwardKinematics(qRad.data(), xBase, footLinkId, x_pose.data());
            if(ans) {
                if(DEBUGGING) fprintf(stderr,"robotStatus::forwardKinematics >> Forward Kinematics computed \n");
                if(DEBUGGING) fprintf(stderr,"robotStatus::forwardKinematics >> pose: %s \n", x_pose.toString().c_str());
                return x_pose;
            }
//            else
//            {
//                x_pose.zero();
//                return x_pose;
//            }
        }
        else {
            fprintf(stderr,"ERROR [robotStatus::forwardKinematics] computing world 2 base rototranslation in robotStatus::forwardKinematics!\n");
        }
    }
    else {
        fprintf(stderr,"ERROR [robotStatus::forwardKinematics] acquiring robot joint angles in robotStatus::forwardKinematics\n");
    }
    x_pose.zero();
    return x_pose;
}
//=========================================================================================================================
JacobianMatrix robotStatus::jacobian(int &lid) {
    if(robotJntAngles(false)) {
        if(world2baseRototranslation(qRad.data())) {
            bool ans = wbInterface->computeJacobian(qRad.data(), xBase, lid, JfootR.data());
            if(ans)
            {
                return JfootR;
            }
//            else
//            {
//                JfootR.setZero();
//                return JfootR;
//            }
        }
        else {
            fprintf(stderr,"ERROR [robotStatus::jacobian] computing world to base rototranslation \n");
        }
    }
    else {
        fprintf(stderr,"ERROR [robotStatus::jacobian] getting robot joint angles to compute Jacobians \n");
    }
    JfootR.setZero();
    return JfootR;
}
//=========================================================================================================================
Vector robotStatus::getEncoders() {
    return qRad;
}
//=========================================================================================================================
VectorXd robotStatus::getJntVelocities() {
    return dqJ;
}
//=========================================================================================================================
Vector robotStatus::getJntAccelerations() {
    return  ddqJ;
}
//=========================================================================================================================
Vector robotStatus::getJntTorques() {
    return tauJ;
}
//=========================================================================================================================
bool robotStatus::setCtrlMode(ControlMode ctrl_mode) {
    if(wbInterface->setControlMode(ctrl_mode)) {
        return true;
    }
    else {
        fprintf(stderr,"ERROR [robotStatus::setCtrlMode] >> Velocity control mode could not be set\n");
        return false;
    }
}
//=========================================================================================================================
void robotStatus::setdqDes(Vector dqD) {
//     int n = dqD.length();
//     new (&dqDesMap)    Map<VectorXd>(dqD.data(),_n);
//     if(DEBUGGING) {
//         for(int i=0; i<dqD.length(); i++) {
//             dqDes[i] = dqD[i];
//         }
//         if(DEBUGGING) fprintf(stderr,"Now printing dqDesMap: \n");
//         for(int i=0; i<dqDesMap.SizeAtCompileTime; i++)
//             fprintf(stderr,"%f \n",dqDesMap(i));
//     }
//     if(!wbInterface->setControlReference(dqDesMap.data()))
//         fprintf(stderr, "ERROR control reference could not be set.\n");
    if(DEBUGGING) fprintf(stderr,"robotStatus::setdqDes >> control reference to be sent is: \n%s\n",dqD.toString().c_str());
    if(!wbInterface->setControlReference(dqD.data()))
        fprintf(stderr, "ERROR [robotStatus::setdqDes] control reference could not be set.\n");
}
//=========================================================================================================================
bool robotStatus::inverseDynamics(double *qrad_input, double *dq_input, double *ddq_input, double *tauJ_computed) {
    bool ans = false;
    if(world2baseRototranslation(qrad_input)) {
        if(DEBUGGING) fprintf(stderr,"robotStatus::inverseDynamics >> world2baseRototranslation computed\n");
        ans = wbInterface->inverseDynamics(qrad_input, xBase, dq_input, dxB.data(), ddq_input, ddxB.data(), grav.data(), tauJ_computed);
        
        if(DEBUGGING)
        {
            fprintf(stderr,"robotStatus::inverseDynamics >> Base vel: %s\n", dxB.toString().c_str());
            fprintf(stderr,"robotStatus::inverseDynamics >> Base acc: %s\n",ddxB.toString().c_str());
        }
    }
    return ans;
}
//=========================================================================================================================
bool robotStatus::dynamicsMassMatrix() {
    bool ans = false;
    if(robotJntAngles(false)) {
        if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsMassMatrix >> robotJntAngles computed\n");
        if(world2baseRototranslation(qRad.data())) {
            if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsMassMatrix >> world2baseRototranslation computed\n");
            ans = wbInterface->computeMassMatrix(qRad.data(),xBase, massMatrix.data());
        }
    }
    return ans;
}
//=========================================================================================================================
MassMatrix robotStatus::getMassMatrix() {
    return massMatrix;
}
//=========================================================================================================================
Vector robotStatus::dynamicsGenBiasForces() {
    bool ans = false;
    // grav is a 3x1 dim array
    if(robotJntAngles(false)) {
        if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> robotJntAngles computed for dynamicsGenBiasForces\n");
        if(robotJntVelocities(false)) {
            if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> robotJntVelocities computed for dynamicsGenBiasForces\n");
            if(world2baseRototranslation(qRad.data())) {
                if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> world2baseRototranslation computed for dynamicsGenBiasForces\n");
                if(robotBaseVelocity()) {
                    
                    if(DEBUGGING) {
                        Vector dqRad(ICUB_DOFS, dqJ.data());
                        fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> Base vel: %s\n", dxB.toString().c_str());
                        fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> Angs: %s\n",qRad.toString().c_str());
                        fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> Vels: %s\n",  dqRad.toString().c_str());
                    }
                    ans = wbInterface->computeGeneralizedBiasForces(qRad.data(), xBase, dqJ.data(), dxB.data(), grav.data(), hterm.data());
                }
            }
        }
    }
    if(ans) {
        if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsGenBiasForces >> h term: %s\n",hterm.toString().c_str());
        return hterm;
    }
    else {
        fprintf(stderr,"ERROR [robotStatus::dynamicsGenBiasForces] Generalized bias forces were not successfully computed\n");
        hterm.resize(ICUB_DOFS+6,0);
        return hterm;
    }
}
#ifdef NEWCODE
//=========================================================================================================================
bool robotStatus::robotBaseVelocity() {
//       ans = wbInterface->getEstimate(ESTIMATE_BASE_VEL, )
    return true;
}
//=========================================================================================================================
bool robotStatus::dynamicsDJdq(int &linkId) {
    bool ans = false;
    if(robotJntAngles(false)) {
        if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsDJd >> robotJntAngles computed for dynamicsDJdq\n");
        if(robotJntVelocities(false)) {
            if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsDJd >> robotJntVelocities computed for dynamicsDJdq\n");
            if(world2baseRototranslation(qRad.data())) {
                footLinkId = linkId;
                if(!robotBaseVelocity()) {
                    fprintf(stderr,"robotStatus::dynamicsDJd >> robotBaseVelocity failed in robotStatus::dynamicsDJd\n");
                    return false;
                }
                // This method does not use yet the last parameter xpose.
                if(DEBUGGING) fprintf(stderr,"robotStatus::dynamicsDJd >> link ID is: %d",footLinkId);
                ans = wbInterface->computeDJdq(qRad.data(), xBase, dqJ.data(), dxB.data(), footLinkId, dJdq.data());
            }
        }
    }
    return ans;
}
//=========================================================================================================================
Vector robotStatus::getDJdq() {
    return dJdq;
}
#endif

//------------------------------------------------------------------------------------------------------------------------//
// END robotStatus implementation ----------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//




//------------------------------------------------------------------------------------------------------------------------//
// COUNTERCLASS IMPLEMENTATION -------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//

counterClass::counterClass()
{
    count++;
}
//=========================================================================================================================
int counterClass::getCount()
{
    return count;
}

// -----------------------------------------------------------------------------------------------------------------------//
// END COUNTERCLASS IMPLEMENTATION----------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//





// -----------------------------------------------------------------------------------------------------------------------//
// START SIMULINK MDL IMPLEMENTATION--------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
    !mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))


// Function: MDL_CHECK_PARAMETERS
// Input parameters for locomotionController in Simulink

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    {
        if(!IS_PARAM_DOUBLE(BLOCK_TYPE_PARAM)) {
            ssSetErrorStatus(S,"mdlCheckParameters: 1st parameter to S-function ");
            return;
        }
    }
}
#endif  /*MDL_CHECK_PARAMETERS*/

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, s, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{

    if(DEBUGGING) fprintf(stderr,"mdlInitializeSizes >> STARTED\n");
    ssSetNumSFcnParams(S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S)!=NULL) {
            return;
        }
        else {
            fprintf(stderr,"mdlInitializeSize >> BLOCK TYPE IS: %d\n", static_cast<int>(mxGetScalar(ssGetSFcnParam(S,BLOCK_TYPE_IDX))));
        }
    } else {
        return; // Parameter mismatch reported by Simulink
    }
#endif

    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // Specify I/O
    if (!ssSetNumInputPorts(S, 5)) return;
    ssSetInputPortWidth(S, 0, 1);              	    		//INPUT for BLOCK TYPE
    ssSetInputPortWidth(S, 1, ICUB_DOFS);    	    		//INPUT for dqDes (control reference, for setting positions, velocities or torques)
    ssSetInputPortWidth(S, 2, ICUB_DOFS);			//INPUT for q (input angles different maybe from current ones)
    ssSetInputPortWidth(S, 3, ICUB_DOFS);			//INPUT for dq (input joint velocities maybe different from current ones)
    ssSetInputPortWidth(S, 4, ICUB_DOFS);			//INPUT for ddq (input joint accelerations maybe different from current ones)
    ssSetInputPortDataType(S, 0, SS_INT8);     	    		//Input data type
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);       		//The input will be used in the output
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    if (!ssSetNumOutputPorts(S,10)) return;
    ssSetOutputPortWidth   (S, 0, ICUB_DOFS);	    		// Robot joint angular positions in radians
    ssSetOutputPortWidth   (S, 1, ICUB_DOFS);	    		// Robot joint angular velocities in radians
    ssSetOutputPortWidth   (S, 2, 7);               		// foot or COM pose from fwdKinematics.
    ssSetOutputPortWidth   (S, 3, 186);             		// 6 x (N+6) Jacobians for a specific link
    ssSetOutputPortWidth   (S, 4, (ICUB_DOFS+6)*(ICUB_DOFS+6));	// Mass matrix of size (N+6 x N+6)
    ssSetOutputPortWidth   (S, 5, ICUB_DOFS+6);		    	// Generalized bias forces of size (N+6 x 1)
    ssSetOutputPortWidth   (S, 6, 6);		    		// dot{J}dot{q} term
    ssSetOutputPortWidth   (S, 7, ICUB_DOFS);			// Joint accelerations
    ssSetOutputPortWidth   (S, 8, ICUB_DOFS);			// Joint torques
    ssSetOutputPortWidth   (S, 9, ICUB_DOFS);			// Compute torques with inverse dynamics
    ssSetOutputPortDataType(S, 0, 0);
    ssSetOutputPortDataType(S, 1, 0);
    ssSetOutputPortDataType(S, 2, 0);
    ssSetOutputPortDataType(S, 3, 0);
    ssSetOutputPortDataType(S, 4, 0);
    ssSetOutputPortDataType(S, 5, 0);
    ssSetOutputPortDataType(S, 6, 0);
    ssSetOutputPortDataType(S, 7, 0);
    ssSetOutputPortDataType(S, 8, 0);
    ssSetOutputPortDataType(S, 9, 0);

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE | //we must be sure that every function we call never throws an exception
                 SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);

    // For FUTURE WORK this flag should be called and debug by correctly programming mdlTerminate.
    //SS_OPTION_CALL_TERMINATE_ON_EXIT); //this calls the terminate function even in case of errors
    if(DEBUGGING) fprintf(stderr,"mdlInitializeSizes >> FINISHED\n\n");

}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    if(DEBUGGING) fprintf(stderr,"mdlInitializeSampleTimes >> STARTED\n");
    // The sampling time of this SFunction must be inherited so that the Soft Real Time sblock can be used.
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // ssSetSampleTime(S, 0, 10.0);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);

    if(DEBUGGING) fprintf(stderr,"mdlInitializeSampleTimes >> FINISHED\n\n");
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    if(DEBUGGING) fprintf(stderr,"mdlStart >> STARTED\n");
    // Counts the times the blocks have been STARTED
    counterClass counter;
    if(DEBUGGING) fprintf(stderr,"mdlStart >> Publicly stating that a new child has been born: %d \n", counter.getCount());
    int_T buflen, status;
    char *String;

    buflen = mxGetN((ssGetSFcnParam(S, STRING_PARAM_IDX)))*sizeof(mxChar)+1;
    String = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, STRING_PARAM_IDX)),String,buflen);
    if (status) {
        ssSetErrorStatus(S,"mdlStart >> Cannot retrieve string from parameter 1!! \n");
        return;
    }
    if(DEBUGGING) fprintf(stderr,"mdlStart >> The string being passed for robotName is - %s\n ", String);

    string robot_name = String;

    status = mxGetString((ssGetSFcnParam(S, LOCAL_PARAM_IDX)),String,buflen);
    if (status) {
        ssSetErrorStatus(S,"mdlStart >> Cannot retrieve string from parameter 2!! \n");
        return;
    }
    if(DEBUGGING) fprintf(stderr,"mdlStart >> The string being passed for local is - %s \n", String);

    string local_name = String;

    real_T block_type = mxGetScalar(ssGetSFcnParam(S,BLOCK_TYPE_IDX));
    fprintf(stderr,"mdlStart >> BLOCK TYPE MASK PARAMETER: %f\n",block_type);

    // This will help determining the kind of block we'll be using
    real_T *x = (real_T*) ssGetDWork(S,0);
    x[0]      = block_type;

    Network yarp;

    if (!yarp.checkNetwork()) {
        ssSetErrorStatus(S,"mdlStart >> YARP server wasn't found active!! \n");
        return;
    }
    else {
        fprintf(stderr,"mdlStart >> YARP is up and running!!\n");
    }
    // -------------------- END YARP INITALIZATION STUFF --------------------

    // INPUT PARAMETER FOR PARAMETRIC FORWARD KINEMATICS and JACOBIANS
    InputPtrsType           u = ssGetInputPortSignalPtrs(S,0);
    InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;

    robotStatus *robot = new robotStatus();
    if(DEBUGGING) fprintf(stderr,"mdlStart >> An object robot of type wholeBodyInterface has been created\n");
    if(DEBUGGING) fprintf(stderr,"mdlStart >> About to configure robot \n");
    robot->setmoduleName(local_name);
    robot->setRobotName(robot_name);

    bool res = robot->robotConfig();
    if(res)
        fprintf(stderr,"mdlStart >> Succesfully exited robotConfig.\n");
    else {
        ssSetErrorStatus(S,"ERROR [mdlStart] in robotConfig.\n");
    }

    res = res && robot->robotInit(static_cast<int>(block_type), static_cast<int>(*uPtrs[0]));
    if(res==true)
        fprintf(stderr,"mdlStart >> Succesfully exiting robotConfig...\n");
    else {
        ssSetErrorStatus(S,"ERROR [mdlStart] in robotInit. \n");
    }

    ssGetPWork(S)[0] = robot;

    //--------------GLOBAL VARIABLES INITIALIZATION --------------
    dotq.Zero(ICUB_DOFS);
    fprintf(stderr,"mdlStart >> FINISHED\n\n");
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    double tinit, tend;
    if(TIMING) tinit = Time::now();
    //Getting type of block
    real_T *block_type = (real_T*) ssGetDWork(S,0);
    int btype = (int) block_type[0];
    if(DEBUGGING) {
        switch(btype)
        {
        case 0:
            fprintf(stderr,"mdlOutputs: This block will retrieve joints angles\n");
            break;
        case 1:
            fprintf(stderr,"mdlOutputs: This block will retrieve joints velocities\n");
            break;
        case 2:
            fprintf(stderr,"mdlOutputs: This block will retrieve parametric forward kinematics\n");
            break;
        case 3:
            fprintf(stderr,"mdlOutputs: This block will retrieve parametric Jacobians\n");
            break;
        case 4:
            fprintf(stderr,"mdlOutputs: This block will set velocities\n");
            break;
        case 5:
            fprintf(stderr,"mdlOutputs: This block will set positions\n");
            break;
        case 6:
            fprintf(stderr,"mdlOutputs: This block will set torques\n");
            break;
        case 7:
            fprintf(stderr,"mdlOutputs: This block will compute generalized bias forces from dynamics\n");
            break;
        case 8:
            fprintf(stderr,"mdlOutputs: This block will compute mass matrix from dynamics\n");
            break;
        case 9:
            fprintf(stderr,"mdlOutputs: This block will compute dJdq\n");
            break;
        case 10:
            fprintf(stderr,"mdlOutputs: This block will retrieve joint accelerations\n");
            break;
        case 11:
            fprintf(stderr,"mldOutputs: This block will retrieve joint torques\n");
            break;
        case 12:
            fprintf(stderr,"mdlOutputs: This block will compute inverse dynamics\n");
            break;
        default:
            fprintf(stderr,"ERROR: [mdlOutputs] The type of this block has not been defined\n");
        }
    }

    // READING FULL ROBOT JOINT ANGLES USING wbi
    robotStatus *robot = (robotStatus *) ssGetPWork(S)[0];
    bool blockingRead = false;

    if(DEBUGGING) fprintf(stderr,"mdlOutputs: wbInterface pointers: %p %p \n",robot->tmpContainer, robot->wbInterface);

//     int linkID;
    const char *linkName;


    // INPUT PARAMETER FOR FORWARD KINEMATICS
    InputPtrsType           u = ssGetInputPortSignalPtrs(S,0);
    InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;
    if(DEBUGGING) fprintf(stderr,"mdlOutputs: Input port value: %d \n", (int)(*uPtrs[0]));

    // This block will compute robot joint angles
    if(btype == 0) {
        if(robot->robotJntAngles(blockingRead))
        {
            qrad = robot->getEncoders();
            if(DEBUGGING) fprintf(stderr,"mdlOutputs: Angles have been retrieved:\n %s \n", qrad.toString().c_str());

            real_T *pY1 = (real_T *)ssGetOutputPortSignal(S,0);
            int_T widthPort = ssGetOutputPortWidth(S,0);
            for(int_T i=0; i<widthPort; i++) {
                pY1[i] = qrad((int) i);
            }
        }
        else {
            fprintf(stderr,"ERROR: [mdlOutputs] Robot Joint Angles could not be computed\n");
        }
    }

    // This block will compute robot joint velocities
    if(btype == 1) {
        if(DEBUGGING) fprintf(stderr,"mdlOutputs: About to send joint velocities to ports...\n");
        if(robot->robotJntVelocities(blockingRead))
        {
            dotq = robot->getJntVelocities();
            real_T *pY2 = (real_T *)ssGetOutputPortSignal(S,1);
            int_T widthPort = ssGetOutputPortWidth(S,1);
            for(int_T i=0; i<widthPort; i++ ) {
                pY2[i] = dotq((int) i);
            }
        }
        else
        {
            fprintf(stderr,"ERROR: [mdlOutputs] Robot joint velocities could not be computed\n");
        }
    }

    int lid = 0;

    // This block will compute forward kinematics of the specified link
    if(btype == 2) {
        switch ((int) *uPtrs[0])
        {
        case 0:
            linkName = "r_sole";
            break;
        case 1:
            linkName = "l_sole";
            break;
        case 2:
            linkName = "com";
            break;
        case 3:
            linkName = "r_gripper";
            break;
        case 4:
            linkName = "l_gripper";
            break;
        case 5:
            linkName = "head";
            break;
        default:
            fprintf(stderr,"ERROR: [mdlOutputs] No body part has been specified to compute forward kinematics\n");
        }
        robot->getLinkId(linkName,lid);

        xpose = robot->forwardKinematics(lid);

        real_T *pY3 = (real_T *)ssGetOutputPortSignal(S,2);
        for(int_T j=0; j<ssGetOutputPortWidth(S,2); j++) {
            pY3[j] = xpose((int) j);
        }
    }

    // This block will compute Jacobians for the specified link
    if(btype == 3) {
        switch ((int) *uPtrs[0])
        {
        case 0:
            linkName = "r_sole";
            break;
        case 1:
            linkName = "l_sole";
            break;
        case 2:
            linkName = "com";
            break;
        case 3:
            linkName = "r_gripper";
            break;
        case 4:
            linkName = "l_gripper";
            break;
        case 5:
            linkName = "head";
            break;
        default:
            fprintf(stderr,"ERROR: [mdlOutputs] No body part has been specified to compute jacobians\n");
        }
        robot->getLinkId(linkName,lid);

        jacob = robot->jacobian(lid);
        if(DEBUGGING) fprintf(stderr,"mdlOutputs: Jacobians Computed Succesfully. Jacobian is: \n");

        real_T *pY4 = (real_T *)ssGetOutputPortSignal(S,3);
        for(int_T j=0; j<ssGetOutputPortWidth(S,3); j++) {
            pY4[j] = jacob(j);
        }
    }

    // This block will set control references for the specified control mode
    if(btype == 4 || btype == 5 || btype == 6) {
        //GET INPUT dqDes
        InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);   //Get the corresponding pointer to "desired position port"
        int nu = ssGetInputPortWidth(S,1);                              //Getting the amount of elements of the input vector/matrix
        Vector dqDestmp;
        dqDestmp.resize(ICUB_DOFS,0.0);
        for(int j=0; j<nu; j++) {                                       //Reading inpute reference
            dqDestmp(j) = (*uPtrs1[j]);
        }
        if(btype == 4) robot->setCtrlMode(CTRL_MODE_VEL);
        if(btype == 5) robot->setCtrlMode(CTRL_MODE_POS);
        if(btype == 6) robot->setCtrlMode(CTRL_MODE_TORQUE);
        robot->setdqDes(dqDestmp);
    }

    Vector h;
    h.resize(ICUB_DOFS+6,0);
    // This block will compute the generalized bias force from the dynamics equation
    if(btype == 7) {
        h = robot->dynamicsGenBiasForces();
        real_T *pY5 = (real_T *)ssGetOutputPortSignal(S,5);
        for(int_T j=0; j<ssGetOutputPortWidth(S,5); j++) {
            pY5[j] = h(j);
        }
    }

    MassMatrix massMatrix;
    // This block will return the mass matrix from the dynamics equation
    if(btype == 8) {
        if(DEBUGGING) fprintf(stderr,"mdlOutputs: About to compute mass matrix\n");
        if(!robot->dynamicsMassMatrix()) {
            fprintf(stderr,"ERROR: [mdlOutputs] Mass matrix was not successfully computed\n");
        }
        massMatrix = robot->getMassMatrix();
        real_T *pY6 = (real_T *)ssGetOutputPortSignal(S,4);
        for(int_T j=0; j<ssGetOutputPortWidth(S,4); j++) {
            pY6[j] = massMatrix(j);
        }
    }

    Vector dJdq;
    dJdq.resize(6,0);
    // This block will compute dJdq from the dynamics equation for the specified link
    if(btype == 9) {
        if(DEBUGGING) fprintf(stderr,"mdlOutputs: About to compute dJdq\n");
        switch ((int) *uPtrs[0])
        {
        case 0:
            linkName = "r_sole";
            break;
        case 1:
            linkName = "l_sole";
            break;
        case 2:
            linkName = "com";
            break;
        case 3:
            linkName = "r_gripper";
            break;
        case 4:
            linkName = "l_gripper";
            break;
        case 5:
            linkName = "head";
            break;
        default:
            fprintf(stderr,"ERROR: [mdlOutputs] No body part has been specified to compute forward kinematics\n");
        }
        robot->getLinkId(linkName,lid);
        if(!robot->dynamicsDJdq(lid)) {
            fprintf(stderr,"ERROR: [mdlOutputs] dynamicsDJdq did not finish successfully\n");
        }
        else {
            dJdq = robot->getDJdq();
        }

        real_T *pY7 = (real_T *)ssGetOutputPortSignal(S,6);
        if(DEBUGGING) fprintf(stderr,"mdlOutputs: djdq computed is: %s \n",dJdq.toString().c_str());
        for(int_T j=0; j<ssGetOutputPortWidth(S,6); j++) {
            pY7[j] = dJdq(j);
        }
    }

    // This block will retrieve joint accelerations
    if(btype == 10) {
        Vector ddqJ;
        ddqJ.resize(ICUB_DOFS,0);
        if(robot->robotJntAccelerations(blockingRead)) {
            ddqJ = robot->getJntAccelerations();
            //Stream joint accelerations
            real_T *pY8 = (real_T*)ssGetOutputPortSignal(S,7);
            for(int_T j=0; j<ssGetOutputPortWidth(S,7); j++) {
                pY8[j] = ddqJ(j);
            }
        }
        else {
            fprintf(stderr,"ERROR: [mdlOutputs] Joint accelerations could not be retrieved\n");
        }
    }

    // This block will retrieve joint torques
    yarp::sig::Vector tauJ(ICUB_DOFS);
    if(btype == 11) {
        if(robot->robotJntTorques(blockingRead)) {
            tauJ = robot->getJntTorques();
            //Stream joint torques
            real_T *pY9 = (real_T*)ssGetOutputPortSignal(S,8);
            for(int_T j=0; j<ssGetOutputPortWidth(S,8); j++) {
                pY9[j] = tauJ(j);
            }
        }
        else {
            fprintf(stderr,"ERROR: [mdlOutputs) Joint torques could not be retrieved\n");
        }
    }

    // This block will compute inverse dynamics
    if(btype == 12) {
        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S,2);   //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth(S,2);                              	//Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize(ICUB_DOFS,0.0);
        for(int j=0; j<nu; j++) {                                       //Reading inpute reference
            qrad_in(j) = (*uPtrs2[j]);
        }

        //READ INPUT JOINT VELOCITIES
        InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs(S,3);   //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth(S,3);                              	//Getting the amount of elements of the input vector/matrix
        Vector dqrad_in;
        dqrad_in.resize(ICUB_DOFS,0.0);
        for(int j=0; j<nu; j++) {                                       //Reading inpute reference
            dqrad_in(j) = (*uPtrs3[j]);
        }

        //READ INPUT JOINT ACCELERATIONS
        InputRealPtrsType uPtrs4 = ssGetInputPortRealSignalPtrs(S,4);   //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth(S,4);                              	//Getting the amount of elements of the input vector/matrix
        Vector ddqrad_in;
        ddqrad_in.resize(ICUB_DOFS,0.0);
        for(int j=0; j<nu; j++) {                                       //Reading inpute reference
            ddqrad_in(j) = (*uPtrs4[j]);
        }

        yarp::sig::Vector tau_computed;
	tau_computed.resize(ICUB_DOFS+6,0.0);
        //TODO bla
        if(robot->inverseDynamics(qrad_in.data(), dqrad_in.data(), ddqrad_in.data(), tau_computed.data())) {
            if(DEBUGGING) {
                fprintf(stderr,"robotStatus::inverseDynamics >> Inverse dynamics has been computed correctly\n");
                fprintf(stderr,"robotStatus::inverseDynamics >> Angs: %s\n",qrad_in.toString().c_str());
                fprintf(stderr,"robotStatus::inverseDynamics >> Vels: %s\n",  dqrad_in.toString().c_str());
                fprintf(stderr,"robotStatus::inverseDynamics >> Accs: %s\n", ddqrad_in.toString().c_str());
            }

	    if(DEBUGGING) fprintf(stderr,"robotStatus::inverseDynamics >> Size of tau_computed is: \n%d\n",tau_computed.size());
            if(DEBUGGING) fprintf(stderr,"robotStatus::inverseDynamics >> Computed torques are: \n%s\n", tau_computed.toString().c_str());
            //Stream computed joint torques by inverseDynamics
            real_T *pY10 = (real_T*)ssGetOutputPortSignal(S,9);
            for(int_T j=0; j<ssGetOutputPortWidth(S,9); j++) {
		/**\todo Decide if we want to stream tau_computed including floating base torques */
                pY10[j] = tau_computed(j+6);
            }
        }
    }

    if(TIMING) tend = Time::now();
    if(TIMING) fprintf(stderr,"Time elapsed: %f \n",tend-tinit);

}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
    // IF YOU FORGET TO DESTROY OBJECTS OR DEALLOCATE MEMORY, MATLAB WILL CRASH.
    // Retrieve and destroy C++ object
    robotStatus *robot = (robotStatus *) ssGetPWork(S)[0];

    if(DEBUGGING) {
        fprintf(stderr,"mdlTerminate: Control variable states: \n");
        fprintf(stderr,"mdlTerminate: robot pointer: %p\n", robot);
    }

    if(robot!=NULL) {
        fprintf(stderr,"mdlTerminate >> Inside robot object %p \n",robot);
        if(robot->decreaseCounter()==0) {
            robot->setCtrlMode(CTRL_MODE_POS);
            delete robot;
            robot->resetCounter();
            robot = NULL;
            ssSetPWorkValue(S,0,NULL);
        }
    }
}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

