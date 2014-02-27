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

#define VERBOSE   1
#define DEBUGGING 1
#define TIMING    0
#define NEWCODE	  1

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


robotStatus::robotStatus():dqDesMap(0,0) {
    creationCounter++;
    fprintf(stderr,"ROBOTSTATUS instantiated %d times\n ",creationCounter);
    wbInterface = NULL;
}
//=========================================================================================================================
robotStatus::~robotStatus() {
    if(tmpContainer!=NULL) {
        creationCounter--;
        fprintf(stderr,"wbInterface in destructor: %p \n",wbInterface);
        if(wbInterface->close()) {
            delete wbInterface;
            fprintf(stderr,"wbInterface has been closed and deleted correctly. %d to go \n",creationCounter);
            wbInterface = NULL;
        }
        else {
            fprintf(stderr,"ERROR: wbInterface couldn't close correctly");
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
int robotStatus::getCounter() {
    return creationCounter;
}
//=========================================================================================================================
int robotStatus::decreaseCounter() {
    creationCounter--;
    return creationCounter;
}
//=========================================================================================================================
bool robotStatus::robotConfig() {
    if(VERBOSE) fprintf(stderr,"Configuring...\n");
    if(tmpContainer!=NULL) {
        wbInterface = (wholeBodyInterface *) tmpContainer;
        if(DEBUGGING) fprintf(stderr,"Copying wholeBodyInterface POINTER!\n");
    }
    else {
        //---------------- CREATION WHOLE BODY INTERFACE ---------------------/
        wbInterface = new icubWholeBodyInterface(moduleName.c_str(),robotName.c_str());
        if(DEBUGGING) fprintf(stderr,"new wbInterface created ...\n");
        tmpContainer = (int *) wbInterface;
        if(DEBUGGING) fprintf(stderr,"icubWholeBodyInterface has been created %p \n", wbInterface);
        //---------------- CONFIGURATION WHOLE BODY INTERFACE ----------------/
        // Add main iCub joints
        wbInterface->addJoints(ICUB_MAIN_JOINTS);
        // Initializing whole body interface
        if(!wbInterface->init()) {
            fprintf(stderr,"ERROR initializing Whole Body Interface!\n");
            return false;
        }
        else {
            fprintf(stderr,"Whole Body Interface correctly initialized, yayyy!!!!\n");
        }

        // Put robot in position mode so that in won't fall at startup assuming it's balanced in its startup position
        if (VERBOSE) fprintf(stderr,"About to set control mode\n");
        setCtrlMode(CTRL_MODE_POS);
    }

    // Initializing private variables. This must be done regardless of the new creation of wbInterface
    actJnts = wbInterface->getJointList().size();
    qRad.resize(actJnts,0.0);
    dqJ.resize(actJnts);

    return true;

}
//=========================================================================================================================
bool robotStatus::robotInit(int btype, int link) {

    // To initialize x_pose which will be used to compute either
    //forward kinematics or jacobians, it is necessary to know for what
    //it is desired. So far, we have it only for right and left leg and
    //center of mass
    const char *linkName;
    int default_size, linkID;

    if(btype == 2 || btype == 3) {
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

    // dot{xBase} We will assume null velocity of the base for now since the estimate hasn't been implemented yet
    dxB.resize(6,0);

    // Generalized bias forces term.
    hterm.resize(6+25,0);

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
bool robotStatus::world2baseRototranslation() {
    int LINK_ID_LEFT_FOOT;
    getLinkId("l_sole",LINK_ID_LEFT_FOOT);
    wbInterface->computeH(qRad.data(), Frame(), LINK_ID_LEFT_FOOT, H_base_leftFoot);
    H_base_leftFoot = H_base_leftFoot*Ha;
    H_base_leftFoot.setToInverse().get4x4Matrix(H_w2b.data());
    if(DEBUGGING) fprintf(stderr,"H_base_leftFoot: %s \n",H_base_leftFoot.toString().c_str());
    xBase.set4x4Matrix(H_w2b.data());
    return true;
}
//=========================================================================================================================
bool robotStatus::robotJntAngles(bool blockingRead) {
    if(DEBUGGING) fprintf(stderr,"About to estimate joint positions\n");
    return wbInterface->getEstimates(ESTIMATE_JOINT_POS, qRad.data(),-1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntVelocities(bool blockingRead) {
    return wbInterface->getEstimates(ESTIMATE_JOINT_VEL, dqJ.data(),-1.0, blockingRead);
}
//=========================================================================================================================
Vector robotStatus::forwardKinematics(int &linkId) {
    if(robotJntAngles(false)) {
        if(world2baseRototranslation())
        {
            footLinkId = linkId;
            if(DEBUGGING) fprintf(stderr,"fwd kinematics will be computed with footLinkId: %d and x_pose: %s \n", footLinkId, x_pose.toString().c_str());

            bool ans = wbInterface->forwardKinematics(qRad.data(), xBase, footLinkId, x_pose.data());
            if(ans) {
                if(DEBUGGING) fprintf(stderr,"Forward Kinematics computed \n");
                if(DEBUGGING) fprintf(stderr,"pose: %s \n", x_pose.toString().c_str());
                return x_pose;
            }
//            else
//            {
//                x_pose.zero();
//                return x_pose;
//            }
        }
        else {
            fprintf(stderr,"ERROR computing world 2 base rototranslation in robotStatus::forwardKinematics!\n");
        }
    }
    else {
        fprintf(stderr,"ERROR acquiring robot joint angles in robotStatus::forwardKinematics\n");
    }
    x_pose.zero();
    return x_pose;
}
//=========================================================================================================================
JacobianMatrix robotStatus::jacobian(int &lid) {
    if(robotJntAngles(false)) {
        if(world2baseRototranslation()) {
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
            fprintf(stderr,"ERROR computing world to base rototranslation \n");
        }
    }
    else {
        fprintf(stderr,"ERROR getting robot joint angles to compute Jacobians \n");
    }
    JfootR.setZero();
    return JfootR;
}
//=========================================================================================================================
Vector robotStatus::getEncoders() {
    if (DEBUGGING) fprintf(stderr,"qrad before returning from getEncoders: %s \n ",qRad.toString().c_str());
    return qRad;
}
//=========================================================================================================================
VectorXd robotStatus::getJntVelocities() {
    return dqJ;
}
//=========================================================================================================================
bool robotStatus::setCtrlMode(ControlMode ctrl_mode) {
    if(wbInterface->setControlMode(ctrl_mode)) {
        return true;
    }
    else {
        cout<<"ERROR: Velocity control mode could not be set"<<endl;
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
    fprintf(stderr,"Sono arrivato!!!!!!\n"); 
    fprintf(stderr,"control reference to be sent is: %s\n",dqD.toString().c_str());
    if(!wbInterface->setControlReference(dqD.data()))
    fprintf(stderr, "ERROR control reference could not be set.\n");    
}
//=========================================================================================================================
bool robotStatus::dynamicsMassMatrix() {
    bool ans = false;
    if(robotJntAngles(false)) {
        if(DEBUGGING) fprintf(stderr,"robotJntAngles computed for dynamicsMassMatrix\n");
        if(world2baseRototranslation()) {
            if(DEBUGGING) fprintf(stderr,"world2baseRototranslation computed for dynamicsMassMatrix\n");
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
    double grav[3]= {0, 0, -9.81};
    if(robotJntAngles(false)) {
        if(DEBUGGING) fprintf(stderr,"robotJntAngles computed for dynamicsGenBiasForces\n");
        if(robotJntVelocities(false)) {
            if(DEBUGGING) fprintf(stderr,"robotJntVelocities computed for dynamicsGenBiasForces\n");
            if(world2baseRototranslation()) {
                if(DEBUGGING) fprintf(stderr,"world2baseRototranslation computed for dynamicsGenBiasForces\n");
                if(robotBaseVelocity()) {
                    if(DEBUGGING) {
		      fprintf(stderr,"robotBaseVelocity computed for dynamicsGenBiasForces\n");		    
		      fprintf(stderr,"Angles: %s\n",qRad.toString().c_str());
		      cerr<<"Velocities: "<<dqJ<<endl;
		      fprintf(stderr,"Base velocity: %s\n",dxB.toString().c_str());
		    }
                    ans = wbInterface->computeGeneralizedBiasForces(qRad.data(), xBase, dqJ.data(), dxB.data(), grav, hterm.data());
                }
            }
        }
    }
    if(ans) {
        fprintf(stderr,"h term: %s\n",hterm.toString().c_str());
        return hterm;
    }
    else {
        fprintf(stderr,"ERROR generalized bias forces were not successfully computed\n");
        hterm.resize(25+6,0);
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
        if(DEBUGGING) fprintf(stderr,"robotJntAngles computed for dynamicsDJdq\n");
        if(robotJntVelocities(false)) {
            if(DEBUGGING) fprintf(stderr,"robotJntVelocities computed for dynamicsDJdq\n");
            if(world2baseRototranslation()) {
                footLinkId = linkId;
                if(!robotBaseVelocity()) {
                    fprintf(stderr,"robotBaseVelocity failed in robotStatus::dynamicsDJd\n");
                    return false;
                }
                // This method does not use yet the last parameter xpose.
                if(DEBUGGING) fprintf(stderr,"link ID is: %d",footLinkId);
                ans = wbInterface->computeDJdq(qRad.data(), xBase, dqJ.data(), dxB.data(), footLinkId, dJdq.data());
            }
        }
    }
    return ans;
}
//=========================================================================================================================
Vector robotStatus::getDJdq(){
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
            ssSetErrorStatus(S,"1st parameter to S-function ");
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

    if(DEBUGGING) fprintf(stderr,"STARTED mdlInitializeSizes\n");
    ssSetNumSFcnParams(S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S)!=NULL) {
            return;
        }
        else {
            fprintf(stderr,"BLOCK TYPE IS: %d\n", static_cast<int>(mxGetScalar(ssGetSFcnParam(S,BLOCK_TYPE_IDX))));
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
    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1);              	    		//Input FOR BLOCK TYPE
    ssSetInputPortWidth(S, 1, ICUB_DOFS);    	    		//INPUT FOR dqDes
    ssSetInputPortDataType(S, 0, SS_INT8);     	    		//Input data type
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);       		//The input will be used in the output
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    if (!ssSetNumOutputPorts(S,7)) return;
    ssSetOutputPortWidth   (S, 0, ICUB_DOFS);	    		// Robot joint angular positions in radians
    ssSetOutputPortWidth   (S, 1, ICUB_DOFS);	    		// Robot joint angular velocities in radians
    ssSetOutputPortWidth   (S, 2, 7);               		// foot or COM pose from fwdKinematics.
    ssSetOutputPortWidth   (S, 3, 186);             		// 6 x (N+6) Jacobians for a specific link
    ssSetOutputPortWidth   (S, 4, (ICUB_DOFS+6)*(ICUB_DOFS+6));	// Mass matrix of size (N+6 x N+6)
    ssSetOutputPortWidth   (S, 5, ICUB_DOFS+6);		    	// Generalized bias forces of size (N+6 x 1)
    ssSetOutputPortWidth   (S, 6, 6);		    		// dot{J}dot{q} term
    ssSetOutputPortDataType(S, 0, 0);
    ssSetOutputPortDataType(S, 1, 0);
    ssSetOutputPortDataType(S, 2, 0);
    ssSetOutputPortDataType(S, 3, 0);
    ssSetOutputPortDataType(S, 4, 0);
    ssSetOutputPortDataType(S, 5, 0);
    ssSetOutputPortDataType(S, 6, 0);

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
    if(DEBUGGING) fprintf(stderr,"FINISHED mdlInitializeSizes\n");

}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    if(DEBUGGING) fprintf(stderr,"STARTED mdlInitializeSampleTimes\n");
    // The sampling time of this SFunction must be inherited so that the Soft Real Time sblock can be used.
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // ssSetSampleTime(S, 0, 10.0);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);

    if(DEBUGGING) fprintf(stderr,"FINISHED mdlInitializeSampleTimes\n");
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    if(DEBUGGING) fprintf(stderr,"STARTED mdlStart ...\n");
    counterClass counter;
    if(DEBUGGING) fprintf(stderr,"Publicly stating that a new child has been born: %d \n", counter.getCount());
    int_T buflen, status;
    char *String;

    buflen = mxGetN((ssGetSFcnParam(S, STRING_PARAM_IDX)))*sizeof(mxChar)+1;
    String = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, STRING_PARAM_IDX)),String,buflen);
    if (status) {
        ssSetErrorStatus(S,"Cannot retrieve string from parameter 1!! \n");
        return;
    }
    if(DEBUGGING) fprintf(stderr,"The string being passed for robotName is - %s\n ", String);
    
    string robot_name = String;

    status = mxGetString((ssGetSFcnParam(S, LOCAL_PARAM_IDX)),String,buflen);
    if (status) {
        ssSetErrorStatus(S,"Cannot retrieve string from parameter 2!! \n");
        return;
    }
    if(DEBUGGING) fprintf(stderr,"The string being passed for local is - %s \n", String);

    string local_name = String;

    real_T block_type = mxGetScalar(ssGetSFcnParam(S,BLOCK_TYPE_IDX));
    fprintf(stderr,"BLOCK TYPE MASK PARAMETER: %f\n",block_type);

    // This will help determining the kind of block we'll be using
    real_T *x = (real_T*) ssGetDWork(S,0);
    x[0]      = block_type;

    Network yarp;

    if (!yarp.checkNetwork()) {
        ssSetErrorStatus(S,"YARP server wasn't found active!! \n");
        return;
    }
    else {
        fprintf(stderr,"YARP is up and running!!\n");
    }
    // -------------------- END YARP INITALIZATION STUFF --------------------

    // INPUT PARAMETER FOR PARAMETRIC FORWARD KINEMATICS and JACOBIANS
    InputPtrsType           u = ssGetInputPortSignalPtrs(S,0);
    InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;

    robotStatus *robot = new robotStatus();
    if(DEBUGGING) fprintf(stderr,"An object robot of type wholeBodyInterface has been created\n");
    if(DEBUGGING) fprintf(stderr,"about to configure robot \n");
    robot->setmoduleName(local_name);
    robot->setRobotName(robot_name);

    bool res = robot->robotConfig();
    if(res)
        fprintf(stderr,"Succesfully exited robotConfig.\n");
    else {
        ssSetErrorStatus(S,"ERROR in robotConfig.\n");
    }

    res = res && robot->robotInit(static_cast<int>(block_type), static_cast<int>(*uPtrs[0]));
    if(res==true)
        fprintf(stderr,"Succesfully exiting robotConfig...\n");
    else {
        ssSetErrorStatus(S,"ERROR in robotInit. \n");
    }

    ssGetPWork(S)[0] = robot;

    //--------------GLOBAL VARIABLES INITIALIZATION --------------
    dotq.Zero(ICUB_DOFS);
    fprintf(stderr,"FINISHED mdlStart. \n");
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
            fprintf(stderr,"This block will retrieve joints angles\n");
            break;
        case 1:
            fprintf(stderr,"This block will retrieve joints velocities\n");
            break;
        case 2:
            fprintf(stderr,"This block will retrieve parametric forward kinematics\n");
            break;
        case 3:
            fprintf(stderr,"This block will retrieve parametric Jacobians\n");
            break;
        case 4:
            fprintf(stderr,"This block will set velocities\n");
            break;
        case 5:
            fprintf(stderr,"This block will set positions\n");
            break;
        case 7:
            fprintf(stderr,"This block will compute generalized bias forces from dynamics\n");
            break;
        case 8:
            fprintf(stderr,"This block will compute mass matrix from dynamics\n");
            break;
        case 9:
            fprintf(stderr,"This block will compute dJdq\n");
            break;
        }
    }

    // READING FULL ROBOT JOINT ANGLES USING wbi
    robotStatus *robot = (robotStatus *) ssGetPWork(S)[0];
    bool blockingRead = false;

    if(DEBUGGING) fprintf(stderr,"wbInterface pointers: %p %p \n",robot->tmpContainer, robot->wbInterface);

    int linkID;
    const char *linkName;


    // INPUT PARAMETER FOR FORWARD KINEMATICS
    InputPtrsType           u = ssGetInputPortSignalPtrs(S,0);
    InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;
    if(DEBUGGING) fprintf(stderr,"Input port value: %d \n", (int)(*uPtrs[0]));

    if(btype == 0) {
        if(robot->robotJntAngles(blockingRead))
        {
            if(DEBUGGING) fprintf(stderr,"global qrad before getting encoders: %s \n", qrad.toString().c_str());
            qrad = robot->getEncoders();
            if(DEBUGGING) fprintf(stderr,"angles have been computed: %s \n", qrad.toString().c_str());

            real_T *pY1 = (real_T *)ssGetOutputPortSignal(S,0);
            int_T widthPort = ssGetOutputPortWidth(S,0);
            for(int_T i=0; i<widthPort; i++) {
                pY1[i] = qrad((int) i);
            }
        }
        else {
            cout<<"ERROR: Robot Joint Angles could not be computed"<<endl;
        }
    }

    if(btype == 1) {
        if(DEBUGGING) cout<<"About to send joint velocities to ports..."<<endl;
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
            cout<<"ERROR: Robot joint velocities could not be computed"<<endl;
        }
    }

    int lid;

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
        }
        robot->getLinkId(linkName,lid);

        xpose = robot->forwardKinematics(lid);

        real_T *pY3 = (real_T *)ssGetOutputPortSignal(S,2);
        for(int_T j=0; j<ssGetOutputPortWidth(S,2); j++) {
            pY3[j] = xpose((int) j);
        }
    }

    if(btype == 3) {
        // JACOBIANS!!!!!
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
        }
        robot->getLinkId(linkName,lid);

        jacob = robot->jacobian(lid);
        if(DEBUGGING) fprintf(stderr,"Jacobians Computed Succesfully. Jacobian is: \n");

        real_T *pY4 = (real_T *)ssGetOutputPortSignal(S,3);
        for(int_T j=0; j<ssGetOutputPortWidth(S,3); j++) {
            pY4[j] = jacob(j);
        }
    }

    if(btype == 4 || btype == 5 || btype == 6) {
        // VELOCITY CONTROL MODE
        //GET INPUT dqDes
        InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);    //Get the corresponding pointer to "desired position port"
        int nu = ssGetInputPortWidth(S,1);                              //Knowing the amount of elements of the input vector/matrix
        Vector dqDestmp;
        dqDestmp.resize(ICUB_DOFS,0.0);
        for(int j=0; j<nu; j++) {                                       //run through all values and do sthg with them
            dqDestmp(j) = (*uPtrs1[j]);
        }
        // SEND REFERENCES
        if(btype == 4) robot->setCtrlMode(CTRL_MODE_VEL);
        if(btype == 5) robot->setCtrlMode(CTRL_MODE_POS);
	if(btype == 6) robot->setCtrlMode(CTRL_MODE_TORQUE);
        robot->setdqDes(dqDestmp);
    }

    Vector h;
    h.resize(ICUB_DOFS+6,0);
    if(btype == 7) {
        h = robot->dynamicsGenBiasForces();
        real_T *pY5 = (real_T *)ssGetOutputPortSignal(S,5);
        for(int_T j=0; j<ssGetOutputPortWidth(S,5); j++) {
            pY5[j] = h(j);
        }
    }

    MassMatrix massMatrix;
    if(btype == 8) {
        if(DEBUGGING) fprintf(stderr,"About to compute mass matrix\n");
        if(!robot->dynamicsMassMatrix()) {
            fprintf(stderr,"ERROR Mass matrix was not successfully computed\n");
        }
        massMatrix = robot->getMassMatrix();
        real_T *pY6 = (real_T *)ssGetOutputPortSignal(S,4);
        for(int_T j=0; j<ssGetOutputPortWidth(S,4); j++) {
            pY6[j] = massMatrix(j);
        }
    }
  
    Vector dJdq;
    dJdq.resize(6,0);
    if(btype == 9) {
        if(DEBUGGING) fprintf(stderr,"About to compute dJdq\n");
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
        }
        
        robot->getLinkId(linkName,lid);
        if(!robot->dynamicsDJdq(lid)){
	  fprintf(stderr,"ERROR dynamicsDJdq did not finish successfully\n");
	}
	else{
	  dJdq = robot->getDJdq();
	}
        
        real_T *pY7 = (real_T *)ssGetOutputPortSignal(S,6);
	if(DEBUGGING) fprintf(stderr,"djdq computed is: %s \n",dJdq.toString().c_str());
        for(int_T j=0; j<ssGetOutputPortWidth(S,6); j++) {
            pY7[j] = dJdq(j);
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

    if(robot!=NULL) {
        fprintf(stderr,"Inside robot object %p \n",robot);
        if(robot->decreaseCounter()==0) {
            robot->setCtrlMode(CTRL_MODE_POS);
            delete robot;
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
