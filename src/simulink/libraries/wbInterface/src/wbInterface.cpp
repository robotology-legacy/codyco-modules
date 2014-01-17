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
#include <Eigen/Core>                               // import most common Eigen types
#include <Eigen/SVD>

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  robotState

// PARAMETERS MASK
#define NPARAMS 3                                   // Number of input parameters

#define BLOCK_TYPE_IDX 0                                  // Index number for first input parameter
#define BLOCK_TYPE_PARAM ssGetSFcnParam(S,BLOCK_TYPE_IDX)    // Get first input parameter from mask

#define STRING_PARAM_IDX 1
#define LENGTH 100

#define LOCAL_PARAM_IDX 2

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "matrix.h"
#include "simstruc.h"

#define VERBOSE 0

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
    !mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))


// Function: MDL_CHECK_PARAMETERS
// Input parameters for locomotionController in Simulink

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    {
        if(!IS_PARAM_DOUBLE(BLOCK_TYPE_PARAM)){
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
    ssSetNumSFcnParams(S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)){
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S)!=NULL){
            return;
        }
        else{
            cout<<"BLOCK TYPE IS: "<<BLOCK_TYPE_PARAM<<endl;
        }
    } else{
        return; // Parameter mismatch reported by Simulink
    }
#endif

    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // Specify I/O
    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1);              //Input FOR BLOCK TYPE
    ssSetInputPortWidth(S, 1, ICUB_DOFS);    //INPUT FOR dqDes
    ssSetInputPortDataType(S, 0, SS_INT8);     //Input data type
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);      //The input will be used in the output
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    if (!ssSetNumOutputPorts(S,4)) return;
    ssSetOutputPortWidth   (S, 0, ICUB_DOFS);
    ssSetOutputPortWidth   (S, 1, ICUB_DOFS);
    ssSetOutputPortWidth   (S, 2, 7);               // foot or COM pose from fwdKinematics.
    ssSetOutputPortWidth   (S, 3, 186);             // 6 x (N+6)
    ssSetOutputPortDataType(S, 0, 0);
    ssSetOutputPortDataType(S, 1, 0);
    ssSetOutputPortDataType(S, 2, 0);
    ssSetOutputPortDataType(S, 3, 0);

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE);

}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // The sampling time of this SFunction must be inherited so that the Soft Real Time sblock can be used.
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // ssSetSampleTime(S, 0, 10.0);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
     counterClass counter;
 //    fprintf(stderr,"Publicly stating that a new child has been born: %d \n", counter.getCount());




     // ######## CHECKING INPUT PARAMETERS ############
     int_T buflen, status;
     char *String;

     buflen = mxGetN((ssGetSFcnParam(S, STRING_PARAM_IDX)))*sizeof(mxChar)+1;
     String = static_cast<char*>(mxMalloc(buflen));
     status = mxGetString((ssGetSFcnParam(S, STRING_PARAM_IDX)),String,buflen);
 //    mexPrintf("The string being passed for robotName is - %s\n ", String);

     string robot_name = String;

     status = mxGetString((ssGetSFcnParam(S, LOCAL_PARAM_IDX)),String,buflen);
 //    mexPrintf("The string being passed for local is - %s \n", String);

     string local_name = String;

     real_T block_type = mxGetScalar(ssGetSFcnParam(S,BLOCK_TYPE_IDX));
     cout<<"BLOCK TYPE MASK PARAMETER: "<<block_type<<endl;

     // This will help determining the kind of block we'll be using
     real_T    *x = (real_T*) ssGetDWork(S,0);
     x[0]         = block_type;

     Network yarp;

     if (!yarp.checkNetwork()){
         ssSetErrorStatus(S,"YARP server wasn't found active!! \n");
         return;
     }
     else{
 //        mexPrintf("YARP is running!!\n");
     }
     // ############ END YARP INITALIZATION STUFF ##############



     // INPUT PARAMETER FOR PARAMETRIC FORWARD KINEMATICS and JACOBIANS
     InputPtrsType           u = ssGetInputPortSignalPtrs(S,0);
     InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;

     robotStatus *robot = new robotStatus();
 //    fprintf(stderr,"An object robot of type wholeBodyInterface has been created\n");

     //    if(counter.getCount()<2){
     // CONFIGURE AND INITIALIZE ROBOT INTERFACE
 //    fprintf(stderr,"about to configure robot \n");
     robot->setmoduleName(local_name);
     robot->setRobotName(robot_name);

     bool res = robot->robotConfig();
     res = res && robot->robotInit(static_cast<int>(block_type), static_cast<int>(*uPtrs[0]));
     if(res==true)
         fprintf(stderr,"Succesfully exiting robotConfig...\n");
     else{
         fprintf(stderr,"ERROR during robotConfig and/or robotInit ... \n");
         return;
     }

     ssGetPWork(S)[0] = robot;

     //    }
     // ########## GLOBAL VARIABLES INITIALIZATION ################
    dotq.Zero(ICUB_DOFS);
    fprintf(stderr,"MDLSTART FINISHES HERE ... \n");
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{

    cout<<"Simulation running ... "<<endl;
     //Getting type of block
     real_T *block_type = (real_T*) ssGetDWork(S,0);
//     cout<<"My block type ID is: "<<block_type[0]<<endl;
     int btype = (int) block_type[0];
//     switch(btype)
//     {
//    case 0:
//         fprintf(stderr,"This block will retrieve joints angles\n");
//         break;
//     case 1:
//         fprintf(stderr,"This block will retrieve joints velocities\n");
//         break;
//     case 2:
//         fprintf(stderr,"This block will retrieve parametric forward kinematics\n");
//         break;
//     case 3:
//         fprintf(stderr,"This block will retrieve parametric Jacobians\n");
//         break;
//     case 4:
//         fprintf(stderr,"This block will set velocities\n");
//         break;
//     }

     // READING FULL ROBOT JOINT ANGLES USING wbi AND wbiy
     robotStatus *robot = (robotStatus *) ssGetPWork(S)[0];
     bool blockingRead = false;

 //    fprintf(stderr,"wbInterface pointers: %p %p \n",robot->i, robot->wbInterface);

     int linkID;
     const char *linkName;


     // INPUT PARAMETER FOR FORWARD KINEMATICS
     InputPtrsType           u = ssGetInputPortSignalPtrs(S,0);
     InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;
     //    fprintf(stderr,"Input port value: %d \n", (int)(*uPtrs[0]));

     if(btype == 0){
 //        cout<<"About to send encoders to ports..."<<endl;
         if(robot->robotJntAngles(blockingRead))
         {
 //            fprintf(stderr,"global qrad before getting encoders: %s \n", qrad.toString().c_str());
             qrad = robot->getEncoders();
 //            fprintf(stderr,"angles have been computed: %s \n", qrad.toString().c_str());

             real_T *pY1 = (real_T *)ssGetOutputPortSignal(S,0);
             int_T widthPort = ssGetOutputPortWidth(S,0);
             for(int_T i=0; i<widthPort; i++){
                 pY1[i] = qrad((int) i);
             }
         }
         else{
             cout<<"ERROR: Robot Joint Angles could not be computed"<<endl;
         }

     }

     if(btype == 1){
 //        cout<<"About to send joint velocities to ports..."<<endl;
         if(robot->robotJntVelocities(blockingRead))
         {
             dotq = robot->getJntVelocities();
             real_T *pY2 = (real_T *)ssGetOutputPortSignal(S,1);
             int_T widthPort = ssGetOutputPortWidth(S,1);
             for(int_T i=0; i<widthPort; i++ ){
                 pY2[i] = dotq((int) i);
             }
         }
         else
         {
             cout<<"ERROR: Robot joint velocities could not be computed"<<endl;
         }
     }

     if(btype == 2 || btype == 3){
         //Interpreting link for either forwardKinematics or Jacobian.
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

         robot->getLinkId(linkName,linkID);
     }

     if(btype == 2){
         xpose = robot->forwardKinematics(linkID);

         real_T *pY3 = (real_T *)ssGetOutputPortSignal(S,2);
         for(int_T j=0; j<ssGetOutputPortWidth(S,2); j++){
             pY3[j] = xpose((int) j);
         }
     }

     if(btype == 3){
         // JACOBIANS!!!!!
         jacob = robot->jacobian(linkID);
         //    fprintf(stderr,"Jacobians Computed Succesfully. Jacobian is: \n");

         real_T *pY4 = (real_T *)ssGetOutputPortSignal(S,3);
         for(int_T j=0; j<ssGetOutputPortWidth(S,3); j++){
             pY4[j] = jacob(j);
         }
     }

     if(btype == 4 || btype == 5 || btype == 6){
        // VELOCITY CONTROL MODE
         //GET INPUT dqDes
         InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);    //Get the corresponding pointer to "desired position port"
         int nu = ssGetInputPortWidth(S,1);                              //Knowing the amount of elements of the input vector/matrix
         Vector dqDestmp;
         dqDestmp.resize(ICUB_DOFS,0.0);
//         for(int j=0; j<nu; j++) {
//             cout<< (*uPtrs1[j]) <<" "<<endl;
//         }
         for(int j=0; j<nu;j++){                                         //run through all values and do sthg with them
             dqDestmp(j) = (*uPtrs1[j]);
         }
         // SEND REFERENCES
         if(btype == 4) robot->setCtrlMode(CTRL_MODE_VEL);
         if(btype == 5) robot->setCtrlMode(CTRL_MODE_POS);
         robot->setdqDes(dqDestmp);
     }
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
        if(robot->decreaseCounter()==0){
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
