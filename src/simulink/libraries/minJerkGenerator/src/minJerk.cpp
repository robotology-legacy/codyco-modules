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

#include "minJerk.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  minJerkGenerator

// PARAMETERS MASK
#define NPARAMS 3                                           // Number of input parameters

#define PARAM_IDX_1 0                                       // Index number for first input parameter
#define PARAM_VAL_1 ssGetSFcnParam(S,PARAM_IDX_1)           // Get min. Jerk Traj dimension
#define PARAM_TASKDOF mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1));

#define PARAM_IDX_2 1
// THIS SHOULD BE CHANGED AND ACQUIRED SOMEHOW FROM THE SOFT REAL TIME BLOCK
#define PARAM_VAL_2 ssGetSFcnParam(S,PARAM_IDX_2)           // Get min. Jerk Traj. Simulation Rate

#define PARAM_IDX_3 2
#define PARAM_VAL_3 ssGetSFcnParam(S,PARAM_IDX_3)

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "matrix.h"
#include "simstruc.h"

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
    !mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))


// Function: MDL_CHECK_PARAMETERS
// Input parameters for locomotionController in Simulink

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    {
        if(!IS_PARAM_DOUBLE(PARAM_VAL_1) || !IS_PARAM_DOUBLE(PARAM_VAL_2) || !IS_PARAM_DOUBLE(PARAM_VAL_3)){
            ssSetErrorStatus(S,"Trajectory dimension and simulation rate should be integers");
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
            cout<<"All parameters have been checked and passed correctly"<<endl;
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
    // - Inputs
    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)));                  //Input
    ssSetInputPortDataType(  S, 0, SS_DOUBLE);                                              //Input data type
    ssSetInputPortDirectFeedThrough(S, 0, 1);                                               //The input will be used in the output

    ssSetInputPortWidth(S, 1, mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)));
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    // - Outputs
    if (!ssSetNumOutputPorts(S,3)) return;
    ssSetOutputPortWidth   (S, 0, mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)));
    ssSetOutputPortWidth   (S, 1, mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)));
    ssSetOutputPortWidth   (S, 2, mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)));               // foot pose from aatics
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    // DWork vectors
    ssSetNumDWork(S, 3);
    ssSetDWorkWidth(S, 0, 3);
    ssSetDWorkWidth(S, 1, 1);
    ssSetDWorkWidth(S, 2, 1);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    ssSetDWorkDataType(S, 1, SS_DOUBLE);
    ssSetDWorkDataType(S, 2, SS_DOUBLE);

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
    // ######### YARP INITIALIZATION STUFF ##################
    fprintf(stderr,"YARP NETWORK INITIALIZED\n");
    Network yarp;

    if (!yarp.checkNetwork()){
        ssSetErrorStatus(S,"YARP server wasn't found active!! \n");
        return;
    }
    else
        cout<<"YARP is running!!\n"<<endl;

    // ######## CHECKING INPUT PARAMETERS ############
    int param_taskDOF     = static_cast<int>(mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)));
    double param_rate        = mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_2));
    double param_trajtime = static_cast<double>(mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_3)));
    cout<<"BLOCK TASK DOF PARAMETER:  "<<param_taskDOF <<endl;
    cout<<"SIMULATION RATE PARAMETER: "<<param_rate    <<endl;
    cout<<"TRAJECTORY TIME PARAMETER: "<<param_trajtime<<endl;

    // This will help determining the kind of minJerk Trajectory we'll be using
    real_T    *x = (real_T*) ssGetDWork(S,0);
    x[0]         = param_taskDOF;
    x[1]         = param_rate;
    x[2]         = param_trajtime;


    // ############ MINIMUM JERK INITIALIZATION ###############
    minJerkTrajGenerator *minJerkTraj = new minJerkTrajGenerator(param_taskDOF);
    fprintf(stderr,"An object minJerkTraj has been created\n");

    //############ DEALING WITH INPUT VALUES ###################
//     // Reading all input ports
//     int_T i,j;
//     int_T nInputPorts = ssGetNumInputPorts(S);                          //First retrieve number of input ports
//     for(i = 0; i < nInputPorts; i++){                                   //Then for each input port
//         InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,i);    //Get the corresponding pointer to that port
//         int_T nu = ssGetInputPortWidth(S,i);                            //Knowing the amount of elements of the input vector/matrix
//         for(j=0; j<nu;j++){                                             //run through all values and do sthg with them
//             if(i==0) minJerkTraj->setInitPos(j,*uPtrs[j]);              //in our case we want to store them as vectors
//         }
//     }
    

//     fprintf(stderr,"initPos contains: %s \n", (minJerkTraj->getInitPos()).toString().c_str());
//     Configure and initalize minJerkTraj
//     if(minJerkTraj->initialize(param_rate, param_trajtime))
//         fprintf(stderr,"successfully initialized minJerkTraj\n");
    // ########### END MINIMUM JERK INITALIZATION #############

    ssGetPWork(S)[0] = minJerkTraj;
    real_T    *flag = (real_T*) ssGetDWork(S,1);
    flag[0]         = 1;
    

    fprintf(stderr,"MDLSTART FINISHES HERE ... \n");
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{

    //MinJerkGenerator
    minJerkTrajGenerator *minJerkTraj = static_cast<minJerkTrajGenerator*>(ssGetPWork(S)[0]);

    real_T *flag = (real_T *) ssGetDWork(S,1);
    real_T    *x = (real_T*) ssGetDWork(S,0);
    
    if(flag[0]){    
    int_T i,j;
    int_T nInputPorts = ssGetNumInputPorts(S);                          //First retrieve number of input ports
    for(i = 0; i < nInputPorts; i++){                                   //Then for each input port
        InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,i);    //Get the corresponding pointer to that port
        int_T nu = ssGetInputPortWidth(S,i);                            //Knowing the amount of elements of the input vector/matrix
        for(j=0; j<nu;j++){                                             //run through all values and do sthg with them
            if(i==0) minJerkTraj->setInitPos(j,*uPtrs[j]);              //in our case we want to store them as vectors
        }
    }    
    
//     fprintf(stderr,"initPos contains: %s \n", (minJerkTraj->getInitPos()).toString().c_str());
    double param_rate     = x[1];
    double param_trajtime = x[2];
    minJerkTraj->initialize(param_rate, param_trajtime);
//         fprintf(stderr,"successfully initialized minJerkTraj\n");
    
    flag[0] = 0;
    }
    //############ DEALING WITH INPUT VALUES ###################
    // Reading DESIRED POS port
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,1);    //Get the corresponding pointer to "desired position port"
    int nu = ssGetInputPortWidth(S,1);                              //Knowing the amount of elements of the input vector/matrix
    for(int j=0; j<nu;j++){                                         //run through all values and do sthg with them
        minJerkTraj->setPos(j,*uPtrs[j]);                           //in our case we want to store them as vectors
    }

    minJerkTraj->updateReferenceTraj();

    // Streaming out encoders and joint velocities

    Vector tmpPos = minJerkTraj->getPosition();
    Vector tmpVel = minJerkTraj->getVelocity();
    Vector tmpAcc = minJerkTraj->getAcceleration();
    for(int i=0; i<tmpPos.length(); i++)
        fprintf(stderr,"%f ",tmpPos[i]);
    printf("\n");
//    fprintf(stderr,"About to send data to ports \n");
    int_T nOutputPorts = ssGetNumOutputPorts(S);   // FOR THE FIRST TWO OUTPUT PORTS (q and dq)
    for(int_T i=0; i<nOutputPorts; i++){
        real_T *pY1 = (real_T *)ssGetOutputPortSignal(S,0);
        real_T *pY2 = (real_T *)ssGetOutputPortSignal(S,1);
        real_T *pY3 = (real_T *)ssGetOutputPortSignal(S,2);
        int_T widthPort = ssGetOutputPortWidth(S,i);
        for(int_T j=0; j<widthPort; j++){
            pY1[j] = tmpPos[j];           //THESE METHODS SHOULD BE MODIFIED ACCORDINGLY!!!!
            pY2[j] = tmpVel[j];
            pY3[j] = tmpAcc[j];
        }
    }

}

static void mdlTerminate(SimStruct *S)
{
    // IF YOU FORGET TO DESTROY OBJECTS OR DEALLOCATE MEMORY, MATLAB WILL CRASH.
    // Retrieve and destroy C++ object
    minJerkTrajGenerator *minJerkTraj = static_cast<minJerkTrajGenerator*>(ssGetPWork(S)[0]);

    if(minJerkTraj!=NULL) {
        fprintf(stderr,"Deleting minJerkTraj object %p \n",minJerkTraj);
        delete minJerkTraj;
        minJerkTraj = NULL;
    }
    if(ssGetPWork(S) != NULL){
        ssSetPWorkValue(S,0,NULL);
    }
    fprintf(stderr,"Everything was closed correctly\n");
}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
