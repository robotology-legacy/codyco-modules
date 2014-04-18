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

#include "yarpWrite.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME yWrite

// PARAMETERS MASK
#define NPARAMS 1                                                   // Number of input parameters

#define PARAM_IDX_1 0                                               // port name


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
    // INPUTS
    if(!ssSetNumInputPorts(S,1)) return;
//    ssSetInputPortWidth(S,0,mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_2)));
//    ssSetInputPortDataType(S,0,SS_DOUBLE);
//    ssSetInputPortDirectFeedThrough(S,0,1);
    ssSetInputPortWidth(S,0,DYNAMICALLY_SIZED);
    ssSetInputPortDirectFeedThrough(S,0,1);
    ssSetInputPortOverWritable(S,0,1);
    ssSetInputPortOptimOpts(S,0,SS_REUSABLE_AND_LOCAL);
    ssSetInputPortSampleTime(S,0,INHERITED_SAMPLE_TIME);

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    // DWork vectors
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);

}

#if defined(MATLAB_MEX_FILE)
# define MDL_SET_INPUT_PORT_WIDTH
  static void mdlSetInputPortWidth(SimStruct *S, int_T port,
                                    int_T inputPortWidth)
  {
      ssSetInputPortWidth(S,port,inputPortWidth);
  }

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
  /* Function: mdlSetDefaultPortDimensionInfo ===========================================
   * Abstract:
   *   In case no ports were specified, the default is an input port of width 2
   *   and an output port of width 1.
   */
  static void mdlSetDefaultPortDimensionInfo(SimStruct        *S)
  {
      ssSetInputPortWidth(S, 0, 1);
  }
#endif

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
    Network::init();

    if (!Network::checkNetwork()){
        ssSetErrorStatus(S,"YARP server wasn't found active!! \n");
        return;
    }
    else
        cout<<"YARP is running!!\n"<<endl;

    int_T buflen, status;
    char *String;

    buflen = mxGetN((ssGetSFcnParam(S, PARAM_IDX_1)))*sizeof(mxChar)+1;
    String = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_1)),String,buflen);
    if (status) {
        ssSetErrorStatus(S,"Cannot retrieve string from parameter 1!! \n");
        return;
    }
    //string port_name = String;

	char *port_name = String;

    cout<<"Port name will be: "<<port_name<<endl;

    // ######## CHECKING INPUT PARAMETERS ############

    Port *port;
    port = new Port;
    port->open(port_name);

    ssGetPWork(S)[0] = port;

}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{

    Port *port = static_cast<Port*>(ssGetPWork(S)[0]);

    Bottle bot;

    // Reading DESIRED POS port
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int nu = ssGetInputPortWidth(S,0);
    for(int j=0; j<nu;j++)
        bot.addDouble(*uPtrs[j]);

    port->write(bot);
    printf("Sent message: %s\n",bot.toString().c_str());

}

//
///* Define to indicate that this S-Function has the mdlG[S]etSimState mothods */
//#define MDL_SIM_STATE
//
///* Function: mdlGetSimState =====================================================
// * Abstract:
// *
// */
//static mxArray* mdlGetSimState(SimStruct* S)
//{
//    // Retrieve C++ object from the pointers vector
//    // DoubleAdder *da = static_cast<DoubleAdder*>(ssGetPWork(S)[0]);
//    // return mxCreateDoubleScalar(da->GetPeak());
//}
///* Function: mdlGetSimState =====================================================
// * Abstract:
// *
// */
//static void mdlSetSimState(SimStruct* S, const mxArray* ma)
//{
//    // Retrieve C++ object from the pointers vector
//    // DoubleAdder *da = static_cast<DoubleAdder*>(ssGetPWork(S)[0]);
//    // da->SetPeak(mxGetPr(ma)[0]);
//}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
    // IF YOU FORGET TO DESTROY OBJECTS OR DEALLOCATE MEMORY, MATLAB WILL CRASH.
    // Retrieve and destroy C++ object
     Port *port = static_cast<Port*>(ssGetPWork(S)[0]);
     port->close();

     if(ssGetPWork(S) != NULL)
         ssSetPWorkValue(S,0,NULL);

    Network::fini();
    fprintf(stderr,"Everything was closed correctly\n");
}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
