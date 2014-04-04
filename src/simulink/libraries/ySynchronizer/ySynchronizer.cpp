#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  ySynchronizer

#define RATE(S) ssGetSFcnParam(S,0)
#define SAMPLING_TIME 1

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*
 *  Include the standard ANSI C header for handling time functions:
 *  ---------------------------------------------------------------
 */
#include <time.h>
#include <yarp/os/Time.h>

static void mdlInitializeSizes(SimStruct *S)
{

    ssSetNumSFcnParams(S, 1);  /* 1 Input parameters: Time Scale Factor, e.g. 1 for real time simulation */

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;

    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 1);	    		// Robot joint angular positions in radians
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 0, 0);
    ssSetOutputPortDataType(S, 1, 0);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 3);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOptions(S, 0);
}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    ssSetRWorkValue(S,0,ssGetTStart(S));    	// Get initial simulation time.
    ssSetRWorkValue(S,1,yarp::os::Time::now());  // Initial real time
    ssSetRWorkValue(S,2,-1);		    	// Real initial time of Matlab step.
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Store real time at the beginning of each simulation step
    double             t_c = yarp::os::Time::now();	                // Real time in current simulation step
    real_T             t_system;
    t_system = (real_T)clock()/CLOCKS_PER_SEC;
//     time_T             t_SimTime = ssGetT(S);				// Useful to retrieve the simulation rate
//     real_T             t_previousSimTime = ssGetRWorkValue(S,0);		// Useful to retrieve the simulation rate
    
    const real_T      *sample_time = mxGetPr(RATE(S));
    
//     Retrieve actual sampling time. For now assume the desired time scale factor is 1, i.e. Time ratio is 1:1
//     real_T             fund_sample_time = t_SimTime - t_previousSimTime;  // 
    
    double             t_realSamplingTime = sample_time[0];

    double             t_prev = static_cast<double>(ssGetRWorkValue(S,2));
    
    // if it's the first simulation step ...
    if(t_prev<0) {
	// ... then wait for the desired period ... 
	yarp::os::Time::delay(static_cast<double>(sample_time[0]));  // First delay before execution of the other blocks
    }
    else {
        // ... If it's not the first simulation step, then delay the simulation for 2Ts - (tc - tprev)
        yarp::os::Time::delay(2*t_realSamplingTime - (t_c - t_prev));
    }
    
    // and store the real time at the beginning of this simulation step ...
    ssSetRWorkValue(S,2,t_c);
    
    // store also the matlab time at the beginning of this simulation step
//     ssSetRWorkValue(S, 0, t_SimTime);
    
    real_T *pY0 = (real_T*)ssGetOutputPortSignal(S,0);
    for(int_T j=0; j<ssGetOutputPortWidth(S,0); j++)
//     pY0[j] = yarp::os::Time::now() - ssGetRWorkValue(S,1);
        pY0[j] = t_c - ssGetRWorkValue(S,1);
	
    real_T *pY1 = (real_T*)ssGetOutputPortSignal(S,1);
    for(int_T j=0; j<ssGetOutputPortWidth(S,1); j++)
      pY1[j] = t_system;

}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

/*
 *  Required S-function trailer:
 *  ----------------------------
 */
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
