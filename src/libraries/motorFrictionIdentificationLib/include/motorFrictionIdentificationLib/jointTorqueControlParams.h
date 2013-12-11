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

#ifndef JOINT_TORQUE_CONTROL_PARAMS
#define JOINT_TORQUE_CONTROL_PARAMS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>

using namespace paramHelp;
using namespace Eigen;
using namespace std;

namespace jointTorqueControl
{

static const int N_DOF = 25;
typedef Eigen::Matrix<double,N_DOF,1>           VectorNd;
typedef Eigen::Matrix<int,N_DOF,1>              VectorNi;

static const int SEND_COMMANDS_ACTIVE = 1;
static const int SEND_COMMANDS_NONACTIVE = 0;

// *** DEFAULT PARAMETER VALUES
static const string		DEFAULT_MODULE_NAME     = "jointTorqueControl";
static const string		DEFAULT_ROBOT_NAME      = "icubSim";                // robot name
static const string		DEFAULT_FILE_NAME       = "100poses_A";             // input params file (trajectory points)
static const int        DEFAULT_CTRL_PERIOD     = 10;                       // controller period in ms
static const int 		DEFAULT_MONITORED_JOINT = 0;
static const int        DEFAULT_SENDCMD     	= SEND_COMMANDS_NONACTIVE;  // 
static const VectorNi	DEFAULT_ACTIVE_JOINTS   = VectorNi::Constant(0); 

static const VectorNd	DEFAULT_KT				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KVP				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KVN				= VectorNd::Constant(0.0);
static const VectorNd	DEFAULT_KCP				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KCN				= VectorNd::Constant(0.0);  

static const VectorNd	DEFAULT_KI				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KP				= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_KS				= VectorNd::Constant(0.0);  

static const VectorNd	DEFAULT_TAUD			= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_TAU				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_VM				= VectorNd::Constant(0.0);

static const VectorNd	DEFAULT_VMAX			= VectorNd::Constant(0.0);
static const double		KT_MAX 		         	= 100.0;     
static const double		KVP_MAX     	     	= 100.0;    
static const double		KVN_MAX         	 	= 100.0;     
static const double		KCP_MAX          		= 100.0;    
static const double		KCN_MAX	        	  	= 100.0;    
static const double		KI_MAX  	        	= 100.0;    
static const double		KP_MAX      	    	= 100.0;    
static const double		KS_MAX          		= 100.0;     
static const double		V_MAX 	       		  	= 100.0;  
static const double		TAUD_MIN          		= 100.0;     
static const double		TAUD_MAX 	        	= 100.0;  
static const double		VM_MIN          		= -1333.0;     
static const double		VM_MAX	 	        	= 1333.0;       


// Streaming parameters


// *** IDs of all the module parameters
enum jointTorqueControlParamId { 
    PARAM_ID_MODULE_NAME,	PARAM_ID_CTRL_PERIOD,	PARAM_ID_ROBOT_NAME,		
	PARAM_ID_AJ,	PARAM_ID_KT,	PARAM_ID_KVP,	PARAM_ID_KVN,	PARAM_ID_KCP,	PARAM_ID_KCN,
    PARAM_ID_KI,	PARAM_ID_KP,	PARAM_ID_KS,	PARAM_ID_VMAX,	PARAM_ID_TAUD,	PARAM_ID_VM,	
	PARAM_ID_TAU,  PARAM_ID_SENDCMD,	PARAM_ID_V_OUT, PARAM_ID_TAU_OUT,PARAM_ID_MONITORED_JOINT,PARAM_ID_SIZE,
};


// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamProxyInterface *const jointTorqueControlParamDescr[]  = 
{ 
//	              	        NAME                ID                          SIZE   		BOUNDS          		                        I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
new ParamProxyBasic<string>("name",           	PARAM_ID_MODULE_NAME,       1,			                                                PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
new ParamProxyBasic<int>(   "period",         	PARAM_ID_CTRL_PERIOD,       1,			ParamBilatBounds<int>(1, 1000),                 PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"), 
new ParamProxyBasic<string>("robot",			PARAM_ID_ROBOT_NAME,        1,			                                                PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
// ************************************************ RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<int>(   "activeJoints",		PARAM_ID_AJ,				N_DOF,		ParamBilatBounds<int>(0, 1),			        PARAM_IN_OUT,       DEFAULT_ACTIVE_JOINTS.data(),	"Vector of nDOF integers representing the joints to control  (1: active, 0: inactive)"), 
new ParamProxyBasic<double>("kt",          	    PARAM_ID_KT,				N_DOF,		ParamBilatBounds<double>(0.0, KT_MAX),		    PARAM_IN_OUT,       DEFAULT_KT.data(),				"Vector of nDOF floats ( see Eq. (1) )"), 
new ParamProxyBasic<double>("kvp",          	PARAM_ID_KVP,				N_DOF,		ParamBilatBounds<double>(0.0, KVP_MAX),		    PARAM_IN_OUT,       DEFAULT_KVP.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
new ParamProxyBasic<double>("kvn",          	PARAM_ID_KVN,				N_DOF,		ParamBilatBounds<double>(0.0, KVN_MAX),		    PARAM_IN_OUT,       DEFAULT_KVN.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
new ParamProxyBasic<double>("kcp",		        PARAM_ID_KCP,				N_DOF,		ParamBilatBounds<double>(0.0, KCP_MAX),		    PARAM_IN_OUT,       DEFAULT_KCP.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
new ParamProxyBasic<double>("kcn",     	        PARAM_ID_KCN,			    N_DOF,		ParamBilatBounds<double>(0.0, KCN_MAX),		    PARAM_IN_OUT,       DEFAULT_KCN.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
new ParamProxyBasic<double>("ki",          	    PARAM_ID_KI,				N_DOF,		ParamBilatBounds<double>(0.0, KI_MAX),		    PARAM_IN_OUT,       DEFAULT_KI.data(),				"Vector of nDOF floats representing the position gains ( see Eq. (x) )"), 
new ParamProxyBasic<double>("kp",          	    PARAM_ID_KP,				N_DOF,		ParamBilatBounds<double>(0.0, KP_MAX),		    PARAM_IN_OUT,       DEFAULT_KP.data(),				"Vector of nDOF floats representing the integral gains ( see Eq. (x) )"), 
new ParamProxyBasic<double>("ks",          	    PARAM_ID_KS,				N_DOF,		ParamBilatBounds<double>(0.0, KS_MAX),		    PARAM_IN_OUT,       DEFAULT_KS.data(),				"Vector of nDOF floats representing the steepnes       ( see Eq. (x) )"), 
new ParamProxyBasic<double>("Vmax",          	PARAM_ID_VMAX,				N_DOF,		ParamBilatBounds<double>(0.0, V_MAX),		    PARAM_IN_OUT,       DEFAULT_VMAX.data(),			"Vector of nDOF positive floats representing the tensions' bounds (|Vm| < Vmax"), 
new ParamProxyBasic<int>("sendCommands",   	 	PARAM_ID_SENDCMD,			1,			ParamBilatBounds<int>(0, 1),				    PARAM_IN_OUT,       &DEFAULT_SENDCMD,				"If equal to 1, send commands for torque control"), 
new ParamProxyBasic<int>("monitoredJoint",     	PARAM_ID_MONITORED_JOINT,	1,			ParamBilatBounds<int>(0, N_DOF),				PARAM_IN_OUT,       &DEFAULT_MONITORED_JOINT,				"Select which joint is monitored"), 
// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("tauD",        	    PARAM_ID_TAUD,         		N_DOF,      ParamBilatBounds<double>(TAUD_MIN, TAUD_MAX),   PARAM_IN_STREAM,    DEFAULT_TAUD.data(),			"Desired torques"),

// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("Vm",				PARAM_ID_VM,				N_DOF,		ParamBilatBounds<double>(VM_MIN, VM_MAX),		PARAM_OUT_STREAM,   DEFAULT_VM.data(),				"Vector of nDOF floats representing the tensions"),
new ParamProxyBasic<double>("tau",          	PARAM_ID_TAU,         		N_DOF,      ParamBilatBounds<double>(TAUD_MIN, TAUD_MAX),	PARAM_OUT_STREAM,   DEFAULT_TAU.data(),        		"Torques"),
// ************************************************* MONITOR  PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("V_out",            PARAM_ID_V_OUT,             1,          ParamBilatBounds<double>(VM_MIN, VM_MAX),       PARAM_MONITOR,      0,                              "PWM of the currently controlled motor"),
new ParamProxyBasic<double>("Tau_out",          PARAM_ID_TAU_OUT,           1,          ParamBilatBounds<double>(-150.0, 150.0),        PARAM_MONITOR,      0,                              "Output torque of the currently controlled motor")	
};

// *** IDs of all the module command
enum jointTorqueControlCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT, 
    COMMAND_ID_SIZE
};
// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription jointTorqueControlCommandDescr[]  = 
{ 
//  	                NAME            ID                          DESCRIPTION
	CommandDescription("start",         COMMAND_ID_START,           "Start the control actions"), 
	CommandDescription("stop",          COMMAND_ID_STOP,            "Stop the control actions"), 
	CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"), 
	CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the control actions and quit the module"), 
};

}

#endif
