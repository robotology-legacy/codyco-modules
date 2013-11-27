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

#ifndef LOCOMOTION_PLAN_CONSTANTS
#define LOCOMOTION_PLAN_CONSTANTS

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

typedef Eigen::Matrix<double,N_DOF,1>          VectorNd;

// *** DEFAULT PARAMETER VALUES
static const string		DEFAULT_MODULE_NAME     = "jointTorqueControl";
static const string		DEFAULT_ROBOT_NAME      = "icubSim";            // robot name
static const string		DEFAULT_FILE_NAME       = "100poses_A";         // input params file (trajectory points)
static const int        DEFAULT_CTRL_PERIOD     = 10;                   // controller period in ms
static const VectorNd	DEFAULT_ACTIVE_JOINTS   = VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KT				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KVP				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KVN				= VectorNd::Constant(0.0);
static const VectorNd	DEFAULT_KCP				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KCN				= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_KI				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_KP				= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_KS				= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_VMAX			= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_TAUD			= VectorNd::Constant(0.0);  
static const VectorNd	DEFAULT_TAU				= VectorNd::Constant(0.0); 
static const VectorNd	DEFAULT_VM				= VectorNd::Constant(0.0);  
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
static const double		VM_MIN          		= 100.0;     
static const double		VM_MAX	 	        	= 100.0;       


// Streaming parameters


// *** IDs of all the module parameters
enum jointTorqueControlParamId { 
    PARAM_ID_MODULE_NAME,	PARAM_ID_CTRL_PERIOD,	PARAM_ID_ROBOT_NAME,		
	PARAM_ID_AJ,	PARAM_ID_KT,	PARAM_ID_KVP,	PARAM_ID_KVN,	PARAM_ID_KCP,	PARAM_ID_KCN,
    PARAM_ID_KI,	PARAM_ID_KP,	PARAM_ID_KS,	PARAM_ID_VMAX,	PARAM_ID_TAUD,	PARAM_ID_VM,	
	PARAM_ID_TAU,	PARAM_ID_SIZE
};


// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamDescription jointTorqueControlParamDescr[]  = 
{ 
//	              	      NAME                   ID                      TYPE          SIZE   			BOUNDS          		        I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
	ParamDescription("name",           	PARAM_ID_MODULE_NAME,       PARAM_DATA_STRING,  1,			PARAM_BOUNDS_INF,                   PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
	ParamDescription("period",         	PARAM_ID_CTRL_PERIOD,       PARAM_DATA_INT,     1,			ParamBounds(1, 1000),               PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"), 
	ParamDescription("robot",			PARAM_ID_ROBOT_NAME,        PARAM_DATA_STRING,  1,			PARAM_BOUNDS_INF,                   PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
// ************************************************ RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
	ParamDescription("Active joints",	PARAM_ID_AJ,			PARAM_DATA_INT,			N_DOF,		ParamBounds(0.0, 1),				PARAM_IN_OUT,       DEFAULT_ACTIVE_JOINTS.data(),	"Vector of nDOF integers representing the joints to control  (1: active, 0: inactive)"), 
	ParamDescription("kt",          	PARAM_ID_KT,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KT_MAX),			PARAM_IN_OUT,       DEFAULT_KT.data(),				"Vector of nDOF floats ( see Eq. (1) )"), 
	ParamDescription("kvp",          	PARAM_ID_KVP,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KVP_MAX),			PARAM_IN_OUT,       DEFAULT_KVP.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
	ParamDescription("kvn",          	PARAM_ID_KVN,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KVN_MAX),			PARAM_IN_OUT,       DEFAULT_KVN.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
	ParamDescription("kcp",		        PARAM_ID_KCP,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KCP_MAX),			PARAM_IN_OUT,       DEFAULT_KCP.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
	ParamDescription("kcn",     	    PARAM_ID_KCN,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KCN_MAX),			PARAM_IN_OUT,       DEFAULT_KCN.data(),				"Vector of nDOF floats ( see Eq. (2) )"), 
	ParamDescription("ki",          	PARAM_ID_KI,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KI_MAX),			PARAM_IN_OUT,       DEFAULT_KI.data(),				"Vector of nDOF floats representing the position gains ( see Eq. (x) )"), 
	ParamDescription("kp",          	PARAM_ID_KP,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KP_MAX),			PARAM_IN_OUT,       DEFAULT_KP.data(),				"Vector of nDOF floats representing the integral gains ( see Eq. (x) )"), 
	ParamDescription("ks",          	PARAM_ID_KS,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, KS_MAX),			PARAM_IN_OUT,       DEFAULT_KS.data(),				"Vector of nDOF floats representing the steepnes       ( see Eq. (x) )"), 
	ParamDescription("Vmax",          	PARAM_ID_VMAX,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(0.0, V_MAX),			PARAM_IN_OUT,       DEFAULT_VMAX.data(),			"Vector of nDOF positive floats representing the tensions' bounds (|Vm| < Vmax"), 

// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
	ParamDescription("tauD",        	PARAM_ID_TAUD,         	PARAM_DATA_FLOAT,		N_DOF,      ParamBounds(TAUD_MIN, TAUD_MAX),	PARAM_IN_STREAM,    DEFAULT_TAUD.data(),			"Desired torques"),

// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
	ParamDescription("Vm",				PARAM_ID_VM,			PARAM_DATA_FLOAT,		N_DOF,		ParamBounds(VM_MIN, VM_MAX),		PARAM_OUT_STREAM,   DEFAULT_VM.data(),				"Vector of nDOF floats representing the tensions"),
	ParamDescription("tau",          	PARAM_ID_TAU,         	PARAM_DATA_FLOAT,		N_DOF,      ParamBounds(TAUD_MIN, TAUD_MAX),	PARAM_OUT_STREAM,   DEFAULT_TAU.data(),        		"Torques"), 
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
