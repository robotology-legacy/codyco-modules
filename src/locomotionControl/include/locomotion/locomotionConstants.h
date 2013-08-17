
#include <paramHelp/paramHelp.h>

using namespace paramHelp;

// Id of all the module parameters
enum LocomotionParamId { 
    PARAM_ID_KP_COM,            PARAM_ID_KP_FOOT,           PARAM_ID_KP_POSTURE, 
    PARAM_ID_TRAJ_TIME_COM,     PARAM_ID_TRAJ_TIME_FOOT,    PARAM_ID_TRAJ_TIME_POSTURE,
    PARAM_ID_ACTIVE_JOINTS,     PARAM_ID_SUPPORT_PHASE,     PARAM_ID_PINV_DAMP,
    PARAM_ID_SIZE
};

const double    KP_MAX = 100.0;    // max value of proportional gains
const int       ICUB_DOFS = 23;    // number of degrees of freedom of iCub

// Description of all the module parameters
const ParamDescription locomotionParamDescr[]  = 
{ 
    ParamDescription("kp_com",          PARAM_ID_KP_COM,            PARAM_TYPE_FLOAT,   ParamSize(2),           ParamBounds(0.0, KP_MAX)), 
    ParamDescription("kp_foot",         PARAM_ID_KP_FOOT,           PARAM_TYPE_FLOAT,   ParamSize(6),           ParamBounds(0.0, KP_MAX)), 
    ParamDescription("kp_posture",      PARAM_ID_KP_POSTURE,        PARAM_TYPE_FLOAT,   ParamSize(ICUB_DOFS),   ParamBounds(0.0, KP_MAX)), 
    ParamDescription("tt_com",          PARAM_ID_TRAJ_TIME_COM,     PARAM_TYPE_FLOAT,   ParamSize(1),           ParamBounds(0.1, PARAM_BOUND_INF)), 
    ParamDescription("tt_foot",         PARAM_ID_TRAJ_TIME_FOOT,    PARAM_TYPE_FLOAT,   ParamSize(1),           ParamBounds(0.1, PARAM_BOUND_INF)), 
    ParamDescription("tt_posture",      PARAM_ID_TRAJ_TIME_POSTURE, PARAM_TYPE_FLOAT,   ParamSize(1),           ParamBounds(0.1, PARAM_BOUND_INF)), 
    ParamDescription("active joints",   PARAM_ID_ACTIVE_JOINTS,     PARAM_TYPE_BOOL,    ParamSize(ICUB_DOFS)), 
    ParamDescription("support phase",   PARAM_ID_SUPPORT_PHASE,     PARAM_TYPE_INT,     ParamSize(1),           ParamBounds(0.0, 2.0)), 
    ParamDescription("pinv damp",       PARAM_ID_PINV_DAMP,         PARAM_TYPE_FLOAT,   ParamSize(1),           ParamBounds(1e-8, 1.0))
};


const int   DEFAULT_CTRL_PERIOD = 10;           // controller period in ms
const char* DEFAULT_ROBOT_NAME  = "icubSim";    // robot name