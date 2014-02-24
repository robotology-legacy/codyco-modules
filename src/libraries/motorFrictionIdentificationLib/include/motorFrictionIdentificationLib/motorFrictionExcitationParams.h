/*
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#ifndef __MOTOR_FRICTION_EXCITATION_PARAMS
#define __MOTOR_FRICTION_EXCITATION_PARAMS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>

using namespace paramHelp;
using namespace Eigen;
using namespace std;

static const int       ICUB_DOFS = 25;    // number of (the main) degrees of freedom of iCub

typedef Eigen::Matrix<double,ICUB_DOFS,1>          VectorNd;

namespace motorFrictionExcitation
{
    
    ///< specify whether or not the commands are sent to the motors
    enum MFE_MotorCommandMode
    {
        DO_NOT_SEND_COMMANDS_TO_MOTORS = 0,
        SEND_COMMANDS_TO_MOTORS = 1
    };
    
    // *** DEFAULT PARAMETER VALUES
    static const string                 DEFAULT_MODULE_NAME     = "motorFrictionExcitation";        ///< name of the module
    static const int                    DEFAULT_CTRL_PERIOD     = 10;                               ///< controller period in ms
    static const string                 DEFAULT_ROBOT_NAME      = "icubSim";                        ///< robot name
    static const string                 DEFAULT_MFI_NAME        = "motorFrictionIdentification";    ///< name of the motorFrictionIdentification module
    static const VectorNd               DEFAULT_Q_MAX           = VectorNd::Constant(150.0);
    static const VectorNd               DEFAULT_Q_MIN           = VectorNd::Constant(-150.0);
    static const int                    DEFAULT_SEND_COMMANDS   = SEND_COMMANDS_TO_MOTORS;
    static const double                 DEFAULT_JNT_LIM_MIN_DIST = 10.0;
    static const double                 DEFAULT_POS_INT_GAIN    = 1e-5;
    
    // *** IDs of all the module command
    enum MotorFrictionExcitationCommandId {
        COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_RESET,   COMMAND_ID_HELP,    COMMAND_ID_QUIT,
        COMMAND_ID_SIZE
    };
    
    // ******************************************************************************************************************************
    // ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
    // ******************************************************************************************************************************
    const CommandDescription motorFrictionExcitationCommandDescr[]  =
    {
        //                  NAME            ID                          DESCRIPTION
        CommandDescription("start",         COMMAND_ID_START,           "Start the controller"),
        CommandDescription("stop",          COMMAND_ID_STOP,            "Stop the controller"),
        CommandDescription("reset",         COMMAND_ID_RESET,           "Reset the excitation counter so that the excitation starts back form the beginning"),
        CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"),
        CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the controller and quit the module"),
    };
    
    ///< IDs of all the module parameters
    enum MotorFrictionExcitationParamId
    {
        PARAM_ID_MODULE_NAME,       PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME,
        PARAM_ID_MOTOR_FRICTION_IDENTIFICATION_NAME,            PARAM_ID_SEND_COMMANDS,
        PARAM_ID_Q_MAX,             PARAM_ID_Q_MIN,
        // Monitor parameters
        PARAM_ID_Q,                 PARAM_ID_PWM_DES,
        PARAM_ID_KT_STD_DEV_THR,    PARAM_ID_FRIC_STD_DEV_THR,
        PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
    };
    
    ///< Ids of the subparameters conposing the struct parameter FreeMotionExcitation
    enum FreeMotionExcitationParamId
    {
        PARAM_ID_JOINT_ID,          PARAM_ID_INIT_Q,            PARAM_ID_A,
        PARAM_ID_A0,                PARAM_ID_W,                 PARAM_ID_JOINT_LIM_THR,
        PARAM_ID_FRIC_PAR_COV_THR,  PARAM_ID_POS_INT_GAIN,
        FREE_MOTION_EXCITATION_PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
    };
    
    // ******************************************************************************************************************************
    // ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
    // ******************************************************************************************************************************
    
    class ContactExcitation
    {
    public:
        ArrayXi jointId;
        ArrayXd initialJointConfiguration;
        ArrayXd paramCovarThresh;
        
        //ContactExcitation();
        bool set(const yarp::os::Bottle &value, yarp::os::Bottle &reply);
        bool setSubparam(const std::string &name, const yarp::os::Bottle &value, yarp::os::Bottle &reply);
        std::string toString() const;
    };
    
    class ContactExcitationList : public std::vector<ContactExcitation>
    {
    public:
        bool readFromConfigFile(yarp::os::ResourceFinder &rf, yarp::os::Bottle &reply);
        std::string toString() const;
    };
    
    class FreeMotionExcitation
    {
    public:
        ArrayXi jointId;
        ArrayXd initialJointConfiguration;
        ArrayXd a;
        ArrayXd a0;
        ArrayXd w;
        ArrayXd jointLimitThresh;
        ArrayXd fricParamCovarThresh;
        ArrayXd ki;
        
        //ContactExcitation();
        bool set(const yarp::os::Bottle &value, yarp::os::Bottle &reply);
        bool setSubparam(const std::string &name, const yarp::os::Bottle &value, yarp::os::Bottle &reply);
        std::string toString() const;
    };
    
    class FreeMotionExcitationList : public std::vector<FreeMotionExcitation>
    {
    public:
        bool readFromConfigFile(yarp::os::ResourceFinder &rf, yarp::os::Bottle &reply);
        std::string toString() const;
    };
    
    const ParamProxyInterface *const motorFrictionExcitationParamDescr[PARAM_ID_SIZE]  =
    {
        // ************************************************* STRUCT PARAMETERS ****************************************************************************************************************************************************************************************************************************************
        //                                        NAME                       ID                          SIZE                I/O ACCESS         DEFAULT VALUE               DESCRIPTION
        // ************************************************* SIMPLE PARAMETERS ****************************************************************************************************************************************************************************************************************************************
        //                          NAME                    ID                          SIZE                BOUNDS                                      I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
        new ParamProxyBasic<string>("name",                 PARAM_ID_MODULE_NAME,       1,                                                              PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"),
        new ParamProxyBasic<int>(   "period",               PARAM_ID_CTRL_PERIOD,       1,                  ParamBilatBounds<int>(1,1000),              PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"),
        new ParamProxyBasic<string>("robot",                PARAM_ID_ROBOT_NAME,        1,                                                              PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"),
        new ParamProxyBasic<string>("motor friction identification name", PARAM_ID_MOTOR_FRICTION_IDENTIFICATION_NAME, 1,                               PARAM_CONFIG,       &DEFAULT_MFI_NAME,              "Name of the instance of the motorFrictionIdentification module"),
        // ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
        new ParamProxyBasic<double>("q max",                PARAM_ID_Q_MAX,             ICUB_DOFS,          ParamBilatBounds<double>(-360.0,360.0),     PARAM_IN_OUT,       DEFAULT_Q_MAX.data(),           "Joint upper bounds"),
        new ParamProxyBasic<double>("q min",                PARAM_ID_Q_MIN,             ICUB_DOFS,          ParamBilatBounds<double>(-360.0,360.0),     PARAM_IN_OUT,       DEFAULT_Q_MIN.data(),           "Joint lower bounds"),
        new ParamProxyBasic<int>(   "send commands",        PARAM_ID_SEND_COMMANDS,     1,                  ParamBilatBounds<int>(0, 1),                PARAM_IN_OUT,       &DEFAULT_SEND_COMMANDS,         "Specify whether to send commands to the motors"),
        // ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
        // ************************************************* STREAMING MONITOR PARAMETERS ****************************************************************************************************************************************************************************************************************************
        new ParamProxyBasic<double>("q",                    PARAM_ID_Q,                 1,                  ParamBilatBounds<double>(-150.0, 150.0),    PARAM_MONITOR,      0,                              "Joint angle of the currently controlled motor"),
        new ParamProxyBasic<double>("pwmDes",               PARAM_ID_PWM_DES,           1,                  ParamBilatBounds<double>(-1333.0, 1333.0),  PARAM_MONITOR,      0,                              "Current desired pwm sent to the motors"),
        new ParamProxyBasic<double>("kt std dev thr",       PARAM_ID_KT_STD_DEV_THR,    1,                  ParamBilatBounds<double>(0.0, 1.0),         PARAM_MONITOR,      0,                              "Threshold of the standard deviation of the parameter kt of the currently excited motor"),
        new ParamProxyBasic<double>("fric std dev thr",     PARAM_ID_FRIC_STD_DEV_THR,  1,                  ParamBilatBounds<double>(0.0, 1.0),         PARAM_MONITOR,      0,                              "Threshold of the standard deviation of the friction parameters of the currently excited motor")
    };
    
}   // end namespace 

#endif
