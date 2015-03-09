/**
 * Copyright (C) 2014 CoDyCo
 * @author: Francesco Romano
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

#ifndef PARAMHELPERCONFIG_H
#define PARAMHELPERCONFIG_H

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>
#include <limits>
#include "config.h"

namespace codyco {
    namespace torquebalancing {
        
        typedef enum {
            //Initialization parameters (for module) //0
            TorqueBalancingModuleParameterModuleName,
            TorqueBalancingModuleParameterRobotName,
            TorqueBalancingModuleParameterControllerPeriod,
            TorqueBalancingModuleParameterModulePeriod,
            TorqueBalancingModuleParameterForcesSmoothingDuration,
            TorqueBalancingModuleParameterJointsSmoothingDuration,
            //RPC parameters  //6
            TorqueBalancingModuleParameterCurrentState,
            //PIDs // 7
            //COM
            TorqueBalancingModuleParameterCOMIntegralLimit,
            TorqueBalancingModuleParameterCOMProportionalGain,
            TorqueBalancingModuleParameterCOMDerivativeGain,
            TorqueBalancingModuleParameterCOMIntegralGain,
            //Hands Position (not used for now)
            TorqueBalancingModuleParameterHandsPositionIntegralLimit,
            TorqueBalancingModuleParameterHandsPositionProportionalGain,
            TorqueBalancingModuleParameterHandsPositionDerivativeGain,
            TorqueBalancingModuleParameterHandsPositionIntegralGain,
            //Hands Force (not used for now)
            TorqueBalancingModuleParameterHandsForceIntegralLimit,
            TorqueBalancingModuleParameterHandsForceProportionalGain,
            TorqueBalancingModuleParameterHandsForceDerivativeGain,
            TorqueBalancingModuleParameterHandsForceIntegralGain,
            //Centroidal gain
            TorqueBalancingModuleParameterCentroidalGain,
            TorqueBalancingModuleParameterImpedanceGain,
            //Additional parameters
            TorqueBalancingModuleParameterTorqueSaturation,
            //MonitoredVariables
            TorqueBalancingModuleParameterMonitorDesiredCOMAcceleration,
            TorqueBalancingModuleParameterMonitorCOMError,
            TorqueBalancingModuleParameterMonitorCOMIntegralError,
            TorqueBalancingModuleParameterMonitorFeetForces,
            TorqueBalancingModuleParameterMonitorOutputTorques,
            
        } TorqueBalancingModuleParameter;
        
        static const int TorqueBalancingModuleParameterSize = 27;
        
        static const std::string defaultModuleName = "torqueBalancing";
        static const std::string defaultRobotName = "icub";
        static const int defaultModuleState = 1;
        static const int defaultControllerPeriod = 10; //ms
        static const double defaultModulePeriod = 1.0; //s
        static const double defaultSmoothingDuration = 5.0; //s
        static const double defaultIntegralLimit = std::numeric_limits<double>::max(); //no limit
        static const Eigen::VectorXd defaultCOMGains = Eigen::VectorXd(3).setZero();
        static const Eigen::VectorXd defaultHandsPositionGains = Eigen::VectorXd::Zero(14);
        static const Eigen::VectorXd defaultHandsForceGains = Eigen::VectorXd::Zero(12);
        static const double defaultCentroidalGain = 0;
//        static const Eigen::VectorXd defaultImpedanceGains = Eigen::VectorXd::Zero(tempactuatedDOFs);
//        static const Eigen::VectorXd defaultSaturation = Eigen::VectorXd::Constant(tempactuatedDOFs, 100);
        
        paramHelp::ParamProxyInterface *const TorqueBalancingModuleParameterDescriptions[]  =
        {
            //config parameters
            new paramHelp::ParamProxyBasic<std::string>("name", TorqueBalancingModuleParameterModuleName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultModuleName, "Name of the instance of the module"),
            new paramHelp::ParamProxyBasic<std::string>("robot", TorqueBalancingModuleParameterRobotName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultRobotName, "Name of the robot"),
            new paramHelp::ParamProxyBasic<int>("period", TorqueBalancingModuleParameterControllerPeriod, 1, paramHelp::ParamConstraint<int>(), paramHelp::PARAM_CONFIG, &defaultControllerPeriod, "Period of the controller"),
            new paramHelp::ParamProxyBasic<double>("forcesSmoothing", TorqueBalancingModuleParameterForcesSmoothingDuration, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_CONFIG, &defaultSmoothingDuration, "Duration of the trajectory smoother for the contact (hands) forces"),
            new paramHelp::ParamProxyBasic<double>("jointsSmoothing", TorqueBalancingModuleParameterJointsSmoothingDuration, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_CONFIG, &defaultSmoothingDuration, "Duration of the trajectory smoother for the joints impedance"),
            //RPC parameters
            new paramHelp::ParamProxyBasic<double>("modulePeriod", TorqueBalancingModuleParameterModulePeriod, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultModulePeriod, "Period of the module. Used to update monitored variables."),
            new paramHelp::ParamProxyBasic<int>("state", TorqueBalancingModuleParameterCurrentState, 1, paramHelp::ParamConstraint<int>(), paramHelp::PARAM_IN_OUT, &defaultModuleState, "State of the module"),
            //COM
            new paramHelp::ParamProxyBasic<double>("comIntLimit", TorqueBalancingModuleParameterCOMIntegralLimit, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultIntegralLimit, "Integral limit on COM PID"),
            new paramHelp::ParamProxyBasic<double>("comKp", TorqueBalancingModuleParameterCOMProportionalGain, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultCOMGains.data(), "Proportional gains of COM PID"),
            new paramHelp::ParamProxyBasic<double>("comKd", TorqueBalancingModuleParameterCOMDerivativeGain, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultCOMGains.data(), "Derivative gains of COM PID"),
            new paramHelp::ParamProxyBasic<double>("comKi", TorqueBalancingModuleParameterCOMIntegralGain, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultCOMGains.data(), "Integral gains of COM PID"),
            //Hands position
            new paramHelp::ParamProxyBasic<double>("handPosIntLimit", TorqueBalancingModuleParameterHandsPositionIntegralLimit, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultIntegralLimit, "Integral limit on Hands position PID"),
            new paramHelp::ParamProxyBasic<double>("handPosKp", TorqueBalancingModuleParameterHandsPositionProportionalGain, 14, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsPositionGains.data(), "Proportional gains of Hands position PID"),
            new paramHelp::ParamProxyBasic<double>("handPosKd", TorqueBalancingModuleParameterHandsPositionDerivativeGain, 14, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsPositionGains.data(), "Derivative gains of Hands position PID"),
            new paramHelp::ParamProxyBasic<double>("handPosKi", TorqueBalancingModuleParameterHandsPositionIntegralGain, 14, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsPositionGains.data(), "Integral gains of Hands position PID"),
            //Hands force
            new paramHelp::ParamProxyBasic<double>("handForceIntLimit", TorqueBalancingModuleParameterHandsForceIntegralLimit, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultIntegralLimit, "Integral limit on Hands forces PID"),
            new paramHelp::ParamProxyBasic<double>("handForceKp", TorqueBalancingModuleParameterHandsForceProportionalGain, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsForceGains.data(), "Proportional gains of Hands forces PID"),
            new paramHelp::ParamProxyBasic<double>("handForceKd", TorqueBalancingModuleParameterHandsForceDerivativeGain, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsForceGains.data(), "Derivative gains of Hands forces PID"),
            new paramHelp::ParamProxyBasic<double>("handForceKi", TorqueBalancingModuleParameterHandsForceIntegralGain, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsForceGains.data(), "Integral gains of Hands forces PID"),
            //gains
            new paramHelp::ParamProxyBasic<double>("kw", TorqueBalancingModuleParameterCentroidalGain, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultCentroidalGain, "Gain for the centroidal-based controller"),
            //Monitored variables
            new paramHelp::ParamProxyBasic<double>("desCOMAcc", TorqueBalancingModuleParameterMonitorDesiredCOMAcceleration, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Desired COM acceleration computed by the controller"),
            new paramHelp::ParamProxyBasic<double>("comError", TorqueBalancingModuleParameterMonitorCOMError, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Instantaneous COM error"),
            new paramHelp::ParamProxyBasic<double>("comIntError", TorqueBalancingModuleParameterMonitorCOMIntegralError, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Integral of COM error"),
            new paramHelp::ParamProxyBasic<double>("feetForces", TorqueBalancingModuleParameterMonitorFeetForces, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Desired feet forces"),
            //Param size Free
            new paramHelp::ParamProxyBasic<double>("kImp", TorqueBalancingModuleParameterImpedanceGain, paramHelp::PARAM_SIZE_FREE, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, 0, "Gain for the impedance control task."),
            //Additional parameters
            new paramHelp::ParamProxyBasic<double>("tsat", TorqueBalancingModuleParameterTorqueSaturation, paramHelp::PARAM_SIZE_FREE, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, 0, "Torque saturations (positives)"),
            new paramHelp::ParamProxyBasic<double>("outTorques", TorqueBalancingModuleParameterMonitorOutputTorques, paramHelp::PARAM_SIZE_FREE, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Output torques of the controller"),
            
        };
        
        
        typedef enum {
            TorqueBalancingModuleCommandStart,
            TorqueBalancingModuleCommandStop,
            TorqueBalancingModuleCommandQuit,
            TorqueBalancingModuleCommandHelp
            
        } TorqueBalancingModuleCommand;
        
        static const int TorqueBalancingModuleCommandSize = 4;
        
        const paramHelp::CommandDescription TorqueBalancingModuleCommandDescriptions[]  =
        {
            paramHelp::CommandDescription("start", TorqueBalancingModuleCommandStart, "Start the control actions"),
            paramHelp::CommandDescription("stop", TorqueBalancingModuleCommandStop, "Stop the control actions"),
            paramHelp::CommandDescription("help", TorqueBalancingModuleCommandHelp, "Get instructions about how to communicate with this module"),
            paramHelp::CommandDescription("quit", TorqueBalancingModuleCommandQuit, "Stop the control actions and quit the module"),
        };
    }
}

#endif /* end of include guard: PARAMHELPERCONFIG_H */
