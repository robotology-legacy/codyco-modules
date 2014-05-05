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

namespace codyco {
    namespace torquebalancing {
        
        typedef enum {
            //Initialization parameters (for module)
            TorqueBalancingModuleParameterModuleName,
            TorqueBalancingModuleParameterRobotName,
            TorqueBalancingModuleParameterPeriod,
            //RPC parameters
            //PIDs
            //COM
            TorqueBalancingModuleParameterCOMIntegralLimit,
            TorqueBalancingModuleParameterCOMProportionalGain,
            TorqueBalancingModuleParameterCOMDerivativeGain,
            TorqueBalancingModuleParameterCOMIntegralGain,
            //Hands Position
            TorqueBalancingModuleParameterHandsPositionIntegralLimit,
            TorqueBalancingModuleParameterHandsPositionProportionalGain,
            TorqueBalancingModuleParameterHandsPositionDerivativeGain,
            TorqueBalancingModuleParameterHandsPositionIntegralGain,
            //Hands Force
            TorqueBalancingModuleParameterHandsForceIntegralLimit,
            TorqueBalancingModuleParameterHandsForceProportionalGain,
            TorqueBalancingModuleParameterHandsForceDerivativeGain,
            TorqueBalancingModuleParameterHandsForceIntegralGain,
            //Centroidal gain
            TorqueBalancingModuleParameterCentroidalGain
            
        } TorqueBalancingModuleParameter;
        
        static const int TorqueBalancingModuleParameterSize = 16;
        
        
        static const std::string defaultModuleName = "torqueBalancing";
        static const std::string defaultRobotName = "icub";
        static const int defaultControllerPeriod = 10;
        static const double defaultIntegralLimit = std::numeric_limits<double>::max(); //no limit
        static const Eigen::VectorXd defaultCOMGains = Eigen::VectorXd(3).setZero();
        static const Eigen::VectorXd defaultHandsPositionGains = Eigen::VectorXd(14).setZero();
        static const Eigen::VectorXd defaultHandsForceGains = Eigen::VectorXd(12).setZero();
        static const double defaultCentroidalGain = 0;
        
        
        const paramHelp::ParamProxyInterface *const TorqueBalancingModuleParameterDescriptions[]  =
        {
            //config parameters
            new paramHelp::ParamProxyBasic<std::string>("name", TorqueBalancingModuleParameterModuleName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultModuleName, "Name of the instance of the module"),
            new paramHelp::ParamProxyBasic<std::string>("robot", TorqueBalancingModuleParameterRobotName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultRobotName, "Name of the robot"),
            new paramHelp::ParamProxyBasic<int>("period", TorqueBalancingModuleParameterPeriod, 1, paramHelp::ParamConstraint<int>(), paramHelp::PARAM_CONFIG, &defaultControllerPeriod, "Period of the controller"),
            //RPC parameters
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
            new paramHelp::ParamProxyBasic<double>("comIntLimit", TorqueBalancingModuleParameterHandsForceIntegralLimit, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultIntegralLimit, "Integral limit on Hands forces PID"),
            new paramHelp::ParamProxyBasic<double>("comKp", TorqueBalancingModuleParameterHandsForceProportionalGain, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsForceGains.data(), "Proportional gains of Hands forces PID"),
            new paramHelp::ParamProxyBasic<double>("comKd", TorqueBalancingModuleParameterHandsForceDerivativeGain, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsForceGains.data(), "Derivative gains of Hands forces PID"),
            new paramHelp::ParamProxyBasic<double>("comKi", TorqueBalancingModuleParameterHandsForceIntegralGain, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultHandsForceGains.data(), "Integral gains of Hands forces PID"),
            //Centroidal gain
            new paramHelp::ParamProxyBasic<double>("kw", TorqueBalancingModuleParameterCentroidalGain, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultCentroidalGain, "Gain for the centroidal-based controller"),
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
