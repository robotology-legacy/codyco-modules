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
            //PIDs
            //COM
            TorqueBalancingModuleParameterCOMIntegralLimit,
            TorqueBalancingModuleParameterCOMProportionalGain,
            TorqueBalancingModuleParameterCOMDerivativeGain,
            TorqueBalancingModuleParameterCOMIntegralGain,
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
            TorqueBalancingModuleParameterMonitorDesiredCOM,

        } TorqueBalancingModuleParameter;

        static const int TorqueBalancingModuleParameterSize = 13;

        static const double defaultIntegralLimit = std::numeric_limits<double>::max(); //no limit
        static const Eigen::VectorXd defaultCOMGains = Eigen::VectorXd(3).setZero();
        static const double defaultCentroidalGain = 0;

        paramHelp::ParamProxyInterface *const TorqueBalancingModuleParameterDescriptions[]  =
        {
            //COM
            new paramHelp::ParamProxyBasic<double>("comIntLimit", TorqueBalancingModuleParameterCOMIntegralLimit, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultIntegralLimit, "Integral limit on COM PID"),
            new paramHelp::ParamProxyBasic<double>("comKp", TorqueBalancingModuleParameterCOMProportionalGain, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultCOMGains.data(), "Proportional gains of COM PID"),
            new paramHelp::ParamProxyBasic<double>("comKd", TorqueBalancingModuleParameterCOMDerivativeGain, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultCOMGains.data(), "Derivative gains of COM PID"),
            new paramHelp::ParamProxyBasic<double>("comKi", TorqueBalancingModuleParameterCOMIntegralGain, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultCOMGains.data(), "Integral gains of COM PID"),
            //gains
            new paramHelp::ParamProxyBasic<double>("kw", TorqueBalancingModuleParameterCentroidalGain, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultCentroidalGain, "Gain for the centroidal-based controller"),
            //Monitored variables
            new paramHelp::ParamProxyBasic<double>("desCOMAcc", TorqueBalancingModuleParameterMonitorDesiredCOMAcceleration, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Desired COM acceleration computed by the controller"),
            new paramHelp::ParamProxyBasic<double>("comError", TorqueBalancingModuleParameterMonitorCOMError, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Instantaneous COM error"),
            new paramHelp::ParamProxyBasic<double>("comIntError", TorqueBalancingModuleParameterMonitorCOMIntegralError, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Integral of COM error"),
            new paramHelp::ParamProxyBasic<double>("feetForces", TorqueBalancingModuleParameterMonitorFeetForces, 12, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Desired feet forces"),
            new paramHelp::ParamProxyBasic<double>("desCOM", TorqueBalancingModuleParameterMonitorDesiredCOM, 3, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_MONITOR, 0, "Desired COM position"),
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
