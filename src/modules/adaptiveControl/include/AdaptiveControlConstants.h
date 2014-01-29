/*
 * Copyright (C) 2014 CoDyCo
 * Author: Daniele Pucci, Francesco Romano
 * email:  daniele.pucci@iit.it, francesco.romano@iit.it
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

#ifndef ADAPTIVECONTROLCONSTANTS_H
#define ADAPTIVECONTROLCONSTANTS_H

#include <paramHelp/paramProxyBasic.h>
#include <string>
#include <cstdio>
#include <Eigen/Core> //forward declaration on matrix type is too complicated :(

#define error_out(...)\
            fprintf(stderr, __VA_ARGS__)
#define info_out(...)\
            fprintf(stdout, __VA_ARGS__)

#define PARAMETERS_SIZE 8
#define ICUB_PART_DOF 6
#define JOINTTORQUECONTROL_DOFS 25

//define eigen common matrices
namespace Eigen {
//    
//    template<typename _Scalar, int _Rows, int _Cols>//, int _Options, int _MaxRows, int _MaxCols>
//    class Matrix;
    
    typedef Matrix<double, 2, PARAMETERS_SIZE> Matrix28d; //define matrix for regressor
    typedef Matrix<double, PARAMETERS_SIZE, 1> Vector8d; //define vector of parameters
    typedef Matrix<double, PARAMETERS_SIZE, PARAMETERS_SIZE> Matrix8d; //define matrix for parameters gain
    typedef Matrix<double, ICUB_PART_DOF, 1> VectorNd;
}

namespace adaptiveControl
{
    //Configuration constants
    const int passiveJointIndex = 0;
    const int activeJointIndex = 3;
    
    const int robotPartStartingIndex = 13;
    
    //various constants
    const double gravity = 9.80665;
    const double pi = 3.1415926536;
    
    inline double convertDegToRad(double degAngle) { return degAngle / 180 * pi; }
    inline double convertRadToDeg(double radAngle) { return radAngle * 180 / pi; }
    inline double hardLimiter(double inputValue, double lowerLimit, double upperLimit)
    { return inputValue > upperLimit ? upperLimit : (inputValue < lowerLimit ? lowerLimit : inputValue); }
//    template <typename Derived>
//    inline void hardLimiter(Eigen::MatrixBase<Derived>& inputValue, double lowerLimit, double upperLimit, Eigen::MatrixBase<Derived>& outputValue)
//    {  }

    struct JointLimit 
    {
    private:
        double delta;
        double center;
    public:
        double min;
        double max;
        
        JointLimit(double min, double max): min(min), max(max) 
        {
            delta = (max - min) / 2;
            center = (max + min) / 2;
        }
        
        bool isInLimit(double currentPosition, double scalingFactor) const
        {
            double newMax = center + delta * scalingFactor;
            double newMin = center - delta * scalingFactor;
            
            return currentPosition < newMax && currentPosition > newMin ? true : false;
        }
    };
    
    const JointLimit kneeJoint(-1.73, 0);
    const JointLimit hipPitchJoint(-0.52, 0.92);
    
    
    // ******************************************************************************************************************************
    // ****************************************** PARAMETER SECTION *****************************************************************
    // ******************************************************************************************************************************
    // *** DEFAULT PARAMETER VALUES
    //config
    static const std::string defaultModuleName = "adaptiveControl";
    static const std::string defaultRobotName = "icubSim"; // robot name
    static const std::string defaultRobotPart = "right_leg";
    static const int defaultModulePeriod = 10;
    static const Eigen::Vector2d defaultLinkLengths = Eigen::Vector2d::Zero();
    static const double defaultIntegralSymmetricLimit = 10;
    static const Eigen::VectorNd defaultHomePositions = Eigen::VectorNd::Zero();
#ifndef ADAPTIVECONTROL_TORQUECONTROL
    static const std::string defaultTorqueControlModuleName = "jtc";
#endif
    //rpc
    static const int defaultOutputEnabled = 0;
    static const double defaultMinDeterminant = 10.0;
    static const double defaultLambdaGain = 1;
    static const double defaultLambdaIntegralGain = 0.1;
    static const Eigen::Vector2d defaultKappaGain = Eigen::Vector2d::Constant(1);
    static const Eigen::Vector2d defaultKappaIntegralGain = Eigen::Vector2d::Constant(1);
    static const Eigen::Vector8d defaultGammaGain = Eigen::Vector8d::Constant(1);
    
    static const double defaultRefBaseline = 0;
    static const double defaultRefAngularVelocity = 0;
    static const double defaultRefFrequency = 0;
    static const double defaultRefPhase = 0;
    static const Eigen::Vector8d defaultInitialPiHat = Eigen::Vector8d::Constant(1);
    static const double defaultInitialXi1 = 1;
    
    // *** IDs of all the module parameters
    enum AdaptiveControlParamID {
        //Configuration IDs
        AdaptiveControlParamIDModuleName,
        AdaptiveControlParamIDRobotName,
        AdaptiveControlParamIDRobotPartName,
        AdaptiveControlParamIDPeriod,
        AdaptiveControlParamIDLinkLengths,
        AdaptiveControlParamIDIntegralSymmetricLimit,
        AdaptiveControlParamIDHomePositions,
#ifndef ADAPTIVECONTROL_TORQUECONTROL
        AdaptiveControlParamIDJointTorqueControlModuleName,
#endif
        //RPC in-out parameters
        AdaptiveControlParamIDOutputEnabled,
        AdaptiveControlParamIDMinDeterminantValue,
        AdaptiveControlParamIDGainLambda,
        AdaptiveControlParamIDGainLambdaIntegral,
        AdaptiveControlParamIDGainKappa,
        AdaptiveControlParamIDGainKappaIntegral,
        AdaptiveControlParamIDGainGamma,
        AdaptiveControlParamIDRefBaseline,
        AdaptiveControlParamIDRefFrequency,
        AdaptiveControlParamIDRefAmplitude,
        AdaptiveControlParamIDRefPhase,
        AdaptiveControlParamIDInitialPiHat,
        AdaptiveControlParamIDInitialXi1,
    };

    // *****************************************************************************************************************************************
    // ****************************************** DESCRIPTION OF ALL THE MODULE AND THREAD PARAMETERS ******************************************
    // *****************************************************************************************************************************************
#ifndef ADAPTIVECONTROL_TORQUECONTROL
    const unsigned short adaptiveControlParamDescriptorsSize = 20;
#else
    const unsigned short adaptiveControlParamDescriptorsSize = 19;
#endif
    const paramHelp::ParamProxyInterface *const adaptiveControlParamDescriptors[]  =
    {
        //NAME, ID, SIZE, BOUNDS, I/O ACCESS, DEFAULT VALUE, DESCRIPTION
        //Configuration parameters (at module launch)
        new paramHelp::ParamProxyBasic<std::string>("name", AdaptiveControlParamIDModuleName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultModuleName, "Name of the instance of the module"),
        new paramHelp::ParamProxyBasic<std::string>("robot", AdaptiveControlParamIDRobotName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultRobotName, "Name of the robot"),
        new paramHelp::ParamProxyBasic<std::string>("part", AdaptiveControlParamIDRobotPartName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultRobotPart, "Robot part: currently only leg is supported, so specify (left|right)_leg"),
        new paramHelp::ParamProxyBasic<int>("period", AdaptiveControlParamIDPeriod, 1, paramHelp::ParamConstraint<int>(), paramHelp::PARAM_CONFIG, &defaultModulePeriod, "Name of the robot"),
        new paramHelp::ParamProxyBasic<double>("linkLengths", AdaptiveControlParamIDLinkLengths, 2, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_CONFIG, defaultLinkLengths.data(), "Length of links"),
        new paramHelp::ParamProxyBasic<double>("intLimit", AdaptiveControlParamIDIntegralSymmetricLimit, 1, paramHelp::ParamLowerBound<double>(0), paramHelp::PARAM_CONFIG, &defaultIntegralSymmetricLimit, "Absolute value of the limit for the integral of the error => the integral will be between -intLimit and intLimit"),
        new paramHelp::ParamProxyBasic<double>("home", AdaptiveControlParamIDHomePositions, ICUB_PART_DOF, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_CONFIG, defaultHomePositions.data(), "Home positions for the robot part"),
#ifndef ADAPTIVECONTROL_TORQUECONTROL
 new paramHelp::ParamProxyBasic<std::string>("jtcName", AdaptiveControlParamIDJointTorqueControlModuleName, 1, paramHelp::ParamConstraint<std::string>(), paramHelp::PARAM_CONFIG, &defaultTorqueControlModuleName, "Name of the torqueControl module"),
#endif
        //RPC in/out parameters (during runtime)
        new paramHelp::ParamProxyBasic<int>("sendCommands", AdaptiveControlParamIDOutputEnabled, 1, paramHelp::ParamBilatBounds<int>(0, 1), paramHelp::PARAM_IN_OUT, &defaultOutputEnabled, "Boolean for enable output to motors"),
        new paramHelp::ParamProxyBasic<double>("minDet", AdaptiveControlParamIDMinDeterminantValue, 1, paramHelp::ParamLowerBound<double>(0), paramHelp::PARAM_IN_OUT, &defaultMinDeterminant, "Minimum value for the determinant of the passive minor of the Mass Matrix"),
        new paramHelp::ParamProxyBasic<double>("lambda", AdaptiveControlParamIDGainLambda, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultLambdaGain, "Lambda gain: rate of convergence of active joints to reference"),
        new paramHelp::ParamProxyBasic<double>("lambdaI", AdaptiveControlParamIDGainLambdaIntegral, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultLambdaIntegralGain, "Lambda Integrale gain: integral gain for the position error"),
        new paramHelp::ParamProxyBasic<double>("kappa", AdaptiveControlParamIDGainKappa, 2, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultKappaGain.data(), "Kappa gain: torque gain"),
        new paramHelp::ParamProxyBasic<double>("kappaI", AdaptiveControlParamIDGainKappaIntegral, 2, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultKappaIntegralGain.data(), "Kappa Integral gain: it acts on s"),
        new paramHelp::ParamProxyBasic<double>("gamma", AdaptiveControlParamIDGainGamma, PARAMETERS_SIZE, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultGammaGain.data(), "Gamma gain: gain in the parameter update rule"),
        new paramHelp::ParamProxyBasic<double>("refBase", AdaptiveControlParamIDRefBaseline, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultRefBaseline, "Baseline for reference signal: r(t) = base + ampl * sin(2 * pi * freq * t + phase)"),
        new paramHelp::ParamProxyBasic<double>("refFreq", AdaptiveControlParamIDRefFrequency, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultRefFrequency, "Frequency for reference signal: r(t) = base + ampl * sin(2 * pi * freq * t + phase)"),
        new paramHelp::ParamProxyBasic<double>("refAmpl", AdaptiveControlParamIDRefAmplitude, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultRefFrequency, "Amplitude for reference signal: r(t) = base + ampl * sin(2 * pi * freq * t + phase)"),
        new paramHelp::ParamProxyBasic<double>("refPhase", AdaptiveControlParamIDRefPhase, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultRefPhase, "Phase for reference signal: r(t) = base + ampl * sin(2 * pi * freq * t + phase)"),
        new paramHelp::ParamProxyBasic<double>("piHat_0", AdaptiveControlParamIDInitialPiHat, PARAMETERS_SIZE, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, defaultInitialPiHat.data(), "Initial values for pihat. They can be set only if the control is off."),
        new paramHelp::ParamProxyBasic<double>("xi1_0", AdaptiveControlParamIDInitialXi1, 1, paramHelp::ParamConstraint<double>(), paramHelp::PARAM_IN_OUT, &defaultInitialXi1, "Initial values for xi1. It can be set only if the control is off."),
    };
    
    
    // *** IDs of all the module command
    enum AdaptiveControlCommandID {
        AdaptiveControlCommandIDStart,
        AdaptiveControlCommandIDStop,
        AdaptiveControlCommandIDHelp,
        AdaptiveControlCommandIDQuit,
        AdaptiveControlCommandIDReset,
    };
    
    // ******************************************************************************************************************************
    // ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
    // ******************************************************************************************************************************
    const unsigned short adaptiveControlCommandDescriptorsSize = 5;
    const paramHelp::CommandDescription adaptiveControlCommandDescriptors[]  =
    {
        //                  NAME            ID                          DESCRIPTION
        paramHelp::CommandDescription("start", AdaptiveControlCommandIDStart, "Start the planner"),
        paramHelp::CommandDescription("stop", AdaptiveControlCommandIDStop, "Stop the planner"),
        paramHelp::CommandDescription("help", AdaptiveControlCommandIDHelp, "Get instructions about how to communicate with this module"),
        paramHelp::CommandDescription("quit", AdaptiveControlCommandIDQuit, "Stop the planner and quit the module"),
        paramHelp::CommandDescription("reset", AdaptiveControlCommandIDReset, "Reset the control thread. It can be done only if the control is in the stop state"),
    };
    
}

#endif
