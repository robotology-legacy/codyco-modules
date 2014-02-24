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

#include "AdaptiveControlThread.h"
#include "AdaptiveControlConstants.h"
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#ifdef ADAPTIVECONTROL_TORQUECONTROL
#include <yarp/dev/ITorqueControl.h>
#else
#include <paramHelp/paramHelperClient.h>
#include <motorFrictionIdentificationLib/jointTorqueControlParams.h>
#endif
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <paramHelp/paramHelperServer.h>
#include <paramHelp/paramProxyInterface.h>
#include <string>
#include <cmath>
#include <Eigen/SVD>

using namespace yarp::os;
using namespace yarp::dev;
using namespace Eigen;
using namespace iCub::ctrl;

namespace adaptiveControl {

		
    AdaptiveControlThread::AdaptiveControlThread(const std::string& threadName,
                                                 const std::string& robotName,
                                                 const std::string& robotPart,
                                                 int periodMilliseconds,
                                                 paramHelp::ParamHelperServer&paramHelperServer,
#ifndef ADAPTIVECONTROL_TORQUECONTROL
                                                 paramHelp::ParamHelperClient& paramHelperClient,
#endif
                                                 const Eigen::Vector2d &linklengths):
    RateThread(periodMilliseconds),
    _period(periodMilliseconds),
    _controlEnabled(false),
    _maxReadFailed(3),
    _failedReads(0),
    _firstRunLoop(true),
    _threadName(threadName),
    _robotName(robotName),
    _robotPart(robotPart),
    _paramServer(paramHelperServer),
#ifndef ADAPTIVECONTROL_TORQUECONTROL
    _paramClient(paramHelperClient),
#endif
    _refAngularVelocity(0),
    _refSystemGain(1),
    _link1Length(linklengths(0)),
    _link2Length(linklengths(1)),
    _errorIntegral(0),
    _kneeTorque(0),
    _outputTau(ICUB_PART_DOF, 0.0)
    {
        _piHat = Vector8d::Zero();
        _dpiHat = Vector8d::Zero();
        _xi = Vector2d::Zero();
        _dxi = Vector2d::Zero();
        _sIntegral.setZero();
        _GammaInput.setZero();
        _Gamma.setZero();
#ifndef ADAPTIVECONTROL_TORQUECONTROL
        _jointTorqueControlTorques.setZero();
#endif
    }
    
    AdaptiveControlThread::~AdaptiveControlThread() { threadRelease(); }
    
    bool AdaptiveControlThread::setInitialConditions(const Eigen::Vector8d& initialPiHat, const double& initialXi1)
    {
        if (_controlEnabled) return false;
        _xi(0) = initialXi1;
         _piHat = initialPiHat;
        return true;
    }
    
    bool AdaptiveControlThread::controlEnabled() { return _controlEnabled; }
    
    void AdaptiveControlThread::resetState()
    {
        _errorIntegral = 0;
        _sIntegral.setZero();
    }
    
#pragma mark - RateThread Overridings
    
    bool AdaptiveControlThread::threadInit()
    {        
        //link parameters
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDMinDeterminantValue, &_minDeterminantValue));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDOutputEnabled, &_outputEnabled));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDIntegralSymmetricLimit, &_integralSaturationLimit));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDHomePositions, _homePositions.data()));
        //Kappa, Gamma, Lambda
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainLambda, &_lambda));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainLambdaIntegral, &_lambdaIntegral));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainKappa, _kappa.data()));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainKappaIntegral, _kappaIntegral.data()));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainGamma, _GammaInput.data()));
        //reference trajectory
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefBaseline, &_refBaseline));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefFrequency, &_refDesiredFrequency));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefAmplitude, &_refAmplitude));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefPhase, &_refPhase));   
        
		YARP_ASSERT(_paramServer.registerCommandCallback(AdaptiveControlCommandIDStart, this));
		YARP_ASSERT(_paramServer.registerCommandCallback(AdaptiveControlCommandIDStop, this));
        
        YARP_ASSERT(_paramServer.registerParamValueChangedCallback(AdaptiveControlParamIDGainGamma, this));
        
#ifndef ADAPTIVECONTROL_TORQUECONTROL
        YARP_ASSERT(_paramClient.linkParam(jointTorqueControl::PARAM_ID_TAU_OFFSET, _jointTorqueControlTorques.data()));
#endif
		
        //open ports and drivers
        //Encoder
        _driver = openDriver(_threadName, _robotName, _robotPart);
        if (!_driver) {
            error_out("Could not open driver %s\n", _robotPart.c_str());
            return false;
        }
        
        if (!_driver->view(_encoders) || !_encoders) {
            error_out("Error initializing encoders for %s\n", _robotPart.c_str());
            return false;
        }
        //velocity estimator
        _velocityEstimator = new AWLinEstimator(16*10/_period, 0.1);
        if (!_velocityEstimator) {
            error_out("Could not initialize velocity estimator");
            return false;
        }
        
        if (!_driver->view(_controlMode) || !_controlMode) {
            error_out("Error initializing control mode interface for %s\n", _robotPart.c_str());
            return false;
        }
        
        if (!_driver->view(_positionControl) || !_positionControl) {
            error_out("Error initializing position control interface for %s\n", _robotPart.c_str());
            return false;
        }
        
#ifdef ADAPTIVECONTROL_TORQUECONTROL
		info_out("Using torque interface\n");
        if (!_driver->view(_torqueControl) || !_torqueControl) {
            error_out("Error initializing torque Control for %s\n", _robotPart.c_str());
            return false;
        }
#endif
        //torque output
        _torqueOutput = new BufferedPort<Bottle>();
        if (!_torqueOutput || !_torqueOutput->open(("/" + _threadName + "/torque:o").c_str())) {
            error_out("Could not open port /%s/torque:o\n", _threadName.c_str());
            return false;
        }
        
         _debugPort = new BufferedPort<yarp::sig::Vector>();
        if (!_debugPort || !_debugPort->open(("/" + _threadName + "/debug:o").c_str())) {
            error_out("Could not open port /%s/debug:o\n", _threadName.c_str());
            return false;
        }
        
//         //TEMP x GAZEBO
//         _speedInput = new BufferedPort<Bottle>();
//         if (!_speedInput || !_speedInput->open(("/" + _threadName + "/speeds:i").c_str())) {
//             error_out("Could not open port _speedInput\n");
//             return false;
//         }
//         Network::connect(("/coman/" + _robotPart + "/analog/speeds:o").c_str(), ("/" + _threadName + "/speeds:i").c_str());
        
        //param server does not call the registered objects on initialization.
        _Gamma = _GammaInput.asDiagonal();
        setRobotToHomePositions();
        
        return true;
    }
    
    void AdaptiveControlThread::run()
    {
        if (_controlEnabled) {
            computeControl();
        }
    }
    
    void AdaptiveControlThread::threadRelease()
    {
        if (_driver) {
            stopControl();
            _driver->close();
            _encoders = NULL;
            _controlMode = NULL;
            _positionControl = NULL;
#ifdef ADAPTIVECONTROL_TORQUECONTROL
            _torqueControl = NULL;         
#endif
			delete _driver; _driver = NULL;
        }
        if (_torqueOutput) {
            _torqueOutput->close();
            delete _torqueOutput; _torqueOutput = NULL;
        }
        
        if (_debugPort) {
            _debugPort->close();
            delete _debugPort; _debugPort = NULL;
        }
        
        if (_velocityEstimator) {
            delete _velocityEstimator; _velocityEstimator = NULL;
        }
        
//         if (_speedInput) {
//             delete _speedInput; _speedInput = NULL;
//         }
    }
    
#pragma mark - Parameter Helper Callbacks
    /****************************************************************/
    /* Parameter Helper Callbacks */
    /****************************************************************/
    
    void AdaptiveControlThread::commandReceived(const paramHelp::CommandDescription &cd, const Bottle &params, Bottle &reply)
    {
        switch(cd.id)
        {
            case AdaptiveControlCommandIDStart:
                startControl();
				reply.addString("Start command received.");
                break;
            case AdaptiveControlCommandIDStop:
                stopControl();
				reply.addString("Stop command received.");
                break;
            default:
                break;
        }
    }
    
    void AdaptiveControlThread::parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface)
    {
        switch (proxyInterface->id) {
            case AdaptiveControlParamIDGainGamma:
                _Gamma = _GammaInput.asDiagonal();
                break;
            default:
                break;
        }
    }
    
#pragma mark - Private methods
    /****************************************************************/
    /* Private methods */
    /****************************************************************/
    
    PolyDriver* AdaptiveControlThread::openDriver(std::string localName, std::string robotName, std::string bodyPartName)
    {
        std::string localPort  = "/" + localName + "/" + bodyPartName;
        std::string remotePort = "/" + robotName + "/" + bodyPartName;
        Property options;
        options.put("robot", robotName.c_str());
        options.put("part", bodyPartName.c_str());
        options.put("device", "remote_controlboard");
        options.put("local", localPort.c_str());
        options.put("remote", remotePort.c_str());
        
        PolyDriver *pd = new PolyDriver(options);
        if(!pd || !(pd->isValid()))
        {
            error_out("Problems instantiating the device driver %s\n", bodyPartName.c_str());
            return NULL;
        }
        return pd;
    }
    
    
    void AdaptiveControlThread::computeRegressor(const Eigen::Vector2d& q,
                                                 const Eigen::Vector2d& dq,
                                                 const Eigen::Vector2d& dq_lin,
                                                 const Eigen::Vector2d& ddq,
                                                 Eigen::Matrix28d& regressor)
    {
        double c1 = cos(q(0));
        double c2 = cos(q(1)), s2 = sin(q(1));
        double c12 = cos(q(0) + q(1));
        
        regressor(0,0) = _link1Length * _link1Length * ddq(0) + _link1Length * gravity * c1;
        regressor(0,1) = 2 * _link1Length * ddq(0) + gravity * c1;
        regressor(0,2) = ddq(0);
        regressor(0,3) = (_link1Length * _link1Length + 2 * _link1Length * _link2Length * c2 + _link2Length * _link2Length) * ddq(0) + (_link1Length * _link2Length * c2 + _link2Length * _link2Length) * ddq(1)
        - _link2Length * _link1Length * s2 * dq(1) * dq_lin(0) - _link1Length * _link2Length * s2 * (dq(0) + dq(1)) * dq_lin(1) + _link1Length * gravity * c1 + _link2Length * gravity * c12;
        regressor(0,4) = 2 * (_link1Length * c2 + _link2Length) * ddq(0) + (_link1Length * c2 + 2 * _link2Length) * ddq(1) - _link1Length * s2 * dq(1) * dq_lin(0)
        - _link1Length * s2 * (dq(0) + dq(1))*dq_lin(1) + gravity * c12;
        regressor(0,5) = ddq(0) + ddq(1);
        regressor(0,6) = dq_lin(0);
        regressor(0,7) = 0.0;
        regressor(1,0) = regressor(1,1) = regressor(1,2) = 0.0;
        regressor(1,3) = (_link1Length * _link2Length * c2 + _link2Length * _link2Length) * ddq(0) + _link2Length * _link2Length * ddq(1) + _link1Length * _link2Length * s2 * dq(0) * dq_lin(0) + _link2Length * gravity * c12;
        regressor(1,4) = (_link1Length * c2 + 2 * _link2Length) * ddq(0) + 2 * _link2Length * ddq(1) + _link1Length * s2 * dq(0) * dq_lin(0) + gravity * c12;
        regressor(1,5) = regressor(0,5);
        regressor(1,6) = 0.0;
        regressor(1,7) = dq_lin(1);
    }
    
    void AdaptiveControlThread::computeControl()
    {
        if (_firstRunLoop) {
            _firstRunLoop = false;
            _initialTime = yarp::os::Time::now();
            _previousTime = 0;
        }
        double now = yarp::os::Time::now() - _initialTime;
        
        double dt = now - _previousTime;
        _previousTime = now;
        
        //update state variables (only if sendCommands = true, otherwise the updating law integrates a constant value)
        double dotOmega = -_refSystemGain * (_refAngularVelocity - 2 * pi * _refDesiredFrequency);
        if (_outputEnabled) {
            _piHat = _piHat + dt * _dpiHat;
            _xi(0) = _xi(0) + dt * _dxi(0);
            _refAngularVelocity = _refAngularVelocity + dt * dotOmega;
        }
        
        //define reference trajectory
        double q_ref = _refBaseline + _refAmplitude * sin(2 * pi * _refDesiredFrequency * now + _refPhase);
        double dq_ref = _refAmplitude * 2 * pi * _refDesiredFrequency * cos(2 * pi * _refDesiredFrequency * now + _refPhase);
        double ddq_ref = -_refAmplitude * pow(2*pi*_refDesiredFrequency, 2) * sin(2*pi*_refDesiredFrequency * now + _refPhase);
        
        _currentRef = q_ref;
        
        //read data: joint positions and velocities
        bool success = readSensors(_q, _dq);
        _q(0) = _dq(0) = 0;
        if (!success) {
            error_out("Failed to retrieve positions from encoders\n");
            ++_failedReads;
            if (_failedReads > _maxReadFailed) {
                error_out("Reading from encoder fails more than %d times in a row. Stop control.\n", _maxReadFailed);
                stopControl();
                return;
            }
        }
        else {
            _failedReads = 0;
        }
        
        double qTilde = _q(1) - q_ref;
//         qTilde = qTilde * qTilde < convertDegToRad(0.1) ? 0 : qTilde;
        
        if (_outputEnabled) {
            _errorIntegral = hardLimiter(_errorIntegral + dt * qTilde, -_integralSaturationLimit, _integralSaturationLimit);
        }
        _xi(1) = dq_ref - _lambda * qTilde - _lambdaIntegral * _errorIntegral;
        
        //define variable 's'
        Vector2d s = _dq - _xi;
        if (_outputEnabled) {
            for (int i = 0; i < 2; i++)
                _sIntegral(i) = hardLimiter(_sIntegral(i) + dt * s(i), -_integralSaturationLimit, _integralSaturationLimit);           
        }
        
        //compute dxi
        double dqError = _dq(1) - dq_ref;
//         dqError = dqError * dqError < convertDegToRad(0.1) ? 0 : dqError;
        _dxi(1) = ddq_ref - _lambda * (dqError);
        _dxi(0) = 0;
        
        //compute regressor
        Matrix28d regressor;
        computeRegressor(_q, _dq, _xi, _dxi, regressor);
        
        //compute torques and send them to actuation
        _kneeTorque = regressor.row(1) * _piHat - _kappa(1) * s(1) - _kappaIntegral(1) * _sIntegral(1);
        writeOutputs();
        
        
        //compute update rule for parameters
        _dpiHat = - _Gamma * regressor.transpose() * s;
        
        writeDebug();
    }
    
    bool AdaptiveControlThread::readSensors(Vector2d& positions, Vector2d& velocities)
    {
        double q[ICUB_PART_DOF], timestamp[ICUB_PART_DOF];
        
        bool result = _encoders->getEncodersTimed(q, timestamp);
        
        if (result) {
            //convert all joints to rads
            for (int i = 0; i < ICUB_PART_DOF; i++) {
                q[i] = convertDegToRad(q[i]);
            }
            //I'm intereste only in two joints
            yarp::sig::Vector myQ(2, 0.0);
            myQ[0] = q[passiveJointIndex];
            myQ[1] = q[activeJointIndex];
            
            AWPolyElement element;
            element.data = myQ;
            element.time = timestamp[0];
            yarp::sig::Vector dq = _velocityEstimator->estimate(element);
            positions(0) = myQ(0);
            positions(1) = myQ(1);
            
            velocities(0) = dq(0);
            velocities(1) = dq(1);
            
//             Bottle *speed = _speedInput->read(false);
//             if (speed) {
//                 yarp::os::Value valPassive = speed->get(passiveJointIndex);
//                 yarp::os::Value valActive = speed->get(activeJointIndex);
//             
//                 if (!valPassive.isNull())
//                     velocities(0) = valPassive.asDouble();
//                 if (!valActive.isNull())
//                     velocities(1) = valActive.asDouble();              
//             }
            
        }
        
        //Check joint limits   
        if (!hipPitchJoint.isInLimit(_q(0), 0.9) || !kneeJoint.isInLimit(_q(1), 0.9)) 
            haltControl(q);
        
//                     if (velocities(0)*velocities(0) < convertDegToRad(5))
//                 velocities(0) = 0; 
//             if (velocities(1)*velocities(1) < convertDegToRad(5))
//                 velocities(1) = 0;
        
        
        return result;
    }
    		
    void AdaptiveControlThread::writeOutputs()
    {
        _outputTau.zero();
#ifndef ADAPTIVECONTROL_TORQUECONTROL
        _jointTorqueControlTorques.setZero();
#endif
        if (_outputEnabled) {
            _outputTau(activeJointIndex) = _kneeTorque;
#ifndef ADAPTIVECONTROL_TORQUECONTROL
            _jointTorqueControlTorques(robotPartStartingIndex + activeJointIndex) = _kneeTorque;
#endif
        }
#ifdef ADAPTIVECONTROL_TORQUECONTROL
        //set directly the torque ref to the control
        _torqueControl->setRefTorques(_outputTau.data());
#else
        _paramClient.sendStreamParams();
#endif
        
        Bottle& torqueBottle = _torqueOutput->prepare();
        torqueBottle.clear();
        torqueBottle.addList().read(_outputTau);
        _torqueOutput->write();
    }
    
    void AdaptiveControlThread::startControl()
    {
        if (!_controlEnabled) {
            //only if the control was in stop state
            _controlEnabled = true;
            _failedReads = 0;
            _firstRunLoop = true;

#ifdef ADAPTIVECONTROL_TORQUECONTROL
            _controlMode->setTorqueMode(passiveJointIndex);
			_torqueControl->setRefTorque(passiveJointIndex, 0);
            _controlMode->setTorqueMode(activeJointIndex);
			_torqueControl->setRefTorque(activeJointIndex, 0);
#endif
			info_out("Control is now enabled\n");
        }
    }
    
    void AdaptiveControlThread::stopControl()
    {
        _controlEnabled = false;
#ifdef ADAPTIVECONTROL_TORQUECONTROL
        for (int i = 0; i < ICUB_PART_DOF; i++) {
            _controlMode->setPositionMode(i);
        }
#endif
        //should I set some default position here?
        
        info_out("Control is now disabled\n");
    }
    
    void AdaptiveControlThread::setRobotToHomePositions() 
    {
        stopControl(); //this stops torque control and sets all joints in position mode
        VectorNd refSpeed = Eigen::VectorNd::Constant(10);
        _positionControl->setRefSpeeds(refSpeed.data());
        
        double homes[ICUB_PART_DOF];
        for (int i = 0; i < ICUB_PART_DOF; i++) {
            homes[i] = convertRadToDeg(_homePositions(i));
        }
        _positionControl->positionMove(homes);

        info_out("Going to Home position: ");
        for (int i = 0; i < ICUB_PART_DOF; i++) {
            info_out("%lf ", homes[i]);
        }
        info_out("\n");
    }
    
    void AdaptiveControlThread::haltControl(double* haltPositions) 
    {
        stopControl();
        if (haltPositions) {
            double newPos[ICUB_PART_DOF];
            for (int i = 0; i < ICUB_PART_DOF; i++) {
                newPos[i] = convertRadToDeg(haltPositions[i]);
            }
            _positionControl->positionMove(newPos);
            info_out("Halting the robot to: ");
            for (int i = 0; i < ICUB_PART_DOF; i++) {
                info_out("%lf ", haltPositions[i]);
            }
            info_out("\n");
        }
    }    
    
    void AdaptiveControlThread::writeDebug() {
		
        //writing also to std::out
//         info_out("P:(%lf, %lf) V:(%lf, %lf) T:(%lf) e:(%lf) pi:(%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf)\n",
//             _q(0), _q(1),
//             _dq(0), _dq(1),
//             _outputTau(activeJointIndex),
//             _currentRef - _q(1),
//             _piHat(0),_piHat(1),_piHat(2),_piHat(3),_piHat(4),_piHat(5),_piHat(6),_piHat(7)
//         );
        
        
		yarp::sig::Vector& vector = _debugPort->prepare();
        vector.clear();
        //write everything inside vector
        //_q(2)
        //_dq(2)
        //_tau(1): only active joint
        //_piHat(8)
        
        vector.push_back(convertRadToDeg(_q(0)));
        vector.push_back(convertRadToDeg(_q(1)));
        vector.push_back(_dq(0));
        vector.push_back(_dq(1));
        vector.push_back(_outputTau(activeJointIndex));
        vector.push_back(convertRadToDeg(_q(1) - _currentRef));
        Vector2d s = _dq - _xi;

        
        
        double norm = 0;
        for (int i = 0; i < 8; i++) {
            norm += _piHat(i) * _piHat(i);
//             vector.push_back(_piHat(i));
        }
        vector.push_back(norm);
        vector.push_back(_massMatrixDeterminant);
        vector.push_back(_minDeterminantValue);
        vector.push_back(_piHatModificationOn ? 1 : 0);
        vector.push_back(_errorIntegral);
        vector.push_back(_sIntegral(0));
        vector.push_back(_sIntegral(1));
		vector.push_back(s(1)*s(1) + s(0)*s(0));
        
         Matrix28d regressor;
        computeRegressor(_q, _dq, _xi, _dxi, regressor);
        
        //compute torques and send them to actuation
        double tau = regressor.row(1) * _piHat;
        vector.push_back(tau);
        
        _debugPort->write();
		
	}

    template <typename Derived1, typename Derived2>
    void AdaptiveControlThread::dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                                                    double dampingFactor,
                                                    Eigen::MatrixBase<Derived2>& Apinv)
    {
        int m = A.rows(), n = A.cols(), k = m < n ? m : n;
        JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
        const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType& singularValues = svd.singularValues();
        MatrixXd sigmaDamped = MatrixXd::Zero(k, k);
        
        double damp = dampingFactor * dampingFactor;
        for (int idx = 0; idx < k; idx++) {
            sigmaDamped(idx, idx) = singularValues(idx) / (singularValues(idx) * singularValues(idx) + damp);
        }
        Apinv   = svd.matrixV() * sigmaDamped * svd.matrixU().transpose();   // damped pseudoinverse
    }

}
