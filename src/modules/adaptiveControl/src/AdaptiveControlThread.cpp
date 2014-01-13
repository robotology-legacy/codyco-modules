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
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <paramHelp/paramHelperServer.h>
#include <string>
#include <cmath>

using namespace yarp::os;
using namespace yarp::dev;
using namespace Eigen;
using namespace iCub::ctrl;

namespace adaptiveControl {
    
    
    AdaptiveControlThread::AdaptiveControlThread(std::string& threadName,
                                                 std::string& robotName,
                                                 int periodMilliseconds,
                                                 paramHelp::ParamHelperServer& paramHelperServer,
                                                 const Eigen::Vector2d &linklengths):
    RateThread(periodMilliseconds),
    _controlEnabled(false),
    _threadName(threadName),
    _robotName(robotName),
    _paramServer(paramHelperServer),
    _link1Length(linklengths(0)),
    _link2Length(linklengths(1)),
    _outputTau(ICUB_PART_DOF, 0.0)
    {
        _dpi = Vector8d::Zero();
        _dxi = Vector2d::Zero();
    }
    
    AdaptiveControlThread::~AdaptiveControlThread()
    { /*should be called after thread release*/ }
    
#pragma mark - RateThread Overridings
    
    bool AdaptiveControlThread::threadInit()
    {
        //link parameters
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDMinDeterminantValue, &_minDeterminantValue));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDOutputEnabled, &_outputEnabled));
        //Kappa, Gamma, Lambda
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainLambda, &_lambda));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainKappa, _kappa.data()));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDGainGamma, _Gamma.data()));
        //reference trajectory
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefBaseline, &_refBaseline));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefFrequency, &_refFrequency));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefAmplitude, &_refAmplitude));
        YARP_ASSERT(_paramServer.linkParam(AdaptiveControlParamIDRefPhase, &_refPhase));
        
        //open ports and drivers
        //Encoder
        _encodersDriver = openDriver(_threadName, _robotName, "left_leg");
        if (!_encodersDriver) {
            error_out("Could not open driver left_leg\n");
            return false;
        }
        
        if (!_encodersDriver->view(_encoders)) {
            error_out("Error initializing encoders for left_leg\n");
            return false;
        }
        //velocity estimator
        _velocityEstimator = new AWLinEstimator(16, 1);
        if (!_velocityEstimator) {
            error_out("Could not initialize velocity estimator");
            return false;
        }
        
        //torque output
        _torqueOutput = new BufferedPort<Bottle>();
        if (!_torqueOutput || _torqueOutput->open("/" + _threadName + "/torque:o")) {
            error_out("Could not open port /%s/torque:o\n", _threadName.c_str());
            return false;
        }

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
        if (_encodersDriver) {
            _encodersDriver->close();
            if (_encoders) { //??
                delete _encoders; _encoders = NULL;
            }
            delete _encodersDriver; _encodersDriver = NULL;
        }
        if (_torqueOutput) {
            _torqueOutput->close();
            delete _torqueOutput; _torqueOutput = NULL;
        }
        if (_velocityEstimator) {
            delete _velocityEstimator; _velocityEstimator = NULL;
        }
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
                _controlEnabled = true;
                break;
            case AdaptiveControlCommandIDStop:
                _controlEnabled = false;
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
        double now = yarp::os::Time::now();
        static double previousTime = now; //move to instance variable?
        
        double dt = now - previousTime;
        previousTime = now;
        
        //update state variables
        _pi = _pi + dt * _dpi;
        _xi(0) = _xi(0) + dt * _dxi(0);
        
        //define reference trajectory
        double q_ref = _refBaseline + _refAmplitude * sin(_refFrequency * now + _refPhase);
        double dq_ref = _refAmplitude * _refFrequency * cos(_refFrequency * now + _refPhase);
        double ddq_ref = -_refAmplitude * _refFrequency * _refFrequency * sin(_refFrequency * now + _refPhase);
        
        //read data: joint positions and velocities
        bool result = readSensors(_q, _dq);
        if (!result) {
#warning what should i do?
        }
        
        _xi(1) = dq_ref - _lambda * (_q(1) - q_ref);
        
        //define variable 's'
        Vector2d s = _dq - _xi;
        
        //extract estimated parameters
        double m1H = _pi(0);
        double l1H = (_pi(1) / m1H) + _link1Length;
        double I1H = _pi(2) - (_pi(1) * _pi(1) / m1H);
        double m2H = _pi(3);
        double l2H = (_pi(4) / m2H) + _link2Length;
        double I2H = _pi(5) - (_pi(4) * _pi(4) / m2H);
        double F1H = _pi(6);
//        double F2H = _pi(7);
        
        double c1 = cos(_q(0));
        double c2 = cos(_q(1)), s2 = sin(_q(1));
        double c12 = cos(_q(0) + _q(1));
        
        //compute M, C, and g
        double m11H = I1H + m1H * l1H * l1H + I2H + m2H*(_link1Length * _link1Length + l2H * l2H + 2 * _link1Length * l2H * c2);
        double m12H = I2H + m2H * (l2H * l2H + _link1Length * l2H * c2);
//        double m22H = I2H + m2H * l2H * l2H;
        
        double hH = -m2H * _link1Length * l2H * s2;
        double C11H = hH * _dq(1);
        double C12H = hH * (_dq(0) + _dq(1));
//        double C21H = -hH * _dq(0);
//        double C22H = 0;
        
        double g1H = (m1H * l1H + m2H * _link1Length) * gravity * c1 + m2H * l2H * gravity * c12;
//        double g2H = m2H * l2H * gravity * c12;
        
        
        //compute dxi
        _dxi(1) = ddq_ref - _lambda * (_dq(1) - dq_ref);
        _dxi(0) =  1/m11H * (_dq(0) - (1 + C11H + F1H) * _xi(0) - m12H * _dxi(1) - C12H * _xi(1) - g1H);
#warning add _kappa(0)
        
        
        //compute regressor
        Matrix28d regressor;
        computeRegressor(_q, _dq, _xi, _dxi, regressor);
        
        //compute torques and send them to actuation
        double tau = regressor.row(1) * _pi - _kappa(1) * s(1);
        if (_outputEnabled) {
            _outputTau(3) = tau;
            writeOutputs();
        }
        
        //compute update rule for parameters
        _dpi = - _Gamma * regressor.transpose() * s;
        
        if (m11H <= _minDeterminantValue) {
            //I'm near a singularity for matrix Mpassive
            //compute delta and Upsilon matrix.
            
            //compute regressor for mass matrix Y_M(q, ei) as difference between the
            // regressor with q and qddot and regressor with only q (=> corresponds to gravity term)
            Eigen::Vector2d zero2Vector = Eigen::Vector2d::Zero();
            Eigen::Vector2d e1 = Eigen::Vector2d::Zero(); e1(0) = 1;
            
            Eigen::Matrix28d Yacc;
            Eigen::Matrix28d Ygravity;
            computeRegressor(_q, zero2Vector, zero2Vector, e1, Yacc);
            computeRegressor(_q, zero2Vector, zero2Vector, zero2Vector, Ygravity);
            
            Eigen::Matrix<double, 1, 8> Ymass = Yacc.row(0) - Ygravity.row(0);
            Eigen::Matrix<double, 8, 1> delta = Ymass.transpose() / m11H;
            
            //compute derivative of m_i w.r.t. q
            //take the i^th column of the whole mass matrix.
            //dmi_dq = [0, -2* m2H * _link1Length * l2H * s2] * dq;
            double derivativeMpFordq = _dq(1) * (-2 * m2H * _link1Length * l2H * s2);
            
            //upsilon is the sum of two terms: 1) the dm_dq times dq  2) the
            //derivative of m w.r.t. its parameters (=> it becomes Y_M) times the
            //(original) update law for the parameters
            double upsilon = derivativeMpFordq - Ymass * _Gamma * regressor.transpose() * s;
            double zeta = upsilon / m11H; //trace(m11Inv * upsilon);
            if (zeta < 0) {
                //double eta = (params(1)/Mdet - zeta )/ (delta'*Gamma* delta);
                double eta = - zeta / (delta.transpose() * _Gamma * delta);
                //only if zeta is less than zero apply modification
                _dpi = _dpi + eta * _Gamma * delta;
            }
            
        }
    }
    
    bool AdaptiveControlThread::readSensors(Vector2d& positions, Vector2d& velocities)
    {
        double q[ICUB_PART_DOF], timestamp[ICUB_PART_DOF];
        
        bool result = _encoders->getEncodersTimed(q, timestamp);
        
        if (result) {
            //I'm intereste only in two joints
            yarp::sig::Vector myQ(2, 0.0);
            myQ[0] = convertDegToRad(q[2]);
            myQ[1] = convertDegToRad(q[3]);
            
            AWPolyElement element(myQ, timestamp[0]);
            yarp::sig::Vector dq = _velocityEstimator->estimate(element);
            positions(0) = myQ(0);
            positions(1) = myQ(1);
            
            velocities(0) = dq(0);
            velocities(1) = dq(1);
            
        }
        return result;
        
        //        AWPolyElement el;
        //        el.data = q;
        //        el.time = qStamps[0];
        
        //        bool icubWholeBodySensors::readEncoders(double *q, double *stamps, bool wait)
        //        {
        //            double qTemp[MAX_NJ], tTemp[MAX_NJ];
        //            bool res = true, update=false;
        //            int i=0;
        //            FOR_ALL_BODY_PARTS_OF(itBp, encoderIdList)
        //            {
        //                assert(ienc[itBp->first]!=NULL);
        //                // read encoders
        //                while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
        //                    Time::delay(WAIT_TIME);
        //
        //                // if read succeeded => update data
        //                if(update)
        //                    for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
        //                    {
        //                        // joints 0 and 2 of the torso are swapped
        //                        qLastRead[itBp->first][itBp->first==TORSO ? 2-i : i]        = CTRL_DEG2RAD*qTemp[i];
        //                        qStampLastRead[itBp->first][itBp->first==TORSO ? 2-i : i]   = tTemp[i];
        //                    }
        //
        //                // copy most recent data into output variables
        //                FOR_ALL_JOINTS(itBp, itJ)
        //                {
        //                    q[i] = qLastRead[itBp->first][*itJ];
        //                    if(stamps!=NULL)
        //                        stamps[i] = qStampLastRead[itBp->first][*itJ];
        //                    i++;
        //                }
        //                res = res && update;    // if read failed => return false
        //            }
        //            return res || wait;
        //        }

    }
    
    void AdaptiveControlThread::writeOutputs()
    {
        
        Bottle& torqueBottle = _torqueOutput->prepare();
        torqueBottle.clear();
        torqueBottle.write(_outputTau);
        _torqueOutput->write();
    }
}
