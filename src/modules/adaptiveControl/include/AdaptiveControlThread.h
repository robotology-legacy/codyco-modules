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

/**
 *
 @ingroup codyco_module
 Copyright (C) 2014 CoDyCo Project
 
 CopyPolicy: Released under the terms of the GNU GPL v2.0.
 
 **/

#ifndef ADAPTIVECONTROLTHREAD_H
#define ADAPTIVECONTROLTHREAD_H

#include "AdaptiveControlConstants.h"
#include <yarp/os/RateThread.h>
#include <string>

#include <Eigen/Core>

namespace paramHelp {
    class ParamHelperServer;
}

namespace yarp {
    namespace dev {
        class PolyDriver;
        class IEncoders;
    }
    namespace os {
        class Bottle;
        template <class T> class BufferedPort;
    }
}

namespace adaptiveControl {
    
    class AdaptiveControlThread: public yarp::os::RateThread {
        
    private:
        //configuration parameters
        std::string &_threadName;
        std::string &_robotName;
        paramHelp::ParamHelperServer &_paramServer;
        
        //in-out variables
        yarp::dev::PolyDriver* _encodersDriver;
        yarp::dev::IEncoders* _encoders;
        yarp::os::BufferedPort<yarp::os::Bottle> *_torqueControl;
        
        double _minDeterminantValue;
        
        //reference trajectory: for now i compute it internally. In the future we can read from a port
        double _refBaseline;
        double _refFrequency; //in rad/s
        double _refAmplitude;
        double _refPhase; //in rad
        
        //geometric parameters
        double _link1Length;
        double _link2Length;
        
        //gains
        double _lambda;
        Eigen::Vector2d _kappa;
        Eigen::Matrix8d _Gamma;
        
        //Variables for computing control
        Eigen::Vector2d _q;
        Eigen::Vector2d _dq;
        Eigen::Vector2d _xi;
        Eigen::Vector8d _pi;
        
        //Streaming output parameters
        Eigen::VectorNd _outputTau;
        
        yarp::dev::PolyDriver* openDriver(std::string localName, std::string robotName, std::string bodyPartName);
        void computeRegressor(const Eigen::Vector2d& q, /* Joint positions*/
                                          const Eigen::Vector2d& dq, /* Joint velocities*/
                                          const Eigen::Vector2d& dq_lin, /* Joint velocities. This is the term which multiplies linearly the C(.) term */
                                          const Eigen::Vector2d& ddq, /* Joint accelerations*/
                              Eigen::Matrix28d& regressor); /* output variable */
    public:
        AdaptiveControlThread(std::string& threadName,
                              std::string& robotName,
                              int periodMilliseconds,
                              paramHelp::ParamHelperServer&paramHelperServer,
                              const Eigen::Vector2d &linklengths);
        ~AdaptiveControlThread();
        
        /* Overrided functions (Rate Thread)*/
        bool threadInit();
        void run();
        void threadRelease();
        
    };
}

#endif
