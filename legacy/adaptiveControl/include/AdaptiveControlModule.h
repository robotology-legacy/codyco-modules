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

#ifndef ADAPTIVECONTROLMODULE_H
#define ADAPTIVECONTROLMODULE_H

#include <string>
#include <yarp/os/RFModule.h>
#include "AdaptiveControlConstants.h"
#include <Eigen/Core>


namespace paramHelp {
    class ParamHelperServer;
    class ParamHelperClient;
}

namespace adaptiveControl
{
    class AdaptiveControlThread;
    
    class AdaptiveControlModule: public yarp::os::RFModule, public paramHelp::CommandObserver
    {
        /* module parameters */
        std::string _moduleName;
        std::string _robotName;
        std::string _robotPart;
        
        int _period;
        
        double _baselineSmootherDuration;
        double _frequencySmootherDuration;
        
        Eigen::Vector2d _linkLengths;
        Eigen::Vector8d _initialPiHat;
        double _initialXi1;
        
        yarp::os::Port _rpcPort;        // a port to handle rpc messages
        AdaptiveControlThread *_controlThread;
        paramHelp::ParamHelperServer* _parameterServer;    // helper class for communication
#ifndef ADAPTIVECONTROL_TORQUECONTROL
        paramHelp::ParamHelperClient* _parameterClient;
        std::string _torqueControlModuleName;
#endif
        
    public:
        AdaptiveControlModule();
        
        bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
        bool interruptModule();                       // interrupt, e.g., the ports
        bool close();                                 // close and shut down the module
        bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
        double getPeriod(){ return 0.1; }
        bool updateModule();
        
        void commandReceived(const paramHelp::CommandDescription &cd, const yarp::os::Bottle &params, yarp::os::Bottle &reply);
        
    };
    
}

#endif
