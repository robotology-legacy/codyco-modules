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
#include "AdaptiveControlModule.h"

#include "AdaptiveControlConstants.h"
#include "AdaptiveControlThread.h"

#include <cstdio>
#include <string>

using namespace yarp::os;
using namespace paramHelp;

namespace adaptiveControl {
    AdaptiveControlModule::AdaptiveControlModule()
    {
        
    }
    
    bool AdaptiveControlModule::configure(ResourceFinder &rf)
    {
        //-------------------------------------------------- PARAMETER HELPER SERVER ---------------------------------------------------------
        _parameterServer = new ParamHelperServer(adaptiveControlParamDescriptors, adaptiveControlParamDescriptorsSize,
                                                 adaptiveControlCommandDescriptors, adaptiveControlCommandDescriptorsSize);
        if (_parameterServer) {
            error_out("Could not initialize parameter server. Closing module");
            return false;
        }
        _parameterServer->linkParam(AdaptiveControlParamIDModuleName, &_moduleName);
        _parameterServer->linkParam(AdaptiveControlParamIDPeriod, &_period);
        _parameterServer->linkParam(AdaptiveControlParamIDRobotName, &_robotName);
        
        _parameterServer->registerCommandCallback(AdaptiveControlCommandIDHelp, this);
        _parameterServer->registerCommandCallback(AdaptiveControlCommandIDQuit, this);
        
        // Read parameters from configuration file (or command line)
        Bottle initMsg;
        _parameterServer->initializeParams(rf, initMsg);
        info_out("*** Parsing configuration file...\n%s\n", initMsg.toString().c_str());
        
        // Open ports for communicating with other modules
        if(!_parameterServer->init(_moduleName)) {
            error_out("Error while initializing parameter server. Closing module.\n");
            return false;
        }
        
        _rpcPort.open(("/" + _moduleName + "/rpc").c_str());
        setName(_moduleName.c_str());
        attach(_rpcPort);
        initMsg.clear();
        
        //--------------------------CONTROL THREAD--------------------------
        _controlThread = new AdaptiveControlThread(_moduleName, _robotName, _period, *_parameterServer);
        if (!_controlThread || !_controlThread->start()) {
            error_out("Error while initializing control thread. Closing module.\n");
            return false;
        }
        
        info_out("Control torque module started\n");
        return true;
    }
    
    bool AdaptiveControlModule::respond(const Bottle& cmd, Bottle& reply)
    {
        _parameterServer->lock();
        if(!_parameterServer->processRpcCommand(cmd, reply))
            reply.addString((std::string("Command ")+ cmd.toString().c_str() + " not recognized.").c_str());
        _parameterServer->unlock();
        
        // if reply is empty put something into it, otherwise the rpc communication gets stuck
        if(reply.size()==0)
            reply.addString((std::string("Command ") + cmd.toString().c_str() + " received.").c_str());
        return true;
    }
    
    void AdaptiveControlModule::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
    {
        switch(cd.id)
        {
            case AdaptiveControlCommandIDHelp:
                _parameterServer->getHelpMessage(reply);
                break;
            case AdaptiveControlCommandIDQuit:
                stopModule();
                reply.addString("Quitting module.");
                break;
        }
    }
    
    bool AdaptiveControlModule::interruptModule()
    {
        if(_controlThread)  _controlThread->stop();
        _rpcPort.interrupt();
        return true;
    }
    
    bool AdaptiveControlModule::close()
    {
        //stop threads
        if(_controlThread) {
            _controlThread->stop();
            delete _controlThread;
            _controlThread = NULL;
        }
        if(_parameterServer) {
            _parameterServer->close();
            delete _parameterServer;
            _parameterServer = NULL;
        }
        
        //closing ports
        _rpcPort.close();
        info_out("about to close\n");
        return true;
    }
    
    bool AdaptiveControlModule::updateModule()
    {
        if (!_controlThread) {
            error_out("%s: Error. Control thread pointer is zero.\n", _moduleName.c_str());
            return false;
        }
        double periodMean = 0, periodStdDeviation = 0;
        double usedMean = 0, usedStdDeviation = 0;
        
        _controlThread->getEstPeriod(periodMean, periodStdDeviation);
        _controlThread->getEstUsed(usedMean, usedStdDeviation);
        
        if(periodMean > 1.3 * _period)
        {
            info_out("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", periodMean, periodStdDeviation, _period);
            info_out("Duration of 'run' method: %3.3f+/-%3.3f.\n", usedMean, usedStdDeviation);
        }
        
        return true;
    }
}
