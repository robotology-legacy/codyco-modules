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

#ifndef WHOLEBODYSENSORTESTERMODULE_H
#define WHOLEBODYSENSORTESTERMODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/os/PortReaderBuffer.h>
#include <paramHelp/paramProxyInterface.h>

#include <map>
#include <string>
#include <Eigen/Core>

#include <wbi/iWholeBodySensors.h>
#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include "WholeBodySensorTesterThread.h"
/*
namespace paramHelp {
    class ParamHelperServer;
}
*/

namespace yarpWbi {
    class yarpWholeBodySensors;
}

namespace yarp {
    namespace os
    {
        class ResourceFinder;
    }
}

/** @brief Main module for the wholeBodyState Tester module.
*
*/
class WholeBodySensorTesterModule : public yarp::os::RFModule
{
public:
    WholeBodySensorTesterModule();
    virtual ~WholeBodySensorTesterModule();
    virtual bool configure(yarp::os::ResourceFinder& resourceFinder);
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /**
    * Cleansups all the variable and threads managed by this module
    */
    void cleanup();

    private:
        class ParamHelperManager;
        bool setupWholeBodyState();


        std::string m_moduleName;
        std::string m_robotName;
        yarp::os::ResourceFinder rf;

        yarpWbi::yarpWholeBodySensors *wbS;
    protected:
        yarp::os::Port      rpcPort;
        std::string         name;
        std::string         contextPath;
        bool                verbose;
        WholeBodySensorTesterThread wholeBodySensorTesterThread;
};

#endif /* end of include guard: WHOLEBODYSENSORTESTERMODULE_H */
