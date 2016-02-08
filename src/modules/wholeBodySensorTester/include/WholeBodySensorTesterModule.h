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
/*
namespace yarp {
    namespace os {
        class Port;
        class Property;

        template <typename T>
        class BufferedPort;
    }

    namespace dev {
        class Pid;
    }
}*/

// namespace codyco {
//     class PIDList;
// 
//     namespace torquebalancing {
// 
//         class ControllerReferences;
//         class TorqueBalancingController;
//         class ControllerDelegate;
//         class ReferenceGenerator;
//         class ReferenceGeneratorInputReader;
// 
// 
//         /** Possible tasks */
//         typedef enum {
//             TaskTypeUnknown, /*!< Default value */
//             TaskTypeCOM, /*!< Center of Mass control task */
//             TaskTypeImpedanceControl,  /*!< Impedance control (same meaning of postural task) */
//         } TaskType;
// 
//         extern const std::string TorquePIDInitialKey; /*!< Key used to save original torque PIDs */
//         extern const std::string TorquePIDDefaultKey; /*!< Key used to save default torque PIDs */


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
//             virtual bool close();
//             virtual double getPeriod();
            virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

            /** Cleanups all the variable and threads managed by this module
             */
            void cleanup();

        private:
            class ParamHelperManager;
            //TODO: move the private part to use a pimpl pattern

            std::string m_moduleName;
            std::string m_robotName;

            // wbi::iWholeBodySensors *wbS;
            yarpWbi::yarpWholeBodySensors *wbS;
            
        protected:
            yarp::os::Port      rpcPort;
            std::string         name;
            std::string         contextPath;
            bool                verbose;    
            WholeBodySensorTesterThread wholeBodySensorTesterThread;

        };
/*
        class TorqueBalancingModule::ParamHelperManager : public
        paramHelp::ParamValueObserver,
        paramHelp::CommandObserver {

            TorqueBalancingModule& m_module;

            bool m_initialized;
            paramHelp::ParamHelperServer* m_parameterServer;

            Eigen::VectorXd m_comProportionalGain;
            Eigen::VectorXd m_comDerivativeGain;
            Eigen::VectorXd m_comIntegralGain;
            double m_comIntegralLimit;

            Eigen::VectorXd m_torqueSaturation;

            double m_centroidalGain;
            Eigen::VectorXd m_impedanceControlGains;

            //Monitored variables
            Eigen::VectorXd m_monitoredDesiredCOMAcceleration;
            Eigen::VectorXd m_monitoredCOMError;
            Eigen::VectorXd m_monitoredCOMIntegralError;
            Eigen::VectorXd m_monitoredFeetForces;
            Eigen::VectorXd m_monitoredOutputTorques;

        public:
            ParamHelperManager(TorqueBalancingModule& module, int actuatedDOFs);

            bool init(yarp::os::ResourceFinder& resourceFinder);
            bool linkVariables();
            bool linkMonitoredVariables();
            bool registerCommandCallbacks();
            void loadDefaultVariables();

            bool processRPCCommand(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
            void sendMonitoredVariables();
            void syncLinkedVariables();

            virtual ~ParamHelperManager();
            virtual void parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface);
            virtual void commandReceived(const paramHelp::CommandDescription& commandDescription, const yarp::os::Bottle& params, yarp::os::Bottle& reply);
        };*/
// }


#endif /* end of include guard: WHOLEBODYSENSORTESTERMODULE_H */
