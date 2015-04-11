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

#ifndef TORQUEBALANCINGMODULE_H
#define TORQUEBALANCINGMODULE_H

#include "Reference.h"
#include "TorqueBalancingController.h"

#include <yarp/os/RFModule.h>
#include <yarp/os/PortReaderBuffer.h>
#include <paramHelp/paramProxyInterface.h>

#include <map>
#include <string>
#include <Eigen/Core>

namespace paramHelp {
    class ParamHelperServer;
}


namespace wbi {
    class wholeBodyInterface;
}

namespace yarp {
    namespace os {
        class Port;
        class Property;

        template <typename T>
        class BufferedPort;
    }
}

namespace codyco {
    namespace torquebalancing {

        class ControllerReferences;
        class TorqueBalancingController;
        class ControllerDelegate;
        class ReferenceGenerator;
        class ReferenceGeneratorInputReader;

        /** Possible tasks */
        typedef enum {
            TaskTypeUnknown, /*!< Default value */
            TaskTypeCOM, /*!< Center of Mass control task */
            TaskTypeImpedanceControl,  /*!< Impedance control (same meaning of postural task) */
        } TaskType;


        /** @brief Main module for the torque balancing module.
         *
         */
        class TorqueBalancingModule : public yarp::os::RFModule,
        codyco::torquebalancing::ReferenceDelegate, codyco::torquebalancing::ControllerDelegate,
        public yarp::os::TypedReaderCallback<yarp::os::Bottle>
        {
        public:
            TorqueBalancingModule();
            virtual ~TorqueBalancingModule();

            virtual bool configure(yarp::os::ResourceFinder& resourceFinder);
            virtual bool updateModule();
            virtual bool close();
            virtual double getPeriod();
            virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

            /** Cleanups all the variable and threads managed by this module
             */
            void cleanup();

            /** Set the active state of all the threads managed by this module
             *
             * @param isActive the new active state for all the threads.
             */
            void setControllersActiveState(bool isActive);

            /** Updates and print the variables to be monitored
             */
            void monitorVariables();

            //Delegate method
            virtual void referenceDidChangeValue(const Reference&);
            virtual void controllerDidStop(TorqueBalancingController&);
            virtual void onRead(yarp::os::Bottle &read);

        private:
            class ParamHelperManager;
            //TODO: move the private part to use a pimpl pattern

            int m_controllerThreadPeriod;
            double m_modulePeriod;
            bool m_active;

            double m_forcesSmootherDuration;
            double m_jointsSmootherDuration;

            std::string m_moduleName;
            std::string m_robotName;

            wbi::wholeBodyInterface* m_robot;

            TorqueBalancingController* m_controller;
            ControllerReferences* m_references;

            std::map<TaskType, ReferenceGeneratorInputReader*> m_generatorReaders;
            std::map<TaskType, ReferenceGenerator*> m_referenceGenerators;

            yarp::os::Port* m_rpcPort;
            yarp::os::BufferedPort<yarp::os::Bottle>* m_constraintsPort;
            yarp::os::BufferedPort<yarp::os::Property>* m_eventsPort;

            ParamHelperManager* m_paramHelperManager;

            Eigen::VectorXd m_jointsConfiguration; /*!< Resting position of the impedance control */
            Eigen::VectorXd m_tempHeptaVector; /*!< Temporary vector of 7 elements */
            Eigen::VectorXd m_comReference; /*!< Reference for the center of mass */
        };

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
        };
    }
}

#endif /* end of include guard: TORQUEBALANCINGMODULE_H */
