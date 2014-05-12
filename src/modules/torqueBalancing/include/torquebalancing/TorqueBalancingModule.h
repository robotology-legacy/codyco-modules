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

#include <yarp/os/RFModule.h>
#include <map>
#include <string>
#include <paramHelp/paramProxyInterface.h>
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
    }
}

namespace codyco {
    namespace torquebalancing {
        
        class ControllerReferences;
        class TorqueBalancingController;
        class ReferenceGenerator;
        class ReferenceGeneratorInputReader;
        
        /** Possible tasks */
        typedef enum {
            TaskTypeCOM, /*!< Center of Mass control task */
            TaskTypeLeftHandPosition, /*!< Left hand position control task */
            TaskTypeRightHandPosition,  /*!< Right hand position control task */
            TaskTypeLeftHandForce,  /*!< Left hand force control task */
            TaskTypeRightHandForce  /*!< Right hand force control task */
        } TaskType;
        
        /** Defines the possible states for the module
         */
        typedef enum {
            TorqueBalancingModuleStateDoubleSupportStable = 0x1, /*!< Robot is standing on its feet, i.e. only COM/Feet forces task */
            TorqueBalancingModuleStateDoubleSupportSeekingContactBothHands = 0x2, /*!< Robot is standing on its feet, and it seeks contacts with both hands */
            TorqueBalancingModuleStateTripleSupportSeekingContactRightHand = 0x4, /*!< Robot is standing on its feet and keep contact with the left hand. The right hand is seeking contact */
            TorqueBalancingModuleStateTripleSupportSeekingContactLeftHand = 0x8, /*!< Robot is standing on its feet and keep contact with the right hand. The left hand is seeking contact */
            TorqueBalancingModuleStateQuadrupleSupport = 0x10 /*!< Robot balances with both its feet and both its hands */
        } TorqueBalancingModuleState;
        
        
        /** @brief Main module for the torque balancing module.
         *
         */
        class TorqueBalancingModule : public yarp::os::RFModule
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
            
        private:
            class ParamHelperManager;
            
            void updateModuleCoordinationStatus();
            
            TorqueBalancingModuleState m_moduleState;
            int m_controllerThreadPeriod;
            double m_modulePeriod;
            bool m_active;
            
            std::string m_moduleName;
            std::string m_robotName;
            
            wbi::wholeBodyInterface* m_robot;
            
            TorqueBalancingController* m_controller;
            ControllerReferences* m_references;
            
            std::map<TaskType, ReferenceGeneratorInputReader*> m_generatorReaders;
            std::map<TaskType, ReferenceGenerator*> m_referenceGenerators;
            
            yarp::os::Port* m_rpcPort;
            
            ParamHelperManager* m_paramHelperManager;
            
        };
        
        class TorqueBalancingModule::ParamHelperManager : public
        paramHelp::ParamValueObserver,
        paramHelp::CommandObserver {
            
            TorqueBalancingModule& m_module;
            
            bool m_initialized;
            paramHelp::ParamHelperServer* m_parameterServer;
            
            Eigen::VectorXd m_comReference;
            Eigen::VectorXd m_handsPositionReference;
            Eigen::VectorXd m_handsForceReference;
            
            Eigen::VectorXd m_comProportionalGain;
            Eigen::VectorXd m_comDerivativeGain;
            Eigen::VectorXd m_comIntegralGain;
            double m_comIntegralLimit;
            
            Eigen::VectorXd m_handsPositionProportionalGain;
            Eigen::VectorXd m_handsPositionDerivativeGain;
            Eigen::VectorXd m_handsPositionIntegralGain;
            double m_handsPositionIntegralLimit;
            
            Eigen::VectorXd m_handsForceProportionalGain;
            Eigen::VectorXd m_handsForceDerivativeGain;
            Eigen::VectorXd m_handsForceIntegralGain;
            double m_handsForceIntegralLimit;
            
            double m_centroidalGain;
            Eigen::VectorXd m_impedanceControlGains;
            
            //Monitored variables
            Eigen::VectorXd m_monitoredDesiredCOMAcceleration;
            Eigen::VectorXd m_monitoredCOMError;
            Eigen::VectorXd m_monitoredCOMIntegralError;
            Eigen::VectorXd m_monitoredFeetForces;
            Eigen::VectorXd m_monitoredOutputTorques;
            
            void sendReferencesToControllers();
            
        public:
            ParamHelperManager(TorqueBalancingModule& module);
            
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
