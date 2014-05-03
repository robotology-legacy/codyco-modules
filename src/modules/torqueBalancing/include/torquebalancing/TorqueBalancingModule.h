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
    class Frame;
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
        
        typedef enum {
            TaskTypeCOM,
            TaskTypeHandsPosition,
            TaskTypeHandsForce
        } TaskType;
        
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
            
            void cleanup();
            
        private:
            
            class ParamHelperManager;
            
            int m_referenceThreadPeriod;
            int m_controllerThreadPeriod;
            
            std::string m_moduleName;
            std::string m_robotName;
            
            wbi::wholeBodyInterface* m_robot;
            wbi::Frame* m_world2BaseFrame;
            
            TorqueBalancingController* m_controller;
            ControllerReferences* m_references;
            
            std::map<TaskType, ReferenceGeneratorInputReader*> m_generatorReaders;
            std::map<TaskType, ReferenceGenerator*> m_referenceGenerators;
            
            paramHelp::ParamHelperServer* m_parameterServer;
            
            yarp::os::Port* m_rpcPort;
            
            ParamHelperManager* m_paramHelperManager;
            
        };
        
        class TorqueBalancingModule::ParamHelperManager : public paramHelp::ParamValueObserver {
            
            TorqueBalancingController& m_controller;
            std::map<TaskType, ReferenceGenerator*>& m_referenceGenerators;
            paramHelp::ParamHelperServer& m_parameterServer;
            
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
            
        public:
            ParamHelperManager(TorqueBalancingController& controller, std::map<TaskType, ReferenceGenerator*>& generators, paramHelp::ParamHelperServer& parameterServer);
            
            bool linkVariables();
                        
            virtual ~ParamHelperManager();
            virtual void parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface);
        };
    }
}

#endif /* end of include guard: TORQUEBALANCINGMODULE_H */
