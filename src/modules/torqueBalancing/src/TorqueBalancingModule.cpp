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

#include "TorqueBalancingModule.h"
#include "TorqueBalancingController.h"
#include "Reference.h"
#include "ReferenceGenerator.h"
#include "ReferenceGeneratorInputReaderImpl.h"
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Port.h>
#include <vector>

#include <paramHelp/paramHelperServer.h>
#include "ParamHelperConfig.h"

#ifdef DEBUG
#include <codyco/Utils.h>
#include <iostream>
#endif

namespace codyco {
    namespace torquebalancing {
        
        //Utility structure used inside the module
        struct TaskInformation {
            TaskType taskType;
            std::string referredLinkName;
            Reference& reference;
        };
        
        
        TorqueBalancingModule::TorqueBalancingModule()
        : m_moduleState(TorqueBalancingModuleStateDoubleSupportStable)
        , m_referenceThreadPeriod(10)
        , m_controllerThreadPeriod(10)
        , m_active(false)
        , m_robot(0)
        , m_controller(0)
        , m_references(0)
        , m_parameterServer(0)
        , m_rpcPort(0)
        , m_paramHelperManager(0) {}
        
        TorqueBalancingModule::~TorqueBalancingModule() { cleanup(); }
        
        bool TorqueBalancingModule::configure(yarp::os::ResourceFinder& rf)
        {
            //PARAMETERS SECTION
            //Creating parameter server
            m_parameterServer = new paramHelp::ParamHelperServer(TorqueBalancingModuleParameterDescriptions,
                                                                 TorqueBalancingModuleParameterSize,
                                                                 TorqueBalancingModuleCommandDescriptions,
                                                                 TorqueBalancingModuleCommandSize);
            if (!m_parameterServer) {
                return false;
            }
            
            bool linkedVariable = true;
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterModuleName, &m_moduleName);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterRobotName, &m_robotName);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterPeriod, &m_controllerThreadPeriod);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterURDFFilePath, &m_urdfFilePath);
            
            if (!linkedVariable) {
                return false;
            }
            
            yarp::os::Bottle replyBottle;
            m_parameterServer->initializeParams(rf, replyBottle);
            
            if (!m_parameterServer->init(m_moduleName)) {
                return false;
            }
            
            //END PARAMETER SECTION
            
            m_rpcPort = new yarp::os::Port();
            if (!m_rpcPort
                || !m_rpcPort->open(("/" + m_moduleName + "/rpc").c_str())) {
                return false;
            }
            setName(m_moduleName.c_str());
            attach(*m_rpcPort);
            
            //Create reference variable
            m_references = new ControllerReferences();
            if (!m_references) {
                return false;
            }
            
            //create reference to wbi
            if (m_urdfFilePath.empty()) {
                m_robot = new wbiIcub::icubWholeBodyInterface(m_moduleName.c_str(), m_robotName.c_str(), iCub::iDynTree::iCubTree_version_tag());
#ifdef DEBUG
                std::cerr << "Initializing wbi with default iCub Model" << std::endl;
#endif
            } else {
                m_robot = new wbiIcub::icubWholeBodyInterface(m_moduleName.c_str(), m_robotName.c_str(), iCub::iDynTree::iCubTree_version_tag(), m_urdfFilePath);
#ifdef DEBUG
                std::cerr << "Initializing wbi with URDF model specified in" << m_urdfFilePath << std::endl;
#endif
            }
            if (!m_robot) {
                return false;
            }
            //add joints
            m_robot->addJoints(wbiIcub::ICUB_MAIN_JOINTS);
            if (!m_robot->init()) {
                return false;
            }
            
            //create generators
            ReferenceGeneratorInputReader* reader = 0;
            ReferenceGenerator* generator = 0;
            
            reader = new COMReader(*m_robot);
            if (reader) {
                m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(TaskTypeCOM, reader));
            } else {
                return false;
            }
            
            generator = new ReferenceGenerator(m_referenceThreadPeriod, m_references->desiredCOMAcceleration(), *reader);
            if (generator) {
                m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(TaskTypeCOM, generator));
            } else {
                return false;
            }
            
            //position tasks
            std::vector<TaskInformation> tasks;
            TaskInformation task1 = {TaskTypeLeftHandPosition, "l_gripper", m_references->desiredLeftHandPosition()};
            tasks.push_back(task1);
            TaskInformation task2 = {TaskTypeRightHandPosition, "r_gripper", m_references->desiredRightHandPosition()};
            tasks.push_back(task2);
            
            for (std::vector<TaskInformation>::iterator it = tasks.begin(); it != tasks.end(); it++) {
                reader = new EndEffectorPositionReader(*m_robot, it->referredLinkName);
                if (reader) {
                    m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(it->taskType, reader));
                } else {
                    return false;
                }
                generator = new ReferenceGenerator(m_referenceThreadPeriod, it->reference, *reader);
                if (generator) {
                    m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(it->taskType, generator));
                } else {
                    return false;
                }
            }
            
            //force tasks
            tasks.clear();
            TaskInformation task3 = {TaskTypeLeftHandForce, "l_gripper", m_references->desiredLeftHandPosition()};
            tasks.push_back(task1);
            TaskInformation task4 = {TaskTypeRightHandForce, "r_gripper", m_references->desiredRightHandPosition()};
            tasks.push_back(task2);
            
            for (std::vector<TaskInformation>::iterator it = tasks.begin(); it != tasks.end(); it++) {
                reader = new EndEffectorForceReader(*m_robot);//, it->referredLinkName);
                if (reader) {
                    m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(it->taskType, reader));
                } else {
                    return false;
                }
                generator = new ReferenceGenerator(m_referenceThreadPeriod, it->reference, *reader);
                if (generator) {
                    m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(it->taskType, generator));
                } else {
                    return false;
                }
            }
            
            m_controller = new TorqueBalancingController(m_controllerThreadPeriod, *m_references, *m_robot);
            if (!m_controller) {
                return false;
            }
            
            //link controller and references variables to param helper manager
            m_paramHelperManager = new ParamHelperManager(*this, *m_controller, m_referenceGenerators, *m_parameterServer);
            if (!m_paramHelperManager || !m_paramHelperManager->linkVariables() || !m_paramHelperManager->registerCommandCallbacks()) {
                return false;
            }
            
            bool threadsStarted = true;
            
            for (std::map<TaskType, ReferenceGenerator*>::iterator it = m_referenceGenerators.begin(); it != m_referenceGenerators.end(); it++) {
                threadsStarted = threadsStarted && it->second->start();
            }
            
            threadsStarted = threadsStarted && m_controller->start();
            
            return threadsStarted;
        }
        
        bool TorqueBalancingModule::updateModule()
        {
            if (!m_controller) {
//                error_out("%s: Error. Control thread pointer is zero.\n", _moduleName.c_str());
                return false;
            }
            double periodMean = 0, periodStdDeviation = 0;
            double usedMean = 0, usedStdDeviation = 0;
            
            m_controller->getEstPeriod(periodMean, periodStdDeviation);
            m_controller->getEstUsed(usedMean, usedStdDeviation);
            
            if (periodMean > 1.3 * m_controllerThreadPeriod) {
//                info_out("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", periodMean, periodStdDeviation, m_controllerThreadPeriod);
//                info_out("Duration of 'run' method: %3.3f+/-%3.3f.\n", usedMean, usedStdDeviation);
            }
            return true;
        }
        
        bool TorqueBalancingModule::close()
        {
            cleanup();
            return true;
        }
        
        void TorqueBalancingModule::cleanup()
        {
            if (m_paramHelperManager) {
                delete m_paramHelperManager;
                m_paramHelperManager = 0;
            }
            
            //Close parameter server
            if (m_parameterServer) {
                m_parameterServer->close();
                delete m_parameterServer;
                m_parameterServer = 0;
            }
            
            if (m_rpcPort) {
                m_rpcPort->close();
                delete m_rpcPort;
                m_rpcPort = 0;
            }
            
            //close controller thread
            if (m_controller) {
                m_controller->stop();
                delete m_controller;
                m_controller = 0;
            }
            
            //close trajectory generators
            for (std::map<TaskType, ReferenceGenerator*>::iterator it = m_referenceGenerators.begin(); it != m_referenceGenerators.end(); it++) {
                it->second->stop();
                delete it->second;
            }
            m_referenceGenerators.clear();
            
            for (std::map<TaskType, ReferenceGeneratorInputReader*>::iterator it = m_generatorReaders.begin(); it != m_generatorReaders.end(); it++) {
                delete it->second;
            }
            m_generatorReaders.clear();
            
            //clear the other variables
            if (m_robot) {
                m_robot->close();
                delete m_robot;
                m_robot = 0;
            }
            
            if (m_references) {
                delete m_references;
                m_references = 0;
            }
        }
        
        void TorqueBalancingModule::setControllersActiveState(bool isActive)
        {
            if (isActive == m_active) return;
            m_active = isActive;
#ifdef DEBUG
            std::cerr << FUNCTION_NAME << ": Module new state is " << (m_active ? "on" : "off") << std::endl;
#endif
            updateModuleCoordinationStatus();
        }
        
        void TorqueBalancingModule::updateModuleCoordinationStatus()
        {
            bool comTaskActive = m_active;
            bool leftHandPositionTaskActive = false;
            bool rightHandPositionTaskActive = false;
            bool leftHandForceTaskActive = false;
            bool rightHandForceTaskActive = false;
            
            switch (m_moduleState) {
                case TorqueBalancingModuleStateDoubleSupportSeekingContactBothHands:
                    leftHandPositionTaskActive = m_active;
                    rightHandForceTaskActive = m_active;
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State Double support with hands" << std::endl;
#endif
                    break;
                case TorqueBalancingModuleStateTripleSupportSeekingContactLeftHand:
                    leftHandPositionTaskActive = m_active;
                    rightHandForceTaskActive = m_active;
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State Triple support. Left hand searches contact" << std::endl;
#endif
                    break;
                case TorqueBalancingModuleStateTripleSupportSeekingContactRightHand:
                    rightHandPositionTaskActive = m_active;
                    leftHandForceTaskActive = m_active;
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State Triple support. Right hand searches contact" << std::endl;
#endif
                    break;
                case TorqueBalancingModuleStateQuadrupleSupport:
                    leftHandForceTaskActive = rightHandForceTaskActive = m_active;
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State Quad support" << std::endl;
#endif
                    break;
                case TorqueBalancingModuleStateDoubleSupportStable:
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State Double support" << std::endl;
#endif

                    break;
                default:
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State not recognized" << std::endl;
#endif
                    break;
            }
            
            bool controlSet = true;
            std::map<TaskType, ReferenceGenerator*>::iterator found;
            bool tasksState[5] = {comTaskActive, leftHandPositionTaskActive, rightHandPositionTaskActive, leftHandForceTaskActive, rightHandForceTaskActive};
            TaskType tasksType[5] = { TaskTypeCOM, TaskTypeLeftHandPosition, TaskTypeRightHandPosition, TaskTypeLeftHandForce, TaskTypeRightHandForce};
            
            for (int i = 0; i < 5; i++) {
                found = m_referenceGenerators.find(tasksType[i]);
                controlSet = controlSet && found != m_referenceGenerators.end();
                if (controlSet)
                    found->second->setActiveState(tasksState[i]);
            }
            m_controller->setActiveState(m_active && controlSet);
        }
        
#pragma mark - ParamHelperManager methods
        
        TorqueBalancingModule::ParamHelperManager::ParamHelperManager(TorqueBalancingModule& module, TorqueBalancingController& controller, std::map<TaskType, ReferenceGenerator*>& generators, paramHelp::ParamHelperServer& parameterServer)
        : m_module(module)
        , m_controller(controller)
        , m_referenceGenerators(generators)
        , m_parameterServer(parameterServer) {}
        
        TorqueBalancingModule::ParamHelperManager::~ParamHelperManager() {}
        
        bool TorqueBalancingModule::ParamHelperManager::linkVariables()
        {
            bool linked = true;
            
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterCurrentState, &m_module.m_moduleState);
            //COM
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterCOMProportionalGain, m_comProportionalGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterCOMDerivativeGain, m_comDerivativeGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterCOMIntegralGain, m_comIntegralGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterCOMIntegralLimit, &m_comIntegralLimit);
            //Hands position
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsPositionDerivativeGain, m_handsPositionProportionalGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsPositionDerivativeGain, m_handsPositionDerivativeGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsPositionIntegralGain, m_handsPositionIntegralGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsPositionIntegralLimit, &m_handsPositionIntegralLimit);
            //Hands forces
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsForceProportionalGain, m_handsForceProportionalGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsForceDerivativeGain, m_handsForceDerivativeGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsForceIntegralGain, m_handsForceIntegralGain.data());
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterHandsForceIntegralLimit, &m_handsForceIntegralLimit);
            //Centroidal momentum
            linked = linked && m_parameterServer.linkParam(TorqueBalancingModuleParameterCentroidalGain, &m_centroidalGain);
            
            return linked;
        }
        
        bool TorqueBalancingModule::ParamHelperManager::registerCommandCallbacks()
        {
            bool commandRegistered = true;
            commandRegistered = commandRegistered && m_parameterServer.registerCommandCallback(TorqueBalancingModuleCommandStart, this);
            commandRegistered = commandRegistered && m_parameterServer.registerCommandCallback(TorqueBalancingModuleCommandStop, this);
            commandRegistered = commandRegistered && m_parameterServer.registerCommandCallback(TorqueBalancingModuleCommandQuit, this);
            commandRegistered = commandRegistered && m_parameterServer.registerCommandCallback(TorqueBalancingModuleCommandHelp, this);
            
            return commandRegistered;
        }
        
        void TorqueBalancingModule::ParamHelperManager::parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface)
        {
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            switch (proxyInterface->id) {
                case TorqueBalancingModuleParameterCurrentState:
                    m_module.updateModuleCoordinationStatus();
                    break;
                    //COM
                case TorqueBalancingModuleParameterCOMProportionalGain:
                    foundController = m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_comProportionalGain);
                    }
                    break;
                case TorqueBalancingModuleParameterCOMDerivativeGain:
                    foundController = m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_comDerivativeGain);
                    }
                    break;
                case TorqueBalancingModuleParameterCOMIntegralGain:
                    foundController = m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_comIntegralGain);
                    }
                    break;
                case TorqueBalancingModuleParameterCOMIntegralLimit:
                    foundController = m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_comIntegralLimit);
                    }
                    break;
                // Hands position
                case TorqueBalancingModuleParameterHandsPositionProportionalGain:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsPositionProportionalGain);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsPositionProportionalGain);
                    }
                    break;
                case TorqueBalancingModuleParameterHandsPositionDerivativeGain:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsPositionDerivativeGain);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsPositionDerivativeGain);
                    }

                    break;
                case TorqueBalancingModuleParameterHandsPositionIntegralGain:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsPositionIntegralGain);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsPositionIntegralGain);
                    }
                    break;
                case TorqueBalancingModuleParameterHandsPositionIntegralLimit:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsPositionIntegralLimit);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsPositionIntegralLimit);
                    }
                    break;
                //Hands forces
                case TorqueBalancingModuleParameterHandsForceProportionalGain:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsForceProportionalGain);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsForceProportionalGain);
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceDerivativeGain:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsForceDerivativeGain);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsForceDerivativeGain);
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceIntegralGain:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsForceIntegralGain);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsForceIntegralGain);
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceIntegralLimit:
                    foundController = m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsForceIntegralLimit);
                    }
                    foundController = m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsForceIntegralLimit);
                    }
                    break;
                //Centroidal
                case TorqueBalancingModuleParameterCentroidalGain:
                    m_controller.setCentroidalMomentumGain(m_centroidalGain);
                    break;
            }
        }
        
        void TorqueBalancingModule::ParamHelperManager::commandReceived(const paramHelp::CommandDescription& commandDescription, const yarp::os::Bottle& params, yarp::os::Bottle& reply)
        {
            switch (commandDescription.id) {
                case TorqueBalancingModuleCommandStart:
                    m_module.setControllersActiveState(true);
                    reply.addString("Controller actived");
                    break;
                case TorqueBalancingModuleCommandStop:
                    m_module.setControllersActiveState(false);
                    reply.addString("Controller stopped");
                    break;
                case TorqueBalancingModuleCommandQuit:
                    m_module.stopModule();
                    reply.addString("Quitting module");
                    break;
                case TorqueBalancingModuleCommandHelp:
                    m_parameterServer.getHelpMessage(reply);
                    break;
                default:
                    break;
            }
            
        }
    }
}
