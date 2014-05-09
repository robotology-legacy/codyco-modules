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

#include <codyco/ModelParsing.h>
#include <iostream>
#include <codyco/Utils.h>


namespace codyco {
    namespace torquebalancing {
        
        //Utility structure used inside the module
        struct TaskInformation {
            TaskType taskType;
            std::string referredLinkName;
            Reference* reference; //I cannot use a reference because object must be Assignable to be used inside std::vector
        };
        
        
        TorqueBalancingModule::TorqueBalancingModule()
        : m_moduleState(TorqueBalancingModuleStateDoubleSupportStable)
        , m_controllerThreadPeriod(10)
        , m_modulePeriod(1.0)
        , m_active(false)
        , m_robot(0)
        , m_controller(0)
        , m_references(0)
        , m_rpcPort(0)
        , m_paramHelperManager(0) {}
        
        TorqueBalancingModule::~TorqueBalancingModule() { cleanup(); }
        
        bool TorqueBalancingModule::configure(yarp::os::ResourceFinder& rf)
        {
            //PARAMETERS SECTION
            //Creating parameter server helper
            //link controller and references variables to param helper manager
            m_paramHelperManager = new ParamHelperManager(*this);
            if (!m_paramHelperManager || !m_paramHelperManager->init(rf)) {
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
            iCub::iDynTree::iCubTree_version_tag iCubVersion;
            codyco::iCubVersionFromRf(rf, iCubVersion);
            m_robot = new wbiIcub::icubWholeBodyInterface(m_moduleName.c_str(), m_robotName.c_str(), iCubVersion);
            
            /*
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
            }*/
            if (!m_robot) {
                return false;
            }
            //add joints
            m_robot->addJoints(wbiIcub::ICUB_MAIN_JOINTS);
            if (!m_robot->init()) {
                return false;
            }
            
            //Sanity checks
            if (wbiIcub::ICUB_MAIN_JOINTS.size() != actuatedDOFs) {
                std::cerr << FUNCTION_NAME << ":Error in initializing wbi, the number of joints is different from the expected" << std::endl;
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
            
            generator = new ReferenceGenerator(m_controllerThreadPeriod, m_references->desiredCOMAcceleration(), *reader);
            if (generator) {
                m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(TaskTypeCOM, generator));
            } else {
                return false;
            }
            
            //position tasks
            std::vector<TaskInformation> tasks;
            TaskInformation task1 = {TaskTypeLeftHandPosition, "l_gripper", &m_references->desiredLeftHandPosition()};
            tasks.push_back(task1);
            TaskInformation task2 = {TaskTypeRightHandPosition, "r_gripper", &m_references->desiredRightHandPosition()};
            tasks.push_back(task2);
            
            for (std::vector<TaskInformation>::iterator it = tasks.begin(); it != tasks.end(); it++) {
                reader = new EndEffectorPositionReader(*m_robot, it->referredLinkName);
                if (reader) {
                    m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(it->taskType, reader));
                } else {
                    return false;
                }
                generator = new ReferenceGenerator(m_controllerThreadPeriod, *(it->reference), *reader);
                if (generator) {
                    m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(it->taskType, generator));
                } else {
                    return false;
                }
            }
            
            //force tasks
            tasks.clear();
            TaskInformation task3 = {TaskTypeLeftHandForce, "l_gripper", &m_references->desiredLeftHandPosition()};
            tasks.push_back(task3);
            TaskInformation task4 = {TaskTypeRightHandForce, "r_gripper", &m_references->desiredRightHandPosition()};
            tasks.push_back(task4);
            
            for (std::vector<TaskInformation>::iterator it = tasks.begin(); it != tasks.end(); it++) {
                reader = new EndEffectorForceReader(*m_robot);//, it->referredLinkName);
                if (reader) {
                    m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(it->taskType, reader));
                } else {
                    return false;
                }
                generator = new ReferenceGenerator(m_controllerThreadPeriod, *(it->reference), *reader);
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
            if (!m_paramHelperManager->linkVariables()
                || !m_paramHelperManager->linkMonitoredVariables()
                || !m_paramHelperManager->registerCommandCallbacks()) {
                return false;
            }
            
            //load initial configuration for the impedance control
            Eigen::VectorXd positions(actuatedDOFs);
            m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, positions.data());
            m_controller->setDesiredJointsConfiguration(positions);
            
            int leftFootLinkID = -1;
            m_robot->getLinkId("l_sole", leftFootLinkID);
            
            wbi::Frame frame;
            m_robot->computeH(positions.data(), wbi::Frame(), leftFootLinkID, frame);
          
            frame = frame * wbi::Frame(wbi::Rotation(0, 0, 1,
                                                     0, -1, 0,
                                                     1, 0, 0));
            frame.setToInverse();
            
            Eigen::VectorXd initialCOM(7);
            //set initial com to be equal to the read one
            m_robot->forwardKinematics(positions.data(), frame, wbi::wholeBodyInterface::COM_LINK_ID, initialCOM.data());
            Eigen::VectorXd comPosition = initialCOM.head(3);
            
            m_references->desiredCOMAcceleration().setValue(comPosition);
            
            bool threadsStarted = true;
            
            for (std::map<TaskType, ReferenceGenerator*>::iterator it = m_referenceGenerators.begin(); it != m_referenceGenerators.end(); it++) {
                threadsStarted = threadsStarted && it->second->start();
            }
            
            threadsStarted = threadsStarted && m_controller->start();
            
            std::cout << "Module " << m_moduleName << " ready." << std::endl;
            
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
        
        double TorqueBalancingModule::getPeriod()
        {
            return m_modulePeriod;
        }
        
        bool TorqueBalancingModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
        {
            if (!m_paramHelperManager->processRPCCommand(command, reply)) {
                reply.addString((std::string("Command ") + command.toString().c_str() + " not recognized.").c_str());
            }
            
            // if reply is empty put something into it, otherwise the rpc communication gets stuck
            if (reply.size() == 0)
                reply.addString((std::string("Command ") + command.toString().c_str() + " received.").c_str());
            return true;
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
        
        void TorqueBalancingModule::monitorVariables()
        {
            m_paramHelperManager->sendMonitoredVariables();
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
        
        TorqueBalancingModule::ParamHelperManager::ParamHelperManager(TorqueBalancingModule& module)
        : m_module(module)
        , m_initialized(false)
        , m_parameterServer(0)
        , m_comReference(3)
        , m_handsPositionReference(14)
        , m_handsForceReference(12)
        , m_comProportionalGain(3)
        , m_comDerivativeGain(3)
        , m_comIntegralGain(3)
        , m_comIntegralLimit(std::numeric_limits<double>::max())
        , m_handsPositionProportionalGain(14)
        , m_handsPositionDerivativeGain(14)
        , m_handsPositionIntegralGain(14)
        , m_handsPositionIntegralLimit(std::numeric_limits<double>::max())
        , m_handsForceProportionalGain(12)
        , m_handsForceDerivativeGain(12)
        , m_handsForceIntegralGain(12)
        , m_handsForceIntegralLimit(std::numeric_limits<double>::max())
        , m_centroidalGain(0)
        , m_impedanceControlGains(actuatedDOFs)
        , m_monitoredDesiredCOMAcceleration(3)
        , m_monitoredCOMError(3) {}
        
        TorqueBalancingModule::ParamHelperManager::~ParamHelperManager()
        {
            if (m_parameterServer) {
                m_parameterServer->close();
                delete m_parameterServer;
                m_parameterServer = 0;
            }
        }
        
        bool TorqueBalancingModule::ParamHelperManager::init(yarp::os::ResourceFinder& resourceFinder)
        {
            if (m_initialized) return false;
            m_parameterServer = new paramHelp::ParamHelperServer(TorqueBalancingModuleParameterDescriptions,
                                                                 TorqueBalancingModuleParameterSize,
                                                                 TorqueBalancingModuleCommandDescriptions,
                                                                 TorqueBalancingModuleCommandSize);
            if (!m_parameterServer) {
                return false;
            }
            
            bool linkedVariable = true;
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterModuleName, &m_module.m_moduleName);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterRobotName, &m_module.m_robotName);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterControllerPeriod, &m_module.m_controllerThreadPeriod);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterURDFFilePath, &m_module.m_urdfFilePath);
            
            if (!linkedVariable) {
                return false;
            }
            
            yarp::os::Bottle replyBottle;
            m_parameterServer->initializeParams(resourceFinder, replyBottle);
            
            if (!m_parameterServer->init(m_module.m_moduleName)) {
                return false;
            }
            m_initialized = true;
            return true;
        }
        
        bool TorqueBalancingModule::ParamHelperManager::linkVariables()
        {
            if (!m_initialized) return false;
            bool linked = true;
            
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCurrentState, &m_module.m_moduleState)
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCurrentState, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterModulePeriod, &m_module.m_modulePeriod);
            //References
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMReference, m_comReference.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMReference, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionReference, m_handsPositionReference.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionReference, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceReference, m_handsForceReference.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceReference, this);
            //COM
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMProportionalGain, m_comProportionalGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMProportionalGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMDerivativeGain, m_comDerivativeGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMDerivativeGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMIntegralGain, m_comIntegralGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMIntegralGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMIntegralLimit, &m_comIntegralLimit)
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMIntegralLimit, this);
            //Hands position
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionDerivativeGain, m_handsPositionProportionalGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionDerivativeGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionDerivativeGain, m_handsPositionDerivativeGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionDerivativeGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionIntegralGain, m_handsPositionIntegralGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionIntegralGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionIntegralLimit, &m_handsPositionIntegralLimit)
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionIntegralLimit, this);
            //Hands forces
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceProportionalGain, m_handsForceProportionalGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceProportionalGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceDerivativeGain, m_handsForceDerivativeGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceDerivativeGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceIntegralGain, m_handsForceIntegralGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceIntegralGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceIntegralLimit, &m_handsForceIntegralLimit)
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceIntegralLimit, this);
            //Centroidal moment / Gains
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCentroidalGain, &m_centroidalGain)
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCentroidalGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterImpedanceGain, m_impedanceControlGains.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterImpedanceGain, this);
            
            return linked;
        }
        
        bool TorqueBalancingModule::ParamHelperManager::linkMonitoredVariables()
        {
            if (!m_initialized) return false;
            bool linked = true;
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterMonitorDesiredCOMAcceleration, m_monitoredDesiredCOMAcceleration.data());
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterMonitorCOMError, m_monitoredCOMError.data());
            return linked;
        }
        
        bool TorqueBalancingModule::ParamHelperManager::registerCommandCallbacks()
        {
            if (!m_initialized) return false;
            bool commandRegistered = true;
            commandRegistered = commandRegistered && m_parameterServer->registerCommandCallback(TorqueBalancingModuleCommandStart, this);
            commandRegistered = commandRegistered && m_parameterServer->registerCommandCallback(TorqueBalancingModuleCommandStop, this);
            commandRegistered = commandRegistered && m_parameterServer->registerCommandCallback(TorqueBalancingModuleCommandQuit, this);
            commandRegistered = commandRegistered && m_parameterServer->registerCommandCallback(TorqueBalancingModuleCommandHelp, this);
            
            return commandRegistered;
        }
        
        bool TorqueBalancingModule::ParamHelperManager::processRPCCommand(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
        {
            assert(m_parameterServer);
            return m_parameterServer->processRpcCommand(command, reply);
        }
        
        void TorqueBalancingModule::ParamHelperManager::sendMonitoredVariables()
        {
            assert(m_parameterServer);
            //copy updated varables to internal monitor variables
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            ReferenceGenerator* comGenerator = 0;
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeCOM)) != m_module.m_referenceGenerators.end()) {
                comGenerator = foundController->second;
            }

            if (comGenerator) {
                m_monitoredDesiredCOMAcceleration = comGenerator->computedReference();
                m_monitoredCOMError = comGenerator->instantaneousError();
            }
            
            //send variables
            m_parameterServer->sendStreamParams();
            
        }
        
        void TorqueBalancingModule::ParamHelperManager::parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface)
        {
            assert(m_parameterServer);
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            switch (proxyInterface->id) {
                case TorqueBalancingModuleParameterCurrentState:
#ifdef DEBUG
                    std::cerr << FUNCTION_NAME << ": State updated to " << m_module.m_moduleState << std::endl;
#endif
                    m_module.updateModuleCoordinationStatus();
                    break;
                    //References
                case TorqueBalancingModuleParameterCOMReference:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setSignalReference(m_comReference);
                    }
                    break;
                case TorqueBalancingModuleParameterHandsPositionReference:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setSignalReference(m_handsPositionReference.head(7));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setSignalReference(m_handsPositionReference.tail(7));
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceReference:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setSignalReference(m_handsPositionReference.head(6));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setSignalReference(m_handsPositionReference.tail(6));
                    }
                    break;
                    //COM
                case TorqueBalancingModuleParameterCOMProportionalGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_comProportionalGain);
                    }
                    break;
                case TorqueBalancingModuleParameterCOMDerivativeGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_comDerivativeGain);
                    }
                    break;
                case TorqueBalancingModuleParameterCOMIntegralGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_comIntegralGain);
                    }
                    break;
                case TorqueBalancingModuleParameterCOMIntegralLimit:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeCOM);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_comIntegralLimit);
                    }
                    break;
                // Hands position
                case TorqueBalancingModuleParameterHandsPositionProportionalGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsPositionProportionalGain.head(7));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsPositionProportionalGain.tail(7));
                    }
                    break;
                case TorqueBalancingModuleParameterHandsPositionDerivativeGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsPositionDerivativeGain.head(7));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsPositionDerivativeGain.tail(7));
                    }

                    break;
                case TorqueBalancingModuleParameterHandsPositionIntegralGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsPositionIntegralGain.head(7));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsPositionIntegralGain.tail(7));
                    }
                    break;
                case TorqueBalancingModuleParameterHandsPositionIntegralLimit:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsPositionIntegralLimit);
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandPosition);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsPositionIntegralLimit);
                    }
                    break;
                //Hands forces
                case TorqueBalancingModuleParameterHandsForceProportionalGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsForceProportionalGain.head(6));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setProportionalGains(m_handsForceProportionalGain.tail(6));
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceDerivativeGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsForceDerivativeGain.head(6));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setDerivativeGains(m_handsForceDerivativeGain.tail(6));
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceIntegralGain:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsForceIntegralGain.head(6));
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralGains(m_handsForceIntegralGain.tail(6));
                    }
                    break;
                case TorqueBalancingModuleParameterHandsForceIntegralLimit:
                    foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsForceIntegralLimit);
                    }
                    foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandForce);
                    if (foundController != m_module.m_referenceGenerators.end()) {
                        foundController->second->setIntegralLimit(m_handsForceIntegralLimit);
                    }
                    break;
                //Centroidal and gains
                case TorqueBalancingModuleParameterCentroidalGain:
                    m_module.m_controller->setCentroidalMomentumGain(m_centroidalGain);
                    break;
                case TorqueBalancingModuleParameterImpedanceGain:
                    m_module.m_controller->setImpedanceGains(m_impedanceControlGains);
                    break;
            }
        }
        
        void TorqueBalancingModule::ParamHelperManager::commandReceived(const paramHelp::CommandDescription& commandDescription, const yarp::os::Bottle& params, yarp::os::Bottle& reply)
        {
            assert(m_parameterServer);
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
                    m_parameterServer->getHelpMessage(reply);
                    break;
                default:
                    break;
            }
            
        }
    }
}
