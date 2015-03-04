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
#include "config.h"
#include "TorqueBalancingController.h"
#include "Reference.h"
#include "ReferenceGenerator.h"
#include "ReferenceGeneratorInputReaderImpl.h"
#include "MinimumJerkTrajectoryGenerator.h"
#include "ParamHelperConfig.h"

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <paramHelp/paramHelperServer.h>
#include <codyco/ModelParsing.h>
#include <codyco/Utils.h>
#include <yarp/os/Port.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <vector>

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
        , m_forcesSmootherDuration(5)
        , m_jointsSmootherDuration(5)
        , m_robot(0)
        , m_controller(0)
        , m_references(0)
        , m_rpcPort(0)
        , m_paramHelperManager(0)
        , m_comReference(3)
        , m_impedanceLeftHandReference(5)
        , m_impedanceRightHandReference(5) {}

        TorqueBalancingModule::~TorqueBalancingModule() { cleanup(); }

        bool TorqueBalancingModule::configure(yarp::os::ResourceFinder& rf)
        {
            //Loading joints information
            yarp::os::Property wbiProperties;
            if (!rf.check("wbi_config_file")) {
                yError("No WBI configuration file found.");
                return false;
            }

            if (!rf.check("wbi_joint_list")) {
                yError("No joint list found. Please specify a joint list in \"wbi_joint_list\"");
                return false;
            }

            if (!wbiProperties.fromConfigFile(rf.findFile("wbi_config_file"))) {
                yError("Not possible to load WBI properties from file.");
                return false;
            }
            wbiProperties.fromString(rf.toString(), false);

            yarp::os::ConstString jointList = rf.find("wbi_joint_list").asString();
            //retrieve all main joints
            wbi::IDList iCubMainJoints;
            if (!yarpWbi::loadIdListFromConfig(jointList, wbiProperties, iCubMainJoints)) {
                yError("Cannot find joint list");
                return false;
            }
            double actuatedDOFs = iCubMainJoints.size();
            
            m_initialJointsConfiguration.resize(actuatedDOFs);
            m_impedanceDoubleSupportReference.resize(actuatedDOFs);
            
            //PARAMETERS SECTION
            //Creating parameter server helper
            //link controller and references variables to param helper manager
            m_paramHelperManager = new ParamHelperManager(*this, actuatedDOFs);
            if (!m_paramHelperManager || !m_paramHelperManager->init(rf)) {
                yError("Could not initialize parameter helper.");
                return false;
            }
            //END PARAMETER SECTION

            m_rpcPort = new yarp::os::Port();
            if (!m_rpcPort
                || !m_rpcPort->open(("/" + m_moduleName + "/rpc").c_str())) {
                yError("Could not open RPC port: /%s/rpc", m_moduleName.c_str());
                return false;
            }
            setName(m_moduleName.c_str());
            attach(*m_rpcPort);

            //Create reference variable
            m_references = new ControllerReferences(actuatedDOFs);
            if (!m_references) {
                yError("Could not create shared references object.");
                return false;
            }

            //Setup streaming
            m_references->desiredCOMPosition().setUpReaderThread(("/" + m_moduleName + "/com:i"));
            m_references->desiredCOMPosition().addDelegate(this);
            m_references->desiredJointsPosition().setUpReaderThread(("/" + m_moduleName + "/qdes:i"));
            m_references->desiredJointsPosition().addDelegate(this);

            //create reference to wbi
            m_robot = new yarpWbi::yarpWholeBodyInterface(m_moduleName.c_str(), wbiProperties);
            if (!m_robot) {
                yError("Could not create wbi object.");
                return false;
            }
            
            //add joints
            m_robot->addJoints(iCubMainJoints);
            if (!m_robot->init()) {
                yError("Could not initialize wbi.");
                return false;
            }
            
            //load initial configuration for the impedance control
            m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, m_initialJointsConfiguration.data());
            m_references->desiredJointsConfiguration().setValue(m_initialJointsConfiguration);

            m_impedanceDoubleSupportReference = m_initialJointsConfiguration; //copy into double support state

//            if (!rf.check("world_frame")) {
//                yError("No world frame specified. Please specify one with \"world_frame\"");
//                return false;
//            }

            yarp::os::ConstString worldFrame = "l_sole";// rf.find("world_frame").asString();

            int worldFrameID = -1;
            m_robot->getFrameList().idToIndex(worldFrame.c_str(), worldFrameID);
            if (worldFrameID < 0) {
                yError("World frame %s not found. Please specify a valid world frame", worldFrame.c_str());
                return false;
            }

            wbi::Frame frame;
            m_robot->computeH(m_initialJointsConfiguration.data(), wbi::Frame(), worldFrameID, frame);

            frame = frame * wbi::Frame(wbi::Rotation(0, 0, 1,
                                                     0, -1, 0,
                                                     1, 0, 0));
            frame.setToInverse();

            Eigen::VectorXd initialCOM(7);
            //set initial com to be equal to the read one
            m_robot->forwardKinematics(m_initialJointsConfiguration.data(), frame, wbi::wholeBodyInterface::COM_LINK_ID, initialCOM.data());
            m_comReference = initialCOM.head(3);


            std::ostringstream outputMessage;
            outputMessage << "Initial COM position: " << m_comReference.transpose();
            yInfo("%s", outputMessage.str().c_str());

            //create smoother
            MinimumJerkTrajectoryGenerator forcesSmoother(6);
            forcesSmoother.initializeTimeParameters(m_controllerThreadPeriod, m_forcesSmootherDuration); //duration to be moved into module (initial) parameters
            MinimumJerkTrajectoryGenerator jointsSmoother(actuatedDOFs);
            jointsSmoother.initializeTimeParameters(m_controllerThreadPeriod, m_jointsSmootherDuration); //duration to be moved into module (initial) parameters

            //create generators
            ReferenceGeneratorInputReader* reader = 0;
            ReferenceGenerator* generator = 0;

            //COM task
            reader = new COMReader(*m_robot, actuatedDOFs);
            if (reader) {
                m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(TaskTypeCOM, reader));
            } else {
                yError("Could not create COM reader object.");
                return false;
            }

            generator = new ReferenceGenerator(m_controllerThreadPeriod, m_references->desiredCOMAcceleration(), *reader, "com pid");
            if (generator) {
                generator->setSignalReference(m_comReference);
                m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(TaskTypeCOM, generator));
            } else {
                yError("Could not create COM controller.");
                return false;
            }

            //Impedance task
            reader = new VoidReader(actuatedDOFs);
            if (reader) {
                m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(TaskTypeImpedanceControl, reader));
            } else {
                yError("Could not create impedance control reader object.");
                return false;
            }

            generator = new ReferenceGenerator(m_controllerThreadPeriod, m_references->desiredJointsConfiguration(), *reader);
            if (generator) {
                //generator->setReferenceFilter(&jointsSmoother);
                generator->setProportionalGains(Eigen::VectorXd::Constant(actuatedDOFs, 1.0));
                m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(TaskTypeImpedanceControl, generator));
            } else {
                yError("Could not create impedance controller object.");
                return false;
            }


//            //position tasks
//            std::vector<TaskInformation> tasks;
//            TaskInformation task1 = {TaskTypeLeftHandPosition, "l_gripper", &m_references->desiredLeftHandPosition()};
//            tasks.push_back(task1);
//            TaskInformation task2 = {TaskTypeRightHandPosition, "r_gripper", &m_references->desiredRightHandPosition()};
//            tasks.push_back(task2);
//
//            for (std::vector<TaskInformation>::iterator it = tasks.begin(); it != tasks.end(); it++) {
//                reader = new EndEffectorPositionReader(*m_robot, it->referredLinkName);
//                if (reader) {
//                    m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(it->taskType, reader));
//                } else {
//                    std::cerr << "Could not create end effector (" << it->referredLinkName << ") position reader object." << std::endl;
//                    return false;
//                }
//                generator = new ReferenceGenerator(m_controllerThreadPeriod, *(it->reference), *reader);
//                if (generator) {
//                    m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(it->taskType, generator));
//                } else {
//                    std::cerr << "Could not create end effector (" << it->referredLinkName << ") position controller object." << std::endl;
//                    return false;
//                }
//            }

            //force tasks
            //Forces are controlled in open-loop right now.
            //I set the proportional gain to the identity, set the feedforward through reference (so it is smoothed)
            //and I do not enable the force reader, so it has no feedback
            std::vector<TaskInformation> tasks;
            TaskInformation task1 = {TaskTypeLeftHandForce, "l_gripper", &m_references->desiredLeftHandForce()};
            tasks.push_back(task1);
            TaskInformation task2 = {TaskTypeRightHandForce, "r_gripper", &m_references->desiredRightHandForce()};
            tasks.push_back(task2);

            reader = new VoidReader(6);
            if (reader) {
                m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(TaskTypeLeftHandForce, reader));
            } else {
                yError("Could not create Force reader object.");
                return false;
            }

            for (std::vector<TaskInformation>::iterator it = tasks.begin(); it != tasks.end(); it++) {
//                reader = new EndEffectorForceReader(*m_robot, it->referredLinkName);
//                if (reader) {
//                    m_generatorReaders.insert(std::pair<TaskType, ReferenceGeneratorInputReader*>(it->taskType, reader));
//                } else {
//                    std::cerr << "Could not create end effector (" << it->referredLinkName << ") force reader object." << std::endl;
//                    return false;
//                }
                generator = new ReferenceGenerator(m_controllerThreadPeriod, *(it->reference), *reader);
                if (generator) {
                    generator->setReferenceFilter(&forcesSmoother);
                    generator->setProportionalGains(Eigen::VectorXd::Constant(6, 1));
                    m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(it->taskType, generator));
                } else {
                    yError("Could not create end effector (%s) force controller object.", it->referredLinkName.c_str());
                    return false;
                }
            }

            m_controller = new TorqueBalancingController(m_controllerThreadPeriod, *m_references, *m_robot, actuatedDOFs);
            if (!m_controller) {
                yError("Could not create TorqueBalancing controller object.");
                return false;
            }

            //link controller and references variables to param helper manager
            if (!m_paramHelperManager->linkVariables()
                || !m_paramHelperManager->linkMonitoredVariables()
                || !m_paramHelperManager->registerCommandCallbacks()) {
                yError("Could not link parameter helper variables.");
                return false;
            }

            //start threads. Controllers start always in inactive state
            //This is needed because they have to be initialized before setting gains, etc..
            bool threadsStarted = true;

            for (std::map<TaskType, ReferenceGenerator*>::iterator it = m_referenceGenerators.begin(); it != m_referenceGenerators.end(); it++) {
                threadsStarted = threadsStarted && it->second->start();
            }
            threadsStarted = threadsStarted && m_controller->start();

            m_paramHelperManager->loadDefaultVariables();


            yInfo("Module %s ready.", m_moduleName.c_str());

            return threadsStarted;
        }

        bool TorqueBalancingModule::updateModule()
        {
            if (!m_controller) {
//                error_out("%s: Error. Control thread pointer is zero.\n", _moduleName.c_str());
                return false;
            }

            m_paramHelperManager->syncLinkedVariables();
            monitorVariables();

            static int counter = 0;
            counter = (counter + 1) % (static_cast<int>(5 / m_modulePeriod)); //every 5 seconds

            if (counter == 0) {
                double periodMean = 0, periodStdDeviation = 0;
                double usedMean = 0, usedStdDeviation = 0;

                m_controller->getEstPeriod(periodMean, periodStdDeviation);
                m_controller->getEstUsed(usedMean, usedStdDeviation);

                if (periodMean > 1.3 * m_controllerThreadPeriod) {
                    yWarning("Control loop is too slow. Real period: %lf +/- %lf. Expected period: %d[ms]\nDuration of 'run' method: %lf +/- %lf", periodMean, periodStdDeviation, m_controllerThreadPeriod, usedMean, usedStdDeviation);
                }
            }

            return true;
        }

        bool TorqueBalancingModule::close()
        {
            setControllersActiveState(false);
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
                m_references->desiredCOMPosition().removeDelegate(this);
                m_references->desiredJointsPosition().removeDelegate(this);
                m_references->desiredCOMPosition().tearDownReaderThread();
                m_references->desiredJointsPosition().tearDownReaderThread();

                delete m_references;
                m_references = 0;
            }
        }

        void TorqueBalancingModule::referenceDidChangeValue(const codyco::torquebalancing::Reference &reference)
        {
            if (!m_references) return;

            TaskType taskType = TaskTypeUnknown;
            if (&reference == &m_references->desiredCOMPosition()) {
                taskType = TaskTypeCOM;
            } else if (&reference == &m_references->desiredJointsPosition()) {
                taskType = TaskTypeImpedanceControl;
            } else return;
            std::map<TaskType, ReferenceGenerator*>::iterator found = m_referenceGenerators.find(TaskTypeCOM);
            if (found != m_referenceGenerators.end()) {
                found->second->setSignalReference(reference.value());
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
            yDebug("Module new state is %s", (m_active ? "on" : "off"));
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
            bool impedanceTaskActive = m_active;
            bool leftHandPositionTaskActive = false;
            bool rightHandPositionTaskActive = false;
            bool leftHandForceTaskActive = false;
            bool rightHandForceTaskActive = false;
            Eigen::VectorXd impedanceReference = m_impedanceDoubleSupportReference;

            switch (m_moduleState) {
                case TorqueBalancingModuleStateDoubleSupportStable:
#ifdef DEBUG
                    yDebug("State Double support");
#endif
                    break;
                case TorqueBalancingModuleStateDoubleSupportSeekingContactBothHands:
//                    leftHandPositionTaskActive = m_active;
//                    rightHandForceTaskActive = m_active;
                    impedanceReference.segment(3, 5) = m_impedanceLeftHandReference;
                    impedanceReference.segment(8, 5) = m_impedanceRightHandReference;
#ifdef DEBUG
                    yDebug("State Double support with hands");
#endif
                    break;
                case TorqueBalancingModuleStateTripleSupportSeekingContactLeftHand:
//                    leftHandPositionTaskActive = m_active;
                    rightHandForceTaskActive = m_active;
                    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, impedanceReference.data());
                    impedanceReference.segment(3, 5) = m_impedanceLeftHandReference;
#ifdef DEBUG
                    yDebug("State Triple support. Left hand searches contact");
#endif
                    break;
                case TorqueBalancingModuleStateTripleSupportSeekingContactRightHand:
//                    rightHandPositionTaskActive = m_active;
                    leftHandForceTaskActive = m_active;
                    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, impedanceReference.data());
                    impedanceReference.segment(8, 5) = m_impedanceRightHandReference;
#ifdef DEBUG
                    yDebug("State Triple support. Right hand searches contact");
#endif
                    break;
                case TorqueBalancingModuleStateQuadrupleSupport:
                    leftHandForceTaskActive = rightHandForceTaskActive = m_active;
                    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, impedanceReference.data());
#ifdef DEBUG
                    yDebug("State Quad support");
#endif
                    break;
                default:
#ifdef DEBUG
                    yDebug("State not recognized");
#endif
                    break;
            }

            bool controlSet = true;
            std::map<TaskType, ReferenceGenerator*>::iterator found;
//            bool tasksState[5] = {comTaskActive, leftHandPositionTaskActive, rightHandPositionTaskActive, leftHandForceTaskActive, rightHandForceTaskActive};
//            TaskType tasksType[5] = { TaskTypeCOM, TaskTypeLeftHandPosition, TaskTypeRightHandPosition, TaskTypeLeftHandForce, TaskTypeRightHandForce};
//
            bool tasksState[4] = {comTaskActive, leftHandForceTaskActive, rightHandForceTaskActive, impedanceTaskActive};
            TaskType tasksType[4] = {TaskTypeCOM, TaskTypeLeftHandForce, TaskTypeRightHandForce, TaskTypeImpedanceControl};
            Eigen::VectorXd* taskReferences[4] = {&m_comReference, 0, 0, &impedanceReference};

            for (int i = 0; i < 4; i++) {
                found = m_referenceGenerators.find(tasksType[i]);
                controlSet = controlSet && found != m_referenceGenerators.end();
                if (controlSet) {
                    found->second->setActiveState(tasksState[i]);
                    if (taskReferences[i])
                        found->second->setSignalReference(*taskReferences[i]);
                }
            }
            m_controller->setActiveState(m_active && controlSet);
        }

#pragma mark - ParamHelperManager methods

        TorqueBalancingModule::ParamHelperManager::ParamHelperManager(TorqueBalancingModule& module, int actuatedDOFs)
        : m_module(module)
        , m_initialized(false)
        , m_parameterServer(0)
        , m_comProportionalGain(3)
        , m_comDerivativeGain(3)
        , m_comIntegralGain(3)
        , m_comIntegralLimit(std::numeric_limits<double>::max())
        , m_torqueSaturation(actuatedDOFs)
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
        , m_monitoredCOMError(3)
        , m_monitoredCOMIntegralError(3)
        , m_monitoredFeetForces(12)
        , m_monitoredOutputTorques(actuatedDOFs)
        {
            //this is totally crazy..
            //indexes to modify are last 3:
            paramHelp::ParamSize newSize = paramHelp::ParamSize(actuatedDOFs, false);
            TorqueBalancingModuleParameterDescriptions[TorqueBalancingModuleParameterSize - 3]->size = newSize;
            TorqueBalancingModuleParameterDescriptions[TorqueBalancingModuleParameterSize - 2]->size = newSize;
            TorqueBalancingModuleParameterDescriptions[TorqueBalancingModuleParameterSize - 1]->size = newSize;
        }

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
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterForcesSmoothingDuration, &m_module.m_forcesSmootherDuration);
            linkedVariable = linkedVariable && m_parameterServer->linkParam(TorqueBalancingModuleParameterJointsSmoothingDuration, &m_module.m_jointsSmootherDuration);

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
            //COM
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMProportionalGain, m_comProportionalGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMProportionalGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMDerivativeGain, m_comDerivativeGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMDerivativeGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMIntegralGain, m_comIntegralGain.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMIntegralGain, this);
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterCOMIntegralLimit, &m_comIntegralLimit)
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterCOMIntegralLimit, this);

            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterTorqueSaturation, m_torqueSaturation.data())
            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterTorqueSaturation, this);

//            //Hands position
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionDerivativeGain, m_handsPositionProportionalGain.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionDerivativeGain, this);
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionDerivativeGain, m_handsPositionDerivativeGain.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionDerivativeGain, this);
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionIntegralGain, m_handsPositionIntegralGain.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionIntegralGain, this);
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsPositionIntegralLimit, &m_handsPositionIntegralLimit)
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsPositionIntegralLimit, this);
            //Hands forces
            //Forces are controlled in open-loop for now
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceProportionalGain, m_handsForceProportionalGain.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceProportionalGain, this);
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceDerivativeGain, m_handsForceDerivativeGain.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceDerivativeGain, this);
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceIntegralGain, m_handsForceIntegralGain.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceIntegralGain, this);
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterHandsForceIntegralLimit, &m_handsForceIntegralLimit)
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterHandsForceIntegralLimit, this);
            //Impedance gains
            //Double support for now is loaded at startup
//            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterDesiredJointsConfigurationStateDoubleSupport,
//                                                            m_module.m_impedanceDoubleSupportReference.data())
//            && m_parameterServer->registerParamValueChangedCallback(TorqueBalancingModuleParameterDesiredJointsConfigurationStateDoubleSupport, this);
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
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterMonitorCOMIntegralError, m_monitoredCOMIntegralError.data());
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterMonitorFeetForces, m_monitoredFeetForces.data());
            linked = linked && m_parameterServer->linkParam(TorqueBalancingModuleParameterMonitorOutputTorques, m_monitoredOutputTorques.data());

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

        void TorqueBalancingModule::ParamHelperManager::loadDefaultVariables()
        {
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            ReferenceGenerator* comGenerator = 0;
            ReferenceGenerator* leftHandPositionGenerator = 0;
            ReferenceGenerator* rightHandPositionGenerator = 0;
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeCOM)) != m_module.m_referenceGenerators.end()) {
                comGenerator = foundController->second;
            }
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition)) != m_module.m_referenceGenerators.end()) {
                leftHandPositionGenerator = foundController->second;
            }
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeRightHandPosition)) != m_module.m_referenceGenerators.end()) {
                rightHandPositionGenerator = foundController->second;
            }

            if (comGenerator) {
                comGenerator->setProportionalGains(m_comProportionalGain);
                comGenerator->setDerivativeGains(m_comDerivativeGain);
                comGenerator->setIntegralGains(m_comIntegralGain);
//                 comGenerator->setAllGains(m_comProportionalGain, m_comDerivativeGain, m_comIntegralGain);
            }
            if (leftHandPositionGenerator) {
                leftHandPositionGenerator->setProportionalGains(m_handsPositionProportionalGain);
                leftHandPositionGenerator->setDerivativeGains(m_handsPositionDerivativeGain);
                leftHandPositionGenerator->setIntegralGains(m_handsPositionIntegralGain);
            }
            if (rightHandPositionGenerator) {
                rightHandPositionGenerator->setProportionalGains(m_handsPositionProportionalGain);
                rightHandPositionGenerator->setDerivativeGains(m_handsPositionDerivativeGain);
                rightHandPositionGenerator->setIntegralGains(m_handsPositionIntegralGain);
            }
            m_module.m_controller->setCentroidalMomentumGain(m_centroidalGain);
            m_module.m_controller->setImpedanceGains(m_impedanceControlGains);
            m_module.m_controller->setTorqueSaturationLimit(m_torqueSaturation);
        }

        bool TorqueBalancingModule::ParamHelperManager::processRPCCommand(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
        {
            assert(m_parameterServer);
            return m_parameterServer->processRpcCommand(command, reply);
        }

        void TorqueBalancingModule::ParamHelperManager::sendMonitoredVariables()
        {
            if (!m_parameterServer || !m_module.m_controller) {
                yError("Error: controller or server are nil! Please restart the module");
                return;
            }
            //copy updated varables to internal monitor variables
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            ReferenceGenerator* comGenerator = 0;
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeCOM)) != m_module.m_referenceGenerators.end()) {
                comGenerator = foundController->second;
            }

            if (comGenerator) {
                m_monitoredDesiredCOMAcceleration = comGenerator->computedReference();
                m_monitoredCOMError = comGenerator->instantaneousError();
                m_monitoredCOMIntegralError = comGenerator->errorIntegral();
            }
            m_monitoredFeetForces = m_module.m_controller->desiredFeetForces();
            m_monitoredOutputTorques = m_module.m_controller->outputTorques();

            //send variables
            m_parameterServer->sendStreamParams();

        }

        void TorqueBalancingModule::ParamHelperManager::syncLinkedVariables()
        {
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            ReferenceGenerator* comGenerator = 0;
//            ReferenceGenerator* leftHandPositionGenerator = 0;
            //ReferenceGenerator* rightHandPositionGenerator = 0;
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeCOM)) != m_module.m_referenceGenerators.end()) {
                comGenerator = foundController->second;
            }
//            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition)) != m_module.m_referenceGenerators.end()) {
//                leftHandPositionGenerator = foundController->second;
//            }
//             if ((foundController = m_module.m_referenceGenerators.find(TaskTypeLeftHandPosition)) != m_module.m_referenceGenerators.end()) {
//                 rightHandPositionGenerator = foundController->second;
//             }

            if (comGenerator) {
                m_comProportionalGain = comGenerator->proportionalGains();
                m_comDerivativeGain = comGenerator->derivativeGains();
                m_comIntegralGain = comGenerator->integralGains();
                m_comIntegralLimit = comGenerator->integralLimit();
            }
//            if (leftHandPositionGenerator) {
//                m_handsPositionProportionalGain = leftHandPositionGenerator->proportionalGains();
//                m_handsPositionDerivativeGain = leftHandPositionGenerator->derivativeGains();
//                m_handsPositionIntegralGain = leftHandPositionGenerator->integralGains();
//                m_handsPositionIntegralLimit = leftHandPositionGenerator->integralLimit();
//            }
//             if (rightHandPositionGenerator) {
//                 rightHandPositionGenerator->setProportionalGains(m_handsPositionProportionalGain);
//                 rightHandPositionGenerator->setDerivativeGains(m_handsPositionDerivativeGain);
//                 rightHandPositionGenerator->setIntegralGains(m_handsPositionIntegralGain);
//             }
        }

        void TorqueBalancingModule::ParamHelperManager::parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface)
        {
            assert(m_parameterServer);
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            switch (proxyInterface->id) {
                case TorqueBalancingModuleParameterCurrentState:
#ifdef DEBUG
                    yDebug("State updated to %d", m_module.m_moduleState);
#endif
                    m_module.updateModuleCoordinationStatus();
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
                //Saturation
                case TorqueBalancingModuleParameterTorqueSaturation:
                    m_module.m_controller->setTorqueSaturationLimit(m_torqueSaturation);
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
