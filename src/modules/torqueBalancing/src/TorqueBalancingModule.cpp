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
#include <codyco/PIDList.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <yarp/dev/ControlBoardPid.h>
#include <iostream>
#include <sstream>
#include <vector>

#define ACTIVATE_CONTACT "activateConstraints"
#define DEACTIVATE_CONTACT "deactivateConstraints"

namespace codyco {
    namespace torquebalancing {

        const std::string TorquePIDInitialKey = "__ORIGINAL_PIDs__";
        const std::string TorquePIDDefaultKey = "__DEFAULT_PIDs__";

        //Utility structure used inside the module
        struct TaskInformation {
            TaskType taskType;
            std::string referredLinkName;
            Reference* reference; //I cannot use a reference because object must be Assignable to be used inside std::vector
        };

        TorqueBalancingModule::TorqueBalancingModule()
        : m_controllerThreadPeriod(10)
        , m_modulePeriod(1.0)
        , m_active(false)
        , m_robot(0)
        , m_controller(0)
        , m_references(0)
        , m_rpcPort(0)
        , m_paramHelperManager(0)
        , m_tempHeptaVector(7)
        , m_comReference(3) {}

        TorqueBalancingModule::~TorqueBalancingModule() { cleanup(); }

        bool TorqueBalancingModule::configure(yarp::os::ResourceFinder& rf)
        {
            using namespace yarp::os;

            //Loading joints information
            yarp::os::Property wbiProperties;
            if (!rf.check("wbi_config_file", "Checking wbi configuration file")) {
                yError("No WBI configuration file found.");
                return false;
            }

            if (!rf.check("wbi_joint_list", "Checking wbi joint list name")) {
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

            m_jointsConfiguration.resize(actuatedDOFs);

            //Load configuration-time parameters
            m_moduleName = rf.check("name", Value("torqueBalancing"), "Looking for module name").asString();
            m_robotName = rf.check("robot", Value("icub"), "Looking for robot name").asString();
            m_controllerThreadPeriod = rf.check("period", Value(10), "Looking for controller period").asInt();
            double dynamicsSmoothing = rf.check("dynSmooth", Value(1.0), "Looking for dynamics smoothing transition time ").asDouble();
            m_modulePeriod = rf.check("modulePeriod", Value(0.25), "Looking for module period").asDouble();
            Value trueValue;
            trueValue.fromString("true");
            bool checkJointLimits = rf.check("check_limits", trueValue, "Looking for joint limits check option").asBool();
            Value falseValue;
            falseValue.fromString("false");
            bool autoStart = rf.check("autostart", falseValue, "Looking for autostart option").asBool();

            //Check smooth parameter
            //Structure is: key: smooth
            //              value: Bottle with: (("type", duration), (...))
            //                     Type is currently only "joints" or "com".
            //                     duration is a double

            bool jointSmooth = false;
            bool comSmooth = false;
            double jointSmoothDuration = -1;
            double comSmoothDuration = -1;

            if (rf.check("smooth", "Looking for smooth parameters")) {
                if (rf.find("smooth").isList()) {
                    Bottle *smoothBottle = rf.find("smooth").asList();
                    //if list has size 2, check if children are string and double
                    if (smoothBottle->size() == 2 && !smoothBottle->get(0).isList()
                        && smoothBottle->get(0).isString() && smoothBottle->get(1).isDouble()) {
                        std::string type = smoothBottle->get(0).asString();
                        double duration = smoothBottle->get(1).asDouble();
                        if (type == "joints") {
                            jointSmooth = true;
                            jointSmoothDuration = duration;
                        } else if (type == "com") {
                            comSmooth = true;
                            comSmoothDuration = duration;
                        }
                    } else {
                        for (int i = 0; i < smoothBottle->size(); i++) {
                            //list of list
                            if (smoothBottle->get(i).isList()) {
                                Bottle *innerList = smoothBottle->get(i).asList();
                                if (innerList->size() == 2
                                    && innerList->get(0).isString()
                                    && innerList->get(1).isDouble()) {
                                    //this can be taken into a method.. I'm duplicating the code above
                                    std::string type = innerList->get(0).asString();
                                    double duration = innerList->get(1).asDouble();
                                    if (type == "joints") {
                                        jointSmooth = true;
                                        jointSmoothDuration = duration;
                                    } else if (type == "com") {
                                        comSmooth = true;
                                        comSmoothDuration = duration;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            //PARAMETERS SECTION
            //Creating parameter server helper
            //link controller and references variables to param helper manager
            m_paramHelperManager = new ParamHelperManager(*this, actuatedDOFs);
            if (!m_paramHelperManager || !m_paramHelperManager->init(rf)) {
                yError("Could not initialize parameter helper.");
                return false;
            }
            //END PARAMETER SECTION
            setName(m_moduleName.c_str());

            m_rpcPort = new yarp::os::Port();
            if (!m_rpcPort
                || !m_rpcPort->open(("/" + getName("/rpc")).c_str())) {
                yError("Could not open RPC port: /%s/rpc", m_moduleName.c_str());
                return false;
            }
            attach(*m_rpcPort);

            m_eventsPort = new yarp::os::BufferedPort<yarp::os::Property>();
            if (!m_eventsPort
                || !m_eventsPort->open(("/" + getName("/events:o")).c_str())) {
                yError("Could not open events port: /%s/events:o", m_moduleName.c_str());
                return false;
            }

            m_constraintsPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
            if (!m_constraintsPort
                || !m_constraintsPort->open(("/" + getName("/constraints:i")).c_str())) {
                yError("Could not open constraint input port: /%s/constraints:i", m_moduleName.c_str());
                return false;
            }
            m_constraintsPort->useCallback(*this);

            //Create reference variable
            m_references = new ControllerReferences(actuatedDOFs);
            if (!m_references) {
                yError("Could not create shared references object.");
                return false;
            }

            //Setup streaming
            if (!m_references->desiredCOM().setUpReaderPort(("/" + getName("/comDes:i")))) {
                yError("CoM streaming port failed to start.");
                return false;
            }
            m_references->desiredCOM().addDelegate(this);
            if (!m_references->desiredJointsPosition().setUpReaderPort(("/" + getName("/qDes:i")))) {
                yError("QDes streaming port failed to start.");
                return false;
            }
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

            //check torque gains after init
            m_defaultTorquePIDsKey = rf.check("defaultTorqueGainsKey", Value::getNullValue(), "Checking default PIDs set");
            if (rf.check("torqueGains", "Looking for torque gains section")) {
                if (!loadTorqueGains(rf.find("torqueGains"))) {
                    yError("A torque gain option is specified, but failed to load");
                    return false;
                }
            }
            yInfo() << "Gains in memory: " << m_torquePIDs.size() << "\n";

            //load initial configuration for the impedance control
            m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsConfiguration.data());
            m_references->desiredJointsConfiguration().setValue(m_jointsConfiguration);

            wbi::Frame frame;
            Eigen::VectorXd baseSerialization(16);
            if (!m_robot->getEstimates(wbi::ESTIMATE_BASE_POS, baseSerialization.data())) {
                yError("Could not compute base estimation.");
                return false;
            }
            
            wbi::frameFromSerialization(baseSerialization.data(), frame);
            //set initial com to be equal to the read one
            if (!m_robot->forwardKinematics(m_jointsConfiguration.data(), frame, wbi::wholeBodyInterface::COM_LINK_ID, m_tempHeptaVector.data())) {
                yError("Could not compute forward kinematics.");
                return false;
            }
            m_comReference = m_tempHeptaVector.head<3>();

            std::ostringstream outputMessage;
            outputMessage << "Initial COM position: " << m_comReference.transpose();
            yInfo("%s", outputMessage.str().c_str());
            

            //Disabled for now
            //            MinimumJerkTrajectoryGenerator jointsSmoother(actuatedDOFs);
            //            jointsSmoother.initializeTimeParameters(m_controllerThreadPeriod, m_jointsSmootherDuration); //duration to be moved into module (initial) parameters

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
                if (comSmooth) {
                    yInfo() << "COM smoothing is ENABLED with duration " << comSmoothDuration;
                    MinimumJerkTrajectoryGenerator comSmoother(3);
                    comSmoother.initializeTimeParameters(m_controllerThreadPeriod/1000, comSmoothDuration);
                    generator->setReferenceFilter(&comSmoother);
                }
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

            generator = new ReferenceGenerator(m_controllerThreadPeriod, m_references->desiredJointsConfiguration(), *reader, "qdes");
            if (generator) {
                if (jointSmooth) {
                    yInfo() << "Joint smoothing is ENABLED with duration " << jointSmoothDuration;
                    MinimumJerkTrajectoryGenerator jointsSmoother(actuatedDOFs);
                    jointsSmoother.initializeTimeParameters(m_controllerThreadPeriod/1000, jointSmoothDuration);
                    generator->setReferenceFilter(&jointsSmoother);
                } else {
                    yInfo() << "Joint smoothing is DISABLED";
                }
                generator->setProportionalGains(Eigen::VectorXd::Constant(actuatedDOFs, 1.0));
                generator->setSignalReference(m_jointsConfiguration);
                m_referenceGenerators.insert(std::pair<TaskType, ReferenceGenerator*>(TaskTypeImpedanceControl, generator));
            } else {
                yError("Could not create impedance controller object.");
                return false;
            }

            //Balancing controller
            m_controller = new TorqueBalancingController(m_controllerThreadPeriod, *m_references, *m_robot, actuatedDOFs, dynamicsSmoothing);
            if (!m_controller) {
                yError("Could not create TorqueBalancing controller object.");
                return false;
            }
            m_controller->setDelegate(this);
            m_controller->setCheckJointLimits(checkJointLimits);

            //link controller and references variables to param helper manager
            if (!m_paramHelperManager->linkVariables()
                || !m_paramHelperManager->linkMonitoredVariables()
                || !m_paramHelperManager->registerCommandCallbacks()) {
                yError("Could not link parameter helper variables.");
                return false;
            }

            //Read initial constraints for the controller
            std::vector<std::string> constraintsLinkName;
            if (rf.check("constraint_links", "Checking rigid constraints")) {
                Bottle &constraints = rf.findGroup("constraint_links");
                if (constraints.size() == 2) {
                    Bottle *list = constraints.get(1).asList();
                    if (list) {
                        for (int i = 0; i < list->size(); i++) {
                            Value &linkName = list->get(i);
                            if (linkName.isString())
                                constraintsLinkName.push_back(linkName.asString());
                        }
                    }
                }
            }

            //Default case: two feet balancing (back compatibily)
            if (constraintsLinkName.empty()) {
                constraintsLinkName.push_back("l_sole");
                constraintsLinkName.push_back("r_sole");
            }
            if (!m_controller->setInitialConstraintSet(constraintsLinkName)) {
                yError("Could not set initial dynamic constraints.");
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

            if (m_torquePIDs.size() > 0) {
                std::string key = m_defaultTorquePIDsKey.isNull() ? TorquePIDDefaultKey : m_defaultTorquePIDsKey.asString().c_str();
                switchToTorqueGainsWithKey(key);
            }

            if (autoStart)
                setControllersActiveState(autoStart);

            return threadsStarted;
        }

        bool TorqueBalancingModule::updateModule()
        {
            if (!m_controller) {
                yError("%s: Error. Control thread pointer is zero.", m_moduleName.c_str());
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
            if (m_torquePIDs.size() > 0) {
                //restore initial PIDs
                switchToTorqueGainsWithKey(TorquePIDInitialKey);
            }
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

            if (m_constraintsPort) {
                m_constraintsPort->close();
                m_constraintsPort->disableCallback();
                delete m_constraintsPort;
                m_constraintsPort = 0;
            }

            if (m_eventsPort) {
                m_eventsPort->close();
                delete m_eventsPort;
                m_eventsPort = 0;
            }

            //close controller thread
            if (m_controller) {
                m_controller->setDelegate(NULL);
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
                m_references->desiredCOM().removeDelegate(this);
                m_references->desiredJointsPosition().removeDelegate(this);
                m_references->desiredCOM().tearDownReaderPort();
                m_references->desiredJointsPosition().tearDownReaderPort();

                delete m_references;
                m_references = 0;
            }
        }

        void TorqueBalancingModule::referenceDidChangeValue(const codyco::torquebalancing::Reference &reference)
        {
            if (!m_references) return;

            TaskType taskType = TaskTypeUnknown;
            if (&reference == &m_references->desiredCOM()) {
                taskType = TaskTypeCOM;
                std::map<TaskType, ReferenceGenerator*>::iterator found = m_referenceGenerators.find(taskType);
                if (found != m_referenceGenerators.end()) {
                    const Eigen::VectorXd &newReference = reference.value();
                    //check if smoother is active.
                    if (found->second->referenceFilter()) {
                        found->second->setSignalReference(newReference.head(3));
                    } else {
                        found->second->setAllReferences(newReference.head(3), newReference.segment(3, 3), newReference.tail(3));
                    }
                }
            } else if (&reference == &m_references->desiredJointsPosition()) {
                taskType = TaskTypeImpedanceControl;
                std::map<TaskType, ReferenceGenerator*>::iterator found = m_referenceGenerators.find(taskType);
                if (found != m_referenceGenerators.end()) {
                    found->second->setSignalReference(reference.value());
                }
            } else return;
        }

        void TorqueBalancingModule::controllerDidStop(TorqueBalancingController& controller)
        {
            setControllersActiveState(false);
            yarp::os::Property &event = m_eventsPort->prepare();
            event.put("STATUS", yarp::os::Value("stop"));
            m_eventsPort->write();
        }

        void TorqueBalancingModule::onRead(yarp::os::Bottle &read)
        {
            //Read of the constraints port
            //expecting size >= 2
            if (read.size() < 2) return;

            std::string command = read.get(0).asString();
            if (command.compare(ACTIVATE_CONTACT) == 0) {
                for (int i = 1; i < read.size(); i++) {
                    m_controller->addDynamicConstraint(read.get(i).asString());
                }
            }

            if (command.compare(DEACTIVATE_CONTACT) == 0) {
                for (int i = 1; i < read.size(); i++) {
                    m_controller->removeDynamicConstraint(read.get(i).asString());
                }
            }
        }

        double TorqueBalancingModule::getPeriod()
        {
            return m_modulePeriod;
        }

        bool TorqueBalancingModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
        {
            if (!m_paramHelperManager->processRPCCommand(command, reply)) {
                if (command.size() > 1 && command.get(0).isString() && command.get(1).isString()) {
                    std::string cmd = command.get(0).asString();
                    if (cmd == "torque_gains_switch") {
                        if (!switchToTorqueGainsWithKey(command.get(1).asString()))
                            reply.addString(("Failed to switch to " + command.get(1).asString()).c_str());
                    }
                } else {
                    reply.addString((std::string("Command ") + command.toString().c_str() + " not recognized.").c_str());
                }
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
            //When the control switch from inactive to active
            //it should discard all previous references: the robot should stay still
            if (isActive) {
                yarp::os::LockGuard guard(dynamic_cast<yarpWbi::yarpWholeBodyInterface*>(m_robot)->getInterfaceMutex());
                m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsConfiguration.data());
                wbi::Frame frame;
                Eigen::VectorXd baseSerialization(16);
                m_robot->getEstimates(wbi::ESTIMATE_BASE_POS, baseSerialization.data());
                wbi::frameFromSerialization(baseSerialization.data(), frame);

                m_robot->forwardKinematics(m_jointsConfiguration.data(), frame, wbi::wholeBodyInterface::COM_LINK_ID, m_tempHeptaVector.data());
                m_comReference = m_tempHeptaVector.head<3>();
            }


            bool comTaskActive = m_active;
            bool impedanceTaskActive = m_active;

            bool controlSet = true;
            std::map<TaskType, ReferenceGenerator*>::iterator found;
            bool tasksState[2] = {comTaskActive, impedanceTaskActive};
            TaskType tasksType[2] = {TaskTypeCOM, TaskTypeImpedanceControl};
            Eigen::VectorXd* taskReferences[2] = {&m_comReference, &m_jointsConfiguration};

            for (int i = 0; i < 2; i++) {
                found = m_referenceGenerators.find(tasksType[i]);
                controlSet = controlSet && found != m_referenceGenerators.end();
                if (controlSet) {
                    found->second->setActiveState(tasksState[i]);
                    if (taskReferences[i])
                        found->second->setSignalReference(*taskReferences[i], true);
                }
            }
            m_controller->setActiveState(m_active && controlSet);
        }

        void TorqueBalancingModule::monitorVariables()
        {
            m_paramHelperManager->sendMonitoredVariables();
        }

        bool TorqueBalancingModule::switchToTorqueGainsWithKey(const std::string &gainKey)
        {
            PidMap::const_iterator found = m_torquePIDs.find(gainKey);
            if (found == m_torquePIDs.end()) {
                yError("Default torque gains not found in the configuration list.");
                return false;
            }
            if (!switchToTorqueGains(found->second)) {
                yError("Failed to switch to torque gains %s", gainKey.c_str());
                return false;
            } else {
                yInfo("Loaded torque gains - %s", gainKey.c_str());
            }
            return true;
        }

        bool TorqueBalancingModule::switchToTorqueGains(const PIDList& pids)
        {
            if (!m_robot) return false;
            yarpWbi::yarpWholeBodyActuators *actuators = ((yarpWbi::yarpWholeBodyInterface*)m_robot)->wholeBodyActuator();
            actuators->setPIDGains(pids.pidList(), wbi::CTRL_MODE_TORQUE);
            return true;
        }


        bool TorqueBalancingModule::loadTorqueGains(yarp::os::Value &gains)
        {
            //gains can be:
            // - filename
            // - List with (key, filename)
            // - List of list of (key, filename)

            //Load original gains from controlboards and save them the original key.
            PIDList originalGains(m_robot->getDoFs());
            yarpWbi::yarpWholeBodyActuators *actuators = ((yarpWbi::yarpWholeBodyInterface*)m_robot)->wholeBodyActuator();
            actuators->getPIDGains(originalGains.pidList(), wbi::CTRL_MODE_TORQUE);
            m_torquePIDs.insert(PidMap::value_type(TorquePIDInitialKey, originalGains));

            //Now load additional gains
            bool result = true;
            if (gains.isString()) {
                PIDList pids(m_robot->getDoFs());
                result = loadTorqueGainsFromFile(gains.asString(), originalGains, pids);
                m_torquePIDs.insert(PidMap::value_type(TorquePIDDefaultKey, pids));
            } else if (gains.isList()) {
                using namespace yarp::os;
                Bottle *list = gains.asList();
                if (list->size() == 2 &&
                    list->get(0).isString() && list->get(1).isString()) {
                    std::string key = list->get(0).asString();
                    std::string filename = list->get(1).asString();
                    PIDList pids(m_robot->getDoFs());
                    result = result && loadTorqueGainsFromFile(filename, originalGains, pids);
                    if (result) m_torquePIDs.insert(PidMap::value_type(key, pids));

                } else {
                    for (int i = 0; i < list->size() && result; i++) {
                        if (list->get(i).isList() &&
                            list->get(i).asList()->size() == 2 &&
                            list->get(i).asList()->get(0).isString() && list->get(i).asList()->get(1).isString()) {
                            std::string key = list->get(i).asList()->get(0).asString();
                            std::string filename = list->get(i).asList()->get(1).asString();
                            PIDList pids(m_robot->getDoFs());
                            result = loadTorqueGainsFromFile(filename, originalGains, pids);
                            if (result) m_torquePIDs.insert(PidMap::value_type(key, pids));
                        } else {
                            yError("Malformed torque gains list");
                            result = false;
                            break;
                        }
                    }
                }
            }
            return result;
        }

        static bool fillPIDWithBottleDescription(const yarp::os::Bottle &bottle, yarp::dev::Pid &pid)
        {
            for (int i = 1; i < bottle.size(); ++i) {
                if (!bottle.get(i).isList()) continue;
                yarp::os::Bottle *gains = bottle.get(i).asList();
                if (gains->size() == 2 && gains->get(0).isString() && gains->get(1).isDouble()) {
                    std::string gainType = gains->get(0).asString();
                    double value = gains->get(1).asDouble();

                    if (gainType == "kp") {
                        pid.setKp(value);
                    } else if (gainType == "kd") {
                        pid.setKd(value);
                    } else if (gainType == "ki") {
                        pid.setKi(value);
                    }
                }
            }
            return true;
        }

        bool TorqueBalancingModule::loadTorqueGainsFromFile(std::string filename,
                                                            const PIDList &originalList,
                                                            PIDList &loadedPIDs)
        {
            //List of list. Each element has a key: joint name, and a list of pairs: kp, kd, ki and its respective value
            using namespace yarp::os;
            yarp::os::ResourceFinder resourceFinder = yarp::os::ResourceFinder::getResourceFinderSingleton();
            Property file;
            file.fromConfigFile(resourceFinder.findFile(filename));

            Bottle externalList;
            externalList.fromString(file.toString());

            bool result = true;
            wbi::IDList jointList = m_robot->getJointList();
            for (int i = 0; i < externalList.size(); ++i) {
                if (!externalList.get(i).isList()) continue;
                Bottle *jointConfig = externalList.get(i).asList();
                if (jointConfig->size() < 2 || !jointConfig->get(0).isString()) continue;
                wbi::ID jointID(jointConfig->get(0).asString());
                int jointIndex = -1;
                if (!jointList.idToIndex(jointID, jointIndex)) continue;
                if (jointIndex < 0 || jointIndex >= jointList.size()) {
                    yWarning("Specified joint %s index is outside joint list size", jointID.toString().c_str());
                    continue;
                }

                loadedPIDs.pidList()[jointIndex] = originalList.pidList()[jointIndex];
                result = result && fillPIDWithBottleDescription(*jointConfig, loadedPIDs.pidList()[jointIndex]);
            }
            return result;
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
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeCOM)) != m_module.m_referenceGenerators.end()) {
                comGenerator = foundController->second;
            }

            if (comGenerator) {
                comGenerator->setProportionalGains(m_comProportionalGain);
                comGenerator->setDerivativeGains(m_comDerivativeGain);
                comGenerator->setIntegralGains(m_comIntegralGain);
                //                 comGenerator->setAllGains(m_comProportionalGain, m_comDerivativeGain, m_comIntegralGain);
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
            if ((foundController = m_module.m_referenceGenerators.find(TaskTypeCOM)) != m_module.m_referenceGenerators.end()) {
                comGenerator = foundController->second;
            }
            if (comGenerator) {
                m_comProportionalGain = comGenerator->proportionalGains();
                m_comDerivativeGain = comGenerator->derivativeGains();
                m_comIntegralGain = comGenerator->integralGains();
                m_comIntegralLimit = comGenerator->integralLimit();
            }
        }

        void TorqueBalancingModule::ParamHelperManager::parameterUpdated(const paramHelp::ParamProxyInterface *proxyInterface)
        {
            assert(m_parameterServer);
            std::map<TaskType, ReferenceGenerator*>::iterator foundController;
            switch (proxyInterface->id) {
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
