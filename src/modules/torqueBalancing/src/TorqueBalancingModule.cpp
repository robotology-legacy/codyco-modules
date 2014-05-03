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

#include <paramHelp/paramHelperServer.h>
#include "ParamHelperConfig.h"

namespace codyco {
    namespace torquebalancing {
        
        TorqueBalancingModule::TorqueBalancingModule()
        : m_referenceThreadPeriod(10)
        , m_controllerThreadPeriod(10)
        , m_robot(0)
        , m_world2BaseFrame(0)
        , m_controller(0)
        , m_references(0)
        , m_parameterServer(0)
        , m_rpcPort(0) {}
        
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
            m_robot = new wbiIcub::icubWholeBodyInterface(m_moduleName.c_str(), m_robotName.c_str());//, iCub::iDynTree::iCubTree_version_tag icub_version, std::string urdf_file);
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
            
            reader = new COMReader(*m_robot, *m_world2BaseFrame);
            if (reader) {
                m_generatorReaders.push_back(reader);
            } else {
                return false;
            }
            
            generator = new ReferenceGenerator(m_referenceThreadPeriod, m_references->desiredCOMAcceleration(), *reader);
            if (generator) {
                m_referenceGenerators.push_back(generator);
            } else {
                return false;
            }
            
            reader = new HandsForceReader(*m_robot, *m_world2BaseFrame);
            if (reader) {
                m_generatorReaders.push_back(reader);
            } else {
                return false;
            }
            generator = new ReferenceGenerator(m_referenceThreadPeriod, m_references->desiredHandsPosition(), *reader);
            if (generator) {
                m_referenceGenerators.push_back(generator);
            } else {
                return false;
            }
            
            reader = new HandsPositionReader(*m_robot, *m_world2BaseFrame);
            if (reader) {
                m_generatorReaders.push_back(reader);
            } else {
                return false;
            }
            generator = new ReferenceGenerator(m_referenceThreadPeriod, m_references->desiredHandsPosition(), *reader);
            if (generator) {
                m_referenceGenerators.push_back(generator);
            } else {
                return false;
            }
            
            m_controller = new TorqueBalancingController(m_controllerThreadPeriod, *m_references);
            if (!m_controller) {
                return false;
            }
            
            return true;
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
            for (std::vector<ReferenceGenerator*>::iterator it = m_referenceGenerators.begin(); it != m_referenceGenerators.end(); it++) {
                (*it)->stop();
                delete *it;
            }
            m_referenceGenerators.clear();
            
            for (std::vector<ReferenceGeneratorInputReader*>::iterator it = m_generatorReaders.begin(); it != m_generatorReaders.end(); it++) {
                delete *it;
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
    }
}