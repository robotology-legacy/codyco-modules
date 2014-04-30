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

namespace codyco {
    namespace torquebalancing {
        
        TorqueBalancingModule::TorqueBalancingModule() {}
        TorqueBalancingModule::~TorqueBalancingModule() {}
        
        bool TorqueBalancingModule::configure(yarp::os::ResourceFinder& rf)
        {
            //Create reference variable
            m_references = new ControllerReferences();
            
            //create generators
            ReferenceGeneratorInputReader* reader = 0;
            ReferenceGenerator* generator = 0;
            
            reader = new COMReader(*m_robot, *m_world2BaseFrame);
            generator = new ReferenceGenerator(10, m_references->desiredCOMAcceleration(), *reader);
            
            m_generatorReaders.push_back(reader);
            m_referenceGenerators.push_back(generator);
            
            reader = new HandsForceReader(*m_robot, *m_world2BaseFrame);
            generator = new ReferenceGenerator(10, m_references->desiredHandsPosition(), *reader);
            
            m_generatorReaders.push_back(reader);
            m_referenceGenerators.push_back(generator);
            
            reader = new HandsPositionReader(*m_robot, *m_world2BaseFrame);
            generator = new ReferenceGenerator(10, m_references->desiredHandsPosition(), *reader);
            
            m_generatorReaders.push_back(reader);
            m_referenceGenerators.push_back(generator);
            
            m_controller = new TorqueBalancingController(10, *m_references);
            
            return true;
        }
        bool TorqueBalancingModule::updateModule() { return true; }
        bool TorqueBalancingModule::close()
        {
            for (std::vector<ReferenceGenerator*>::iterator it = m_referenceGenerators.begin(); it != m_referenceGenerators.end(); it++) {
                
//                it->stop();
                delete *it;
            }
            m_referenceGenerators.clear();
            
            for (std::vector<ReferenceGeneratorInputReader*>::iterator it = m_generatorReaders.begin(); it != m_generatorReaders.end(); it++) {
                delete *it;
            }
            m_generatorReaders.clear();
            
            return true;
        }
    }
}