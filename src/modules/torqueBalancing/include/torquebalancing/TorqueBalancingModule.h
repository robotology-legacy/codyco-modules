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
#include <vector>

namespace wbi {
    class wholeBodyInterface;
    class Frame;
}

namespace codyco {
    namespace torquebalancing {
        
        class ControllerReferences;
        class TorqueBalancingController;
        class ReferenceGenerator;
        class ReferenceGeneratorInputReader;
        
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
            
        private:
            wbi::wholeBodyInterface* m_robot;
            wbi::Frame* m_world2BaseFrame;
            
            TorqueBalancingController* m_controller;
            ControllerReferences* m_references;
            
            std::vector<ReferenceGeneratorInputReader*> m_generatorReaders;
            std::vector<ReferenceGenerator*> m_referenceGenerators;
        };
    }
}

#endif /* end of include guard: TORQUEBALANCINGMODULE_H */
