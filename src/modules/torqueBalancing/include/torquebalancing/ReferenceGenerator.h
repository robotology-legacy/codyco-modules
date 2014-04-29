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

#ifndef REFERENCEGENERATOR_H
#define REFERENCEGENERATOR_H

#include <yarp/os/RateThread.h>
#include "Reference.h"

namespace codyco {
    namespace torquebalancing {
        
        template<class ReferenceType>
        class ReferenceGenerator: public ::yarp::os::RateThread
        {
        public:
            ReferenceGenerator(int period, Reference<ReferenceType>& reference);
            
            virtual bool threadInit();
            virtual void threadRelease();
            virtual void run();
            
        private:
            Reference<ReferenceType>& m_reference;
        };
        
        template<class ReferenceType>
        ReferenceGenerator<ReferenceType>::ReferenceGenerator(int period, Reference<ReferenceType>& reference)
        : RateThread(period)
        , m_reference(reference) {}
        
    }
}


#endif /* end of include guard: REFERENCEGENERATOR_H */
