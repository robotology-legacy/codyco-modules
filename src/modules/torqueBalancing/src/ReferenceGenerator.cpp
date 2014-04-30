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

#include "ReferenceGenerator.h"
#include "Reference.h"
#include <wbi/wholeBodyInterface.h>

namespace codyco {
    namespace torquebalancing {
        
        ReferenceGenerator::ReferenceGenerator(int period, Reference& reference, ReferenceGeneratorInputReader& reader)
        : RateThread(period)
        , m_reference(reference)
        , m_reader(reader)
        , m_proportionalGains(reference.value().size())
        , m_derivativeGains(reference.value().size())
        , m_integralGains(reference.value().size()){}

        bool ReferenceGenerator::threadInit()
        {
            return true;
        }
        
        void ReferenceGenerator::threadRelease()
        {
            
        }
        
        void ReferenceGenerator::run()
        {
            
            //compute pid
            
            
        }
        
        ReferenceGeneratorInputReader::~ReferenceGeneratorInputReader() {}

        
    }
}