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

#ifndef CONFIG_H
#define CONFIG_H

namespace codyco {
    namespace torquebalancing {
        extern const int actuatedDOFs; /*!< number of actuated degree of freedom */
        extern const int totalDOFs; /*!< total number of degree of freedom. For a free floating robot this is usually the number of actuated joints plus 6*/
        extern const double PseudoInverseTolerance;
        
    }
}

#endif //CONFIG_H
