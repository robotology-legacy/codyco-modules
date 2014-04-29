/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Francesco Romano
 *
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

#ifndef CODYCOLIB_MATH_H
#define CODYCOLIB_MATH_H

namespace Eigen {
    template<typename Derived> class MatrixBase;
}

namespace codyco {
    namespace math {
        
        template <typename Derived1, typename Derived2>
        void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                                 double dampingFactor,
                                 Eigen::MatrixBase<Derived2>& Apinv);
        
        template <typename Derived1, typename Derived2>
        void pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                           double tolerance,
                           Eigen::MatrixBase<Derived2>& Apinv);
    }
}
#endif
