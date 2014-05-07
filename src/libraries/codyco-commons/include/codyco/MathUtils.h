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

#include <Eigen/Core>

//namespace Eigen {
//    template<typename Derived> class MatrixBase;
//    template<typename _Scalar, int _Rows, int _Cols>//, int _Options, int _MaxRows, int _MaxCols>
//    class Matrix;
//    template<typename PlainObjectType> class Ref;
//    extern const int Dynamic = -1;
//}

namespace codyco {
    namespace math {
        
        template <typename Derived1, typename Derived2>
        void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                                 double dampingFactor,
                                 Eigen::MatrixBase<Derived2>& Apinv,
                                 unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV);
        
//        template <typename Derived1, typename Derived2>
//        void pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
//                           double tolerance,
//                           Eigen::MatrixBase<Derived2>& Apinv);
        
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           double tolerance,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV);
    }
}
#endif
