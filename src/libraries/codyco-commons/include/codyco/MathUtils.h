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
#include <Eigen/SVD>

//namespace Eigen {
//    template<typename Derived> class MatrixBase;
//    template<typename _Scalar, int _Rows, int _Cols>//, int _Options, int _MaxRows, int _MaxCols>
//    class Matrix;
//    template<typename PlainObjectType> class Ref;
//    extern const int Dynamic = -1;
//}

namespace codyco {
    namespace math {
        
        /** Check if a double is isnan.
         * We reimplement this function to avoid depending on C++11 (std::isnan) 
         * or C99 isnan macro.
         *
         * @param value value to be checked for nan
         * @return true if the value is a nan, false otherwise.
         */
        inline bool isnan(double value)
        {
            volatile double currentValue = value;
            return currentValue != currentValue;
        }
        
        template <typename Derived1, typename Derived2>
        void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                                 double dampingFactor,
                                 Eigen::MatrixBase<Derived2>& Apinv,
                                 unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV);
        
//        template <typename Derived1, typename Derived2>
//        void pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
//                           double tolerance,
//                           Eigen::MatrixBase<Derived2>& Apinv);
        
        /** @brief Computes the truncated pseudo inverse of a matrix
         *
         * This function computes the truncated pseudo inverse of a matrix.
         * Singular values less than the input tolerance will be set to sero.
         * By default the pseudo inverse will be computed by using the thin U and V unitary matrices.
         * @todo add default tolerance value.
         * @note this method will allocate memory for the SVD decomposition. If want to avoid this use 
         * codyco::math::pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>&, Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject>&,
         * Eigen::Ref<Eigen::MatrixXd>, double, unsigned int)
         *
         * @param A the matrix to be pseudoinverted
         * @param Apinv the matrix in which to save the pseudoinversion of A. The size must be correct (same as \f$A^\top\f$)
         * @param tolerance tolerance to be used for the truncation
         * @param computationOptions Eigen options for the computation. By default compute the thin U and V matrices.
         */
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV);
        
        /** @brief Computes the truncated pseudo inverse of a matrix
         *
         * This function computes the truncated pseudo inverse of a matrix.
         * Singular values less than the input tolerance will be set to sero.
         * By default the pseudo inverse will be computed by using the thin U and V unitary matrices.
         * @todo add default tolerance value.
         *
         * @param A the matrix to be pseudoinverted
         * @param svdDecomposition the decomposition object (already allocated) to be used
         * @param Apinv the matrix in which to save the pseudoinversion of A. The size must be correct (same as \f$A^\top\f$)
         * @param tolerance tolerance to be used for the truncation
         * @param computationOptions Eigen options for the computation. By default compute the thin U and V matrices.
         */
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject>& svdDecomposition,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV);
        
        /** @brief Computes the skew-symmetric (3D) matrix form of 
         *  the input 3D vector for cross-product computation
         *
         * This function computes the skew symmetric matrix form of the input
         * vector needed for cross-product computation, i.e. 
         * given the following cross product
         * \f[
         *      \vec{a} \times \vec{b} = \left[ \vec{a} \right]_\times \vec{b}
         * \f]
         * this function returns \f$ \left[ \vec{a} \right]_\times \f$.
         *
         * @param vector 3D vector
         * @param skewSymmetrixMatrix resultant skew symmetric matrix
         */
        void skewSymmentricMatrixFrom3DVector(const Eigen::Ref<const Eigen::Vector3d>& vector, Eigen::Ref<Eigen::Matrix3d> skewSymmetricMatrix);
    }
}
#endif
