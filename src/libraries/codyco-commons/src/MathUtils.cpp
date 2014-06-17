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

#include "MathUtils.h"
#include <Eigen/SVD>

namespace codyco {

    namespace math {
        
        template <typename Derived1, typename Derived2>
        void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                                 double dampingFactor,
                                 Eigen::MatrixBase<Derived2>& Apinv,
                                 unsigned int computationOptions)
        {
            using namespace Eigen;
            
            int m = A.rows(), n = A.cols(), k = m < n ? m : n;
            JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(computationOptions);
            const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType& singularValues = svd.singularValues();
            MatrixXd sigmaDamped = MatrixXd::Zero(k, k);
            
            double damp = dampingFactor * dampingFactor;
            for (int idx = 0; idx < k; idx++) {
                sigmaDamped(idx, idx) = singularValues(idx) / (singularValues(idx) * singularValues(idx) + damp);
            }
            Apinv = svd.matrixV() * sigmaDamped * svd.matrixU().transpose();   // damped pseudoinverse
        }
        
//        template <typename Derived1, typename Derived2>
//        void pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
//                           double tolerance,
//                           Eigen::MatrixBase<Derived2>& Apinv)
//        {
//            using namespace Eigen;
//            JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
//            typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType singularValues = svd.singularValues();
//            
//            for (int idx = 0; idx < singularValues.size(); idx++) {
//                singularValues(idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
//            }
//            
//            Apinv = svd.matrixV() * singularValues.asDiagonal() * svd.matrixU().adJoint();   //pseudoinverse
////            using namespace Eigen;
////            JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
////            typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType singularValues = svd.singularValues();
////            //I don't get why I cannot use the singular vector as in http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
////            MatrixXd singularInverse = singularValues.asDiagonal();
////            
////            for (int idx = 0; idx < singularValues.size(); idx++) {
////                singularInverse(idx, idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
////            }
////            
////            //            for (int idx = 0; idx < singularValues.size(); idx++) {
////            //                singularValues(idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
////            //            }
////            //            auto var = svd.matrixV() * sigmaDamped;
////            //            auto var2 = sigmaDamped * svd.matrixU().transpose();
////            //
////            //            Apinv = var * svd.matrixU().transpose();
////            
////            Apinv = (svd.matrixV() * singularInverse * svd.matrixU().adjoint());   //pseudoinverse
//        }
        
        
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           unsigned int computationOptions)

        {
            Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject> svdDecomposition(A.rows(), A.cols());
            pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);
        }
        
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           unsigned int computationOptions)
        {
            using namespace Eigen;
            svdDecomposition.compute(A, computationOptions);
            JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
            for (int idx = 0; idx < singularValues.size(); idx++) {
                singularValues(idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
            }
            Apinv = svdDecomposition.matrixV() * singularValues.asDiagonal() * svdDecomposition.matrixU().adjoint();
        }
        
        void skewSymmentricMatrixFrom3DVector(const Eigen::Ref<const Eigen::Vector3d>& vector, Eigen::Ref<Eigen::Matrix3d> skewSymmetricMatrix)
        {
            skewSymmetricMatrix.setZero();
            //            S = [   0,   -w(3),    w(2);
            //                 w(3),   0,     -w(1);
            //                 -w(2),  w(1),     0   ];
            skewSymmetricMatrix(0, 1) = -vector(2);
            skewSymmetricMatrix(0, 2) = vector(1);
            skewSymmetricMatrix(1, 2) = -vector(0);
            skewSymmetricMatrix.bottomLeftCorner<2, 2>() = -skewSymmetricMatrix.topRightCorner<2, 2>().transpose();
        }
    }
}
