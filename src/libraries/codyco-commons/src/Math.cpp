#include "codyco/Math.h"
#include <Eigen/SVD>

namespace codyco {

    namespace math {
        
        template <typename Derived1, typename Derived2>
        void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                                                        double dampingFactor,
                                                        Eigen::MatrixBase<Derived2>& Apinv)
        {
            using namespace Eigen;
            
            int m = A.rows(), n = A.cols(), k = m < n ? m : n;
            JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
            const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType& singularValues = svd.singularValues();
            MatrixXd sigmaDamped = MatrixXd::Zero(k, k);
            
            double damp = dampingFactor * dampingFactor;
            for (int idx = 0; idx < k; idx++) {
                sigmaDamped(idx, idx) = singularValues(idx) / (singularValues(idx) * singularValues(idx) + damp);
            }
            Apinv = svd.matrixV() * sigmaDamped * svd.matrixU().transpose();   // damped pseudoinverse
        }
        
        void pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                           double tolerance,
                           Eigen::MatrixBase<Derived2>& Apinv)
        {
            using namespace Eigen;
            JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
            typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType singularValues = svd.singularValues();
            
            for (int idx = 0; idx < singularValues.size(); idx++) {
                singularValues(idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
            }
            
            Apinv = svd.matrixV() * singularValues.asDiagonal() * svd.matrixU().adJoint();   //pseudoinverse
        }
    }
}
