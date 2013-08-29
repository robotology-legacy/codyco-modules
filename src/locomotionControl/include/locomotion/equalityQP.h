/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#ifndef EQUALITY_QP
#define EQUALITY_QP

#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>

using namespace Eigen;
using namespace std;

namespace locomotion
{

/** Compute the truncated pseudoinverse of the specified matrix A. 
  * This version of the function takes addtional input matrices to avoid allocating memory and so improve 
  * the efficiency of the computation. 
  * @param A Input mXn matrix.
  * @param tol Input threshold for the singular values of the truncated pseudoinverse.
  * @param U Output mXm matrix, left eigenvectors of A. 
  * @param V Output nXn matrix, right eigenvectors of A. 
  * @param Spinv Output kXk matrix (with k=min(m,n)), truncated pseudoinverse of the singular value matrix of A. 
  * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
  * @param sv Output (optional) k-dim vector (with k=min(m,n)), singular values of A. */
void pinvTrunc(const MatrixXd &A, double tol, MatrixXd &Apinv, VectorXd *sv=0);

/** Compute two different pseudoinverses of the specified matrix A: a truncated pseudoinverse and a
  * damped pseudoinverse. The difference between the two versions is that the truncated version sets to zero
  * all the singular values that are less than a certain threshold (tol), whereas the damped version
  * uses computes this expression: \f[ A^+ = A^T(AA^T+\lambda I)^{-1}\f], where \f[ \lambda \f] is the damping 
  * factor. Both pseudoinverses are computed from the singular value decomposition of A.
  * @param A Input mXn matrix.
  * @param tol Input threshold for the singular values of the truncated pseudoinverse.
  * @param damp Input damping factor for the damped pseudoinverse.
  * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
  * @param ApinvDamp Output nXm matrix, damped pseudoinverse of A.
  * @param sv Output (optional) k-dim vector (with k=min(m,n)), singular values of A. */
void pinvDampTrunc(const MatrixXd &A, double tol, double damp, MatrixXd &Apinv, MatrixXd &ApinvDamp, VectorXd *sv=0);

}

#endif
