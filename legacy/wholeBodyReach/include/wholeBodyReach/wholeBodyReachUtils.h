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

#ifndef WHOLE_BODY_REACH_UTILS
#define WHOLE_BODY_REACH_UTILS

#include <Eigen/Core>                               // import most common Eigen types
#include <Eigen/SVD>
#include <vector>
#include <string>
#include <iterator>     // std::ostream_iterator
#include <iCub/skinDynLib/common.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <yarp/os/BufferedPort.h>
#include <wbi/wbi.h>
#include <wholeBodyReach/wholeBodyReachConstants.h>

namespace Eigen
{
    typedef Matrix<double,6,1>                      Vector6d;
    typedef Matrix<double,7,1>                      Vector7d;
    typedef Matrix<double,ICUB_DOFS,1>              VectorNd;
    typedef Matrix<int,ICUB_DOFS,1>                 VectorNi;
    typedef Matrix<double,6,Dynamic,RowMajor>       JacobianMatrix;     // a Jacobian is 6 rows and N columns

    typedef Matrix<double,Dynamic,Dynamic,RowMajor> MatrixRXd;     /// Dynamic matrix with row-major storage order
    typedef Matrix<double,3,3,RowMajor>             MatrixR3d;
    typedef Matrix<double,4,4,RowMajor>             MatrixR4d;
    typedef Matrix<double,6,6,RowMajor>             MatrixR6d;

    typedef Ref<VectorXd>                           VectorRef;      /// Type used to pass Eigen vectors by reference
    typedef Ref<MatrixRXd>                          MatrixRef;      /// Type used to pass Eigen matrices by reference
    typedef const Ref<const VectorXd>&              VectorConst;    /// Type used to pass Eigen vectors by const reference
    typedef const Ref<const MatrixRXd>&             MatrixConst;    /// Type used to pass Eigen matrices by const reference

    typedef JacobiSVD<MatrixRXd>                    SVD;            /// svd of RowMajor matrix
    typedef LDLT<MatrixRXd>                         Cholesky;       /// Cholesky decomposition of RowMajor matrix

//    class JacobiSVDExtended{};
}

namespace wholeBodyReach
{
    /** Utility class for reading the skin contacts.
     */
    class SkinContactReader
    {
        std::string         _name;
        std::string         _robotName;

        // input port reading external contacts
        yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> *port_ext_contacts;
        // last list of contacts
        iCub::skinDynLib::skinContactList     contactList;
        // list of contacts on interested link
        iCub::skinDynLib::skinContactList     partContList;
        // contact chosen for the controller
        iCub::skinDynLib::skinContact*        contact;
    public:
        SkinContactReader(std::string name, std::string robotName);

        /** Open the port and connect to wholeBodyDynamics. */
        bool init(std::string wholeBodyDynamicsName);

        /** Read the skin contact.
         * @param blocking If true the reading is blocking.
         * @return True if the reading succeeded, false otherwise. */
        bool readContacts(bool blocking=false);

        /** Returns true if there is at least one contact on the specified skin part. */
        bool isThereContactOnSkinPart(iCub::skinDynLib::SkinPart sp);

        /** Get the contact (if any) on the specified skin part.
         * In case of more contacts get the one that activates the most number of taxels. */
        bool getContactOnSkinPart(iCub::skinDynLib::SkinPart sp, iCub::skinDynLib::skinContact &c);

        /**
         * Return true if _robotName is icubSim or icubGazeboSim, false otherwise
         */
        bool isRobotSimulator(std::string _robotName);
    };

    Eigen::VectorXd svdSolveWithDamping(const Eigen::JacobiSVD<Eigen::MatrixRXd>& svd, Eigen::VectorConst b, double damping=0.0);

    Eigen::VectorXd svdSolveTransposeWithDamping(const Eigen::JacobiSVD<Eigen::MatrixRXd>& svd, Eigen::VectorConst b, double damping=0.0);

    /** Compute the truncated pseudoinverse of the specified matrix A.
     * This version of the function takes addtional input matrices to avoid allocating memory and so improve
     * the efficiency of the computation.
     * @param A Input mXn matrix.
     * @param tol Input threshold for the singular values of the truncated pseudoinverse.
     * @param Spinv Output kXk matrix (with k=min(m,n)), truncated pseudoinverse of the singular value matrix of A.
     * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
     * @param sv Output (optional) k-dim vector (with k=min(m,n)), singular values of A. */
    void pinvTrunc(const Eigen::MatrixRXd &A, double tol, Eigen::MatrixRXd &Apinv, Eigen::MatrixRXd &Spinv, Eigen::VectorXd &sv);

    /** Compute two different pseudoinverses of the specified matrix A: a truncated pseudoinverse and a
     * damped pseudoinverse. The difference between the two versions is that the truncated version sets to zero
     * all the singular values that are less than a certain threshold (tol), whereas the damped version
     * uses computes this expression: \f[ A^+ = A^T(AA^T+\lambda I)^{-1}\f], where \f[ \lambda \f] is the damping
     * factor. Both pseudoinverses are computed from the singular value decomposition of A.
     * @param A Input mXn matrix.
     * @param tol Input threshold for the singular values of the truncated pseudoinverse.
     * @param damp Input damping factor for the damped pseudoinverse.
     * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
     * @param ApinvDamp Output nXm matrix, damped pseudoinverse of A.*/
    void pinvDampTrunc(const Eigen::MatrixRXd &A, double tol, double damp, Eigen::MatrixRXd &Apinv, Eigen::MatrixRXd &ApinvDamp);

    Eigen::MatrixRXd nullSpaceProjector(const Eigen::Ref<const Eigen::MatrixRXd> &A, double tol);

    Eigen::MatrixRXd pinvDampedEigen(const Eigen::Ref<const Eigen::MatrixRXd> &A, double damp);

    void adjointInv(const Eigen::Ref<const Eigen::Vector3d> &p, Eigen::Ref<Eigen::MatrixR6d> A);



    std::string toString(Eigen::MatrixConst m, int precision=2, const char* endRowStr="\n", int maxRowsPerLine=10);

    std::string jointToString(const Eigen::VectorXd &j, int precision=1);

    /** Convert a generic variable into a string. */
    template <class T> inline std::string toString(const T& t)
    { std::ostringstream ss; ss << t; return ss.str(); }

    /** Convert a generic vector into a string */
    template <class T> inline std::string toString(const std::vector<T>& v, const char *separator=" ")
    { std::ostringstream s; std::copy(v.begin(), v.end(), std::ostream_iterator<T>(s, separator)); return s.str(); }




    /** Tolerance for considering two values equal */
    const double ZERO_TOL = 1e-5;

    void assertEqual(const Eigen::MatrixRXd &A, const Eigen::MatrixRXd &B, std::string testName, double tol = ZERO_TOL);

    void testFailed(std::string testName);


}

#endif
