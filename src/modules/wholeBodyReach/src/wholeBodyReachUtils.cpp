/*
Copyright (c) 2010-2013 Tommaso Urli

Tommaso Urli    tommaso.urli@uniud.it   University of Udine

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include <stdio.h>
#include <iostream>
#include <iomanip>      // std::setprecision
#include "wholeBodyReach/wholeBodyReachUtils.h"
#include "wholeBodyReach/wholeBodyReachConstants.h"

using namespace std;
using namespace wholeBodyReach;
using namespace Eigen;

VectorXd wholeBodyReach::svdSolveTransposeWithDamping(const JacobiSVD<MatrixRXd>& A, VectorConst b, double damping)
{
    eigen_assert(A.computeU() && A.computeV() && "svdSolveTransposeWithDamping() requires both unitaries U and V to be computed (thin unitaries suffice).");
    assert(A.cols()==b.size());
    
    //    cout<<"b = "<<toString(b,1)<<endl;
    VectorXd tmp(A.rows());
    int nzsv = A.nonzeroSingularValues();
    tmp.noalias() = A.matrixV().leftCols(nzsv).adjoint() * b;
    //    cout<<"V^T*b = "<<toString(tmp,1)<<endl;
    double sv, d2 = damping*damping;
    for(int i=0; i<nzsv; i++)
    {
        sv = A.singularValues()(i);
        tmp(i) *= sv/(sv*sv + d2);
    }
    //    cout<<"S^+ U^T b = "<<toString(tmp,1)<<endl;
    VectorXd res = A.matrixU().leftCols(nzsv) * tmp;
    
    //    getLogger().sendMsg("sing val = "+toString(A.singularValues(),3), MSG_STREAM_INFO);
    //    getLogger().sendMsg("solution with damp "+toString(damping)+" = "+toString(res.norm()), MSG_STREAM_INFO);
    //    getLogger().sendMsg("solution without damping  ="+toString(A.solve(b).norm()), MSG_STREAM_INFO);
    
    return res;
}

//**************************************************************************************************
VectorXd wholeBodyReach::svdSolveWithDamping(const JacobiSVD<MatrixRXd>& A, VectorConst b, double damping)
{
    eigen_assert(A.computeU() && A.computeV() && "svdSolveWithDamping() requires both unitaries U and V to be computed (thin unitaries suffice).");
    assert(A.rows()==b.size());
    
    //    cout<<"b = "<<toString(b,1)<<endl;
    VectorXd tmp(A.cols());
    int nzsv = A.nonzeroSingularValues();
    tmp.noalias() = A.matrixU().leftCols(nzsv).adjoint() * b;
    //    cout<<"U^T*b = "<<toString(tmp,1)<<endl;
    double sv, d2 = damping*damping;
    for(int i=0; i<nzsv; i++)
    {
        sv = A.singularValues()(i);
        tmp(i) *= sv/(sv*sv + d2);
    }
    //    cout<<"S^+ U^T b = "<<toString(tmp,1)<<endl;
    VectorXd res = A.matrixV().leftCols(nzsv) * tmp;
    
    //    getLogger().sendMsg("sing val = "+toString(A.singularValues(),3), MSG_STREAM_INFO);
    //    getLogger().sendMsg("solution with damp "+toString(damping)+" = "+toString(res.norm()), MSG_STREAM_INFO);
    //    getLogger().sendMsg("solution without damping  ="+toString(A.solve(b).norm()), MSG_STREAM_INFO);
    
    return res;
}


//*************************************************************************************************************************
//************************************************* OLD STUFF *************************************************************
//*************************************************************************************************************************
void wholeBodyReach::pinvTrunc(const MatrixRXd &A, double tol, MatrixRXd &Apinv, MatrixRXd &Spinv, VectorXd &sv)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    Spinv.setZero(k,k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    sv = svd.singularValues();
    // compute pseudoinverse of singular value matrix
    for (int c=0;c<k; c++)
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    // compute pseudoinverse
    Apinv = svd.matrixV() * Spinv  * svd.matrixU().transpose();
}


//*************************************************************************************************************************
void wholeBodyReach::pinvDampTrunc(const MatrixRXd &A, double tol, double damp, MatrixRXd &Apinv, MatrixRXd &ApinvDamp)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    MatrixRXd Spinv = MatrixRXd::Zero(k,k);
    MatrixRXd SpinvD = MatrixRXd::Zero(k,k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    VectorXd sv = svd.singularValues();
    // compute pseudoinverses of singular value matrix
    double damp2 = damp*damp;
    for (int c=0;c<k; c++)
    {
        SpinvD(c,c) = sv(c) / (sv(c)*sv(c) + damp2);
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    }
    // compute pseudoinverses
    Apinv       = svd.matrixV() * Spinv  * svd.matrixU().transpose();   // truncated pseudoinverse
    ApinvDamp   = svd.matrixV() * SpinvD * svd.matrixU().transpose();   // damped pseudoinverse
}

//**************************************************************************************************
Eigen::MatrixRXd wholeBodyReach::pinvDampedEigen(const Eigen::Ref<const Eigen::MatrixRXd> &A, double damp)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    VectorXd SpinvD = VectorXd::Zero(k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    VectorXd sv = svd.singularValues();
    // compute pseudoinverse of singular value matrix
    double damp2 = damp*damp;
    for (int c=0;c<k; c++)
        SpinvD(c) = sv(c) / (sv(c)*sv(c) + damp2);
    // compute damped pseudoinverse
    return svd.matrixV() * SpinvD.asDiagonal() * svd.matrixU().transpose();
}

//**************************************************************************************************
Eigen::MatrixRXd wholeBodyReach::nullSpaceProjector(const Eigen::Ref<const MatrixRXd> &A, double tol)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    MatrixRXd Spinv = MatrixRXd::Zero(k,k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    VectorXd sv = svd.singularValues();
    // compute pseudoinverse of singular value matrix
    for (int c=0;c<k; c++)
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    // compute pseudoinverse
    MatrixRXd N = MatrixRXd::Identity(n,n);
    N -= svd.matrixV() * Spinv  * svd.matrixU().transpose() * A;
    return N;
}

//*************************************************************************************************************************
void wholeBodyReach::assertEqual(const MatrixRXd &A, const MatrixRXd &B, string testName, double tol)
{
    if(A.cols() != B.cols() || A.rows()!=B.rows())
    {
        cout<< testName<< ": dim(A) != dim(B): " << A.rows()<< "x"<<A.cols()<<" != "<< B.rows()<< "x"<<B.cols()<<endl;
        return testFailed(testName);
    }
    for(int r=0; r<A.rows(); r++)
        for(int c=0; c<A.cols(); c++)
            if(abs(A(r,c)-B(r,c))>tol)
            {
                printf("%s: element %d,%d is different, absolute difference is %f\n", testName.c_str(), r, c, abs(A(r,c)-B(r,c)));
                return testFailed(testName);
            }
}

//*************************************************************************************************************************
void wholeBodyReach::testFailed(string testName)
{
    printf("Test %s ***FAILED*** !!!\n", testName.c_str());
    assert(false);
}

//*************************************************************************************************************************
std::string wholeBodyReach::toString(Eigen::MatrixConst m, int precision, const char* endRowStr, int maxColsPerLine)
{
    // if m is a column vector print it as a row vector
    if(m.cols()==1)
        return toString(m.transpose(), precision, endRowStr, maxColsPerLine);
    
    string ret = "";
    if(m.rows()>1 && m.cols()>maxColsPerLine)
    {
        return ret+"("+toString(maxColsPerLine)+" cols)\n" +
        toString(m.leftCols(maxColsPerLine),precision,endRowStr,maxColsPerLine) + "\n" +
        toString(m.rightCols(m.cols()-maxColsPerLine),precision,endRowStr,maxColsPerLine);
    }
    char tmp[350];
    for(int i=0;i<m.rows();i++)
    {
        for(int j=0;j<m.cols();j++)
        {
            sprintf(tmp, "% .*lf\t", precision, m(i,j));
            ret+=tmp;
        }
        ret = ret.substr(0,ret.length()-1);     // remove the last character (tab)
        if(i<m.rows()-1)                          // if it is not the last row
            ret+= endRowStr;
    }
    return ret; //.substr(0, ret.length()-1);
}

//*************************************************************************************************************************
std::string wholeBodyReach::jointToString(const Eigen::VectorXd &j, int precision)
{
    if(j.size()!=ICUB_DOFS && j.size()!=ICUB_DOFS+6)
        cout<<"Error in size of joint vector: "<<j.size()<<endl;
    
    int index=0;
    string ret = "";
    char tmp[350];
    if(j.size()==ICUB_DOFS+6)
    {
        ret += "base(";
        for(int i=0;i<6;i++)
        {
            sprintf(tmp, "% .*lf ", precision, j(index));
            ret+=tmp;
            index++;
        }
        ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
        ret += ")\t";
    }
    ret += "torso(";
    for(int i=0;i<3;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tl_arm(";
    for(int i=0;i<5;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tr_arm(";
    for(int i=0;i<5;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tl_leg(";
    for(int i=0;i<6;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tr_leg(";
    for(int i=0;i<6;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret += ")";
    return ret;
}

