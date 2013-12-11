#include "util.h"
#include "yarp/sig/Vector.h"
#include "yarp/sig/Matrix.h"
#include "yarp/math/Math.h"
#include "yarp/math/SVD.h"

using namespace yarp::sig;
using namespace yarp::math;

void pinvDampTrunc(const Matrix &A, Matrix &Apinv, Matrix &ApinvDamp, double tol, double damp)
{
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    Vector sv(k);
    pinvDampTrunc(A, Apinv, ApinvDamp, sv, tol, damp);
}

void pinvDampTrunc(const Matrix &A, Matrix &Apinv, Matrix &ApinvDamp, Vector &sv, double tol, double damp)
{
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    Matrix U(m,k), V(n,k), Spinv=zeros(k,k), SpinvD=zeros(k,k);
    pinvDampTrunc(A, Apinv, ApinvDamp, sv, tol, damp, U, V, Spinv, SpinvD);
}

void pinvDampTrunc(const Matrix &A, Matrix &Apinv, Matrix &ApinvDamp, Vector &sv, double tol, double damp, 
    Matrix &U, Matrix &V, Matrix &Spinv, Matrix &SpinvD)
{
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    yarp::math::SVD(A, U, sv, V);
    
    double damp2 = damp*damp;
    for (int c=0;c<k; c++)
    {
        SpinvD(c,c) = sv(c) / (sv(c)*sv(c) + damp2);
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    }

    Apinv = V*Spinv*U.transposed();
    ApinvDamp = V*SpinvD*U.transposed();
}