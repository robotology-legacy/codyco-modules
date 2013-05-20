#ifndef UTIL_h
#define UTIL_h

#include "yarp/sig/Vector.h"
#include "yarp/sig/Matrix.h"

using namespace yarp::sig;
//using namespace yarp::math;

/** 
 * Compute both the damped pseudoinverse and the truncated pseudoinverse of a specified matrix
*/
void pinvDampTrunc(const Matrix &A, Matrix &Apinv, Matrix &ApinvDamp, double tol, double damp);
// this version returns also the singular value of A
void pinvDampTrunc(const Matrix &A, Matrix &Apinv, Matrix &ApinvDamp, Vector &sv, double tol, double damp);
// this version is more efficient because it takes as inputs the matrices U,V ans S so that it doesn't instantiate them
void pinvDampTrunc(const Matrix &A, Matrix &Apinv, Matrix &ApinvDamp, Vector &sv, double tol, double damp, 
                    Matrix &U, Matrix &V, Matrix &Spinv, Matrix &SpinvD);


#endif //UTIL_H