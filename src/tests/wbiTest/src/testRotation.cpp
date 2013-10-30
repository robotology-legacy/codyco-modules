/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <wbiTest/testRotation.h>
#include <wbi/wbiUtil.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace wbi;

#define TOL         1e-8
#ifndef M_PI
    #define M_PI        3.14159265358979323846
#endif

bool wbiTest::testRotation()
{
    int err = 0;    // number of failed tests

    // Generate random data
    double ang1 = Rand::scalar()*2*M_PI;
    double ang2 = Rand::scalar()*2*M_PI;
    double ang3 = Rand::scalar()*2*M_PI;
    double v[3] = {3.2, 6.5, 1.1};
    double w[3] = {1.2, 4.5, 7.1};
    double tmp4[4];
    double tmp3[3];
    Vector q = Rand::vector(4);             // quaternion
    q /= norm(q);
    Vector aa = Rand::vector(4);            // axis-angle
    aa.setSubvector(0, aa.subVector(0,2)/norm(aa.subVector(0,2)));
    Vector rpy = Rand::vector(3);           // roll-pitch-yaw
    Vector zyz = Rand::vector(3);           // Euler zyz
    Vector zyx = Rand::vector(3);           // Euler zyx

    // Create rotations
    Rotation Rx = Rotation::rotX(ang1);                 // random rotation about X
    Rotation Ry = Rotation::rotY(ang2);                 // random rotation about Y
    Rotation Rz = Rotation::rotZ(ang3);                 // random rotation about Z
    Rotation Rquat = Rotation::quaternion(q.data());    // quaternion
    Rotation Raa = Rotation::axisAngle(aa.data());      // axis angle
    Rotation Rrpy = Rotation::RPY(rpy.data());          // RPY
    Rotation Rzyz = Rotation::eulerZYZ(zyz.data());     // ZYZ
    Rotation Rzyx = Rotation::eulerZYX(zyx.data());     // ZYX
    Rotation I;                                         // identity rotation

    // TEST CONSTRUCTORS OF ROTATION MATRICES FROM DIFFERENT ROTATION REPRESENTATIONS
    err += assertTrue(isRotationMatrixProper(Rx), "Rotation about X") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Ry), "Rotation about Y") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Rz), "Rotation about Z") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Rquat), "Rotation from quaternion") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Raa), "Rotation from axis angle") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Rrpy), "Rotation from rpy") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Rzyz), "Rotation from zyz") ? 0 : 1;
    err += assertTrue(isRotationMatrixProper(Rzyx), "Rotation from zyx") ? 0 : 1;

    // TEST CONSTRUCTORS OF ELEMENTARY ROTATION MATRICES (ROTATIONS ABOUT PRINCIPAL AXES)
    err += assertEqual(Rotation::rotX(ang1)*Rotation::rotX(-ang1), Rotation::identity(), "RotX(a)*RotX(-a) = I") ? 0 : 1;
    err += assertEqual(Rotation::rotY(ang2)*Rotation::rotY(-ang2), Rotation::identity(), "RotY(a)*RotY(-a) = I") ? 0 : 1;
    err += assertEqual(Rotation::rotZ(ang3)*Rotation::rotZ(-ang3), Rotation::identity(), "RotZ(a)*RotZ(-a) = I") ? 0 : 1;

    // TEST EXTRACTION OF DIFFERENT ROTATION REPRESENTATIONS FROM ROTATION MATRICES
    Rquat.getQuaternion(tmp4);
    err += assertEqual(tmp4, q.data(), 4, "Rot(q).getQuaternion() = q") ? 0 : 1;
    Raa.getAxisAngle(tmp4);
    err += assertEqual(tmp4, aa.data(), 4, "Rot(aa).getAxisAngle() = aa") ? 0 : 1;
    Rrpy.getRPY(tmp3);
    err += assertEqual(tmp3, rpy.data(), 3, "Rot(rpy).getRPY() = rpy") ? 0 : 1;
    Rzyz.getEulerZYZ(tmp3);
    err += assertEqual(tmp3, zyz.data(), 3, "Rot(zyz).getEulerZYZ() = zyz") ? 0 : 1;
    Rzyx.getEulerZYX(tmp3);
    err += assertEqual(tmp3, zyx.data(), 3, "Rot(zyx).getEulerZYX() = zyx") ? 0 : 1;

    // TEST VECTOR ROTATION AND ROTATION INVERSE
    err += assertEqual(Rx.getInverse().getInverse(), Rx, "inv(inv(R)) = R") ? 0 : 1;
    I.rotate(v, tmp3);
    err += assertEqual(tmp3, v, 3, "I*v = v") ? 0 : 1;
    Rquat.rotate(v, tmp3);
    Rquat.rotateInverse(tmp3);
    err += assertEqual(v, tmp3, 3, "R^T * (R * v) = v") ? 0 : 1;
    Raa.rotateInverse(w, tmp3);
    Raa.rotate(tmp3);
    err += assertEqual(w, tmp3, 3, "R * (R^T * v) = v") ? 0 : 1;
    err += assertEqual(I.getInverse(), I, "inv(I) = I") ? 0 : 1;

    // TEST COPY CONSTRUCTOR AND OPERATORS =, == AND !=
    err += assertTrue(Rzyz==Rzyz, "R = R") ? 0 : 1;
    err += assertTrue(Rzyz!=Rzyx, "R1 != R2") ? 0 : 1;
    Rotation R1 = Raa;
    err += assertEqual(Raa, R1, "copy constructor") ? 0 : 1;
    Rotation R2; R2 = Raa;
    err += assertEqual(Raa, R2, "assignment operator") ? 0 : 1;

    cout<<"\nNumber of failed tests: "<< err<< endl;
    return err==0;
}

bool wbiTest::assertTrue(bool condition, const string &testName)
{
    if(!condition)
        cout<<"*TEST FAILED*:  "<<testName<<endl;
#ifdef VERBOSE
    else    
        cout<<"test succeeded: "<<testName<<endl;
#endif
        
    return condition;
}

bool wbiTest::assertEqual(const wbi::Rotation &R1, const wbi::Rotation &R2, const std::string &testName, double eps)
{
    bool res = assertTrue(isEqual(R1,R2,eps), testName);
    if(!res) cout<<"R1!=R2, where R1=\n"<<R1.toString()<<"\nR2:\n"<<R2.toString()<<endl;
    return res;
}

bool wbiTest::assertEqual(const double x[], const double y[], int size, const std::string &testName, double eps)
{
    bool res = assertTrue(isEqual(x,y,size,eps), testName);
    if(!res) 
        cout<<"x!=y, where x="<<toString(x,3)<<" and y="<<toString(y,3)<<endl;
    return res;
}

bool wbiTest::isEqual(const double x[], const double y[], int size, double eps)
{
    bool res = true;
    for(int i=0;i<size && res;i++)
        res = res && equal(x[i],y[i],eps);
    return res;
}

bool wbiTest::isRotationMatrixProper(const Rotation &R)
{
    int res = 0;
    double x[3], y[3], z[3];
    R.getUnitX(x);
    R.getUnitY(y);
    R.getUnitZ(z);
    res += assertTrue(equal(norm3d(x),1.0), "norm(x)=1") ? 0 : 1;
    res += assertTrue(equal(norm3d(y),1.0), "norm(y)=1") ? 0 : 1;
    res += assertTrue(equal(norm3d(z),1.0), "norm(z)=1") ? 0 : 1;
    res += assertTrue(equal(dot3d(x,y), 0.0), "x orthogonal y") ? 0 : 1;
    res += assertTrue(equal(dot3d(x,z), 0.0), "x orthogonal z") ? 0 : 1;
    res += assertTrue(equal(dot3d(y,z), 0.0), "y orthogonal z") ? 0 : 1;
    if(res>0)
        cout<< (R.toString(4)) <<endl;
    return res==0;
}

string wbiTest::toString(const double *x, int size)
{
    stringstream s;
    for(int i=0;i<size;i++)
        s<<x[i]<<" ";
    return s.str();
}