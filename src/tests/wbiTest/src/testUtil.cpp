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

bool wbiTest::assertEqual(const wbi::Frame &F1, const wbi::Frame &F2, const std::string &testName, double eps)
{
    bool res = assertTrue(isEqual(F1,F2,eps), testName);
    if(!res) cout<<"F1!=F2, where F1=\n"<<F1.toString()<<"\nF2:\n"<<F2.toString()<<endl;
    return res;
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

bool wbiTest::isRototranslationMatrixProper(const Frame &F)
{
    int res = 0;
    double x[3], y[3], z[3], M[16];
    F.get4x4Matrix(M);
    x[0]=M[0]; x[1]=M[4]; x[2]=M[8];
    y[0]=M[1]; y[1]=M[5]; y[2]=M[9];
    z[0]=M[2]; z[1]=M[6]; z[2]=M[10];
    res += assertTrue(equal(norm3d(x),1.0), "norm(x)=1") ? 0 : 1;
    res += assertTrue(equal(norm3d(y),1.0), "norm(y)=1") ? 0 : 1;
    res += assertTrue(equal(norm3d(z),1.0), "norm(z)=1") ? 0 : 1;
    res += assertTrue(equal(dot3d(x,y), 0.0), "x orthogonal y") ? 0 : 1;
    res += assertTrue(equal(dot3d(x,z), 0.0), "x orthogonal z") ? 0 : 1;
    res += assertTrue(equal(dot3d(y,z), 0.0), "y orthogonal z") ? 0 : 1;
    res += assertTrue(equal(M[12],0.0), "M[12]=0") ? 0 : 1;
    res += assertTrue(equal(M[13],0.0), "M[13]=0") ? 0 : 1;
    res += assertTrue(equal(M[14],0.0), "M[14]=0") ? 0 : 1;
    res += assertTrue(equal(M[15],1.0), "M[15]=1") ? 0 : 1;
    if(res>0)
        cout<< (F.toString(4)) <<endl;
    return res==0;
}

string wbiTest::toString(const double *x, int size)
{
    stringstream s;
    for(int i=0;i<size;i++)
        s<<x[i]<<" ";
    return s.str();
}