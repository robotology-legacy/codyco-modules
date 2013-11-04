/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef _TEST_UTIL_
#define _TEST_UTIL_

#include <string>
#include <wbi/wbiUtil.h>

namespace wbiTest
{
//#define VERBOSE
    std::string toString(const double *x, int size);

    bool isEqual(const double x[], const double y[], int size, double eps=EPSILON);

    bool assertEqual(const wbi::Frame &F1, const wbi::Frame &F2, const std::string &testName, double eps=EPSILON);
    bool assertEqual(const wbi::Rotation &R1, const wbi::Rotation &R2, const std::string &testName, double eps=EPSILON);
    bool assertEqual(const double x[], const double y[], int size, const std::string &testName, double eps=EPSILON);

    inline bool isSymmetric(const double M[9]){ return wbi::equal(M[1],M[3]) && wbi::equal(M[2],M[6]) && wbi::equal(M[5],M[7]); }

    inline bool isSkewSymmetric(const double M[9]){ return wbi::equal(M[1],-M[3]) && wbi::equal(M[2],-M[6]) && wbi::equal(M[5],-M[7]); }

    inline double dot3d(const double x[3], const double y[3]){ return x[0]*y[0]+x[1]*y[1]+x[2]*y[2]; }

    bool assertTrue(bool condition, const std::string &testName);

    /** Test whether the specified rotation matrix is properly defined. */
    bool isRotationMatrixProper(const wbi::Rotation &R);

    /** Test whether the specified frame is properly defined. */
    bool isRototranslationMatrixProper(const wbi::Frame &F);
}


#endif