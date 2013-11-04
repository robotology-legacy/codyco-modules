/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <wbiTest/testFrame.h>
#include <wbi/wbiUtil.h>
#include <wbiTest/testUtil.h>
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

#define N_TEST 100
#define TOL         1e-8
#ifndef M_PI
    #define M_PI        3.14159265358979323846
#endif

bool wbiTest::testFrame()
{
    cout<< "*** TEST FRAME ***\n";
    int err = 0;    // number of failed tests
    double tmp3[3], tmp3b[3];
    Vector rpy, zyz, p1, p2, v;
    
    Frame I, F, G;
    for(int i=0; i<N_TEST && err==0; i++)
    {
        rpy = Rand::vector(3);
        zyz = Rand::vector(3);
        v  = Rand::vector(3);
        p1 = Rand::vector(3);
        p2 = Rand::vector(3);
        F = Frame(Rotation::RPY(rpy.data()), p1.data());
        G = Frame(Rotation::eulerZYZ(zyz.data()), p2.data());

        // TEST CONSTRUCTORS OF ROTATION MATRICES FROM DIFFERENT ROTATION REPRESENTATIONS
        err += assertTrue(isRototranslationMatrixProper(F), "4x4 matrix is proper") ? 0 : 1;

        // TEST VECTOR ROTOTRANSLATION AND ITS INVERSE
        err += assertEqual(F.getInverse().getInverse(), F, "inv(inv(F)) = F") ? 0 : 1;
        err += assertEqual(F.getInverse()*F, Frame::identity(), "inv(F)*F = I") ? 0 : 1;
        err += assertEqual(F*F.getInverse(), Frame::identity(), "F*inv(F) = I") ? 0 : 1;

        F.rototranslate(v.data(), tmp3);
        F.rototranslateInverse(tmp3);
        err += assertEqual(v.data(), tmp3, 3, "inv(F) * (F * v) = v") ? 0 : 1;

        F.rototranslateInverse(v.data(), tmp3);
        F.rototranslate(tmp3);
        err += assertEqual(v.data(), tmp3, 3, "F * (inv(F) * v) = v") ? 0 : 1;

        (F*G).rototranslate(v.data(), tmp3);
        G.rototranslate(v.data(), tmp3b);
        F.rototranslate(tmp3b);
        err += assertEqual(tmp3b, tmp3, 3, "F*(G*v) = (F*G)*v") ? 0 : 1;

        I.rototranslate(v.data(), tmp3);
        err += assertEqual(tmp3, v.data(), 3, "I*v = v") ? 0 : 1;

        err += assertEqual(I.getInverse(), I, "inv(I) = I") ? 0 : 1;

        // TEST COPY CONSTRUCTOR AND OPERATORS =, == AND !=
        err += assertTrue(F==F, "F = F") ? 0 : 1;
        err += assertTrue(G!=F, "F1 != F2") ? 0 : 1;

        Frame F2 = F;
        err += assertEqual(F2, F, "copy constructor") ? 0 : 1;

        Frame G2; G2 = G;
        err += assertEqual(G, G2, "assignment operator") ? 0 : 1;
    }

    cout<<"\nNumber of failed tests: "<< err<< endl;
    return err==0;
}
