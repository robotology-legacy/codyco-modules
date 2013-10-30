/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for wholeBodyInterface.
 */

#include <wbiTest/testRotation.h>
#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace std;
using namespace wbiTest;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    if(!testRotation())
        printf("\n*** SOME OF THE ROTATION TESTS HAVE FAILED! ***\n");

    char c[10];
    cout<<"TEST FINISHED. ENTER ANY STRING AND PRESS ENTER TO EXIT.";
    cin.getline(c,10);
    return 0;
}


