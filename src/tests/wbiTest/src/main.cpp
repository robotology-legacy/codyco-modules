/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for wholeBodyInterface.
 */

#include <wbiTest/testRotation.h>
#include <wbiTest/testFrame.h>
#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace std;
using namespace wbiTest;

int main(int argc, char * argv[])
{
    if(!testRotation())
        printf("\n*** Some tests on the Rotation class have FAILED! ***\n");
    if(!testFrame())
        printf("\n*** Some tests on the Frame class have FAILED! ***\n");

    char c[10];
    cout<<"\nTEST FINISHED. Enter any string and press ENTER to exit the program.";
    cin.getline(c,10);
    return 0;
}


