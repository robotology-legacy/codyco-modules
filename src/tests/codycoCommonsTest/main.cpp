/*
 * Author: Francesco Romano
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <Eigen/Core>
#include <codyco/MathUtils.h>
#include <iostream>

using namespace Eigen;

const double tolerance = 1e-8;


int main(int argc, char** argv)
{

    MatrixXd A(6, 7);
    A << 0.8147,    0.2785,    0.9572,    0.7922,    0.6787,    0.7060,    0.6948,
         0.9058,    0.5469,    0.4854,    0.9595,    0.7577,    0.0318,    0.3171,
         0.1270,    0.9575,    0.8003,    0.6557,    0.7431,    0.2769,    0.9502,
         0.9134,    0.9649,    0.1419,    0.0357,    0.3922,    0.0462,    0.0344,
         0.6324,    0.1576,    0.4218,    0.8491,    0.6555,    0.0971,    0.4387,
         0.0975,    0.9706,    0.9157,    0.9340,    0.1712,    0.8235,    0.3816;

    std::cout << "Input matrix is " << A << std::endl;
    
    MatrixXd pinvCheck(7, 6);
    pinvCheck << 0.3135,   -0.8134,   -0.5936,    0.8587,    1.1097,   -0.0815,
                -0.7506,   -0.5176,    0.2558,    0.7262,    0.5323,    0.4765,
                 2.6581,    7.9809,    0.8252,   -3.0226,  -10.5329,   -1.3595,
                -1.2197,   -1.2398,   -0.5389,    0.1343,    3.0770,    0.9465,
                 0.3896,    1.3872,    0.5693,   -0.4822,   -1.4685,   -0.7333,
                -0.4030,   -4.7659,   -0.8150,    1.6561,    5.1195,    1.1294,
                -0.8698,   -4.9317,    0.3431,    1.5009,    5.7192,    0.2668;
    
    MatrixXd Apinv(7, 6);
    
    codyco::math::pseudoInverse(A, Apinv, tolerance);
    
    std::cout << "Pseudo inverse " << Apinv << std::endl;
    
    std::cout << std::endl << "Error with matlab " << Apinv - pinvCheck << std::endl;
    
    return 0;
}


