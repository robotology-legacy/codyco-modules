/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef _MOTOR_FRICTION_EXCITATION_CONSTANTS
#define _MOTOR_FRICTION_EXCITATION_CONSTANTS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include <yarp/sig/Matrix.h>
#include <motorFrictionIdentificationLib/motorFrictionExcitationParams.h>

using namespace Eigen;


// define some types
typedef Eigen::Matrix<double,1,1>                  Vector1d;
typedef Eigen::Matrix<int,ICUB_DOFS,1>             VectorNi;

namespace motorFrictionExcitation
{
/** Types of printed messages */
enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

// *** CONSTANTS
static const int        PRINT_PERIOD    = 2000;         ///< period of debug prints (in ms)
static const int        PRINT_MSG_LEVEL = MSG_DEBUG;    ///< only messages whose type is greater than or equal to PRINT_MSG_LEVEL are printed
static const double     PWM_MAX         = 400;          ///< max motor PWM allowed (in [-1333; 1333])
static const double     MAX_POS_INTEGRAL = 50.0;        ///< max position error integral (in PWM units)

}   // end namespace 

#endif
