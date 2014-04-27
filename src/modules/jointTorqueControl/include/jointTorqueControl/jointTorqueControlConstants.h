/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Daniele Pucci
 * email:  daniele.pucci@iit.it
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

#ifndef __JOINT_TORQUE_CONTROL_CONSTANTS
#define __JOINT_TORQUE_CONTROL_CONSTANTS

#include <motorFrictionIdentificationLib/jointTorqueControlParams.h>

namespace jointTorqueControl
{

#ifndef M_PI
    #define M_PI        3.14159265358979323846264338328
#endif

#define CTRL_RAD2DEG    (180.0/M_PI)
#define CTRL_DEG2RAD    (M_PI/180.0)

static const int        PRINT_PERIOD                   = 2000;     ///< period of debug prints (in ms)
static const double     TORQUE_INTEGRAL_SATURATION     = 10.0;     ///< value at which the torque error integral is saturated
static const double     JOINT_VEL_ESTIMATION_WINDOW    = 25.0;     ///< max size of the moving window used for velocity estimation
static const int        START_INDEX_FOR_TORS0          = 0;        ///< The index identifying the starting index of the torso in the activeJoint vector
static const int        START_INDEX_FOR_LEFT_SHOULDER  = 3;        ///< The index identifying the starting index of the left shoulder in the activeJoint vector
static const int        START_INDEX_FOR_RIGHT_SHOULDER = 8;        ///< The index identifying the starting index of the right shoulder in the activeJoint vector
static const double     PULLEY_RADIUS_ROLL_JOINT    = 0.08;         ///< Radius roll joint pulley.
static const double     PULLEY_RADIUS_ROLL_MOTOR    = 0.044;        ///< Radius roll motor pulley.
static const double     TRANSMISSION_RATIO_SHOULDER = 1.6;          ///< Transmission ratio for shoulders.

/** Saturate the specified value between the specified bounds. */
inline double saturation(const double x, const double xMax, const double xMin)
{
 	return x>xMax ? xMax : (x<xMin?xMin:x);
}

inline double sign(const double x)
{
    return x>=0.0 ? 1.0 : -1.0;
}

};

#endif
