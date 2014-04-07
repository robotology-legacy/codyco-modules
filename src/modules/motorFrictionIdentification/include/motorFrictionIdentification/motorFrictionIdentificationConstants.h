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

#ifndef _MOTOR_FRICTION_IDENTIFICATION_CONSTANTS
#define _MOTOR_FRICTION_IDENTIFICATION_CONSTANTS

#include <Eigen/Core>

namespace motorFrictionIdentification
{
/** Types of printed messages */
enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

// *** CONSTANTS
static const int        PRINT_PERIOD                = 1000;         ///< period of debug prints (in ms)
static const int        PRINT_MSG_LEVEL             = MSG_DEBUG;    ///< only messages whose type is greater than or equal to PRINT_MSG_LEVEL are printed
static const double     MODULE_PERIOD               = 20.0;         ///< period of the module (in sec)
static const double     TORQUE_SENSOR_SATURATION    = 13.0;         ///< value at which the joint torque sensors saturate
static const double     STD_DEV_SATURATION          = 0.3;          ///< value at which the standard deviation written on the output monitoring port are saturated
static const double     ZERO_TORQUE_THRESHOLD       = 0.07;         ///< Lower threshold for a torque measure in order to consider it zero.
static const double     PULLEY_RADIUS_ROLL_JOINT    = 0.08;         ///< Radius roll joint pulley.
static const double     PULLEY_RADIUS_ROLL_MOTOR    = 0.044;        ///< Radius roll motor pulley.

template<class VectorType>
inline void resizeAndSetToZero(VectorType &v, unsigned int size)
{
    v.resize(size);
    v.setZero();
}

template<class MatrixType>
inline void resizeAndSetToZero(MatrixType &m, unsigned int rowNum, unsigned int colNum)
{
    m.resize(rowNum, colNum);
    m.setZero();
}

}   // end namespace 

#endif
