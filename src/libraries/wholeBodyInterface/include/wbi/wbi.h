/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Andrea Del Prete, Silvio Traversaro, Marco Randazzo
 * email: andrea.delprete@iit.it - silvio.traversaro@iit.it - marco.randazzo@iit.it
 */

/**
 * \defgroup wbi wbi
 *
 * @ingroup codyco_libraries
 *
 * THIS CODE IS UNDER DEVELOPMENT!
 * Library defining a general interface for communicating with a floating-base rigid robot.
 * The interface is divided into four main parts:
 * - sensor: read sensor data (e.g. encoders, force/torque sensors, IMUs)
 * - state estimation: read estimations of the state of the robot (e.g. joint pos/vel/acc, external forces)
 * - actuator: send commands to the low-level motor controllers
 * - model: access the kinematic/dynamic model of the robot
 * The robot has n joints and n+6 DoFs, because of the 6 additional DoFs representing the position and 
 * orientation of its floating-base. 
 *
 * \section dep_sec Dependencies
 * None.
 *
 * \section intro_sec Description
 * We assume the robot is divided into subparts, which we call "body parts" (e.g. left arm, right leg, torso).
 * Each body part has an unique integer identifier.
 * In each body part there exists a unique local identifier associated to any object (e.g. joint, sensor, motor) 
 * that belongs to that body part.
 * Each object also has a unique global identifier, which defines how the objects are serialized at whole-body level.
 *
 * All angles are expressed in radians.
 *
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \author Andrea Del Prete, Silvio Traversaro, Marco Randazzo - name.surname@iit.it
 *
 * Copyright (C) 2013-.. CODYCO
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 **/

#ifndef WBI_H
#define WBI_H

//this header includes all the headers of the wbi library

#include <wbi/wholeBodyInterface.h>
#include <wbi/iWholeBodyModel.h>
#include <wbi/iWholeBodySensors.h>
#include <wbi/iWholeBodyStates.h>
#include <wbi/iWholeBodyActuators.h>
#include <wbi/wbiUtil.h>
#include <wbi/wbiConstants.h>

#endif

