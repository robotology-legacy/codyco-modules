/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email: marco.randazzo@iit.it
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

#include "wbiy/wbiy.h"
#include <string>

using namespace std;
using namespace wbi;
using namespace wbiy;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#define MAX_NJ 20
#define WAIT_TIME 0.001

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ROBOT WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyStates::icubWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow)
{

}

bool icubWholeBodyStates::init(){ return false; }
int icubWholeBodyStates::getDoFs(){ return 0; }
bool icubWholeBodyStates::removeJoint(const wbi::LocalId &j){ return false; }
bool icubWholeBodyStates::addJoint(const wbi::LocalId &j){ return false; }
int icubWholeBodyStates::addJoints(const wbi::LocalIdList &j){ return 0; }

bool icubWholeBodyStates::getQ(double *q, double time, bool wait){ return false; }
bool icubWholeBodyStates::getDq(double *dq, double time, bool wait){ return false; }
bool icubWholeBodyStates::getDqMotors(double *dqM, double time, bool wait){ return false; }
bool icubWholeBodyStates::getD2q(double *d2q, double time, bool wait){ return false; }
bool icubWholeBodyStates::getPwm(double *pwm, double time, bool wait){ return false; }
bool icubWholeBodyStates::getInertial(double *inertial, double time, bool wait){ return false; }
bool icubWholeBodyStates::getFTsensors(double *ftSens, double time, bool wait){ return false; }
bool icubWholeBodyStates::getTorques(double *tau, double time, bool wait){ return false; }
