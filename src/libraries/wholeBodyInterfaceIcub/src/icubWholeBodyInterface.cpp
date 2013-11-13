/**
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
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

#include "wbiIcub/wholeBodyInterfaceIcub.h"
#include <iCub/skinDynLib/common.h>
#include <string>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;

#define MAX_NJ 20
#define WAIT_TIME 0.001

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY INTERFACE
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyInterface::icubWholeBodyInterface(const char* _name, const char* _robotName, int head_version, int legs_version)
{
    actuatorInt = new icubWholeBodyActuators((_name+string("actuator")).c_str(), _robotName);
    stateInt = new icubWholeBodyStates((_name+string("state")).c_str(), _robotName, 0.0);
    modelInt = new icubWholeBodyModel((_name+string("model")).c_str(), _robotName, head_version, legs_version);
}

bool icubWholeBodyInterface::init()
{
    bool ok = actuatorInt->init();
    if(ok) ok = stateInt->init();
    return ok ? modelInt->init() : false;
}

bool icubWholeBodyInterface::close()
{
    bool ok = actuatorInt->close();
    ok = ok && stateInt->close();
    return ok && modelInt->close();
}

bool icubWholeBodyInterface::removeJoint(const LocalId &j)
{
    bool ok = actuatorInt->removeActuator(j);
    if(ok) stateInt->removeEstimate(ESTIMATE_JOINT_POS, j); // removing pos removes also vel and acc estimation
    return ok ? modelInt->removeJoint(j) : false;
}

bool icubWholeBodyInterface::addJoint(const LocalId &j)
{
    bool ok = actuatorInt->addActuator(j);
    if(ok) stateInt->addEstimate(ESTIMATE_JOINT_POS, j);    // adding pos adds also vel and acc estimation
    return ok ? modelInt->addJoint(j) : false;
}

int icubWholeBodyInterface::addJoints(const LocalIdList &jList)
{
    int res1 = actuatorInt->addActuators(jList);
    int res2 = stateInt->addEstimates(ESTIMATE_JOINT_POS, jList);   // adding pos adds also vel and acc estimation
    int res3 = modelInt->addJoints(jList);
    assert(res1==res2);
    assert(res2==res3);
    return res1;
}
