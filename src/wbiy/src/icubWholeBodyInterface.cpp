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
#include <iCub/skinDynLib/common.h>
#include <string>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiy;
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
    vector<string> bodyPartNames(BodyPart_s, BodyPart_s + sizeof(BodyPart_s) / sizeof(string) );
    actuatorInt = new yarpWholeBodyActuators((_name+string("actuator")).c_str(), _robotName, bodyPartNames);
    stateInt = new icubWholeBodyStates((_name+string("state")).c_str(), _robotName, 0.0);
    modelInt = new icubWholeBodyModel((_name+string("model")).c_str(), _robotName, bodyPartNames, head_version,legs_version);
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

int icubWholeBodyInterface::getDoFs()
{
    return actuatorInt->getDoFs();
}

bool icubWholeBodyInterface::removeJoint(const LocalId &j)
{
    bool ok = actuatorInt->removeJoint(j);
    if(ok) stateInt->removeJoint(j);
    return ok ? modelInt->removeJoint(j) : false;
}

bool icubWholeBodyInterface::addJoint(const LocalId &j)
{
    bool ok = actuatorInt->addJoint(j);
    if(ok) stateInt->addJoint(j);
    return ok ? modelInt->addJoint(j) : false;
}

int icubWholeBodyInterface::addJoints(const LocalIdList &jList)
{
    int res1 = actuatorInt->addJoints(jList);
    int res2 = stateInt->addJoints(jList);
    int res3 = modelInt->addJoints(jList);
    assert(res1==res2);
    assert(res2==res3);
    return res1;
}
