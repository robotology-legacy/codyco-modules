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
#include <yarp/os/Os.h>
#include <string>
#include <cassert>

#include <yarp/os/Time.h>

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
icubWholeBodyInterface::icubWholeBodyInterface(const char* _name, const char* _robotName, iCub::iDynTree::iCubTree_version_tag icub_version)
{
    actuatorInt = new icubWholeBodyActuators((_name+string("actuator")).c_str(), _robotName);
    stateInt = new icubWholeBodyStates((_name+string("state")).c_str(), _robotName, 0.0);
    modelInt = new icubWholeBodyModel((_name+string("model")).c_str(), _robotName, icub_version);
}

#ifdef CODYCO_USES_URDFDOM
icubWholeBodyInterface::icubWholeBodyInterface(const char* _name, const char* _robotName, iCub::iDynTree::iCubTree_version_tag icub_version, std::string urdf_file)
{
    actuatorInt = new icubWholeBodyActuators((_name+string("actuator")).c_str(), _robotName);
    stateInt = new icubWholeBodyStates((_name+string("state")).c_str(), _robotName, 0.0);
    modelInt = new icubWholeBodyModel((_name+string("model")).c_str(), _robotName, icub_version, urdf_file);
}
#endif

bool icubWholeBodyInterface::init()
{
    double wait_workaround = 0.5;
    bool ok = actuatorInt->init();
    if(!ok) printf("[icubWholeBodyInterface] Error while initializing actuator interface.\n");
    yarp::os::Time::delay(wait_workaround);
    if(ok) ok = stateInt->init();
    if(!ok) printf("[icubWholeBodyInterface] Error while initializing state interface.\n");
    yarp::os::Time::delay(wait_workaround);
    if(ok) ok = modelInt->init();
    if(!ok) printf("[icubWholeBodyInterface] Error while initializing model interface.\n");
    return ok;
}

bool icubWholeBodyInterface::close()
{
    bool ok = true;

    if( actuatorInt ) { ok = actuatorInt->close(); delete actuatorInt; actuatorInt=0; }
    if( stateInt ) { ok = stateInt->close(); delete stateInt; stateInt=0; }
    if( modelInt ) { ok = modelInt->close(); delete modelInt; modelInt=0; }

    return ok;
}

bool icubWholeBodyInterface::removeJoint(const LocalId &j)
{
    bool ok = actuatorInt->removeActuator(j);
    for(int i=0; ok && i<JOINT_ESTIMATE_TYPES_SIZE; i++)
        ok = stateInt->removeEstimate(jointEstimateTypes[i], j);
    // removing pos removes also vel and acc estimation
    return ok ? modelInt->removeJoint(j) : false;
}

bool icubWholeBodyInterface::addJoint(const LocalId &j)
{
    bool ok = actuatorInt->addActuator(j);
    for(int i=0; ok && i<JOINT_ESTIMATE_TYPES_SIZE; i++)
        ok = stateInt->addEstimate(jointEstimateTypes[i], j);
    return ok ? modelInt->addJoint(j) : false;
}

int icubWholeBodyInterface::addJoints(const LocalIdList &jList)
{
    int res1 = actuatorInt->addActuators(jList);
    for(int i=0; i<JOINT_ESTIMATE_TYPES_SIZE; i++)
        stateInt->addEstimates(jointEstimateTypes[i], jList);
    // adding pos adds also vel and acc estimation
    int res4 = modelInt->addJoints(jList);
    assert(res1==res4);
    return res1;
}

bool icubWholeBodyInterface::setActuactorConfigurationParameter(const std::string& parameterName, const yarp::os::Value& parameterValue)
{
    if (actuatorInt)
        return actuatorInt->setConfigurationParameter(parameterName, parameterValue);
    return false;
}
