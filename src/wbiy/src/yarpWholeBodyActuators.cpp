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
#include <iCub/ctrl/math.h>
#include <string>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiy;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

#define MAX_NJ 20
#define WAIT_TIME 0.001
#define DEFAULT_REF_SPEED 10.0

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyActuators::yarpWholeBodyActuators(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames)
: name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), dof(0), initDone(false) {}

bool yarpWholeBodyActuators::openDrivers(int bp)
{
    itrq[bp]=0; iimp[bp]=0; icmd[bp]=0; ivel[bp]=0; ipos[bp]=0; iopl[bp]=0;  dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], bodyPartNames[bp].c_str()))
        return false;
    
    bool ok = dd[bp]->view(itrq[bp]) && dd[bp]->view(iimp[bp]) && dd[bp]->view(icmd[bp])
              && dd[bp]->view(ivel[bp]) && dd[bp]->view(ipos[bp]) && dd[bp]->view(iopl[bp]);
    if(!ok)
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", bodyPartNames[bp].c_str());
        return false;
    }
    
    return true;
}

bool yarpWholeBodyActuators::init()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
        ok = ok && openDrivers(itBp->first);
    initDone = true;
    return ok;
}

bool yarpWholeBodyActuators::close()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
    {
        assert(dd[itBp->first]!=NULL);
        ok = dd[itBp->first]->close();
    }
    return ok;
}

bool yarpWholeBodyActuators::removeJoint(const LocalId &j)
{
    if(!jointIdList.removeId(j))
        return false;
    dof--;
    return true;
}

bool yarpWholeBodyActuators::addJoint(const LocalId &j)
{
    // if initialization was done and drivers of specified body part are not open, then open them
    // if initialization was not done, drivers will be opened during initialization
    if(initDone && !jointIdList.containsBodyPart(j.bodyPart))
        if(!openDrivers(j.bodyPart))
            return false;
    
    if(!jointIdList.addId(j))
        return false;
    
    dof++;
    return true;
}

int yarpWholeBodyActuators::addJoints(const LocalIdList &jList)
{
    // if initialization was done and drivers of specified body part are not open, then open them
    // if initialization was not done, drivers will be opened during initialization
    if(initDone)
        for(LocalIdList::const_iterator it=jList.begin(); it!=jList.end(); it++)
            if(!jointIdList.containsBodyPart(it->first))
                if(!openDrivers(it->first))
                    return 0;
    int count = jointIdList.addIdList(jList);
    dof += count;
    return count;
}

bool yarpWholeBodyActuators::setControlMode(int controlMode, int joint)
{
    if(joint>=dof)
        return false;
    
    if(joint<0)
    {
        bool ok = true;
        switch(controlMode)
        {
            case CTRL_MODE_POS:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setPositionMode(itBp->first==TORSO ? 2-(*itJ) : *itJ); // icub's torso joints are in reverse order
                break;
                
            case CTRL_MODE_VEL:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setVelocityMode(itBp->first==TORSO ? 2-(*itJ) : *itJ); // icub's torso joints are in reverse order
                break;
                
            case CTRL_MODE_TORQUE:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setTorqueMode(itBp->first==TORSO ? 2-(*itJ) : *itJ); // icub's torso joints are in reverse order
                break;
                
            case CTRL_MODE_OPEN_LOOP:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setOpenLoopMode(itBp->first==TORSO ? 2-(*itJ) : *itJ); // icub's torso joints are in reverse order
                break;
                
            default:
                return false;
        }
        return ok;
    }
    
    LocalId li = jointIdList.globalToLocalId(joint);
    int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
    switch(controlMode)
    {
        case CTRL_MODE_POS:         return icmd[li.bodyPart]->setPositionMode(i);
        case CTRL_MODE_VEL:         return icmd[li.bodyPart]->setVelocityMode(i);
        case CTRL_MODE_TORQUE:      return icmd[li.bodyPart]->setTorqueMode(i);
        case CTRL_MODE_OPEN_LOOP:   return icmd[li.bodyPart]->setOpenLoopMode(i);
    }
    
    return false;
}

bool yarpWholeBodyActuators::setTorqueRef(double *taud, int joint)
{
    if(joint> (int)dof)
        return false;
    
    if(joint>=0)
    {
        LocalId li = jointIdList.globalToLocalId(joint);
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        return itrq[li.bodyPart]->setRefTorque(i, *taud);
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        ok = ok && itrq[itBp->first]->setRefTorque(j, taud[i]);
        i++;
    }
    
    return ok;
}

bool yarpWholeBodyActuators::setPosRef(double *qd, int joint)
{
    if(joint>=dof)
        return false;
    
    if(joint>=0)
    {
        LocalId li = jointIdList.globalToLocalId(joint);
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        return ipos[li.bodyPart]->positionMove(i, CTRL_RAD2DEG*(*qd));
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        ok = ok && ipos[itBp->first]->positionMove(j, CTRL_RAD2DEG*qd[i]);
        i++;
    }
    
    return ok;
}

bool yarpWholeBodyActuators::setVelRef(double *dqd, int joint)
{
    if(joint>=dof)
        return false;
    
    if(joint>=0)
    {
        LocalId li = jointIdList.globalToLocalId(joint);
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        return ivel[li.bodyPart]->velocityMove(i, CTRL_RAD2DEG*(*dqd));
    }
    
    unsigned int i = 0;
    bool ok = true;
    FOR_ALL(itBp, itJ)
    {
        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        ok = ok && ivel[itBp->first]->velocityMove(j, CTRL_RAD2DEG*dqd[i]);
        i++;
    }
    
    return ok;
}

bool yarpWholeBodyActuators::setPwmRef(double *pwmd, int joint)
{
    if(joint>=dof)
        return false;
    
    if(joint>=0)
    {
        LocalId li = jointIdList.globalToLocalId(joint);
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        return iopl[li.bodyPart]->setOutput(i, *pwmd);
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        ok = ok && iopl[itBp->first]->setOutput(j, pwmd[i]);
        i++;
    }
    
    return ok;
}

bool yarpWholeBodyActuators::setReferenceSpeed(double *rspd, int joint)
{
    if(joint>=dof)
        return false;
    
    if(joint>=0)
    {
        LocalId li = jointIdList.globalToLocalId(joint);
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        return ipos[li.bodyPart]->setRefSpeed(i, CTRL_RAD2DEG*(*rspd));
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        ok = ok && ipos[itBp->first]->setRefSpeed(j, CTRL_RAD2DEG*rspd[i]);
        i++;
    }
    
    return ok;
}