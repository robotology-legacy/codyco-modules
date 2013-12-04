/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete, Marco Randazzo
 * email: andrea.delprete@iit.it marco.randazzo@iit.it
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
#include <iCub/ctrl/math.h>
#include <string>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

#define MAX_NJ 20               ///< max number of joints in a body part
#define WAIT_TIME 0.001         ///< waiting time in seconds before retrying to perform an operation that has failed
#define DEFAULT_REF_SPEED 10.0  ///< default reference joint speed for the joint position control

///< iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
///< iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyActuators::icubWholeBodyActuators(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames)
: initDone(false), dof(0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames){}

bool icubWholeBodyActuators::openDrivers(int bp)
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

bool icubWholeBodyActuators::init()
{
    bool ok = true;
    int tmp[MAX_NJ];
    FOR_ALL_BODY_PARTS(itBp)
    {
        ok = ok && openDrivers(itBp->first);
        if(ok)
        {
            icmd[itBp->first]->getControlModes(tmp);
            for(vector<int>::const_iterator itJ=itBp->second.begin(); itJ!=itBp->second.end(); itJ++)
                currentCtrlModes[LocalId(itBp->first,*itJ)] = yarpToWbiCtrlMode(tmp[itBp->first==TORSO?2-*itJ:*itJ]);
        }
    }
    initDone = true;
    return ok;
}

bool icubWholeBodyActuators::close()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
    {
        assert(dd[itBp->first]!=NULL);
        ok = dd[itBp->first]->close();
    }
    return ok;
}

bool icubWholeBodyActuators::removeActuator(const LocalId &j)
{
    if(!jointIdList.removeId(j))
        return false;
    dof--;
    return true;
}

bool icubWholeBodyActuators::addActuator(const LocalId &j)
{
    // if initialization was done and drivers of specified body part are not open, then open them
    if(initDone && !jointIdList.containsBodyPart(j.bodyPart))
        if(!openDrivers(j.bodyPart))
            return false;
    
    // if initialization was not done, drivers will be opened during initialization
    if(!jointIdList.addId(j))
        return false;
    
    if(initDone)
    {
        int tmp=-1;
        icmd[j.bodyPart]->getControlMode(j.bodyPart==TORSO?2-j.index:j.index, &tmp);
        currentCtrlModes[j] = yarpToWbiCtrlMode(tmp);
    }
    
    dof++;
    return true;
}

int icubWholeBodyActuators::addActuators(const LocalIdList &jList)
{
    // if initialization was done and drivers of specified body part are not open, then open them
    // if initialization was not done, drivers will be opened during initialization
    if(initDone)
    {
        int tmp[MAX_NJ];
        for(LocalIdList::const_iterator it=jList.begin(); it!=jList.end(); it++)
            if(!jointIdList.containsBodyPart(it->first))
            {
                if(!openDrivers(it->first))
                    return 0;
                
                icmd[it->first]->getControlModes(tmp);
                for(vector<int>::const_iterator itJ=it->second.begin(); itJ!=it->second.end(); itJ++)
                    currentCtrlModes[LocalId(it->first,*itJ)] = yarpToWbiCtrlMode(tmp[it->first==TORSO?2-*itJ:*itJ]);
            }
    }
    int count = jointIdList.addIdList(jList);
    dof += count;

    return count;
}

bool icubWholeBodyActuators::setControlMode(ControlMode controlMode, double *ref, int joint)
{
    if(joint>=dof)
        return false;
    
    bool ok = true;
    if(joint<0)     ///< set all joints to the specified control mode
    {
        switch(controlMode)
        {
            case CTRL_MODE_POS:
                FOR_ALL(itBp, itJ)
                    if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode)
                        ok = ok && icmd[itBp->first]->setPositionMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                        ///< icub's torso joints are in reverse order
                break;
                
            case CTRL_MODE_VEL:
                FOR_ALL(itBp, itJ)
                    if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode)
                        ok = ok && icmd[itBp->first]->setVelocityMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                break;
                
            case CTRL_MODE_TORQUE:
                FOR_ALL(itBp, itJ)
                    if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode)
                        ok = ok && icmd[itBp->first]->setTorqueMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                break;
                
            case CTRL_MODE_MOTOR_PWM:
                if(!isRobotSimulator(robot)) ///< iCub simulator does not implement PWM motor control
                    FOR_ALL(itBp, itJ)
                        if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode)
                            ok = ok && icmd[itBp->first]->setOpenLoopMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                break;

            default:
                return false;
        }
        if(ok)
        {
            FOR_ALL(itBp, itJ)
                currentCtrlModes[LocalId(itBp->first, *itJ)] = controlMode;
            if(ref!=0)
                ok = ok && setControlReference(ref);
        }
        return ok;
    }
    
    ///< Set the specified joint to the specified control mode
    LocalId li = jointIdList.globalToLocalId(joint);
    if(currentCtrlModes[li]!=controlMode)   ///< check that joint is not already in the specified control mode
    {
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        switch(controlMode)
        {
            case CTRL_MODE_POS:         ok = icmd[li.bodyPart]->setPositionMode(i); break;
            case CTRL_MODE_VEL:         ok = icmd[li.bodyPart]->setVelocityMode(i); break;
            case CTRL_MODE_TORQUE:      ok = icmd[li.bodyPart]->setTorqueMode(i);   break;
            ///< iCub simulator does not implement PWM motor control
            case CTRL_MODE_MOTOR_PWM:   ok = isRobotSimulator(robot) ? true : icmd[li.bodyPart]->setOpenLoopMode(i); break;
            default: break;
        }
        if(ok)
            currentCtrlModes[li] = controlMode;
    }
    if(ok &&ref!=0)
        ok = setControlReference(ref, joint);   ///< set specified control reference (if any)
    return ok;
}

bool icubWholeBodyActuators::setControlReference(double *ref, int joint)
{
    if(joint> (int)dof)
        return false;
    
    bool ok = true;
    if(joint>=0)    // set control reference for the specified joint
    {
        LocalId li = jointIdList.globalToLocalId(joint);
        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        switch(currentCtrlModes[li])
        {
            case CTRL_MODE_POS:         return ipos[li.bodyPart]->positionMove(i, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_VEL:         return ivel[li.bodyPart]->velocityMove(i, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_TORQUE:      return itrq[li.bodyPart]->setRefTorque(i, *ref);
            ///< iCub simulator does not implement PWM motor control
            case CTRL_MODE_MOTOR_PWM:   return isRobotSimulator(robot) ? true : iopl[li.bodyPart]->setOutput(i, *ref);
            default: break;
        }
        return false;
    }
    // set control references for all joints

    ///< on robot use new method which set all joint vel of one body part at the same time (much faster!)
    if(!isRobotSimulator(robot))
    {
        double spd[MAX_NJ];     // vector of reference joint speeds
        int jointIds[MAX_NJ];   // vector of joint ids
        int i=0;                // counter of controlled joints
        FOR_ALL_BODY_PARTS(itBp)
        {
            int njVelCtrl = 0;              // number of joints that are velocity controlled
            int nj = itBp->second.size();   // number of joints of this body part
            for(int j=0;j<nj;j++)
            {
                if(currentCtrlModes[LocalId(itBp->first, itBp->second[j])]==CTRL_MODE_VEL)
                {
                    // icub's torso joints are in reverse order
                    jointIds[j] = itBp->first==TORSO ? 2-itBp->second[j] : itBp->second[j];
                    spd[j] = CTRL_RAD2DEG*ref[i];           // convert joint vel from rad to deg
                    njVelCtrl++;
                }
                i++;
            }
            ok = ok && ivel[itBp->first]->velocityMove(njVelCtrl, jointIds, spd);
        }
    }
    
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        switch(currentCtrlModes[LocalId(itBp->first,*itJ)])
        {
            case CTRL_MODE_POS:         
                ok = ok && ipos[itBp->first]->positionMove(j, CTRL_RAD2DEG*ref[i]); 
                break;
            case CTRL_MODE_VEL:         
                if(isRobotSimulator(robot)) ///< velocity controlled joints have already been managed (for the real robot)
                    ok = ok && ivel[itBp->first]->velocityMove(j, CTRL_RAD2DEG*ref[i]); 
                break;
            case CTRL_MODE_TORQUE:      
                ok = ok && itrq[itBp->first]->setRefTorque(j, ref[i]);              
                break;
            case CTRL_MODE_MOTOR_PWM:   
                if(!isRobotSimulator(robot)) ///< iCub simulator does not implement PWM motor control
                    ok = ok && iopl[itBp->first]->setOutput(j, ref[i]); 
                break;
            default: 
                printf("[icubWholeBodyActuators::setControlReference] ERROR: unmanaged control mode.\n"); 
                return false;
        }
        i++;
    }
    
    return ok;
}
        
bool icubWholeBodyActuators::setControlParam(ControlParam paramId, double *value, int joint)
{
    switch(paramId)
    {
    case CTRL_PARAM_REF_VEL: return setReferenceSpeed(value, joint);
    default: break;
    }
    return false;
}

bool icubWholeBodyActuators::setReferenceSpeed(double *rspd, int joint)
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

ControlMode icubWholeBodyActuators::yarpToWbiCtrlMode(int yarpCtrlMode)
{
    switch(yarpCtrlMode)
    {
    case VOCAB_CM_TORQUE:   return CTRL_MODE_TORQUE;
    case VOCAB_CM_POSITION: return CTRL_MODE_POS;
    case VOCAB_CM_VELOCITY: return CTRL_MODE_VEL;
    case VOCAB_CM_OPENLOOP: return CTRL_MODE_MOTOR_PWM;
    }
    return CTRL_MODE_UNKNOWN;
}

//
//bool icubWholeBodyActuators::setTorqueRef(double *taud, int joint)
//{
//    if(joint> (int)dof)
//        return false;
//    
//    if(joint>=0)
//    {
//        LocalId li = jointIdList.globalToLocalId(joint);
//        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
//        return itrq[li.bodyPart]->setRefTorque(i, *taud);
//    }
//    
//    bool ok = true;
//    unsigned int i=0;
//    FOR_ALL(itBp, itJ)
//    {
//        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
//        ok = ok && itrq[itBp->first]->setRefTorque(j, taud[i]);
//        i++;
//    }
//    
//    return ok;
//}
//
//bool icubWholeBodyActuators::setPosRef(double *qd, int joint)
//{
//    if(joint>=dof)
//        return false;
//    
//    if(joint>=0)
//    {
//        LocalId li = jointIdList.globalToLocalId(joint);
//        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
//        return ipos[li.bodyPart]->positionMove(i, CTRL_RAD2DEG*(*qd));
//    }
//    
//    bool ok = true;
//    unsigned int i=0;
//    FOR_ALL(itBp, itJ)
//    {
//        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
//        ok = ok && ipos[itBp->first]->positionMove(j, CTRL_RAD2DEG*qd[i]);
//        i++;
//    }
//    
//    return ok;
//}
//
//bool icubWholeBodyActuators::setVelRef(double *dqd, int joint)
//{
//    if(joint>=dof)
//        return false;
//    
//    if(joint>=0)
//    {
//        LocalId li = jointIdList.globalToLocalId(joint);
//        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
//        return ivel[li.bodyPart]->velocityMove(i, CTRL_RAD2DEG*(*dqd));
//    }
//    
//    int i = 0;
//    bool ok = true;
//    if(isRobotSimulator(robot)) // on simulator use old method which set one joint vel at a time
//    {
//        FOR_ALL(itBp, itJ)
//        {
//            int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
//            ok = ok && ivel[itBp->first]->velocityMove(j, CTRL_RAD2DEG*dqd[i]);
//            i++;
//        }
//        return ok;
//    }
//
//    // on robot use new method which set all joint vel of one body part at the same time (much faster!)
//    double spd[MAX_NJ];
//    int nj, jointIds[MAX_NJ];
//    FOR_ALL_BODY_PARTS(itBp)
//    {
//        nj = itBp->second.size();   // number of joints of this body part
//        for(int j=0;j<nj;j++)
//        {
//            if(itBp->first==TORSO)
//                jointIds[j] = 2-itBp->second[j];    // icub's torso joints are in reverse order
//            else
//                jointIds[j] = itBp->second[j];
//            spd[j] = CTRL_RAD2DEG*dqd[i];           // convert joint vel from rad to deg
//            i++;
//        }
//        ok = ok && ivel[itBp->first]->velocityMove(nj, jointIds, spd);
//    }
//    return ok;
//}
//
//bool icubWholeBodyActuators::setPwmRef(double *pwmd, int joint)
//{
//    if(joint>=dof)
//        return false;
//    
//    if(joint>=0)
//    {
//        LocalId li = jointIdList.globalToLocalId(joint);
//        int i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
//        return iopl[li.bodyPart]->setOutput(i, *pwmd);
//    }
//    
//    bool ok = true;
//    unsigned int i=0;
//    FOR_ALL(itBp, itJ)
//    {
//        int j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
//        ok = ok && iopl[itBp->first]->setOutput(j, pwmd[i]);
//        i++;
//    }
//    
//    return ok;
//}

