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
#include <yarp/os/Property.h>
#include <string>
#include <cassert>

//*********TEMP**************//
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
#include <paramHelp/paramHelperClient.h>
#include <motorFrictionIdentificationLib/jointTorqueControlParams.h>
#endif
//*********END TEMP**********//

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


//constants
const std::string icubWholeBodyActuators::icubWholeBodyActuatorsUseExternalTorqueModule = "icubWholeBodyActuatorsUseExternalTorqueModuleKey";
const std::string icubWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleName = "icubWholeBodyActuatorsExternalTorqueModuleNameKey";

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyActuators::icubWholeBodyActuators(const char* _name, 
                                               const char* _robotName, 
                                               const std::vector<std::string> &_bodyPartNames)
: initDone(false), dof(0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), reverse_torso_joints(true)
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
,_torqueModuleConnection(0)
#endif
{}

icubWholeBodyActuators::icubWholeBodyActuators(const char* _name,
                                               const char* _robotName, 
                                               const yarp::os::Property & yarp_wbi_properties)
: initDone(false), dof(0), name(_name), robot(_robotName)
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
,_torqueModuleConnection(0)
#endif
{
    yarp::os::Property yarp_wbi_properties_not_const = yarp_wbi_properties;
    loadBodyPartsFromConfig(yarp_wbi_properties_not_const,bodyPartNames);
    Bottle bot = yarp_wbi_properties_not_const.findGroup("WBI_YARP_BODY_PARTS_REMAPPING");
    if( bot.check("reverse_torso_joints") ) {
        reverse_torso_joints = true;
    } else {
        reverse_torso_joints = false;
    }
}



icubWholeBodyActuators::~icubWholeBodyActuators()
{
    close();
}

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
            for(vector<int>::const_iterator itJ=itBp->second.begin(); itJ!=itBp->second.end(); itJ++) {
                if( reverse_torso_joints ) {
                    currentCtrlModes[LocalId(itBp->first,*itJ)] = yarpToWbiCtrlMode(tmp[itBp->first==TORSO?2-*itJ:*itJ]);
                } else {
                    currentCtrlModes[LocalId(itBp->first,*itJ)] = yarpToWbiCtrlMode(tmp[*itJ]);
                }
            }
        }
    }
    
    #ifdef WBI_ICUB_COMPILE_PARAM_HELP
    ///TEMP
    if (_torqueModuleConnection) {
        _torqueModuleConnection->close();
        delete _torqueModuleConnection; _torqueModuleConnection = NULL;
    }
    if (ok) {
        //read options
        yarp::os::Value found = configurationParameters.find(icubWholeBodyActuatorsUseExternalTorqueModule.c_str());
        if (!found.isNull() && found.isBool() && found.asBool()) {
            found = configurationParameters.find(icubWholeBodyActuatorsExternalTorqueModuleName.c_str());
            if (found.isNull()) {
                ok = false;
            }
            else {
                
                _torqueModuleConnection = new paramHelp::ParamHelperClient(jointTorqueControl::jointTorqueControlParamDescr, jointTorqueControl::PARAM_ID_SIZE,
                                                                           jointTorqueControl::jointTorqueControlCommandDescr, jointTorqueControl::COMMAND_ID_SIZE);
                
                Bottle initMsg;
                if (!_torqueModuleConnection || !_torqueModuleConnection->init(name, found.asString().c_str(), initMsg)) {
                    ok = false;
                }
                else {
                    _torqueRefs.resize(jointTorqueControl::N_DOF);
                    ok = _torqueModuleConnection->linkParam(jointTorqueControl::PARAM_ID_TAU_OFFSET, _torqueRefs.data());
                }
            }
        }
    }
    ///END TEMP
    #endif
    initDone = true;
    return ok;
}

bool icubWholeBodyActuators::close()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
    {
        if( dd[itBp->first]!= 0 ) {
            ok = dd[itBp->first]->close();
            delete dd[itBp->first];
            dd[itBp->first] = 0;
        }
    }
    #ifdef WBI_ICUB_COMPILE_PARAM_HELP
    ///TEMP
    if (_torqueModuleConnection) {
        _torqueModuleConnection->close();
        delete _torqueModuleConnection; _torqueModuleConnection = NULL;
    }
    #endif
    
    return ok;
}

bool icubWholeBodyActuators::setConfigurationParameter(const std::string &parameterName, const yarp::os::Value &parameterValue)
{
    /*Note to developers: we can move the functionalities offered by the configuration map to an external class in order to make it more generic */
    if (initDone) return false;
    //check allowed parameters
    if (parameterName.compare(icubWholeBodyActuatorsUseExternalTorqueModule) == 0) {
        if (parameterValue.isBool()) {
            configurationParameters.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    }
    else if (parameterName.compare(icubWholeBodyActuatorsExternalTorqueModuleName) == 0) {
        //simply check value has some length
        if (parameterValue.isString() && parameterValue.asString().length() > 0) {
            configurationParameters.put(parameterName.c_str(), parameterValue);
            return true;
        }
        return false;
    }
    
    return false;
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
        if( reverse_torso_joints ) {
            icmd[j.bodyPart]->getControlMode(j.bodyPart==TORSO?2-j.index:j.index, &tmp);
        } else {
            icmd[j.bodyPart]->getControlMode(j.index, &tmp);
        }
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
                for(vector<int>::const_iterator itJ=it->second.begin(); itJ!=it->second.end(); itJ++) {
                    if( reverse_torso_joints ) {
                        currentCtrlModes[LocalId(it->first,*itJ)] = yarpToWbiCtrlMode(tmp[it->first==TORSO?2-*itJ:*itJ]);
                    } else {
                        currentCtrlModes[LocalId(it->first,*itJ)] = yarpToWbiCtrlMode(tmp[*itJ]);
                    }
                }
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
                FOR_ALL(itBp, itJ) {
                    if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode) {
                        if( reverse_torso_joints ) {
                            ok = ok && icmd[itBp->first]->setPositionMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                            ///< icub's torso joints are in reverse order
                        } else {
                            ok = ok && icmd[itBp->first]->setPositionMode(*itJ);
                        }
                    }
                }
                break;
                
            case CTRL_MODE_VEL:
                FOR_ALL(itBp, itJ) {
                    if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode) {
                        if( reverse_torso_joints ) {
                            ok = ok && icmd[itBp->first]->setVelocityMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                        } else {
                            ok = ok && icmd[itBp->first]->setVelocityMode(*itJ);
                        }
                    }
                }
                break;
                
            case CTRL_MODE_TORQUE:
                FOR_ALL(itBp, itJ) {
                    if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode) {
                        #ifdef WBI_ICUB_COMPILE_PARAM_HELP
                        if (_torqueModuleConnection) {
                            //if torque control connection is true I do not set the torqueMode
                            ok = ok && true;
                        }
                        else
                        #endif
                        {
                            if( reverse_torso_joints ) {
                                ok = ok && icmd[itBp->first]->setTorqueMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                            } else {
                                ok = ok && icmd[itBp->first]->setTorqueMode(*itJ);
                            }
                        }
                    }
                }
                
                break;
                
            case CTRL_MODE_MOTOR_PWM:
                if(!isRobotSimulator(robot)) ///< iCub simulator does not implement PWM motor control
                    FOR_ALL(itBp, itJ) {
                        if(currentCtrlModes[LocalId(itBp->first,*itJ)]!=controlMode) {
                            if( reverse_torso_joints ) {
                                ok = ok && icmd[itBp->first]->setOpenLoopMode(itBp->first==TORSO ? 2-(*itJ) : *itJ);
                            } else {
                                ok = ok && icmd[itBp->first]->setOpenLoopMode(*itJ);
                            }
                        }
                    }
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
        int i;
        if( reverse_torso_joints ) {
            i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        } else {
            i = li.index;
        }
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
        int i;
        if( reverse_torso_joints ) {
            i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        } else {
            i = li.index;
        }
        switch(currentCtrlModes[li])
        {
            case CTRL_MODE_POS:         return ipos[li.bodyPart]->positionMove(i, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_VEL:         return ivel[li.bodyPart]->velocityMove(i, CTRL_RAD2DEG*(*ref));
            case CTRL_MODE_TORQUE:
            {
                #ifdef WBI_ICUB_COMPILE_PARAM_HELP
                //TEMP
                if (_torqueModuleConnection) {
                    _torqueRefs.zero();
                    //convert to global (25 dof)
                    int gid = ICUB_MAIN_JOINTS.localToGlobalId(li);
                    if (gid < 0 || gid >= jointTorqueControl::N_DOF) {
                        return false;
                    }
                    else {
                        _torqueRefs[gid] = *ref;
                        return _torqueModuleConnection->sendStreamParams();
                    }
                }
                else
                //END TEMP
                #endif
                    return itrq[li.bodyPart]->setRefTorque(i, *ref);
            }
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
                    if( reverse_torso_joints ) {
                        jointIds[j] = itBp->first==TORSO ? 2-itBp->second[j] : itBp->second[j];
                    } else {
                        jointIds[j] = itBp->second[j];
                    }
                    spd[j] = CTRL_RAD2DEG*ref[i];           // convert joint vel from rad to deg
                    njVelCtrl++;
                }
                i++;
            }
            ok = ok && ivel[itBp->first]->velocityMove(njVelCtrl, jointIds, spd);
        }
    }
    
    #ifdef WBI_ICUB_COMPILE_PARAM_HELP
    //TEMP
    if (_torqueModuleConnection) {
        _torqueRefs.zero();
    }
    //END TEMP
    #endif
    
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j;
        if( reverse_torso_joints ) {
            j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        } else {
            j = *itJ;
        }
        LocalId localID = LocalId(itBp->first,*itJ);
        switch(currentCtrlModes[localID])
        {
            case CTRL_MODE_POS:         
                ok = ok && ipos[itBp->first]->positionMove(j, CTRL_RAD2DEG*ref[i]); 
                break;
            case CTRL_MODE_VEL:         
                if(isRobotSimulator(robot)) ///< velocity controlled joints have already been managed (for the real robot)
                    ok = ok && ivel[itBp->first]->velocityMove(j, CTRL_RAD2DEG*ref[i]); 
                break;
            case CTRL_MODE_TORQUE:
            {
                #ifdef WBI_ICUB_COMPILE_PARAM_HELP
                //TEMP
                //to keep consistency: set only if ok is true
                if (_torqueModuleConnection && ok) {
                    int gid = ICUB_MAIN_JOINTS.localToGlobalId(localID);
                    if (gid < 0 || gid >= jointTorqueControl::N_DOF) {
                        ok = false;
                    }
                    else {
                        _torqueRefs[gid] = ref[i];
                        ok = true;
                    }
                }
                else
                #endif
                    ok = ok && itrq[itBp->first]->setRefTorque(j, ref[i]);
            }
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
    
    #ifdef WBI_ICUB_COMPILE_PARAM_HELP
    //TEMP
    if (_torqueModuleConnection) {
        ok = ok && _torqueModuleConnection->sendStreamParams();
    }
    //END TEMP
    #endif
    
    return ok;
}
        
bool icubWholeBodyActuators::setControlParam(ControlParam paramId, const void *value, int joint)
{
    switch(paramId)
    {
        case CTRL_PARAM_REF_VEL: return setReferenceSpeed((double*)value, joint);
        case CTRL_PARAM_KP: return false;
        case CTRL_PARAM_KD: return false;
        case CTRL_PARAM_KI: return false;
        case CTRL_PARAM_OFFSET: return false;
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
        int i;
        if( reverse_torso_joints ) {
            i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
        } else {
            i = li.index;
        }
        return ipos[li.bodyPart]->setRefSpeed(i, CTRL_RAD2DEG*(*rspd));
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        int j;
        if( reverse_torso_joints ) {
            j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        } else {
            j = *itJ;
        }
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


bool icubWholeBodyActuators::setPIDGains(const double *pValue, const double *dValue, const double *iValue, int joint)
{
    //The FOR_ALL atomicity is debated in github.. currently do the same as the rest of the library
    bool result = true;
    if (joint < 0) {
        return false;
//        FOR_ALL(itBp, itJ) {
//            switch (currentCtrlModes[LocalId(itBp->first,*itJ)]) {
//                case wbi::CTRL_MODE_TORQUE:
//                {
//                    //this is wrong...
////                    Pid currentPid;
////                    result = itrq[itBp->first]->getTorquePid(joint, &currentPid);
////                    if (!result) break;
////                    if (pValue != NULL)
////                        currentPid.kp = *pValue;
////                    if (dValue != NULL)
////                        currentPid.kd = *dValue;
////                    if (iValue != NULL)
////                        currentPid.ki = *iValue;
////                    result = itrq[itBp->first]->setTorquePid(joint, currentPid);
////                    break;
//                }
//                default:
//                    break;
//            }
//        }
    }
    else {
        LocalId li = jointIdList.globalToLocalId(joint);
        switch (currentCtrlModes[li]) {
            case wbi::CTRL_MODE_TORQUE:
            {
                Pid currentPid;
                result = itrq[li.bodyPart]->getTorquePid(joint, &currentPid);
                if (!result) break;
                if (pValue != NULL)
                    currentPid.kp = *pValue;
                if (dValue != NULL)
                    currentPid.kd = *dValue;
                if (iValue != NULL)
                    currentPid.ki = *iValue;
                result = itrq[li.bodyPart]->setTorquePid(joint, currentPid);
                break;
            }
            default:
                break;
        }
    }
    return result;
}


bool icubWholeBodyActuators::setControlOffset(const double *value, int joint)
{
    //The FOR_ALL atomicity is debated in github.. currently do the same as the rest of the library
    if (!value) return false;
    
    bool result = true;
    if (joint < 0) {
        //todo
        result = false;
    }
    else {
        LocalId li = jointIdList.globalToLocalId(joint);
        switch (currentCtrlModes[li]) {
            case wbi::CTRL_MODE_TORQUE:
            {
                Pid currentPid;
                result = itrq[li.bodyPart]->getTorquePid(joint, &currentPid);
                if (!result) break;
                currentPid.offset = *value;
                result = itrq[li.bodyPart]->setTorquePid(joint, currentPid);
                break;
            }
            default:
                break;
        }
    }
    return result;
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
//        int i;
//        if( reverse_torso_joints ) {
//            i = li.bodyPart==TORSO ? 2-li.index : li.index; // icub's torso joints are in reverse order
//        } else {
//            i = li.index;
//        }
//        return itrq[li.bodyPart]->setRefTorque(i, *taud);
//    }
//    
//    bool ok = true;
//    unsigned int i=0;
//    FOR_ALL(itBp, itJ)
//    {
//        int j;
//        if( reverse_torso_joints ) {
//           j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
//        } else {
//           j = *itJ;
//        }
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
//        assert(false) (add reverse_torso_joints logic)
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
//        assert(false) (add reverse_torso_joints logic)
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
//        assert(false) (add reverse_torso_joints logic)
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
//        assert(false) (add reverse_torso_joints logic)
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
//        assert(false) (add reverse_torso_joints logic)
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

