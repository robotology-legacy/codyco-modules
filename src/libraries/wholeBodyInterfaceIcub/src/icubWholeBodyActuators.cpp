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
: m_commandedParts(0), initDone(false), dof(0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), reverse_torso_joints(true)
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
,_torqueModuleConnection(0)
#endif
{}

icubWholeBodyActuators::icubWholeBodyActuators(const char* _name,
                                               const char* _robotName, 
                                               const yarp::os::Property & yarp_wbi_properties)
: m_commandedParts(0), initDone(false), dof(0), name(_name), robot(_robotName)
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
,_torqueModuleConnection(0)
#endif
{
    yarp::os::Property yarp_wbi_properties_not_const = yarp_wbi_properties;
    loadBodyPartsFromConfig(yarp_wbi_properties_not_const,bodyPartNames);
    loadReverseTorsoJointsFromConfig(yarp_wbi_properties_not_const,reverse_torso_joints);
}



icubWholeBodyActuators::~icubWholeBodyActuators()
{
    close();
}

bool icubWholeBodyActuators::openDrivers(int bp)
{
    if( bp >= (int)bodyPartNames.size() || bp < 0 ) { 
        std::cerr << "icubWholeBodyActuators::openDrivers error: called with bodypart " << bp << 
                     " but the total number of bodyparts is " << bodyPartNames.size() << std::endl;
        return false;
    }
    itrq[bp]=0; iimp[bp]=0; icmd[bp]=0; ivel[bp]=0; ipos[bp]=0; iopl[bp]=0;  dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], bodyPartNames[bp].c_str()))
        return false;
    
    bool ok = dd[bp]->view(itrq[bp]) && dd[bp]->view(iimp[bp]) && dd[bp]->view(icmd[bp])
              && dd[bp]->view(ivel[bp]) && dd[bp]->view(ipos[bp]) && dd[bp]->view(iopl[bp])
              && dd[bp]->view(positionDirectInterface[bp]);
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
    if (ok && dof > 0) {
        m_commandedParts = new unsigned char[dof];
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
    if (m_commandedParts) {
        delete [] m_commandedParts;
        m_commandedParts = 0;
    }
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
    if (initDone) return false;
    
    if(!jointIdList.removeId(j))
        return false;
    dof--;
    return true;
}

bool icubWholeBodyActuators::addActuator(const LocalId &j)
{
    if (initDone) return false;

    if(!jointIdList.addId(j))
        return false;
    
    dof++;
    return true;
}

int icubWholeBodyActuators::addActuators(const LocalIdList &jList)
{
    if (initDone) return false;

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
            case CTRL_MODE_DIRECT_POSITION:
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
                if(!isICubSimulator(robot)) ///< iCub simulator does not implement PWM motor control
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
            case CTRL_MODE_POS:
            case CTRL_MODE_DIRECT_POSITION:
                ok = icmd[li.bodyPart]->setPositionMode(i); break;
            case CTRL_MODE_VEL:         ok = icmd[li.bodyPart]->setVelocityMode(i); break;
            case CTRL_MODE_TORQUE:      ok = icmd[li.bodyPart]->setTorqueMode(i);   break;
            ///< iCub simulator does not implement PWM motor control
            case CTRL_MODE_MOTOR_PWM:   ok = isICubSimulator(robot) ? true : icmd[li.bodyPart]->setOpenLoopMode(i); break;
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
            case CTRL_MODE_DIRECT_POSITION: return positionDirectInterface[li.bodyPart]->setPosition(i, CTRL_RAD2DEG*(*ref));
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
            case CTRL_MODE_MOTOR_PWM:   return isICubSimulator(robot) ? true : iopl[li.bodyPart]->setOutput(i, *ref);
            default: break;
        }
        return false;
    }
    // set control references for all joints
    ///< on robot use new method which set all joint vel of one body part at the same time (much faster!)
//    if(!isRobotSimulator(robot)) //Why not on robot simulator? Shouldn't iCub_SIM implement these methods?
//    {
        double speedReferences[MAX_NJ];     // vector of reference joint speeds
        double torqueReferences[MAX_NJ]; //vector of reference joint torques
        memset(torqueReferences, 0, sizeof(double) * MAX_NJ); //set to zero all the references torques
        double positionReferences[MAX_NJ]; //vector of reference positions
        memset(positionReferences, 0, sizeof(double) * MAX_NJ); //set to zero all the references positions
    
        memset(m_commandedParts, 0, sizeof(unsigned char) * dof); //reset the command map
        
        int velocityJointIDs[MAX_NJ];   // vector of joint ids for velocity move (implementing more advanced velocity function)
        unsigned int i = 0;                // counter of controlled joints
        FOR_ALL_BODY_PARTS(itBp)
        {
            //allow setting of reference on the whole part only if all joints are the same
            wbi::ControlMode partControlMode = CTRL_MODE_UNKNOWN;
            int njVelCtrl = 0;              // number of joints that are velocity controlled
            int jointsInPart = itBp->second.size();   // number of joints of this body part
            int jointIndex = 0;
            for(int j = 0; j < jointsInPart; j++)
            {
                LocalId localId = LocalId(itBp->first, itBp->second[j]);
                wbi::ControlMode currentControlMode = currentCtrlModes[localId];
                
                if (partControlMode != CTRL_MODE_UNKNOWN
                    && partControlMode != currentControlMode) {
                    //a different control mode for a joint in the part
                    partControlMode = CTRL_MODE_UNKNOWN;
                    i += jointsInPart - j; //to be checked
                    break;
                    //should save parts done
                }
                
                jointIndex = j;
                if (reverse_torso_joints) {
                    jointIndex = itBp->first == TORSO ? 2 - j : j; // icub's torso joints are in reverse order
                }
                
                if(currentControlMode == CTRL_MODE_VEL)
                {
                    partControlMode = CTRL_MODE_VEL;
                    velocityJointIDs[jointIndex] = jointIndex;
                    speedReferences[jointIndex] = CTRL_RAD2DEG * ref[i];           // convert joint vel from rad to deg
                    njVelCtrl++;
                }
                else if (currentControlMode == CTRL_MODE_TORQUE) {
                    partControlMode = CTRL_MODE_TORQUE;
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
                    if (_torqueModuleConnection) {
                        //in this case skip to second part
                        i += jointsInPart - j; //to be checked
                        partControlMode = CTRL_MODE_UNKNOWN;
                        break;
                    }
#endif
                    torqueReferences[jointIndex] = ref[i];           // convert joint vel from rad to deg
                }
                else if (currentControlMode == CTRL_MODE_POS) {
                    partControlMode = CTRL_MODE_POS;
                    positionReferences[jointIndex] = CTRL_RAD2DEG * ref[i];
                }
                else if (currentControlMode == wbi::CTRL_MODE_DIRECT_POSITION) {
                    partControlMode = CTRL_MODE_DIRECT_POSITION;
                    positionReferences[jointIndex] = CTRL_RAD2DEG * ref[i];
                }
                i++;
            }
            switch (partControlMode) {
                case wbi::CTRL_MODE_VEL:
                    ok = ok && ivel[itBp->first]->velocityMove(njVelCtrl, velocityJointIDs, speedReferences);
                    //save joints commanded
                    memset(m_commandedParts + i - jointsInPart, 1, sizeof(unsigned char) * jointsInPart);
                    break;
                case wbi::CTRL_MODE_TORQUE:
                    ok = ok && itrq[itBp->first]->setRefTorques(torqueReferences);
                    memset(m_commandedParts + i - jointsInPart, 1, sizeof(unsigned char) * jointsInPart);
                    break;
                case wbi::CTRL_MODE_POS:
                    ok = ok && ipos[itBp->first]->positionMove(positionReferences);
                    memset(m_commandedParts + i - jointsInPart, 1, sizeof(unsigned char) * jointsInPart);
                    break;
                case wbi::CTRL_MODE_DIRECT_POSITION:
                    ok = ok && positionDirectInterface[itBp->first]->setPositions(positionReferences);
                    memset(m_commandedParts + i - jointsInPart, 1, sizeof(unsigned char) * jointsInPart);
                    break;
                default:
                    break;
            }
        }
//    }
    
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
    //TEMP
    if (_torqueModuleConnection) {
        _torqueRefs.zero();
    }
    //END TEMP
#endif
    
    i = 0;
    FOR_ALL(itBp, itJ)
    {
        if (m_commandedParts[i]) { //skip if joint is already controlled
            i++;
            continue;
        }
        int j;
        if( reverse_torso_joints ) {
            j = itBp->first==TORSO ? 2-(*itJ) : *itJ; // icub's torso joints are in reverse order
        } else {
            j = *itJ;
        }
        LocalId localID = LocalId(itBp->first,*itJ);
        printf("[%s:%d]Setting single-part mode for part-joint %d-%d\n", __FILE__, __LINE__, itBp->first, *itJ);
        switch(currentCtrlModes[localID])
        {
            case CTRL_MODE_POS:         
                ok = ok && ipos[itBp->first]->positionMove(j, CTRL_RAD2DEG*ref[i]); 
                break;
            case CTRL_MODE_DIRECT_POSITION:         
                ok = ok && positionDirectInterface[itBp->first]->setPosition(j, CTRL_RAD2DEG*ref[i]); 
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
                if(!isICubSimulator(robot)) ///< iCub simulator does not implement PWM motor control
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
        case CTRL_PARAM_OFFSET: return setControlOffset((double*)value, joint);
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
