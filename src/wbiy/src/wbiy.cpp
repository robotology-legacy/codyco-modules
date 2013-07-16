/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
 *
 * Further modifications
 *
 * Copyright (C) 2013 CoDyCo Consortium
 * Author: Serena Ivaldi
 * email: serena.ivaldi@isir.upmc.fr
 *
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

#include <wbiy/wbiy.h>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace wbi;
using namespace wbiy;

#define MAX_NJ 20
#define WAIT_TIME 0.001

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)

//---------------------------------------------------
bool wbiy::openPolyDriver(const string &localName, const string &robotName, PolyDriver *pd, int bp, const std::string &bodyPartName)
{
    if(bodyPartName.empty())
    {
        fprintf(stderr,"Problems instantiating the device driver -- bodyPart was not initialized properly \n");
        return false;
    }
    string part = bodyPartName;
    string localPort  = "/" + localName + "/" + part;
    string remotePort = "/" + robotName + "/" + part;
    Property options;
    options.put("robot",robotName.c_str());
    options.put("part",part.c_str());
    options.put("device","remote_controlboard");
    options.put("local",localPort.c_str());
    options.put("remote",remotePort.c_str());

    pd = new PolyDriver(options);
    if(!pd || !(pd->isValid()))
    {
        fprintf(stderr,"Problems instantiating the device driver %s\n", part.c_str());
        return false;
    }
    return true;
}
//---------------------------------------------------
bool wbiy::updateLocal2GlobalIndex(JointIds &jId, unsigned int dof, jointMap &l2g, vector<JointId> &g2l)
{
    JointId localId;
    int globalId = 0;
    g2l.resize(dof);

    FOR_ALL_OF(itBp, joint, jId)
    {
        localId.bp = itBp->first;
        localId.joint = *joint;

        l2g[localId] = globalId;
        g2l[globalId] = localId;

        globalId++;
    }
    return true;
}
//---------------------------------------------------

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY SENSORS
// *********************************************************************************************************************
// *********************************************************************************************************************

//---------------------------------------------------
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const char* _robotName): initDone(false), dof(0) {}
//---------------------------------------------------
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const char* _robotName, const JointIds &jids): initDone(false), dof(0), jointIdList(jids)
{
    FOR_ALL_BODY_PARTS(itBp)
        dof += itBp->second.size();
}
//---------------------------------------------------
bool yarpWholeBodySensors::updateJointList()
{
    return updateLocal2GlobalIndex(jointIdList, dof, local2globalIndex, global2localIndex);
}
//---------------------------------------------------
void yarpWholeBodySensors::setBodyPartName(int bodyPart, const std::string &nameBodyPart)
{
    bodyPartsName[bodyPart] = nameBodyPart;
}
//---------------------------------------------------
bool yarpWholeBodySensors::openDrivers(int bp)
{
    ienc[bp]=0; iopl[bp]=0;  dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], bp, bodyPartsName[bp]))
        return false;
    
    bool ok = dd[bp]->view(ienc[bp]) && dd[bp]->view(iopl[bp]);
    if(!ok)
    {
        fprintf(stderr, "Problem initializing drivers of body part %d (%s)\n", bp, bodyPartsName[bp].c_str());
        return false;
    }
    
    int nj=0;
    ienc[bp]->getAxes(&nj);
    bodyPartAxes[bp] = nj;
    return true;
}
//---------------------------------------------------
bool yarpWholeBodySensors::init()
{
    updateJointList();
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
        ok = ok && openDrivers(itBp->first);
    initDone = true;
    return ok;
}
//---------------------------------------------------
bool yarpWholeBodySensors::removeJoint(const JointId &j)
{
    if(!jointIdList.removeJoint(j))
        return false;

    dof--;
    updateJointList();
    return false;
}
//---------------------------------------------------
bool yarpWholeBodySensors::addJoint(const JointId &j)
{
    bool ok = true;
    // if initialization was done and drivers of specified body part are not open, then open them
    // if initialization was not done, drivers will be opened during initialization
    if(initDone && !jointIdList.containsBodyPart(j.bp))
        ok = openDrivers(j.bp);

    jointIdList.addJoint(j);
    dof++;
    ok = ok && updateJointList();
    return ok;
}
//---------------------------------------------------
bool yarpWholeBodySensors::addJoints(const JointIds &jList)
{
    bool ok = true;

    for(JointIds::const_iterator it=jList.begin(); it!=jList.end(); it++)
    {
        if(initDone && !jointIdList.containsBodyPart(it->first))
            ok = ok && openDrivers(it->first);
        FOR_ALL_JOINTS(it, itJ)
        {
            if(jointIdList.addJoint(JointId(it->first, *itJ)))
                dof++;
        }
    }
    updateJointList();
    return ok;
}
//---------------------------------------------------
bool yarpWholeBodySensors::readEncoders(double *q, double *stamps, bool wait)
{
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS(itBp)
    {
        while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
            {
                qLastRead[itBp->first][i]            = qTemp[i];
                qStampLastRead[itBp->first][i]   = tTemp[i];
            }
        
        FOR_ALL_JOINTS(itBp, itJ)
        {
            q[i]        = qLastRead[itBp->first][*itJ];
            stamps[i]   = qStampLastRead[itBp->first][*itJ];
            i++;
        }
        res &= update;
    }
    return res || wait;
}
//---------------------------------------------------
bool yarpWholeBodySensors::readPwm(double *pwm, double *stamps, bool wait)
{
    double pwmTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS(itBp)
    {
        while( !(update=iopl[itBp->first]->getOutputs(pwmTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
                pwmLastRead[itBp->first][i] = pwmTemp[i];
        
        FOR_ALL_JOINTS(itBp, itJ)
        {
            pwm[i]      = pwmLastRead[itBp->first][*itJ];
            stamps[i]   = 0.0;
            i++;
        }
        res &= update;
    }
    return res || wait;
}
//---------------------------------------------------
bool yarpWholeBodySensors::readInertial(double *inertial, double *stamps, bool wait)
{
    return false;
}
//---------------------------------------------------
bool yarpWholeBodySensors::readFTsensors(double *ftSens, double *stamps, bool wait)
{
    return false;
}
//---------------------------------------------------

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY ACTUATOR
// *********************************************************************************************************************
// *********************************************************************************************************************

//---------------------------------------------------
bool yarpWholeBodyActuators::openDrivers(int bp)
{
    itrq[bp]=0; iimp[bp]=0; icmd[bp]=0; ivel[bp]=0; ipos[bp]=0; iopl[bp]=0;  dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], bp, bodyPartsName[bp]))
        return false;
    
    bool ok =   dd[bp]->view(itrq[bp]) && dd[bp]->view(iimp[bp]) && dd[bp]->view(icmd[bp]);
    ok = ok  && dd[bp]->view(ivel[bp]) && dd[bp]->view(ipos[bp]) && dd[bp]->view(iopl[bp]);
    if(!ok)
    {
        fprintf(stderr, "Problem initializing drivers of body part %d (%s)\n", bp, bodyPartsName[bp].c_str());
        return false;
    }
    return true;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::init()
{
    updateJointList();
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
        ok = ok && openDrivers(itBp->first);
    initDone = true;
    return ok;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::removeJoint(const JointId &j)
{
    if(!jointIdList.removeJoint(j))
        return false;

    dof--;
    updateJointList();
    return false;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::addJoint(const JointId &j)
{
    bool ok = true;
    // if initialization was done and drivers of specified body part are not open, then open them
    // if initialization was not done, drivers will be opened during initialization
    if(initDone && !jointIdList.containsBodyPart(j.bp))
        ok = openDrivers(j.bp);

    jointIdList.addJoint(j);
    dof++;
    ok = ok && updateJointList();
    return ok;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::addJoints(const JointIds &jList)
{
    bool ok = true;

    for(JointIds::const_iterator it=jList.begin(); it!=jList.end(); it++)
    {
        if(initDone && !jointIdList.containsBodyPart(it->first))
            ok = ok && openDrivers(it->first);
        FOR_ALL_JOINTS(it, itJ)
        {
            if(jointIdList.addJoint(JointId(it->first, *itJ)))
                dof++;
        }
    }
    updateJointList();
    return ok;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::setControlMode(int controlMode, int joint)
{
    switch(controlMode)
    {
    case CTRL_MODE_POS:
        if(joint<0)
            FOR_ALL(itBp, itJ)
                icmd[itBp->first]->setPositionMode(*itJ);
        else
            icmd[global2localIndex[joint].bp]->setPositionMode(global2localIndex[joint].joint);
        break;

    case CTRL_MODE_VEL:
        if(joint<0)
            FOR_ALL(itBp, itJ)
                icmd[itBp->first]->setVelocityMode(*itJ);
        else
            icmd[global2localIndex[joint].bp]->setVelocityMode(global2localIndex[joint].joint);
        break;

    case CTRL_MODE_TORQUE:
        if(joint<0)
            FOR_ALL(itBp, itJ)
                icmd[itBp->first]->setTorqueMode(*itJ);
        else
            icmd[global2localIndex[joint].bp]->setTorqueMode(global2localIndex[joint].joint);
        break;

    case CTRL_MODE_OPEN_LOOP:
        if(joint<0)
            FOR_ALL(itBp, itJ)
                icmd[itBp->first]->setOpenLoopMode(*itJ);
        else
            icmd[global2localIndex[joint].bp]->setOpenLoopMode(global2localIndex[joint].joint);
        break;

    default:
        return false;
    }
    
    return true;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::setTorqueRef(double *taud, int joint)
{
    if(joint> (int)dof)
        return false;

    if(joint>=0)
        itrq[global2localIndex[joint].bp]->setRefTorque(global2localIndex[joint].joint, *taud);
    else
    {
        unsigned int i=0;
        FOR_ALL(itBp, itJ)
        {
            itrq[itBp->first]->setRefTorque(*itJ, taud[i]);
            i++;
        }
    }

    return true;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::setPosRef(double *qd, int joint)
{
    if(joint> (int)dof)
        return false;

    if(joint>=0)
        ipos[global2localIndex[joint].bp]->positionMove(global2localIndex[joint].joint, *qd);
    else
    {
        unsigned int i=0;
        FOR_ALL(itBp, itJ)
        {
            ipos[itBp->first]->positionMove(*itJ, qd[i]);
            i++;
        }
    }

    return true;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::setVelRef(double *dqd, int joint)
{
    if(joint> (int)dof)
        return false;

    if(joint>=0)
        ivel[global2localIndex[joint].bp]->velocityMove(global2localIndex[joint].joint, *dqd);
    else
    {
        unsigned int i=0;
        FOR_ALL(itBp, itJ)
        {
            ivel[itBp->first]->velocityMove(*itJ, dqd[i]);
            i++;
        }
    }

    return true;
}
//---------------------------------------------------
bool yarpWholeBodyActuators::setPwmRef(double *pwmd, int joint)
{
    if(joint> (int)dof)
        return false;

    if(joint>=0)
        iopl[global2localIndex[joint].bp]->setOutput(global2localIndex[joint].joint, *pwmd);
    else
    {
        unsigned int i=0;
        FOR_ALL(itBp, itJ)
        {
            iopl[itBp->first]->setOutput(*itJ, pwmd[i]);
            i++;
        }
    }

    return true;
}
//---------------------------------------------------


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ROBOT WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
robotWholeBodyStates::robotWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow)
    :name(_name), robot(_robotName), estWind(estimationTimeWindow){}

robotWholeBodyStates::robotWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow, const JointIds &jids)
    :name(_name), robot(_robotName), estWind(estimationTimeWindow)
{
    jointIdList = jids;
    dof = jointIdList.getDoFs();
}


