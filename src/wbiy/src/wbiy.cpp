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

bool wbiy::openPolyDriver(const string &localName, const string &robotName, PolyDriver* &pd, const string &bodyPartName)
{
    string localPort  = "/" + localName + "/" + bodyPartName;
    string remotePort = "/" + robotName + "/" + bodyPartName;
    Property options;
    options.put("robot",robotName.c_str());
    options.put("part",bodyPartName.c_str());
    options.put("device","remote_controlboard");
    options.put("local",localPort.c_str());
    options.put("remote",remotePort.c_str());
    
    pd = new PolyDriver(options);
    if(!pd || !(pd->isValid()))
    {
        fprintf(stderr,"Problems instantiating the device driver %s\n", bodyPartName.c_str());
        return false;
    }
    return true;
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY SENSORS
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames)
: name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), dof(0), initDone(false) {}

bool yarpWholeBodySensors::openDrivers(int bp)
{
    ienc[bp]=0; iopl[bp]=0;  dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], bodyPartNames[bp]))
        return false;
    
    if(dd[bp]==0)
        printf("dd is zero\n");
    bool ok = dd[bp]->view(ienc[bp]);
    if(robot!="icubSim")
        dd[bp]->view(iopl[bp]);
    if(!ok)
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", bodyPartNames[bp].c_str());
        return false;
    }
    
    int nj=0;
    ienc[bp]->getAxes(&nj);
    bodyPartAxes[bp] = nj;
    return true;
}

bool yarpWholeBodySensors::init()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
        ok = ok && openDrivers(itBp->first);
    initDone = true;
    return ok;
}

bool yarpWholeBodySensors::removeJoint(const LocalId &j)
{
    if(!jointIdList.removeId(j))
        return false;
    dof--;
    return true;
}

bool yarpWholeBodySensors::addJoint(const LocalId &j)
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

int yarpWholeBodySensors::addJoints(const LocalIdList &jList)
{
    if(initDone)
        for(LocalIdList::const_iterator it=jList.begin(); it!=jList.end(); it++)
            if(!jointIdList.containsBodyPart(it->first))
                if(!openDrivers(it->first))
                    return 0;
    int count = jointIdList.addIdList(jList);
    dof += count;
    return count;
}

bool yarpWholeBodySensors::readEncoders(double *q, double *stamps, bool wait)
{
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS(itBp)
    {
//        while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
        while( !(update=ienc[itBp->first]->getEncoders(qTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
            {
                qLastRead[itBp->first][i]            = qTemp[i];
                qStampLastRead[itBp->first][i]   = tTemp[i];
            }
        
        if(stamps!=NULL)
            FOR_ALL_JOINTS(itBp, itJ)
            {
                q[i]        = qLastRead[itBp->first][*itJ];
                stamps[i]   = qStampLastRead[itBp->first][*itJ];
                i++;
            }
        else
            FOR_ALL_JOINTS(itBp, itJ)
            {
                q[i]        = qLastRead[itBp->first][*itJ];
                i++;
            }
        res = res && update;
    }
    return res || wait;
}

bool yarpWholeBodySensors::readPwm(double *pwm, double *stamps, bool wait)
{
    double pwmTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS(itBp)
    {
        // read data
        while( !(update=iopl[itBp->first]->getOutputs(pwmTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        // if reading has succeeded, update last read data
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
                pwmLastRead[itBp->first][i] = pwmTemp[i];
        
        // copy data in output vector
        FOR_ALL_JOINTS(itBp, itJ)
        {
            pwm[i]      = pwmLastRead[itBp->first][*itJ];
            // stamps[i]   = ?;
            i++;
        }
        res = res && update;
    }
    return res || wait;
}

bool yarpWholeBodySensors::readInertial(double *inertial, double *stamps, bool wait)
{
    return false;
}

bool yarpWholeBodySensors::readFTsensors(double *ftSens, double *stamps, bool wait)
{
    return false;
}

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
                    ok = ok && icmd[itBp->first]->setPositionMode(*itJ);
                break;
                
            case CTRL_MODE_VEL:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setVelocityMode(*itJ);
                break;
                
            case CTRL_MODE_TORQUE:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setTorqueMode(*itJ);
                break;
                
            case CTRL_MODE_OPEN_LOOP:
                FOR_ALL(itBp, itJ)
                    ok = ok && icmd[itBp->first]->setOpenLoopMode(*itJ);
                break;
                
            default:
                return false;
        }
        return ok;
    }
    
    LocalId li = jointIdList.globalToLocalId(joint);
    switch(controlMode)
    {
        case CTRL_MODE_POS:         return icmd[li.bodyPart]->setPositionMode(li.index);
        case CTRL_MODE_VEL:         return icmd[li.bodyPart]->setVelocityMode(li.index);
        case CTRL_MODE_TORQUE:      return icmd[li.bodyPart]->setTorqueMode(li.index);
        case CTRL_MODE_OPEN_LOOP:   return icmd[li.bodyPart]->setOpenLoopMode(li.index);
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
        return itrq[li.bodyPart]->setRefTorque(li.index, *taud);
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        ok = ok && itrq[itBp->first]->setRefTorque(*itJ, taud[i]);
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
        return ipos[li.bodyPart]->positionMove(li.index, *qd);
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        ok = ok && ipos[itBp->first]->positionMove(*itJ, qd[i]);
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
        return ivel[li.bodyPart]->velocityMove(li.index, *dqd);
    }
    
    unsigned int i = 0;
    bool ok = true;
    FOR_ALL(itBp, itJ)
    {
        ok = ok && ivel[itBp->first]->velocityMove(*itJ, dqd[i]);
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
        return iopl[li.bodyPart]->setOutput(li.index, *pwmd);
    }
    
    bool ok = true;
    unsigned int i=0;
    FOR_ALL(itBp, itJ)
    {
        ok = ok && iopl[itBp->first]->setOutput(*itJ, pwmd[i]);
        i++;
    }
    
    return ok;
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ROBOT WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
robotWholeBodyStates::robotWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow)
{

}

bool robotWholeBodyStates::init(){ return false; }
int robotWholeBodyStates::getDoFs(){ return 0; }
bool robotWholeBodyStates::removeJoint(const wbi::LocalId &j){ return false; }
bool robotWholeBodyStates::addJoint(const wbi::LocalId &j){ return false; }
int robotWholeBodyStates::addJoints(const wbi::LocalIdList &j){ return 0; }

bool robotWholeBodyStates::getQ(double *q, double time, bool wait){ return false; }
bool robotWholeBodyStates::getDq(double *dq, double time, bool wait){ return false; }
bool robotWholeBodyStates::getDqMotors(double *dqM, double time, bool wait){ return false; }
bool robotWholeBodyStates::getD2q(double *d2q, double time, bool wait){ return false; }
bool robotWholeBodyStates::getPwm(double *pwm, double time, bool wait){ return false; }
bool robotWholeBodyStates::getInertial(double *inertial, double time, bool wait){ return false; }
bool robotWholeBodyStates::getFTsensors(double *ftSens, double time, bool wait){ return false; }
bool robotWholeBodyStates::getTorques(double *tau, double time, bool wait){ return false; }



// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY MODEL
// *********************************************************************************************************************
// *********************************************************************************************************************
bool icubWholeBodyModel::init()
{
    return false;
}

int icubWholeBodyModel::getDoFs()
{
    return 0;
}

bool icubWholeBodyModel::removeJoint(const wbi::LocalId &j)
{
    return false;
}

bool icubWholeBodyModel::addJoint(const wbi::LocalId &j)
{
    return false;
}

int icubWholeBodyModel::addJoints(const wbi::LocalIdList &j)
{
    return false;
}

bool icubWholeBodyModel::getJointLimits(double *qMin, double *qMax, int joint)
{
    return false;    
}

bool icubWholeBodyModel::computeH(double *q, double *xBase, int linkId, double *H)
{
    return false;    
}

bool icubWholeBodyModel::computeJacobian(double *q, double *xBase, int linkId, double *J, double *pos)
{
    return false;    
}

bool icubWholeBodyModel::computeDJdq(double *q, double *xB, double *dq, double *dxB, int linkId, double *dJdq, double *pos)
{
    return false;    
}

bool icubWholeBodyModel::forwardKinematics(double *q, double *xB, int linkId, double *x)
{
    return false;
}

bool icubWholeBodyModel::inverseDynamics(double *q, double *xB, double *dq, double *dxB, double *ddq, double *ddxB, double *tau)
{
    return false;    
}

bool icubWholeBodyModel::directDynamics(double *q, double *xB, double *dq, double *dxB, double *M, double *h)
{
    return false;
}



// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY INTERFACE
// *********************************************************************************************************************
// *********************************************************************************************************************

bool icubWholeBodyInterface::init()
{
    return yarpWholeBodyActuators::init();
}

int icubWholeBodyInterface::getDoFs()
{
    return yarpWholeBodyActuators::dof;
}

bool icubWholeBodyInterface::removeJoint(const LocalId &j)
{
    return yarpWholeBodyActuators::removeJoint(j);
}

bool icubWholeBodyInterface::addJoint(const LocalId &j)
{
    return yarpWholeBodyActuators::addJoint(j);
}

int icubWholeBodyInterface::addJoints(const LocalIdList &jList)
{
    return yarpWholeBodyActuators::addJoints(jList);
}



// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY STATES
// *********************************************************************************************************************
// *********************************************************************************************************************
//icubWholeBodyStates::icubWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow)
//:name(_name), robot(_robotName), estWind(estimationTimeWindow){}
//
//icubWholeBodyStates::icubWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow, const LocalIdList &jids)
//:name(_name), robot(_robotName), estWind(estimationTimeWindow)
//{
//    jointIdList = jids;
//    dof = jointIdList.getDoFs();
//}


//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName): name(_name), robot(_robotName)
//{
//    jointIdList.resize(5);
//    jointIdList[0] = LocalIdList(TORSO);
//    jointIdList[1] = LocalIdList(LEFT_ARM);
//    jointIdList[2] = LocalIdList(RIGHT_ARM);
//    jointIdList[3] = LocalIdList(LEFT_LEG);
//    jointIdList[4] = LocalIdList(RIGHT_LEG);
//}
//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName, LocalIdList bp1): name(_name), robot(_robotName)
//{
//    jointIdList.resize(1); jointIdList[0] = bp1.bp;
//}
//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName, LocalIdList bp1, LocalIdList bp2): 
//                                    name(_name), robot(_robotName)
//{
//    jointIdList.resize(2); jointIdList[0] = bp1; jointIdList[1] = bp2;
//}
//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName, LocalIdList bp1, LocalIdList bp2, LocalIdList bp3): 
//                                    name(_name), robot(_robotName)
//{
//    jointIdList.resize(3); jointIdList[0] = bp1.bp; jointIdList[1] = bp2.bp; jointIdList[2] = bp3.bp;
//}
//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName, LocalIdList bp1, LocalIdList bp2, 
//                                   LocalIdList bp3, LocalIdList bp4): name(_name), robot(_robotName)
//{
//    jointIdList.resize(4); jointIdList[0] = bp1; jointIdList[1] = bp2; jointIdList[2] = bp3; jointIdList[3] = bp4;
//}
//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName, LocalIdList bp1, LocalIdList bp2, 
//                                   LocalIdList bp3, LocalIdList bp4, LocalIdList bp5): name(_name), robot(_robotName)
//{
//    jointIdList.resize(5); jointIdList[0] = bp1; jointIdList[1] = bp2; jointIdList[2] = bp3; jointIdList[3] = bp4; jointIdList[4] = bp5;
//}
//
//wholeBodyInterface::wholeBodyInterface(const char* _name, const char* _robotName, LocalIdList bp1, LocalIdList bp2, 
//                                   LocalIdList bp3, LocalIdList bp4, LocalIdList bp5, LocalIdList bp6): name(_name), robot(_robotName)
//{
//    jointIdList.resize(6); jointIdList[0] = bp1; jointIdList[1] = bp2; jointIdList[2] = bp3; jointIdList[3] = bp4; jointIdList[4] = bp5; jointIdList[5] = bp6;
//}
//
//bool wholeBodyInterface::init()
//{
//    std::string part;    
//    std::string localPort;
//    std::string remotePort;
//    LocalId localId;
//    vector<unsigned int> *jl;
//    int globalId = 0, nj=0;
//    BodyPart i;
//    bool ok = true;
//
//    for (unsigned int iii=0; iii<jointIdList.size(); iii++)
//    {
//        i = jointIdList[iii].bp;
//        
//        localId.bp = i;
//        jl = &jointIdList[iii].joints;
//        for(vector<unsigned int>::const_iterator it=jl->begin(); it!=jl->end(); it++)
//        {
//            localId.joint = *it;
//            local2globalIndex[localId] = globalId;
//            global2localIndex.push_back(localId);
//            globalId++;
//        }
//        
//        ipos[i]=0;  itrq[i]=0;  iimp[i]=0;  icmd[i]=0;  ienc[i]=0;  ienct[i]=0;
//        ipid[i]=0;  ivel[i]=0;  iamp[i]=0;  iopl[i]=0;  dd[i]=0;
//
//        part = BodyPart_s[i];
//        localPort  = "/" + name + "/" + part;
//        remotePort = "/" + robot + "/" + part;
//        options[i].put("robot",robot.c_str());
//        options[i].put("part",part.c_str());
//        options[i].put("device","remote_controlboard");
//        options[i].put("local",localPort.c_str());
//        options[i].put("remote",remotePort.c_str());
//
//        dd[i] = new PolyDriver(options[i]);
//        if(!dd[i] || !(dd[i]->isValid()))
//            fprintf(stderr,"Problems instantiating the device driver %s\n", part.c_str());
//        
//        ok = ok & dd[i]->view(ipos[i]);
//        ok = ok & dd[i]->view(itrq[i]);
//        ok = ok & dd[i]->view(iimp[i]);
//        ok = ok & dd[i]->view(icmd[i]);
//        ok = ok & dd[i]->view(ivel[i]);
//        ok = ok & dd[i]->view(ienc[i]);
//        ok = ok & dd[i]->view(ienct[i]);
//        ok = ok & dd[i]->view(ipid[i]);
//        ok = ok & dd[i]->view(iamp[i]);
//        ok = ok & dd[i]->view(iopl[i]);
//        if(!ok)
//            printf("Problem initializing drivers of %s\n", part.c_str());
//        else
//        {
//            ienc[i]->getAxes(&nj);
//            bodyPartAxes[i] = nj;
//        }
//
//        
//        gravityPorts[i] = new BufferedPort<Vector>;
//        ok = ok & gravityPorts[i]->open(("/"+name+"/"+part+"_gravity_torques:i").c_str());
//        ok = ok & Network::connect(("/gravityCompensator/"+part+"_torques:o").c_str(), gravityPorts[i]->getName().c_str());
//        Vector *temp;
//        int counter = 10;
//        while( (temp=gravityPorts[i]->read(false))==0 && counter>0) { counter--; }
//        if(temp!=0)
//            gravityTorques[i] = *temp;
//        else
//            printf("Problem reading gravity torques for %s\n", part.c_str());
//    }
//
//    return ok;
//}