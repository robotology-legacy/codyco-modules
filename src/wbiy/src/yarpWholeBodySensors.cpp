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
#include <cassert>

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

bool wbiy::isRobotSimulator(const string &robotName)
{
    return robotName=="icubSim";
}

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

    bool ok = dd[bp]->view(ienc[bp]);
    if(!isRobotSimulator(robot))
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

bool yarpWholeBodySensors::close()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
    {
        assert(dd[itBp->first]!=NULL);
        ok = ok && dd[itBp->first]->close();
    }
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
        assert(ienc[itBp->first]!=NULL);
        while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
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
