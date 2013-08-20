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
#include <sstream>
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

string wbiy::getPortName(const LocalId &lid, const id_2_PortName *id2port, const int size)
{
    int i=0;
    do
    {
        if(id2port[i].id == lid)
            return id2port[i].portName;
        i++;
    }
    while(i<size);
    return "";
}

string wbiy::getPortName(const LocalId &lid, const vector<id_2_PortName> id2port)
{ 
    return getPortName(lid, &id2port[0], id2port.size());
}

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
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames, 
    const std::vector<id_2_PortName> &_ftSens_2_port, const std::vector<id_2_PortName> &_imu_2_port)
: name(_name), robot(_robotName), bodyPartNames(_bodyPartNames), ftSens_2_port(_ftSens_2_port), imu_2_port(_imu_2_port), 
dof(0), initDone(false) {}

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

bool yarpWholeBodySensors::openImu(const LocalId &i)
{
    string remotePort = "/" + robot + getPortName(i, imu_2_port);
    stringstream localPort; 
    localPort << "/" << name << "/imu" << i.bodyPart << "_" << i.index << ":i";
    portsIMU[i] = new BufferedPort<Vector>();
    if(!portsIMU[i]->open(localPort.str().c_str())) // open local input port
        return false;
    if(!Network::exists(remotePort.c_str()))        // check remote output port exists
        return false;
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp", true))   // connect remote to local port
        return false;
    return true;
}

bool yarpWholeBodySensors::openFTsens(const LocalId &i)
{
    string remotePort = "/" + robot + getPortName(i, ftSens_2_port);
    stringstream localPort; 
    localPort << "/" << name << "/ftSens" << i.bodyPart << "_" << i.index << ":i";
    portsFTsens[i] = new BufferedPort<Vector>();
    if(!portsFTsens[i]->open(localPort.str().c_str()))  // open local input port
        return false;
    if(!Network::exists(remotePort.c_str()))            // check remote output port exists
        return false;
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp"))   // connect remote to local port
        return false;
    return true;
}

bool yarpWholeBodySensors::init()
{
    bool initDone = true;
    FOR_ALL_BODY_PARTS(itBp)
        initDone = initDone && openDrivers(itBp->first);

    for(LocalIdList::iterator itBp=ftSensIdList.begin(); itBp!=ftSensIdList.end(); itBp++)
        for(vector<int>::iterator itId=itBp->second.begin(); itId!=itBp->second.end(); itId++)
            initDone = initDone && openFTsens(LocalId(itBp->first,*itId));
    
    for(LocalIdList::iterator itBp=imuIdList.begin(); itBp!=imuIdList.end(); itBp++)
        for(vector<int>::iterator itId=itBp->second.begin(); itId!=itBp->second.end(); itId++)
            initDone = initDone && openImu(LocalId(itBp->first,*itId));

    return initDone;
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
    /* Shorter implementation */
    /*bool res;
    dof -= res=jointIdList.removeId(j) ? 1 : 0;
    return res;*/
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

bool yarpWholeBodySensors::addIMU(const wbi::LocalId &i)
{
    // if initialization was done, then open port of specified IMU
    // if initialization was not done, ports will be opened during initialization
    if(initDone && !imuIdList.containsId(i))
        if(!openImu(i))
            return false;
    return imuIdList.addId(i);
}

bool yarpWholeBodySensors::addFTsensor(const wbi::LocalId &i)
{
    // if initialization was done, then open port of specified F/T sensor
    // if initialization was not done, ports will be opened during initialization
    if(initDone && !ftSensIdList.containsId(i))
        if(!openFTsens(i))
            return false;
    return ftSensIdList.addId(i);
}

bool yarpWholeBodySensors::readEncoders(double *q, double *stamps, bool wait)
{
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS(itBp)
    {
        assert(ienc[itBp->first]!=NULL);
        // read encoders
        while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        // if read succeeded => update data
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
            {
                qLastRead[itBp->first][i]        = qTemp[i];
                qStampLastRead[itBp->first][i]   = tTemp[i];
            }
        
        // copy most recent data into output variables
        FOR_ALL_JOINTS(itBp, itJ)
        {
            q[i] = qLastRead[itBp->first][*itJ];
            if(stamps!=NULL)
                stamps[i] = qStampLastRead[itBp->first][*itJ];
            i++;
        }
        res = res && update;    // if read failed => return false
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

bool yarpWholeBodySensors::readIMU(double *inertial, double *stamps, bool wait)
{
    return false;
}

bool yarpWholeBodySensors::readFTsensors(double *ftSens, double *stamps, bool wait)
{
    return false;
}
