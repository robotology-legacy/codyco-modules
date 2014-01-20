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

#include "wbiIcub/wholeBodyInterfaceIcub.h"
#include <yarp/os/Time.h>
#include <iCub/ctrl/math.h>
#include <string>
#include <sstream>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

#define MAX_NJ 20 ///< Maxinum number of joints for body part (used for buffers to avoid dynamic memory allocation)
#define WAIT_TIME 0.001

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY SENSORS
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodySensors::icubWholeBodySensors(const char* _name, const char* _robotName): initDone(false), name(_name), robot(_robotName)
{
    bodyPartNames = vector<string>(BodyPart_s, BodyPart_s + sizeof(BodyPart_s) / sizeof(string) );
    ftSens_2_port = vector<id_2_PortName>(icub_FTsens_2_PortName, icub_FTsens_2_PortName + sizeof(icub_FTsens_2_PortName)/sizeof(id_2_PortName));
    imu_2_port = vector<id_2_PortName>(icub_IMU_2_PortName, icub_IMU_2_PortName + sizeof(icub_IMU_2_PortName)/sizeof(id_2_PortName));
}

icubWholeBodySensors::icubWholeBodySensors(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames, 
                                           const std::vector<id_2_PortName> &_ftSens_2_port, const std::vector<id_2_PortName> &_imu_2_port)
    : initDone(false), name(_name), robot(_robotName) , bodyPartNames(_bodyPartNames), ftSens_2_port(_ftSens_2_port), imu_2_port(_imu_2_port)
{}

bool icubWholeBodySensors::init()
{
    bool initDone = true;
    FOR_ALL_BODY_PARTS_OF(itBp, encoderIdList)
        initDone = initDone && openEncoder(itBp->first);

    FOR_ALL_BODY_PARTS_OF(itBp, pwmSensIdList)
        initDone = initDone && openPwm(itBp->first);
    
    FOR_ALL_BODY_PARTS_OF(itBp, torqueSensorIdList)
        initDone = initDone && openTorqueSensor(itBp->first);

    for(LocalIdList::iterator itBp=ftSensIdList.begin(); itBp!=ftSensIdList.end(); itBp++)
        for(vector<int>::iterator itId=itBp->second.begin(); itId!=itBp->second.end(); itId++)
            initDone = initDone && openFTsens(LocalId(itBp->first,*itId));
    
    for(LocalIdList::iterator itBp=imuIdList.begin(); itBp!=imuIdList.end(); itBp++)
        for(vector<int>::iterator itId=itBp->second.begin(); itId!=itBp->second.end(); itId++)
            initDone = initDone && openImu(LocalId(itBp->first,*itId));

    return initDone;
}

bool icubWholeBodySensors::close()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS_OF(itBp, encoderIdList)
    {
        assert(dd[itBp->first]!=NULL);
        ok = ok && dd[itBp->first]->close();
        dd[itBp->first] = NULL;
    }

    FOR_ALL_BODY_PARTS_OF(itBp, pwmSensIdList)
    {
        if(dd[itBp->first]!=NULL)
        {
            ok = ok && dd[itBp->first]->close();
            dd[itBp->first] = NULL;
        }
    }
    
    FOR_ALL_BODY_PARTS_OF(itBp, torqueSensorIdList)
    {
        if(dd[itBp->first])
        {
            ok = ok && dd[itBp->first]->close();
            dd[itBp->first] = NULL;
        }
    }
    
    return ok;
}

bool icubWholeBodySensors::addSensor(const SensorType st, const LocalId &sid)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return addEncoder(sid);
    case SENSOR_PWM:            return addPwm(sid);
    case SENSOR_IMU:            return addIMU(sid);
    case SENSOR_FORCE_TORQUE:   return addFTsensor(sid);
    case SENSOR_TORQUE:         return addTorqueSensor(sid);
    default: break;
    }
    return false;
}

int icubWholeBodySensors::addSensors(const SensorType st, const LocalIdList &sids)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return addEncoders(sids);
    case SENSOR_PWM:            return addPwms(sids);
    case SENSOR_IMU:            return addIMUs(sids);
    case SENSOR_FORCE_TORQUE:   return addFTsensors(sids);
    case SENSOR_TORQUE:         return addTorqueSensors(sids);
    default: break;
    }
    return false;
}

bool icubWholeBodySensors::removeSensor(const SensorType st, const LocalId &sid)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return encoderIdList.removeId(sid);;
    case SENSOR_PWM:            return pwmSensIdList.removeId(sid);
    case SENSOR_IMU:            return imuIdList.removeId(sid);
    case SENSOR_FORCE_TORQUE:   return ftSensIdList.removeId(sid);
    case SENSOR_TORQUE:         return torqueSensorIdList.removeId(sid);
    default:break;
    }
    return false;
}

const LocalIdList& icubWholeBodySensors::getSensorList(const SensorType st)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return encoderIdList;
    case SENSOR_PWM:            return pwmSensIdList;
    case SENSOR_IMU:            return imuIdList;
    case SENSOR_FORCE_TORQUE:   return ftSensIdList;
    case SENSOR_TORQUE:         return torqueSensorIdList;
    default:break;
    }
    return emptyList;
}
        
int icubWholeBodySensors::getSensorNumber(const SensorType st)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return encoderIdList.size();
    case SENSOR_PWM:            return pwmSensIdList.size();
    case SENSOR_IMU:            return imuIdList.size();
    case SENSOR_FORCE_TORQUE:   return ftSensIdList.size();
    case SENSOR_TORQUE:         return torqueSensorIdList.size();
    default: break;
    }
    return 0;
}

bool icubWholeBodySensors::readSensor(const SensorType st, const LocalId &sid, double *data, double *stamps, bool blocking)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return readEncoder(sid, data, stamps, blocking);
    case SENSOR_PWM:            return readPwm(sid, data, stamps, blocking);
    case SENSOR_IMU:            return readIMU(sid, data, stamps, blocking);
    case SENSOR_FORCE_TORQUE:   return readFTsensor(sid, data, stamps, blocking);
    case SENSOR_TORQUE:         return readTorqueSensor(sid, data, stamps, blocking);
    default: break;
    }
    return false;
}
        
bool icubWholeBodySensors::readSensors(const SensorType st, double *data, double *stamps, bool blocking)
{
    switch(st)
    {
    case SENSOR_ENCODER:        return readEncoders(data, stamps, blocking);
    case SENSOR_PWM:            return readPwms(data, stamps, blocking);
    case SENSOR_IMU:            return readIMUs(data, stamps, blocking);
    case SENSOR_FORCE_TORQUE:   return readFTsensors(data, stamps, blocking);
    case SENSOR_TORQUE:         return readTorqueSensors(data, stamps, blocking);
    default: break;
    }
    return false;
}

/********************************************************************************************************************************************/
/**************************************************** PRIVATE METHODS ***********************************************************************/
/********************************************************************************************************************************************/

bool icubWholeBodySensors::openEncoder(const int bp)
{
    // check whether the encoder interface is already open
    if(ienc[bp]!=0) return true;
    // check whether the poly driver is already open (here I assume the elements of dd are initialized to 0)
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], bodyPartNames[bp])) return false;
    // open the encoder interface
    if(!dd[bp]->view(ienc[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", bodyPartNames[bp].c_str());
        return false;
    }
    ///< store the number of joints in this body part
    int nj=0;
    ienc[bp]->getAxes(&nj);
    bodyPartAxes[bp] = nj;
    return true;
}

bool icubWholeBodySensors::openPwm(const int bp)
{
    ///< check whether the motor PWM interface is already open
    if(iopl[bp]!=0)             return true;
    ///< check that we are not in simulation, because iCub simulator does not implement pwm control
    if(isRobotSimulator(robot)) return true;
    ///< if necessary open the poly driver
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], bodyPartNames[bp]))
        return false;

    if(!dd[bp]->view(iopl[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", bodyPartNames[bp].c_str());
        return false;
    }
    return true;
}

bool icubWholeBodySensors::openImu(const LocalId &i)
{
    imuLastRead[i].resize(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0);
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

bool icubWholeBodySensors::openFTsens(const LocalId &i)
{
    ftSensLastRead[i].resize(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0);
    if(isRobotSimulator(robot)) // icub simulator doesn't have force/torque sensors
        return true;
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

bool icubWholeBodySensors::openTorqueSensor(const int bp)
{
//    torqueSensorsLastRead[bp].resize(6,0.0);
    ///< check that we are not in simulation, because iCub simulator does not implement torque sensors
    if(isRobotSimulator(robot))
        return true;
    
    ///< check whether the joint control interface is already open
    if(itrq[bp])
        return true;
    
    ///< if necessary open the poly driver
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], bodyPartNames[bp]))
        return false;
    
    if(!dd[bp]->view(itrq[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", bodyPartNames[bp].c_str());
        return false;
    }
    return true;
}

/********************************************** ADD *******************************************************/

bool icubWholeBodySensors::addEncoder(const LocalId &j)
{
    // if initialization was done and drivers of specified body part are not open, then open them
    if(initDone && !encoderIdList.containsBodyPart(j.bodyPart))
        if(!openEncoder(j.bodyPart))
            return false;
    // if initialization was not done, drivers will be opened during initialization
    return encoderIdList.addId(j);
}

int icubWholeBodySensors::addEncoders(const LocalIdList &jList)
{
    if(initDone)
        for(LocalIdList::const_iterator it=jList.begin(); it!=jList.end(); it++)
            if(!encoderIdList.containsBodyPart(it->first))
                if(!openEncoder(it->first))
                    return 0;
    return encoderIdList.addIdList(jList);
}

bool icubWholeBodySensors::addPwm(const LocalId &j)
{
    // if initialization was done and drivers of specified body part are not open, then open them
    // if initialization was not done, drivers will be opened during initialization
    if(initDone && !pwmSensIdList.containsBodyPart(j.bodyPart))
        if(!openPwm(j.bodyPart))
            return false;
    
    return pwmSensIdList.addId(j);
}

int icubWholeBodySensors::addPwms(const LocalIdList &jList)
{
    if(initDone)
        for(LocalIdList::const_iterator it=jList.begin(); it!=jList.end(); it++)
            if(!pwmSensIdList.containsBodyPart(it->first))
                if(!openPwm(it->first))
                    return 0;
    return pwmSensIdList.addIdList(jList);
}

bool icubWholeBodySensors::addIMU(const wbi::LocalId &i)
{
    // if initialization was done, then open port of specified IMU
    // if initialization was not done, ports will be opened during initialization
    if(initDone && !imuIdList.containsId(i))
        if(!openImu(i))
            return false;
    return imuIdList.addId(i);
}

int icubWholeBodySensors::addIMUs(const wbi::LocalIdList &jList)
{
    // if initialization was done, then open port of specified IMU
    // if initialization was not done, ports will be opened during initialization
    if(initDone)
        for(LocalIdList::const_iterator itBp=jList.begin(); itBp!=jList.end(); itBp++)
            for(vector<int>::const_iterator itId=itBp->second.begin(); itId!=itBp->second.end(); itId++)
                if(!imuIdList.containsId(LocalId(itBp->first,*itId)))
                    if(!openImu(LocalId(itBp->first,*itId)))
                        return 0;
    return imuIdList.addIdList(jList);
}

bool icubWholeBodySensors::addFTsensor(const wbi::LocalId &i)
{
    // if initialization was done, then open port of specified F/T sensor
    // if initialization was not done, ports will be opened during initialization
    if(initDone && !ftSensIdList.containsId(i))
        if(!openFTsens(i))
            return false;
    return ftSensIdList.addId(i);
}

int icubWholeBodySensors::addFTsensors(const wbi::LocalIdList &jList)
{
    // if initialization was done, then open port of specified IMU
    // if initialization was not done, ports will be opened during initialization
    if(initDone)
        for(LocalIdList::const_iterator itBp=jList.begin(); itBp!=jList.end(); itBp++)
            for(vector<int>::const_iterator itId=itBp->second.begin(); itId!=itBp->second.end(); itId++)
                if(!ftSensIdList.containsId(LocalId(itBp->first,*itId)))
                    if(!openFTsens(LocalId(itBp->first,*itId)))
                        return 0;
    return ftSensIdList.addIdList(jList);
}

bool icubWholeBodySensors::addTorqueSensor(const wbi::LocalId &i)
{
    // if initialization was done, then open port of specified joint torque sensor
    // if initialization was not done, ports will be opened during initialization
    if(initDone && !torqueSensorIdList.containsBodyPart(i.bodyPart))
        if(!openTorqueSensor(i.bodyPart))
            return false;
    
    return torqueSensorIdList.addId(i);
}

int icubWholeBodySensors::addTorqueSensors(const wbi::LocalIdList &jList)
{
    // if initialization was done, then open port of specified torque sensors
    // if initialization was not done, ports will be opened during initialization
    if(initDone)
        for(LocalIdList::const_iterator it = jList.begin(); it != jList.end(); it++)
            if(!torqueSensorIdList.containsBodyPart(it->first))
                if(!openTorqueSensor(it->first))
                    return 0;
    return torqueSensorIdList.addIdList(jList);
}

/********************************************** READ *******************************************************/

bool icubWholeBodySensors::readEncoders(double *q, double *stamps, bool wait)
{
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, encoderIdList)
    {
        assert(ienc[itBp->first]!=NULL);
        // read encoders
        while( !(update=ienc[itBp->first]->getEncodersTimed(qTemp, tTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        // if read succeeded => update data
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
            {
                assert( i < qLastRead[itBp->first].size() );
                qLastRead[itBp->first][itBp->first==TORSO ? 2-i : i] = CTRL_DEG2RAD*qTemp[i];;
                qStampLastRead[itBp->first][itBp->first==TORSO ? 2-i : i]   = tTemp[i];
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

bool icubWholeBodySensors::readPwms(double *pwm, double *stamps, bool wait)
{
    ///< check that we are not in simulation, because iCub simulator does not implement pwm control
    if(isRobotSimulator(robot)) 
    {
        memset(pwm, 0, sizeof(double) * pwmSensIdList.size());
        return true;
    }

    double pwmTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, pwmSensIdList)
    {
        // read data
        while( !(update=iopl[itBp->first]->getOutputs(pwmTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        // if reading has succeeded, update last read data
        if(update)
            for(unsigned int i=0; i<bodyPartAxes[itBp->first]; i++)
                pwmLastRead[itBp->first][itBp->first==TORSO ? 2-i : i] = pwmTemp[i];    // joints of the torso are in reverse order
        
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

bool icubWholeBodySensors::readIMUs(double *inertial, double *stamps, bool wait)
{
    Vector *v;
    int i=0;    // sensor index
    for(map<LocalId,BufferedPort<Vector>*>::iterator it=portsIMU.begin(); it!=portsIMU.end(); it++)
    {
        v = it->second->read(wait);
        if(v!=NULL)
        {
            imuLastRead[it->first] = *v;
        }
        convertIMU(inertial,imuLastRead[it->first].data());
        i++;
    }
    return true;
}

bool icubWholeBodySensors::readFTsensors(double *ftSens, double *stamps, bool wait)
{
    ///< iCub simulator does not implement the force/torque sensors
    if(isRobotSimulator(robot))
    {
        memset(ftSens, 0, sizeof(double) * portsFTsens.size());
        return true;
    }

    Vector *v;
    int i=0;    // sensor index
    for(map<LocalId,BufferedPort<Vector>*>::iterator it=portsFTsens.begin(); it!=portsFTsens.end(); it++)
    {
        v = it->second->read(wait);
        if(v!=NULL)
        {
            ftSensLastRead[it->first] = *v;
        }
        memcpy(&ftSens[i*6], ftSensLastRead[it->first].data(), 6);
        i++;
    }
    return true;
}

bool icubWholeBodySensors::readTorqueSensors(double *jointSens, double *stamps, bool wait)
{
    if(isRobotSimulator(robot)) 
    {
        memset(jointSens, 0, sizeof(double) * torqueSensorIdList.size());
        return true;
    }
    
    double torqueTemp[MAX_NJ];
    bool res = true, update=false;
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, torqueSensorIdList)
    {
        // read data
        while( !(update = itrq[itBp->first]->getTorques(torqueTemp)) && wait)
            Time::delay(WAIT_TIME);
        
        // if reading has succeeded, update last read data
        if(update)
            for(unsigned int i = 0; i < bodyPartAxes[itBp->first]; i++)
                torqueSensorsLastRead[itBp->first][itBp->first == TORSO ? 2 - i : i] = torqueTemp[i];    // joints of the torso are in reverse order
        
        // copy data in output vector
        FOR_ALL_JOINTS(itBp, itJ)
        {
            jointSens[i] = torqueSensorsLastRead[itBp->first][*itJ];
            i++;
        }
        res = res && update;
    }
    return res || wait;
}

bool icubWholeBodySensors::readEncoder(const LocalId &sid, double *q, double *stamps, bool wait)
{
    double qTemp[MAX_NJ], tTemp[MAX_NJ];
    bool update=false;
    assert(ienc[sid.bodyPart]!=NULL);
    // read encoders
    while( !(update=ienc[sid.bodyPart]->getEncodersTimed(qTemp, tTemp)) && wait)
        Time::delay(WAIT_TIME);
        
    // if read succeeded => update data
    if(update)
        for(unsigned int i=0; i<bodyPartAxes[sid.bodyPart]; i++)
        {
            // joints 0 and 2 of the torso are swapped
            qLastRead[sid.bodyPart][sid.bodyPart==TORSO ? 2-i : i]        = CTRL_DEG2RAD*qTemp[i];
            qStampLastRead[sid.bodyPart][sid.bodyPart==TORSO ? 2-i : i]   = tTemp[i];
        }
        
    // copy most recent data into output variables
    q[0] = qLastRead[sid.bodyPart][sid.index];
    if(stamps!=NULL)
        stamps[0] = qStampLastRead[sid.bodyPart][sid.index];

    return update || wait;  // if read failed => return false
}

bool icubWholeBodySensors::readPwm(const LocalId &sid, double *pwm, double *stamps, bool wait)
{
    if(isRobotSimulator(robot))
    {
        pwm[0] = 0.0;   // iCub simulator does not have pwm sensors
        return true;    // does not return false, so programs can be tested in simulation
    }
    double pwmTemp[MAX_NJ];
    bool update=false;
    assert(iopl[sid.bodyPart]!=NULL);
    // read motor PWM
    while( !(update=iopl[sid.bodyPart]->getOutputs(pwmTemp)) && wait)
        Time::delay(WAIT_TIME);
    
    // if read succeeded => update data
    if(update) // joints 0 and 2 of the torso are swapped
        pwmLastRead[sid.bodyPart][sid.bodyPart==TORSO ? 2-sid.index : sid.index] = pwmTemp[sid.index];
    
    // copy most recent data into output variables
    pwm[0] = pwmLastRead[sid.bodyPart][sid.index];
    
    return update || wait;  // if read failed => return false
}

bool icubWholeBodySensors::convertIMU(double * wbi_imu_readings, const double * yarp_imu_readings)
{
    //wbi orientation is expressed in axis-angle, yarp orientation in euler angles (roll pitch yaw)
    //wbi  : orientation(4) - linear acceleration (3) - angular velocity    (3) - magnetometer (3)
    //yarp : orientation(3) - linear acceleration (3) - angular velocity    (3) - magnetometer (3) 
    Rotation imu_orientation = Rotation::eulerZYX(yarp_imu_readings[0],yarp_imu_readings[1],yarp_imu_readings[2]);
    imu_orientation.getAxisAngle(wbi_imu_readings+0);
    memcpy(wbi_imu_readings+4,yarp_imu_readings+3,9*sizeof(double));
    return true;
}

bool icubWholeBodySensors::readIMU(const LocalId &sid, double *inertial, double *stamps, bool wait)
{
    Vector *v = portsIMU[sid]->read(wait);
    if(v!=NULL)
        imuLastRead[sid] = *v;
    
    convertIMU(inertial,imuLastRead[sid].data());
    
    return true;
}

bool icubWholeBodySensors::readFTsensor(const LocalId &sid, double *ftSens, double *stamps, bool wait)
{
    if(isRobotSimulator(robot))    // icub simulator doesn't have force/torque sensors
    {
        ftSens[0] = 0.0;
        return true;
    }

    Vector *v = portsFTsens[sid]->read(wait);
    if(v!=NULL)
        ftSensLastRead[sid] = *v;
    memcpy(&ftSens[0], ftSensLastRead[sid].data(), 6);

    return true;
}

bool icubWholeBodySensors::readTorqueSensor(const LocalId &sid, double *jointTorque, double *stamps, bool wait)
{
    if(isRobotSimulator(robot))
    {
        jointTorque[0] = 0.0;   // iCub simulator does not have joint torque sensors
        return true;            // does not return false, so programs can be tested in simulation
    }
    double torqueTemp;
    bool update=false;
    assert(itrq[sid.bodyPart]!=0);

    // read joint torque
    int jointIndex = sid.bodyPart==TORSO ? 2-sid.index : sid.index;
    while(!(update = itrq[sid.bodyPart]->getTorque(jointIndex, &torqueTemp)) && wait)
        Time::delay(WAIT_TIME);
    
    // if read succeeded => update data
    if(update) // joints 0 and 2 of the torso are swapped
        torqueSensorsLastRead[sid.bodyPart][jointIndex] = torqueTemp;
    
    // copy most recent data into output variables
    jointTorque[0] = torqueSensorsLastRead[sid.bodyPart][jointIndex];
    
    return update || wait;  // if read failed => return false
}
