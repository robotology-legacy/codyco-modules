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
#include <string>
#include <cassert>

using namespace std;
using namespace wbi;
using namespace wbiy;
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
icubWholeBodyInterface::icubWholeBodyInterface(const char* _name, const char* _robotName)
{
    vector<string> bodyPartNames(BodyPart_s, BodyPart_s + sizeof(BodyPart_s) / sizeof(string) );
    actuatorInt = new yarpWholeBodyActuators((_name+string("sens")).c_str(), _robotName, bodyPartNames);
    stateInt = new icubWholeBodyStates((_name+string("state")).c_str(), _robotName, 0.0);
    modelInt = new icubWholeBodyModel();
}

bool icubWholeBodyInterface::init()
{
    bool ok = actuatorInt->init();
    if(ok) ok = stateInt->init();
    return ok ? modelInt->init() : false;
}

int icubWholeBodyInterface::getDoFs()
{
    return actuatorInt->getDoFs();
}

bool icubWholeBodyInterface::removeJoint(const LocalId &j)
{
    bool ok = actuatorInt->removeJoint(j);
    if(ok) stateInt->removeJoint(j);
    return ok ? modelInt->removeJoint(j) : false;
}

bool icubWholeBodyInterface::addJoint(const LocalId &j)
{
    bool ok = actuatorInt->addJoint(j);
    if(ok) stateInt->addJoint(j);
    return ok ? modelInt->addJoint(j) : false;
}

int icubWholeBodyInterface::addJoints(const LocalIdList &jList)
{
    int res1 = actuatorInt->addJoints(jList);
    int res2 = stateInt->addJoints(jList);
    int res3 = modelInt->addJoints(jList);
    assert(res1==res2);
    //assert(res2==res3);   // temporarely comment this until model interface is ready
    return res1;
}

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