/* 
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "wbiIcub/wholeBodyInterfaceIcub.h"

#include <wholeBodyDynamicsTree/wholeBodyDynamicsThread.h>
#include <wholeBodyDynamicsTree/wholeBodyDynamicsModule.h>

using namespace yarp::dev;
using namespace wbiIcub;

wholeBodyDynamicsModule::wholeBodyDynamicsModule()
{
    wbdThread      = 0;
    robotInterface  = 0;
    period          = 10;
}
    
bool wholeBodyDynamicsModule::configure(ResourceFinder &rf)
{

    //--------------------------WHOLE BODY STATES INTERFACE--------------------------
    robotInterface = new icubWholeBodyStatesLocal(moduleName.c_str(), robotName.c_str());
    robotInterface->addEstimates(wbi::ESTIMATE_JOINT_POS,wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
    robotInterface->addEstimates(wbi::ESTIMATE_IMU,wbiIcub::ICUB_MAIN_IMUS);
    robotInterface->addEstimates(wbi::ESTIMATE_FORCE_TORQUE, wbiIcub::ICUB_MAIN_FOOT_FTS);

    if(!robotInterface->init()){ fprintf(stderr, "Error while initializing whole body state interface. Closing module\n"); return false; }

    //--------------------------WHOLE BODY DYNAMICS THREAD--------------------------
    wbdThread = new wholeBodyDynamicsThread(moduleName, robotName, period, robotInterface);
    if(!wbdThread->start()){ fprintf(stderr, "Error while initializing wholeBodyDynamics thread. Closing module.\n"); return false; }
    
    fprintf(stderr,"wholeBodyDynamicsThread started\n");


    return true;
}

bool wholeBodyDynamicsModule::respond(const Bottle& cmd, Bottle& reply) 
{
}


bool wholeBodyDynamicsModule::interruptModule()
{
    if(wbdThread)
        wbdThread->suspend();
    rpcPort.interrupt();
    return true;
}

bool wholeBodyDynamicsModule::close()
{
//stop threads
    if(wbdThread){     wbdThread->stop();         delete wbdThread;      wbdThread = 0;     }
    if(robotInterface)
    { 
        bool res=robotInterface->close();    
        if(res)
            printf("Error while closing robot estimator\n");
        delete robotInterface;  
        robotInterface = 0; 
    }

    //closing ports
    rpcPort.close();

    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*period)
        printf("Next time you could set a lower period to improve the wholeBodyDynamics performance.\n");
    else if(avgTime>1.3*period)
        printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

    return true;
}

bool wholeBodyDynamicsModule::updateModule()
{
    if (wbdThread==0)
    {
        printf("wholeBodyDynamicsThread pointers are zero\n");
        return false;
    }

    wbdThread->getEstPeriod(avgTime, stdDev);
    wbdThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
//#ifndef NDEBUG
    if(avgTime > 1.3 * period)
    {
        printf("[WARNING] wholeBodyDynamics loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
        printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }
//#endif

    return true;
}