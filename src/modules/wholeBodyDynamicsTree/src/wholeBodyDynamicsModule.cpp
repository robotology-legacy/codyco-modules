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
    estimationInterface  = 0;
    period          = 10;
}

bool wholeBodyDynamicsModule::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}
    
bool wholeBodyDynamicsModule::configure(ResourceFinder &rf)
{
    if( rf.check("robot") ) {
        robotName = rf.find("robot").asString();
    } else {
        std::cerr << "wholeBodyDynamicsModule::configure failed: robot parameter not found. Closing module." << std::endl;
        return false;
    }
    
    if( rf.check("name") ) {
        moduleName = rf.find("name").asString();
        setName(moduleName.c_str());
    } else {
        std::cerr << "wholeBodyDynamicsModule::configure failed: name parameter not found. Closing module." << std::endl;
        return false;
    }
    
    //Checking iCub parts version 
    /// \todo this part should be replaced by a more general way of accessing robot parameters
    ///       namely urdf for structure parameters and robotInterface xml (or runtime interface) to get available sensors
    iCub::iDynTree::iCubTree_version_tag icub_version;
    icub_version.head_version = 2;
    if( rf.check("headV1") ) {
        icub_version.head_version = 1;
    }
    if( rf.check("headV2") ) {
        icub_version.head_version = 2;
    }
    
    icub_version.legs_version = 2;
    if( rf.check("legsV1") ) {
        icub_version.legs_version = 1;
    }
    if( rf.check("legsV2") ) {
        icub_version.legs_version = 2;
    }
    
    /// \note if feet_version are 2, the presence of FT sensors in the feet is assumed
    icub_version.feet_ft = false;
    if( rf.check("feetV1") ) {
        icub_version.feet_ft = false;
    }
    if( rf.check("feetV2") ) {
        icub_version.feet_ft = true;
    }
    
    //--------------------------RPC PORT--------------------------
    attach(rpcPort);
    std::string rpcPortName= "/";
    rpcPortName+= getName();
    rpcPortName += "/rpc:i";
    if (!rpcPort.open(rpcPortName.c_str())) {           
        std::cerr << getName() << ": Unable to open port " << rpcPortName << std::endl;  
        return false;
    }

    //--------------------------WHOLE BODY STATES INTERFACE--------------------------
    estimationInterface = new icubWholeBodyStatesLocal(moduleName.c_str(), robotName.c_str(), icub_version);
    
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_POS,wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_VEL,wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_ACC,wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);
    if( icub_version.feet_ft ) {
        int added_ft_sensors = estimationInterface->addEstimates(wbi::ESTIMATE_FORCE_TORQUE,wbiIcub::ICUB_MAIN_FOOT_FTS);
        if( added_ft_sensors != wbiIcub::ICUB_MAIN_FOOT_FTS.size() ) {
            std::cout << "Error in adding F/T estimates" << std::endl;
            return false;
        }
    } else {
        int added_ft_sensors = estimationInterface->addEstimates(wbi::ESTIMATE_FORCE_TORQUE,wbiIcub::ICUB_MAIN_FTS);
        if( added_ft_sensors != wbiIcub::ICUB_MAIN_FTS.size() ) {
            std::cout << "Error in adding F/T estimates" << std::endl;
            return false;
        }
    }
    estimationInterface->addEstimates(wbi::ESTIMATE_IMU,wbiIcub::ICUB_MAIN_IMUS);
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_TORQUE, wbiIcub::ICUB_MAIN_DYNAMIC_JOINTS);

    if(!estimationInterface->init()){ std::cerr << getName() << ": Error while initializing whole body estimator interface. Closing module" << std::endl; return false; }

    //--------------------------WHOLE BODY DYNAMICS THREAD--------------------------
    wbdThread = new wholeBodyDynamicsThread(moduleName, robotName, period, estimationInterface, icub_version);
    if(!wbdThread->start()){ std::cerr << getName() << ": Error while initializing whole body estimator interface. Closing module" << std::endl;; return false; }
    
    fprintf(stderr,"wholeBodyDynamicsThread started\n");


    return true;
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
    if(wbdThread)
    {
        std::cout << getName() << ": closing wholeBodyDynamicsThread" << std::endl;
        wbdThread->stop();
        delete wbdThread;
        wbdThread = 0;     
    }
    if(estimationInterface)
    { 
        std::cout << getName() << ": closing wholeBodyStateLocal interface" << std::endl;
        bool res=estimationInterface->close();    
        if(!res)
            printf("Error while closing robot estimator\n");
        delete estimationInterface;  
        estimationInterface = 0; 
    }

    //closing ports
    std::cout << getName() << ": closing RPC port interface" << std::endl;
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

////////////////// RPC METHODS /////////////////////////////////////////
bool wholeBodyDynamicsModule::calib(const std::string& calib_code)
{
    if(wbdThread) {
        std::cout << getName() << ": calibration for " << calib_code << "requested" << std::endl;
        wbdThread->calibrateOffset(calib_code);
        wbdThread->waitCalibrationDone();
        return true;
    } else {
        std::cout << getName() << ": calib failed, no wholeBodyDynamicsThread available" << std::endl;
        return false;
    }
}


bool wholeBodyDynamicsModule::quit()
{
    return close();
}


