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

#include <yarp/os/LogStream.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "yarpWholeBodyInterface/yarpWholeBodyInterface.h"

#include "wholeBodyDynamicsTree/wholeBodyDynamicsThread.h"
#include "wholeBodyDynamicsTree/wholeBodyDynamicsModule.h"

using namespace yarp::dev;
using namespace yarpWbi;
using namespace yarp::os;
using namespace wbi;

wholeBodyDynamicsModule::wholeBodyDynamicsModule()
{
    wbdThread      = 0;
    sensors  = 0;
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

    //Loading thread period
    if( rf.check("rate") && rf.find("rate").isInt() )
    {
        period = rf.find("rate").asInt();
    }


    bool fixed_base = false;
    bool fixed_base_calibration = false;
    std::string fixed_link;
    std::string fixed_link_calibration;
    if( rf.check("assume_fixed") )
    {
        fixed_link = rf.find("assume_fixed").asString().c_str();
        if( fixed_link != "root_link" &&
            fixed_link != "l_sole" &&
            fixed_link != "r_sole" &&
            fixed_link != "r_foot_dh_frame" &&
            fixed_link != "l_foot_dh_frame" )
        {
            yError() << "assume_fixed option found, but disabled because " << fixed_link << " is not a recognized fixed_link ";
            return false;
        } else {
            yInfo() << "assume_fixed option found, using " << fixed_link << " as fixed link as a kinematic root instead of the imu.";
            fixed_base = true;
            fixed_base_calibration = true;
            fixed_link_calibration = fixed_link;
            // \todo TODO workaround for heidelberg
            if( fixed_link == "l_sole" )
            {
                fixed_link = fixed_link_calibration = "l_foot_dh_frame";
            }

            if( fixed_link == "r_sole" )
            {
                fixed_link = fixed_link_calibration = "r_foot_dh_frame";
            }
        }
    }
    else
    {
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
    yarp::os::Property yarpWbiOptions;
    //Get wbi options from the canonical file
    if( !rf.check("wbi_conf_file") )
    {
        fprintf(stderr,"[ERR] wholeBodyDynamicsThread: impossible to open wholeBodyInterface: wbi_conf_file option missing");
        return false;
    }
    std::string wbiConfFile = rf.findFile("wbi_conf_file");
    yarpWbiOptions.fromConfigFile(wbiConfFile);

    //Overwrite the parameters in the wbi_conf_file with stuff from the wbd options file
    yarpWbiOptions.fromString(rf.toString(),false);

    //List of joints used in the dynamic model of the robot
    IDList RobotDynamicModelJoints;
    std::string RobotDynamicModelJointsListName = rf.check("torque_estimation_joint_list",
                                                           yarp::os::Value("ROBOT_DYNAMIC_MODEL_JOINTS"),
                                                           "Name of the list of joint used for torque estimation").asString().c_str();

    if( !loadIdListFromConfig(RobotDynamicModelJointsListName,rf,RobotDynamicModelJoints) )
    {
        if( !loadIdListFromConfig(RobotDynamicModelJointsListName,yarpWbiOptions,RobotDynamicModelJoints) )
        {
            fprintf(stderr, "[ERR] wholeBodyDynamicsModule: impossible to load wbiId joint list with name %s\n",RobotDynamicModelJointsListName.c_str());
            return false;
        }
    }

    //Add to the options some wbd specific stuff
    if( fixed_base )
    {
        yarpWbiOptions.put("fixed_base",fixed_link);
    }

    if( rf.check("IDYNTREE_SKINDYNLIB_LINKS") )
    {
        yarp::os::Property & prop = yarpWbiOptions.addGroup("IDYNTREE_SKINDYNLIB_LINKS");
        prop.fromString(rf.findGroup("IDYNTREE_SKINDYNLIB_LINKS").tail().toString());
    }
    else
    {
        fprintf(stderr, "[ERR] wholeBodyDynamicsModule: impossible to load IDYNTREE_SKINDYNLIB_LINKS group, exiting");

    }

    if( rf.check("WBD_SUBTREES") )
    {
        yarp::os::Property & prop = yarpWbiOptions.addGroup("WBD_SUBTREES");
        prop.fromString(rf.findGroup("WBD_SUBTREES").tail().toString());
    }
    else
    {
        fprintf(stderr, "[ERR] wholeBodyDynamicsModule: impossible to load WBD_SUBTREES group, exiting");
    }

    if( rf.check("WBD_OUTPUT_TORQUE_PORTS") )
    {
        yarp::os::Property & prop = yarpWbiOptions.addGroup("WBD_OUTPUT_TORQUE_PORTS");
        prop.fromString(rf.findGroup("WBD_OUTPUT_TORQUE_PORTS").tail().toString());
    }
    else
    {
        fprintf(stderr, "[ERR] wholeBodyDynamicsModule: impossible to load WBD_OUTPUT_TORQUE_PORTS group, exiting");
    }

    if( rf.check("WBD_OUTPUT_EXTERNAL_WRENCH_PORTS") )
    {
        yarp::os::Property & prop = yarpWbiOptions.addGroup("WBD_OUTPUT_EXTERNAL_WRENCH_PORTS");
        prop.fromString(rf.findGroup("WBD_OUTPUT_EXTERNAL_WRENCH_PORTS").tail().toString());
    }

    //Loading thread period
    if( rf.check("rate") && rf.find("rate").isInt() )
    {
        period = rf.find("rate").asInt();
    }

    if( rf.check("calibration_support_link") )
    {
        yarpWbiOptions.put("calibration_support_link",rf.find("calibration_support_link").asString());
    }
    else
    {
        yarpWbiOptions.put("calibration_support_link","root_link");
    }

    sensors = new yarpWholeBodySensors(moduleName.c_str(), yarpWbiOptions);

    sensors->addSensors(wbi::SENSOR_ENCODER,RobotDynamicModelJoints);
    //estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_VEL,RobotDynamicModelJoints);
    //estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_ACC,RobotDynamicModelJoints);

     //List of 6-axis Force-Torque sensors in the robot
    IDList RobotFTSensors;
    std::string RobotFTSensorsListName = "ROBOT_MAIN_FTS";
    if( !loadIdListFromConfig(RobotFTSensorsListName,yarpWbiOptions,RobotFTSensors) )
    {
        yError("wholeBodyDynamicsTree: impossible to load wbiId list with name %s\n",RobotFTSensorsListName.c_str());
    }
    sensors->addSensors(wbi::SENSOR_FORCE_TORQUE,RobotFTSensors);

    //List of IMUs sensors in the robot
    IDList RobotIMUSensors;
    std::string RobotIMUSensorsListName = "ROBOT_MAIN_IMUS";
    if( !loadIdListFromConfig(RobotIMUSensorsListName,yarpWbiOptions,RobotIMUSensors) )
    {
        yError("wholeBodyDynamicsTree: impossible to load wbiId list with name %s\n",RobotFTSensorsListName.c_str());
    }
    sensors->addSensors(wbi::SENSOR_IMU,RobotIMUSensors);

    if(!sensors->init())
    {
        yError() << getName() << ": Error while initializing whole body estimator interface.Closing module";
        return false;
    }

    //--------------------------WHOLE BODY DYNAMICS THREAD--------------------------
    wbdThread = new wholeBodyDynamicsThread(moduleName,
                                            robotName,
                                            period,
                                            sensors,
                                            yarpWbiOptions,
                                            fixed_base_calibration,
                                            fixed_link_calibration);
    if(!wbdThread->start())
    {
        yError() << getName()
                          << ": Error while initializing whole body estimator thread."
                          << "Closing module";
        return false;
    }

    yInfo() << "wholeBodyDynamicsThread started";


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
    // Get for the last time time stats
    wbdThread->getEstPeriod(avgTime, stdDev);
    wbdThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    //stop threads
    if(wbdThread)
    {
        yInfo() << getName() << ": closing wholeBodyDynamicsThread";
        wbdThread->stop();
        delete wbdThread;
        wbdThread = 0;
    }
    if(sensors)
    {
        yInfo() << getName() << ": closing wholeBodySensors interface";
        bool res=sensors->close();
        if(!res)
            yError("Error while closing robot sensors interface\n");
        delete sensors;
        sensors = 0;
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
        yError("wholeBodyDynamicsThread pointers are zero\n");
        return false;
    }

    wbdThread->getEstPeriod(avgTime, stdDev);
    wbdThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    if(avgTime > 1.3 * period)
    {
        yWarning("[WARNING] wholeBodyDynamics loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
        yInfo("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }


    return true;
}

////////////////// RPC METHODS /////////////////////////////////////////
bool wholeBodyDynamicsModule::calib(const std::string& calib_code, const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        yInfo() << getName() << ": calibration for " << calib_code << "requested";
        wbdThread->calibrateOffset(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        yInfo() << getName() << ": calib failed, no wholeBodyDynamicsThread available";
        return false;
    }
}

////////////////// RPC METHODS /////////////////////////////////////////
bool wholeBodyDynamicsModule::calibStanding(const std::string& calib_code,
                                                   const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        yInfo() << getName() << ": double support calibration for " << calib_code << "requested";
        wbdThread->calibrateOffsetOnDoubleSupport(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        yError() << getName() << ": double support calib failed, no wholeBodyDynamicsThread available";
        return false;
    }
}

bool wholeBodyDynamicsModule::calibStandingLeftFoot(const std::string& calib_code,
                                                   const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        yInfo() << getName() << ": single support left foot calibration for " << calib_code << "requested";
        wbdThread->calibrateOffsetOnLeftFootSingleSupport(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        yError() << getName() << ": calib failed, no wholeBodyDynamicsThread available";
        return false;
    }
}

bool wholeBodyDynamicsModule::calibStandingRightFoot(const std::string& calib_code,
                                                   const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        yInfo() << getName() << ": single support right foot calibration for " << calib_code << "requested";
        wbdThread->calibrateOffsetOnRightFootSingleSupport(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        yDebug() << getName() << ": calib failed, no wholeBodyDynamicsThread available";
        return false;
    }
}

bool wholeBodyDynamicsModule::resetOffset(const std::string& calib_code)
{
    if(wbdThread) {
        yInfo() << getName() << ": offset reset for " << calib_code << "requested";
        wbdThread->resetOffset(calib_code);
        return true;
    } else {
        yError() << getName() << ": calib failed, no wholeBodyDynamicsThread available";
        return false;
    }
}

bool wholeBodyDynamicsModule::resetSimpleLeggedOdometry(const std::string& initial_world_frame,
                                                        const std::string& initial_fixed_link)
{
    if(wbdThread) {
        return wbdThread->resetSimpleLeggedOdometry(initial_world_frame,initial_fixed_link);
    } else {
        return false;
    }
}


bool wholeBodyDynamicsModule::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link)
{
    if(wbdThread) {
        return wbdThread->changeFixedLinkSimpleLeggedOdometry(new_fixed_link);
    } else {
        return false;
    }
}





bool wholeBodyDynamicsModule::quit()
{
    return this->close();
}


