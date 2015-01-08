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

#include <yarp/os/Log.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "yarpWholeBodyInterface/yarpWholeBodyInterface.h"

#include <wholeBodyDynamicsTree/wholeBodyDynamicsThread.h>
#include <wholeBodyDynamicsTree/wholeBodyDynamicsModule.h>

using namespace yarp::dev;
using namespace yarpWbi;
using namespace yarp::os;
using namespace wbi;

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

    bool fixed_base = false;
    std::string fixed_link;
    if( rf.check("assume_fixed") )
    {
        fixed_link = rf.find("assume_fixed").asString().c_str();
        if( fixed_link != "root_link" &&
            fixed_link != "l_sole" &&
            fixed_link != "r_sole" )
        {
            std::cout << "assume_fixed option found, but disabled because " << fixed_link << " is not a recognized fixed_link " << std::endl;
            return false;
        } else {
            std::cout << "assume_fixed option found, using " << fixed_link << " as fixed link as a kinematic root instead of the imu." << std::endl;
            fixed_base = true;
        }
    }

    bool fixed_base_calibration = false;
    std::string fixed_link_calibration;
    if( rf.check("assume_fixed_base_calibration") )
    {
        fixed_link_calibration = rf.find("assume_fixed_base_calibration").asString().c_str();
        if( fixed_link_calibration != "root_link" &&
            fixed_link_calibration != "l_sole" &&
            fixed_link_calibration != "r_sole" )
        {
            std::cout << "assume_fixed_base_calibration option found, but disabled because " << fixed_link_calibration << " is not a recognized fixed_link " << std::endl;
            return false;
        } else {
            std::cout << "assume_fixed_base_calibration option found, using " << fixed_link_calibration << " as fixed link as a kinematic root instead of the imu for calibration." << std::endl;
            fixed_base_calibration = true;
        }
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

    //Overwrite the robot parameter that could be present in wbi_conf_file
    yarpWbiOptions.put("robot",rf.find("robot").asString());

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



    estimationInterface = new yarpWholeBodyStatesLocal(moduleName.c_str(), yarpWbiOptions);

    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_POS,RobotDynamicModelJoints);
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_VEL,RobotDynamicModelJoints);
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_ACC,RobotDynamicModelJoints);

     //List of 6-axis Force-Torque sensors in the robot
    IDList RobotFTSensors;
    std::string RobotFTSensorsListName = "ROBOT_MAIN_FTS";
    if( !loadIdListFromConfig(RobotFTSensorsListName,yarpWbiOptions,RobotFTSensors) )
    {
        fprintf(stderr, "[ERR] wholeBodyDynamicsTree: impossible to load wbiId list with name %s\n",RobotFTSensorsListName.c_str());
    }
    estimationInterface->addEstimates(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,RobotFTSensors);

    //List of IMUs sensors in the robot
    IDList RobotIMUSensors;
    std::string RobotIMUSensorsListName = "ROBOT_MAIN_IMUS";
    if( !loadIdListFromConfig(RobotIMUSensorsListName,yarpWbiOptions,RobotIMUSensors) )
    {
        fprintf(stderr, "[ERR] wholeBodyDynamicsTree: impossible to load wbiId list with name %s\n",RobotFTSensorsListName.c_str());
    }
    estimationInterface->addEstimates(wbi::ESTIMATE_IMU,RobotIMUSensors);

    //Add torque estimation
    estimationInterface->addEstimates(wbi::ESTIMATE_JOINT_TORQUE, RobotDynamicModelJoints);

    if(!estimationInterface->init()){ std::cerr << getName() << ": Error while initializing whole body estimator interface. Closing module" << std::endl; return false; }

    bool use_ang_vel_acc = true;
    if( rf.check("enable_w0_dw0") )
    {
        std::cout << "[INFO] enable_w0_dw0 option found, enabling the use of IMU angular velocity/acceleration." << std::endl;
        use_ang_vel_acc = true;
        estimationInterface->setEstimationParameter(wbi::ESTIMATE_JOINT_TORQUE,wbi::ESTIMATION_PARAM_ENABLE_OMEGA_IMU_DOMEGA_IMU,&use_ang_vel_acc);
    }

    if( rf.check("disable_w0_dw0") )
    {
        std::cout << "[INFO] disable_w0_dw0 option found, disabling the use of IMU angular velocity/acceleration." << std::endl;
        use_ang_vel_acc = false;
        estimationInterface->setEstimationParameter(wbi::ESTIMATE_JOINT_TORQUE,wbi::ESTIMATION_PARAM_ENABLE_OMEGA_IMU_DOMEGA_IMU,&use_ang_vel_acc);
    }

    if( rf.check("min_taxel") )
    {
        int taxel_threshold = rf.find("min_taxel").asInt();
        std::cout << "[INFO] min_taxel option found, ignoring skin contacts with less then "
                  << taxel_threshold << " active taxels will be ignored." << std::endl;
        use_ang_vel_acc = false;
        estimationInterface->setEstimationParameter(wbi::ESTIMATE_JOINT_TORQUE,
                                                    wbi::ESTIMATION_PARAM_MIN_TAXEL,
                                                    &taxel_threshold);
    }
    else
    {
        int taxel_threshold = 0;
        estimationInterface->setEstimationParameter(wbi::ESTIMATE_JOINT_TORQUE,
                                                    wbi::ESTIMATION_PARAM_MIN_TAXEL,
                                                    &taxel_threshold);
    }


    bool autoconnect = false;
    if( rf.check("autoconnect") )
    {
        std::cout << "[INFO] autoconnect option found, enabling the autoconnection." << std::endl;
        autoconnect = true;
    }

    bool zmp_test_mode = false;
    std::string zmp_test_feet = "";
    if( rf.check("zmp_test_left") )
    {
        std::cout << "[INFO] zmp_test_left option found, enabling testing output of debug quantities related to left leg" << std::endl;
        zmp_test_mode = true;
        zmp_test_feet = "left";
    }

    if( rf.check("zmp_test_right") )
    {
        std::cout << "[INFO] zmp_test_right option found, enabling testing output of debug quantities related to right leg" << std::endl;
        zmp_test_mode = true;
        zmp_test_feet = "right";
    }

    bool output_clean_ft = false;
    if( rf.check("output_clean_ft") )
    {
        std::cout << "[INFO] output_clean_ft option found, enabling output of filtered and without offset ft sensors" << std::endl;
        output_clean_ft = true;
    }

    //--------------------------WHOLE BODY DYNAMICS THREAD--------------------------
    wbdThread = new wholeBodyDynamicsThread(moduleName,
                                            robotName,
                                            period,
                                            estimationInterface,
                                            yarpWbiOptions,
                                            autoconnect,
                                            fixed_base_calibration,
                                            fixed_link_calibration,
                                            zmp_test_mode,
                                            zmp_test_feet,
                                            output_clean_ft
                                           );
    if(!wbdThread->start()){ std::cerr << "[ERR]" << getName() << ": Error while initializing whole body estimator interface. Closing module" << std::endl;; return false; }

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
bool wholeBodyDynamicsModule::calib(const std::string& calib_code, const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        std::cout << getName() << ": calibration for " << calib_code << "requested" << std::endl;
        wbdThread->calibrateOffset(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        std::cout << getName() << ": calib failed, no wholeBodyDynamicsThread available" << std::endl;
        return false;
    }
}

////////////////// RPC METHODS /////////////////////////////////////////
bool wholeBodyDynamicsModule::calibStanding(const std::string& calib_code,
                                                   const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        std::cout << getName() << ": double support calibration for " << calib_code << "requested" << std::endl;
        wbdThread->calibrateOffsetOnDoubleSupport(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        std::cout << getName() << ": double support calib failed, no wholeBodyDynamicsThread available" << std::endl;
        return false;
    }
}

bool wholeBodyDynamicsModule::calibStandingLeftFoot(const std::string& calib_code,
                                                   const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        std::cout << getName() << ": single support left foot calibration for " << calib_code << "requested" << std::endl;
        wbdThread->calibrateOffsetOnLeftFootSingleSupport(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        std::cout << getName() << ": calib failed, no wholeBodyDynamicsThread available" << std::endl;
        return false;
    }
}

bool wholeBodyDynamicsModule::calibStandingRightFoot(const std::string& calib_code,
                                                   const int32_t nr_of_samples)
{
    if(wbdThread)
    {
        std::cout << getName() << ": single support right foot calibration for " << calib_code << "requested" << std::endl;
        wbdThread->calibrateOffsetOnRightFootSingleSupport(calib_code,nr_of_samples);
        wbdThread->waitCalibrationDone();
        return true;
    }
    else
    {
        std::cout << getName() << ": calib failed, no wholeBodyDynamicsThread available" << std::endl;
        return false;
    }
}

bool wholeBodyDynamicsModule::resetOffset(const std::string& calib_code)
{
    if(wbdThread) {
        std::cout << getName() << ": offset reset for " << calib_code << "requested" << std::endl;
        wbdThread->resetOffset(calib_code);
        return true;
    } else {
        std::cout << getName() << ": calib failed, no wholeBodyDynamicsThread available" << std::endl;
        return false;
    }
}


bool wholeBodyDynamicsModule::quit()
{
    return this->close();
}


