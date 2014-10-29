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

#include "wholeBodyDynamicsTree/wholeBodyDynamicsThread.h"
#include <yarpWholeBodyInterface/yarpWholeBodyStatesLocal.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/SVD.h>

#include <string.h>
#include <ctime>


using namespace yarp::math;
using namespace yarp::sig;
using namespace yarpWbi;
using namespace std;

iCubTreeStatus::iCubTreeStatus(int nrOfDOFs, int nrOfFTsensors)
{
    setNrOfDOFs(nrOfDOFs);
    setNrOfFTSensors(nrOfFTsensors);
}

bool iCubTreeStatus::zero()
{
    domega_imu.zero();
    omega_imu.zero();
    proper_ddp_imu.zero();
    wbi_imu.zero();
    q.zero();
    dq.zero();
    ddq.zero();
    for(unsigned int i=0; i < estimated_ft_sensors.size(); i++ ) {
        estimated_ft_sensors[i].zero();
        measured_ft_sensors[i].zero();
    }
    return true;
}

bool iCubTreeStatus::setNrOfDOFs(int nrOfDOFs)
{
    domega_imu.resize(3);
    omega_imu.resize(3);
    proper_ddp_imu.resize(3);
    wbi_imu.resize(wbi::sensorTypeDescriptions[wbi::SENSOR_IMU].dataSize);
    q.resize(nrOfDOFs);
    dq.resize(nrOfDOFs);
    ddq.resize(nrOfDOFs);

    zero();
    return true;
}

bool iCubTreeStatus::setNrOfFTSensors(int nrOfFTsensors)
{
    estimated_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    measured_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    zero();
    return true;
}

//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name,
                                                 string _robotName,
                                                 int _period,
                                                 yarpWholeBodyStatesLocal *_wbs,
                                                 yarp::os::Property & _yarp_wbi_opts,
                                                 bool _autoconnect,
                                                 bool _assume_fixed_base_calibration,
                                                 std::string _fixed_link_calibration,
                                                 bool _zmp_test_mode,
                                                 std::string _zmp_test_feet
                                                )
    :  RateThread(_period),
       name(_name),
       robotName(_robotName),
       estimator(_wbs),
       yarp_options(_yarp_wbi_opts),
       printCountdown(0),
       PRINT_PERIOD(2000),
       samples_requested_for_calibration(200),
       max_samples_for_calibration(2000),
       samples_used_for_calibration(0),
       assume_fixed_base_calibration(_assume_fixed_base_calibration),
       autoconnect(_autoconnect),
       fixed_link_calibration(_fixed_link_calibration),
       zmp_test_mode(_zmp_test_mode)
    {

       std::cout << "Launching wholeBodyDynamicsThread with name : " << _name << " and robotName " << _robotName << " and period " << _period << std::endl;

       if( !_yarp_wbi_opts.check("urdf_file") )
       {
            std::cerr << "[ERR] yarpWholeBodyStatesLocal error: urdf_file not found in configuration files" << std::endl;
            return;
       }

    std::string urdf_file = _yarp_wbi_opts.find("urdf_file").asString().c_str();
    yarp::os::ResourceFinder rf;
    std::string urdf_file_path = rf.findFile(urdf_file.c_str());

       if( assume_fixed_base_calibration ) {
           icub_model_calibration = new iCub::iDynTree::iCubTree(urdf_file_path,fixed_link_calibration);
       } else {
           icub_model_calibration = new iCub::iDynTree::iCubTree(urdf_file_path);
       }



    //Resize buffer vectors
    all_torques.resize(_wbs->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE));

    LAExternalWrench.resize(6);
    RAExternalWrench.resize(6);
    LLExternalWrench.resize(6);
    RLExternalWrench.resize(6);

    LACartesianExternalWrench.resize(6);
    RACartesianExternalWrench.resize(6);
    LLCartesianExternalWrench.resize(6);
    RLCartesianExternalWrench.resize(6);

    iCubGuiBase.resize(6);

    //Copied from old wholeBodyDynamics
    std::string robot_name = robotName;
    std::string local_name = name;

    port_external_wrench_RA = new BufferedPort<Vector>;
    port_external_wrench_LA = new BufferedPort<Vector>;
    port_external_wrench_RL = new BufferedPort<Vector>;
    port_external_wrench_LL = new BufferedPort<Vector>;

    port_external_cartesian_wrench_RA = new BufferedPort<Vector>;
    port_external_cartesian_wrench_LA = new BufferedPort<Vector>;
    port_external_cartesian_wrench_RL = new BufferedPort<Vector>;
    port_external_cartesian_wrench_LL = new BufferedPort<Vector>;


    port_contacts = new BufferedPort<skinContactList>;


    port_contacts->open(string("/"+local_name+"/contacts:o").c_str());


    port_external_wrench_RA->open(string("/"+local_name+"/right_arm/endEffectorWrench:o").c_str());
    port_external_wrench_LA->open(string("/"+local_name+"/left_arm/endEffectorWrench:o").c_str());
    port_external_wrench_RL->open(string("/"+local_name+"/right_leg/endEffectorWrench:o").c_str());
    port_external_wrench_LL->open(string("/"+local_name+"/left_leg/endEffectorWrench:o").c_str());
    /*
    port_external_wrench_RF->open(string("/"+local_name+"/right_foot/endEffectorWrench:o").c_str());
    port_external_wrench_LF->open(string("/"+local_name+"/left_foot/endEffectorWrench:o").c_str());
    */

    port_external_cartesian_wrench_RA->open(string("/"+local_name+"/right_arm/cartesianEndEffectorWrench:o").c_str());
    port_external_cartesian_wrench_LA->open(string("/"+local_name+"/left_arm/cartesianEndEffectorWrench:o").c_str());
    port_external_cartesian_wrench_RL->open(string("/"+local_name+"/right_leg/cartesianEndEffectorWrench:o").c_str());
    port_external_cartesian_wrench_LL->open(string("/"+local_name+"/left_leg/cartesianEndEffectorWrench:o").c_str());


    //port_all_velocities->open(string("/"+local_name+"/all_velocities:o").c_str());

    //Open port for iCubGui
    port_icubgui_base = new BufferedPort<Vector>;
    port_icubgui_base->open(string("/"+local_name+"/base:o"));

    //Open port for output joint forcetorque


    if(zmp_test_mode)
    {
        if( _zmp_test_feet == "left" )
        {
            foot_under_zmp_test = LEFT_FOOT;
            icub_model_zmp = new iCub::iDynTree::iCubTree(urdf_file_path);
            icub_model_zmp->setFloatingBaseLink(icub_model_zmp->getLinkIndex("l_sole"));
        }
        else
        {
            foot_under_zmp_test = RIGHT_FOOT;
            icub_model_zmp = new iCub::iDynTree::iCubTree(urdf_file_path);
            icub_model_zmp->setFloatingBaseLink(icub_model_zmp->getLinkIndex("r_sole"));
        }
        port_joint_ankle_cartesian_wrench = new BufferedPort<Vector>;
        port_joint_ankle_cartesian_wrench_from_model = new BufferedPort<Vector>;
        port_joint_foot_cartesian_wrench = new BufferedPort<Vector>;
        port_joint_foot_cartesian_wrench_from_model = new BufferedPort<Vector>;
        port_joint_ankle_cartesian_wrench->open("/"+local_name+"/joint_ankle_cartesian_wrench:o");
        port_joint_ankle_cartesian_wrench_from_model->open("/"+local_name+"/joint_ankle_cartesian_wrench_from_model:o");
        port_joint_foot_cartesian_wrench->open("/"+local_name+"/joint_foot_cartesian_wrench:o");
        port_joint_foot_cartesian_wrench_from_model->open("/"+local_name+"/joint_foot_cartesian_wrench_from_model:o");

    }
}

//*************************************************************************************************************************
wbi::wbiId wholeBodyDynamicsThread::convertFTiDynTreeToFTwbi(int ft_sensor_id)
{
    wbi::wbiId ret;
    estimator->getEstimateList(wbi::ESTIMATE_FORCE_TORQUE_SENSOR).numericIdToWbiId(ft_sensor_id,ret);
    return ret;
}

void checkFTSensorExist(std::string ft_sensor_name, wbi::wbiIdList & all_fts, std::vector<int> & ft_id_list, iCub::iDynTree::iCubTree * icub_model_calibration)
{
    if( all_fts.containsId(ft_sensor_name) )
    {
        int numeric_id;
        all_fts.wbiIdToNumericId(ft_sensor_name,numeric_id);
        YARP_ASSERT(icub_model_calibration->getFTSensorIndex(ft_sensor_name) == numeric_id);
        ft_id_list.push_back(numeric_id);
    }
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::threadInit()
{
    // Load output torque ports informations
    yarp::os::Bottle & output_torques_bot = yarp_options.findGroup("WBD_OUTPUT_TORQUE_PORTS");
    if( output_torques_bot.isNull() )
    {
        std::cerr << "[ERR] WBD_OUTPUT_TORQUE_PORTS group not found in wholeBodyDynamics configuration, exiting" << std::endl;
        return false;
    }

    int nr_of_output_torques_ports = output_torques_bot.size() - 1;

    wbiIdList torque_list = estimator->getEstimateList(wbi::ESTIMATE_JOINT_TORQUE);
    for(int output_torque_port = 1; output_torque_port <= output_torques_bot.size(); output_torque_port++)
    {
        outputTorquePortInformation torque_port_struct;
        yarp::os::Bottle * torque_port = output_torques_bot.get(output_torque_port).asList();
        if( torque_port == NULL || torque_port->isNull() || torque_port->size() != 3 )
        {
            std::cerr << "[ERR] malformed WBD_OUTPUT_TORQUE_PORTS group found in wholeBodyDynamics configuration, exiting" << std::endl;
            return false;
        }
        torque_port_struct.port_name = torque_port->get(0).asString();
        torque_port_struct.magic_number = torque_port->get(1).asInt();
        yarp::os::Bottle * torque_ids = torque_port->get(2).asList();
        for(int i = 0; i < torque_ids->size(); i++ )
        {
            std::string torque_wbi_id = torque_ids->get(i).asString();
            int torque_wbi_numeric_id;
            torque_list.wbiIdToNumericId(torque_wbi_id,torque_wbi_numeric_id);
            torque_port_struct.wbi_numeric_ids_to_publish.push_back(torque_wbi_numeric_id);
        }
        assert(torque_ids->size() == torque_port_struct.wbi_numeric_ids_to_publish.size() );
        torque_port_struct.output_vector.resize(torque_port_struct.wbi_numeric_ids_to_publish.size());
    }

    assert(nr_of_output_torques_ports == output_torque_ports.size());


    //Calibration variables
    int nrOfAvailableFTSensors = estimator->getEstimateNumber(wbi::ESTIMATE_FORCE_TORQUE_SENSOR);
    if( nrOfAvailableFTSensors != icub_model_calibration->getNrOfFTSensors() ) {
        std::cout << "wholeBodyDynamicsThread::threadInit() error: number of FT sensors different between model (" <<
        icub_model_calibration->getNrOfFTSensors() << ") and interface (" << nrOfAvailableFTSensors << " ) " << std::endl;
        return false;
    }

    offset_buffer.resize(nrOfAvailableFTSensors,yarp::sig::Vector(6,0.0));
    calibrate_ft_sensor.resize(nrOfAvailableFTSensors,false);
    tree_status.setNrOfDOFs(icub_model_calibration->getNrOfDOFs());
    tree_status.setNrOfFTSensors(nrOfAvailableFTSensors);

    //Get list of ft sensors for calibration shortcut
    wbi::wbiIdList ft_list = estimator->getEstimateList(wbi::ESTIMATE_FORCE_TORQUE_SENSOR);

    checkFTSensorExist("r_foot_ft_sensor",ft_list,feet_fts,icub_model_calibration);
    checkFTSensorExist("l_foot_ft_sensor",ft_list,feet_fts,icub_model_calibration);
    checkFTSensorExist("l_leg_ft_sensor",ft_list,legs_fts,icub_model_calibration);
    checkFTSensorExist("r_leg_ft_sensor",ft_list,legs_fts,icub_model_calibration);
    checkFTSensorExist("l_arm_ft_sensor",ft_list,arms_fts,icub_model_calibration);
    checkFTSensorExist("r_arm_ft_sensor",ft_list,arms_fts,icub_model_calibration);

    //Find end effector ids
    int max_id = 100;
    root_link_idyntree_id = icub_model_calibration->getLinkIndex("root_link");
    YARP_ASSERT(root_link_idyntree_id >= 0 && root_link_idyntree_id < max_id );
    left_hand_link_idyntree_id = icub_model_calibration->getLinkIndex("r_hand");
    YARP_ASSERT(left_hand_link_idyntree_id >= 0 && left_hand_link_idyntree_id < max_id );
    right_hand_link_idyntree_id = icub_model_calibration->getLinkIndex("r_hand");
    YARP_ASSERT(right_hand_link_idyntree_id >= 0 && right_hand_link_idyntree_id < max_id );
    left_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("l_foot");
    YARP_ASSERT(left_foot_link_idyntree_id >= 0  && left_foot_link_idyntree_id < max_id);
    right_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("r_foot");
    YARP_ASSERT(right_foot_link_idyntree_id >= 0 && right_foot_link_idyntree_id < max_id);

    left_gripper_frame_idyntree_id = icub_model_calibration->getLinkIndex("l_gripper");
    YARP_ASSERT(left_gripper_frame_idyntree_id >= 0 && left_gripper_frame_idyntree_id < max_id);
    right_gripper_frame_idyntree_id = icub_model_calibration->getLinkIndex("r_gripper");
    YARP_ASSERT(right_hand_link_idyntree_id >= 0 && right_gripper_frame_idyntree_id < max_id);
    left_sole_frame_idyntree_id = icub_model_calibration->getLinkIndex("l_sole");
    YARP_ASSERT(left_sole_frame_idyntree_id >= 0 && left_sole_frame_idyntree_id < max_id);
    right_sole_frame_idyntree_id = icub_model_calibration->getLinkIndex("r_sole");
    YARP_ASSERT(right_sole_frame_idyntree_id >= 0 && right_sole_frame_idyntree_id < max_id);

    KDL::CoDyCo::TreePartition icub_partition = icub_model_calibration->getKDLUndirectedTree().getPartition();

    std::cout << icub_partition.toString() << std::endl;

    left_hand_link_id = icub_partition.getLocalLinkIndex(left_hand_link_idyntree_id);
    YARP_ASSERT(left_hand_link_id == 6);
    YARP_ASSERT(left_hand_link_id >= 0);
    right_hand_link_id = icub_partition.getLocalLinkIndex(right_hand_link_idyntree_id);
    YARP_ASSERT(right_hand_link_id >= 0);
    left_foot_link_id = icub_partition.getLocalLinkIndex(left_foot_link_idyntree_id);
    right_foot_link_id = icub_partition.getLocalLinkIndex(right_foot_link_idyntree_id);

    left_gripper_frame_id = icub_partition.getLocalLinkIndex(left_gripper_frame_idyntree_id);
    right_gripper_frame_id = icub_partition.getLocalLinkIndex(right_gripper_frame_idyntree_id);
    left_sole_frame_id = icub_partition.getLocalLinkIndex(left_sole_frame_idyntree_id);
    right_sole_frame_id = icub_partition.getLocalLinkIndex(right_sole_frame_idyntree_id);

    if( zmp_test_mode )
    {
        switch(foot_under_zmp_test)
        {
            LEFT_FOOT:
                ankle_joint_idyntree_id = icub_model_calibration->getJunctionIndex("l_ankle_roll");
                foot_sole_fake_joint_idyntree_id = icub_model_calibration->getJunctionIndex("l_sole_joint");
                foot_sole_link_idyntree_id = icub_model_calibration->getLinkIndex("l_sole");
            break;
            RIGHT_FOOT:
                ankle_joint_idyntree_id = icub_model_calibration->getJunctionIndex("r_ankle_roll");
                foot_sole_fake_joint_idyntree_id = icub_model_calibration->getJunctionIndex("r_sole_joint");
                foot_sole_link_idyntree_id = icub_model_calibration->getLinkIndex("r_sole");
            break;
        }
    }

    //Open and connect all the ports
    if (autoconnect)
    {
        std::cout << "[INFO] wholeBodyDynamicsThread: autoconnect option enabled, autoconnecting." << std::endl;
        for(int output_torque_port_i = 0; output_torque_port_i < output_torque_ports.size(); output_torque_port_i++ )
        {
            std::string port_name = output_torque_ports[output_torque_port_i].port_name;
            std::string local_port = "/" + name + "/" + port_name + "/Torques:o";
            std::string robot_port = "/" + robotName  + "/joint_vsens/" + port_name + ":i";
            output_torque_ports[output_torque_port_i].output_port.open(local_port);
            if( Network::exists(robot_port) )
            {
                Network::connect(local_port,robot_port,"tcp",false);
            }
        }
     }

    //Start with calibration
    calibrateOffset("all",samples_requested_for_calibration);
    printf("\n\n");
    return true;
}


//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffset(const std::string calib_code, int samples_to_use)
{
    run_mutex.lock();
    samples_requested_for_calibration= samples_to_use;
    std::cout << "[INFO] wholeBodyDynamicsThread::calibrateOffset called with code " << calib_code << std::endl;
    if( samples_requested_for_calibration <= 0 ) {
        std::cout << "[INFO] wholeBodyDynamicsThread::calibrateOffset error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples." << std::endl;
        return false;
    }

    if( !this->decodeCalibCode(calib_code) )
    {
        return false;
    }

    //Resetting the offset for the sensor being calibrated, to get the raw values
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        if( calibrate_ft_sensor[ft_id] )
        {
            double ft_off[6];
            estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);


            std::cout << "[INFO] wholeBodyDynamicsThread::calibrateOffset: current calibration for FT " << ft_id << " is " <<
                         ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5] << std::endl;
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;

            estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
        }
    }

    //Changing the base of the calibration model to the root link
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("root_link"));


    calibration_mutex.lock();
    std::cout << "wholeBodyDynamicsThread::calibrateOffset " << calib_code  << " called successfully, starting calibration." << std::endl;
    wbd_mode = CALIBRATING;
    run_mutex.unlock();

    return true;
}

bool wholeBodyDynamicsThread::decodeCalibCode(const std::string calib_code)
{
    if( calib_code == "all" ) {
        for(int ft_sens = 0; ft_sens < calibrate_ft_sensor.size(); ft_sens++ )
        {
            calibrate_ft_sensor[ft_sens] = true;
        }
    } else if ( calib_code == "arms" ) {
        if( arms_fts.size() == 0 )
        {
            std::cout << "wholeBodyDynamicsThread::decodeCalibCode error: requested feet calibration, but arms FT sensor are not available." << std::endl;
            return false;
        }
        for(int i = 0; i < arms_fts.size(); i++ )
        {
            int ft_sens = arms_fts[i];
            calibrate_ft_sensor[ft_sens] = true;
        }
    } else if ( calib_code == "legs" ) {
        if( legs_fts.size() == 0 )
        {
            std::cout << "wholeBodyDynamicsThread::decodeCalibCode error: requested feet calibration, but legs FT sensor are not available." << std::endl;
            return false;
        }
        for(int i = 0; i < legs_fts.size(); i++ )
        {
            int ft_sens = legs_fts[i];
            calibrate_ft_sensor[ft_sens] = true;
        }
    } else if ( calib_code == "feet" ) {
        if( feet_fts.size() == 0 )
        {
            std::cout << "wholeBodyDynamicsThread::decodeCalibCode error: requested feet calibration, but feet FT sensor are not available." << std::endl;
            return false;
        }
        for(int i = 0; i < feet_fts.size(); i++ )
        {
            int ft_sens = feet_fts[i];
            calibrate_ft_sensor[ft_sens] = true;
        }
    } else {
        std::cout << "wholeBodyDynamicsThread::decodeCalibCode error: code " << calib_code << " not available" << std::endl;
        return false;
    }
}

void wholeBodyDynamicsThread::disableCalibration()
{
    for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
        offset_buffer[ft_sensor_id].zero();
        calibrate_ft_sensor[ft_sensor_id] = false;
    }
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffsetOnDoubleSupport(const std::string calib_code, int samples_to_use)
{
    run_mutex.lock();
    samples_requested_for_calibration= samples_to_use;
    std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnDoubleSupport called with code " << calib_code << std::endl;
    if( samples_requested_for_calibration <= 0 )
    {
        std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnDoubleSupport error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples." << std::endl;
        return false;
    }

    if( !this->decodeCalibCode(calib_code) )
    {
        return false;
    }

    //Resetting the offset for the sensor being calibrated, to get the raw values
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        if( calibrate_ft_sensor[ft_id] )
        {
            double ft_off[6];
            estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);

            std::cout << "wholeBodyDynamicsThread::calibrateOffset: current calibration for FT " << ft_id << " is " <<
                         ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5] << std::endl;
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;
            estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
        }
    }

    calibration_mutex.lock();
    std::cout << "wholeBodyDynamicsThread::calibrateOffset " << calib_code  << " called successfully, starting calibration." << std::endl;
    wbd_mode = CALIBRATING_ON_DOUBLE_SUPPORT;
    run_mutex.unlock();

    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport(const std::string calib_code, int samples_to_use)
{
    run_mutex.lock();
    samples_requested_for_calibration= samples_to_use;
    std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport called with code " << calib_code << std::endl;
    if( samples_requested_for_calibration <= 0 )
    {
        std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples." << std::endl;
        return false;
    }

    if( !this->decodeCalibCode(calib_code) )
    {
        return false;
    }

    //Resetting the offset for the sensor being calibrated, to get the raw values
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        if( calibrate_ft_sensor[ft_id] )
        {
            double ft_off[6];
            estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);

            std::cout << "wholeBodyDynamicsThread::calibrateOffset: current calibration for FT " << ft_id << " is " <<
                         ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5] << std::endl;
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;
            estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
        }
    }

     //Changing the base of the calibration model to the left foot
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("l_sole"));


    calibration_mutex.lock();
    std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport " << calib_code  << " called successfully, starting calibration." << std::endl;
    wbd_mode = CALIBRATING;
    run_mutex.unlock();

    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport(const std::string calib_code, int samples_to_use)
{
    run_mutex.lock();
    samples_requested_for_calibration= samples_to_use;
    std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport called with code " << calib_code << std::endl;
    if( samples_requested_for_calibration <= 0 )
    {
        std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples." << std::endl;
        return false;
    }

    if( !this->decodeCalibCode(calib_code) )
    {
        return false;
    }

    //Resetting the offset for the sensor being calibrated, to get the raw values
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        if( calibrate_ft_sensor[ft_id] )
        {
            double ft_off[6];
            estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);

            std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport: current calibration for FT " << ft_id << " is " <<
                         ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5] << std::endl;
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;
            estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
        }
    }

    //Changing the base of the calibration model to the left foot
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("r_sole"));

    calibration_mutex.lock();
    std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport " << calib_code  << " called successfully, starting calibration." << std::endl;
    wbd_mode = CALIBRATING;
    run_mutex.unlock();

    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::resetOffset(const std::string calib_code)
{
    run_mutex.lock();
    std::cout << "wholeBodyDynamicsThread::resetOffset called with code " << calib_code << std::endl;

    if( !this->decodeCalibCode(calib_code) )
    {
        return false;
    }

    //Resetting the offset
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ ) {
        if( calibrate_ft_sensor[ft_id] ) {
            double ft_off[6];
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;
            estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
        }
    }

    std::cout << "wholeBodyDynamicsThread::resetOffset " << calib_code  << " called successfully." << std::endl;

    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        calibrate_ft_sensor[ft_id] = false;
    }

    run_mutex.unlock();

    return true;
}


//*************************************************************************************************************************
bool wholeBodyDynamicsThread::waitCalibrationDone()
{
    calibration_mutex.lock();
    assert(wbd_mode == NORMAL);
    calibration_mutex.unlock();
    std::cout << "wholeBodyDynamicsThread::waitCalibrationDone() returning: calibration finished with success" << std::endl;
    return true;
}

//****************************************************************************
void wholeBodyDynamicsThread::getEndEffectorWrenches()
{
    bool ret;
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,tree_status.q.data());
    if(!ret)
    {
        std::cout << "wholeBodyDynamicsThread::getEndEffectorWrenches(): Unable to get estimates of joint positions " << std::endl;
        return;
    }
    YARP_ASSERT(ret);
    icub_model_calibration->setAng(tree_status.q);

    ret = estimator->getEstimate(wbi::ESTIMATE_EXTERNAL_FORCE_TORQUE, left_gripper_frame_id, LAExternalWrench.data());
    if(!ret)
    {
        std::cout << "wholeBodyDynamicsThread::getEndEffectorWrenches(): Unable to get estimates of left gripper" << std::endl;
        return;
    }
    YARP_ASSERT(ret);
    transform_mat_buffer = icub_model_calibration->getPosition(root_link_idyntree_id,left_gripper_frame_idyntree_id);
    LACartesianExternalWrench.setSubvector(0,transform_mat_buffer.submatrix(0,2,0,2)*LAExternalWrench.subVector(0,2));
    LACartesianExternalWrench.setSubvector(3,transform_mat_buffer.submatrix(0,2,0,2)*LAExternalWrench.subVector(3,5));

    ret = estimator->getEstimate(wbi::ESTIMATE_EXTERNAL_FORCE_TORQUE, right_gripper_frame_id, RAExternalWrench.data());
    YARP_ASSERT(ret);
    transform_mat_buffer = icub_model_calibration->getPosition(root_link_idyntree_id,right_gripper_frame_idyntree_id);
    RACartesianExternalWrench.setSubvector(0,transform_mat_buffer.submatrix(0,2,0,2)*RAExternalWrench.subVector(0,2));
    RACartesianExternalWrench.setSubvector(3,transform_mat_buffer.submatrix(0,2,0,2)*RAExternalWrench.subVector(3,5));

    ret = estimator->getEstimate(wbi::ESTIMATE_EXTERNAL_FORCE_TORQUE, left_sole_frame_id, LLExternalWrench.data());
    YARP_ASSERT(ret);
    transform_mat_buffer = icub_model_calibration->getPosition(root_link_idyntree_id,left_sole_frame_idyntree_id);
    LLCartesianExternalWrench.setSubvector(0,transform_mat_buffer.submatrix(0,2,0,2)*LLExternalWrench.subVector(0,2));
    LLCartesianExternalWrench.setSubvector(3,transform_mat_buffer.submatrix(0,2,0,2)*LLExternalWrench.subVector(3,5));


    ret = estimator->getEstimate(wbi::ESTIMATE_EXTERNAL_FORCE_TORQUE, right_sole_frame_id, RLExternalWrench.data());
    YARP_ASSERT(ret);
    transform_mat_buffer = icub_model_calibration->getPosition(root_link_idyntree_id,right_sole_frame_idyntree_id);
    RLCartesianExternalWrench.setSubvector(0,transform_mat_buffer.submatrix(0,2,0,2)*RLExternalWrench.subVector(0,2));
    RLCartesianExternalWrench.setSubvector(3,transform_mat_buffer.submatrix(0,2,0,2)*RLExternalWrench.subVector(3,5));
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishTorques()
{
    //Converting torques from the serialization used in wholeBodyStates interface to the port used by the robot

    assert(part_torques.size() == Torques_ports.size());

    for(int output_torque_port_id = 0;
        output_torque_port_id < output_torque_ports.size();
        output_torque_port_id++ )
    {
        for(int output_vector_index = 0;
            output_vector_index < output_torque_ports[output_torque_port_id].wbi_numeric_ids_to_publish.size();
            output_vector_index++)
        {
            int torque_wbi_numeric_id = output_torque_ports[output_torque_port_id].wbi_numeric_ids_to_publish[output_vector_index];
            output_torque_ports[output_torque_port_id].output_vector[output_vector_index] = all_torques[torque_wbi_numeric_id];
        }

        writeTorque(output_torque_ports[output_torque_port_id].output_vector,
                    output_torque_ports[output_torque_port_id].magic_number,
                    &(output_torque_ports[output_torque_port_id].output_port));
    }

    /// \todo remove this hardcoded dependency on the serialization used in the interface (using getEstimateList)

    /// \note The torso has a weird dependency, basically the joint serialization in the robot and the one on the model are reversed
    //TOTorques[0] = all_torques[2];
    //TOTorques[1] = all_torques[1];
    //TOTorques[2] = all_torques[0];

    //for(int i=0; i < 3; i++ ) {
    //    HDTorques[i] = all_torques[3+i];
    //}

    //for(int i=0; i < 7; i++ ) {
    //    LATorques[i] = all_torques[3+3+i];
    //}

    //for(int i=0; i < 7; i++ ) {
    //    RATorques[i] = all_torques[3+3+7+i];
    //}

    //for(int i=0; i < 6; i++ ) {
    //    LLTorques[i] = all_torques[3+3+7+7+i];
    //}

    //for(int i=0; i < 6; i++ ) {
    //    RLTorques[i] = all_torques[3+3+7+7+6+i];
    //}

    //Parameters copied from old wholeBodyDynamics
    //writeTorque(TOTorques, 4, port_TOTorques);
    //writeTorque(HDTorques, 0, port_HDTorques);
    //writeTorque(LATorques, 1, port_LATorques);
    //writeTorque(RATorques, 1, port_RATorques);
    //writeTorque(LATorques, 3, port_LWTorques);
    //writeTorque(RATorques, 3, port_RWTorques);
    //writeTorque(LLTorques, 2, port_LLTorques);
    //writeTorque(RLTorques, 2, port_RLTorques);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishContacts()
{
    broadcastData<skinContactList>(external_forces_list, port_contacts);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishEndEffectorWrench()
{
    broadcastData<yarp::sig::Vector>(LAExternalWrench, port_external_wrench_LA);
    broadcastData<yarp::sig::Vector>(RAExternalWrench, port_external_wrench_RA);
    broadcastData<yarp::sig::Vector>(LLExternalWrench, port_external_wrench_LL);
    broadcastData<yarp::sig::Vector>(RLExternalWrench, port_external_wrench_RL);

    broadcastData<yarp::sig::Vector>(LACartesianExternalWrench, port_external_cartesian_wrench_LA);
    broadcastData<yarp::sig::Vector>(RACartesianExternalWrench, port_external_cartesian_wrench_RA);
    broadcastData<yarp::sig::Vector>(LLCartesianExternalWrench, port_external_cartesian_wrench_LL);
    broadcastData<yarp::sig::Vector>(RLCartesianExternalWrench, port_external_cartesian_wrench_RL);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishBaseToGui()
{
    bool ret;
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,tree_status.q.data());
    YARP_ASSERT(ret);

    //For the icubGui, the world is the root frame when q == 0
    //So we have to find the transformation between the root now
    //and the root when q == 0
    icub_model_calibration->setAng(tree_status.q);

    // {}^{leftFoot} H_{currentRoot}
    KDL::Frame H_leftFoot_currentRoot
       = icub_model_calibration->getPositionKDL(left_foot_link_idyntree_id,root_link_idyntree_id);

    tree_status.q.zero();
    icub_model_calibration->setAng(tree_status.q);

    //{}^world H_{leftFoot}
    KDL::Frame H_world_leftFoot
        = icub_model_calibration->getPositionKDL(root_link_idyntree_id,left_foot_link_idyntree_id);

    KDL::Frame H_world_currentRoot
        = H_world_leftFoot*H_leftFoot_currentRoot;

    iCubGuiBase.zero();

    //Set angular part
    double roll,pitch,yaw;
    H_world_currentRoot.M.GetRPY(roll,pitch,yaw);
    //H_world_currentRoot.M.Inverse().GetRPY(roll,pitch,yaw);

    const double RAD2DEG = 180.0/(3.1415);

    iCubGuiBase[0] = RAD2DEG*roll;
    iCubGuiBase[1] = RAD2DEG*pitch;
    iCubGuiBase[2] = RAD2DEG*yaw;

    /*
    iCubGuiBase[0] = RAD2DEG*roll;
    iCubGuiBase[1] = RAD2DEG*pitch;
    iCubGuiBase[2] = RAD2DEG*yaw;
    */

    //Set linear part (iCubGui wants the root offset in millimeters)
    const double METERS2MILLIMETERS = 1000.0;
    iCubGuiBase[3] = METERS2MILLIMETERS*H_world_currentRoot.p(0);
    iCubGuiBase[4] = METERS2MILLIMETERS*H_world_currentRoot.p(1);
    iCubGuiBase[5] = METERS2MILLIMETERS*H_world_currentRoot.p(2);

    //Add offset to avoid lower forces to be hided by the floor
    iCubGuiBase[5] = iCubGuiBase[5] + 1000.0;

    broadcastData<yarp::sig::Vector>(iCubGuiBase,port_icubgui_base);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishAnkleFootForceTorques()
{
    YARP_ASSERT(zmp_test_mode);

    yarp::sig::Vector joint_ankle_cartesian_wrench(6,0.0);
    yarp::sig::Vector joint_ankle_cartesian_wrench_from_model(6,0.0);
    yarp::sig::Vector joint_foot_cartesian_wrench(6,0.0);
    yarp::sig::Vector joint_foot_cartesian_wrench_from_model(6,0.0);

    //Compute model forces
    bool ret;
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,tree_status.q.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_VEL,tree_status.dq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_ACC,tree_status.ddq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_IMU,tree_status.wbi_imu.data());
    YARP_ASSERT(ret);

    //std::cout << "wholeBodyDynamicsThread::calibration_run(): estimates obtained" << std::endl;

    //Setting imu proper acceleration from measure (assuming omega e domega = 0)
    //acceleration are measures 4:6 (check wbi documentation)
    tree_status.proper_ddp_imu[0] = tree_status.wbi_imu[4];
    tree_status.proper_ddp_imu[1] = tree_status.wbi_imu[5];
    tree_status.proper_ddp_imu[2] = tree_status.wbi_imu[6];

    tree_status.omega_imu[0] = 0.0*tree_status.wbi_imu[7];
    tree_status.omega_imu[1] = 0.0*tree_status.wbi_imu[8];
    tree_status.omega_imu[2] = 0.0*tree_status.wbi_imu[9];

    //Estimating sensors
    //icub_model_zmp->setInertialMeasure(0.0*tree_status.omega_imu,0.0*tree_status.domega_imu,tree_status.proper_ddp_imu);
    //icub_model_zmp->setAng(tree_status.q);
    //icub_model_zmp->setDAng(0.0*tree_status.dq);
    //icub_model_zmp->setD2Ang(0.0*tree_status.ddq);

    //icub_model_zmp->kinematicRNEA();
    //icub_model_zmp->dynamicRNEA();

    //joint_ankle_cartesian_wrench_from_model = icub_model_zmp->getJointForceTorque(ankle_joint_idyntree_id,foot_sole_link_idyntree_id);
    //joint_foot_cartesian_wrench_from_model = icub_model_zmp->getJointForceTorque(foot_sole_fake_joint_idyntree_id,foot_sole_link_idyntree_id);

    // \todo TODO enable back this estimation
    //YARP_ASSERT(estimator->getEstimateJointForceTorque(ankle_joint_idyntree_id,joint_ankle_cartesian_wrench.data(),foot_sole_link_idyntree_id));
    //YARP_ASSERT(estimator->getEstimateJointForceTorque(foot_sole_fake_joint_idyntree_id,joint_foot_cartesian_wrench.data(),foot_sole_link_idyntree_id));

    //broadcastData<yarp::sig::Vector>(joint_ankle_cartesian_wrench,port_joint_ankle_cartesian_wrench);
    //broadcastData<yarp::sig::Vector>(joint_ankle_cartesian_wrench_from_model,port_joint_ankle_cartesian_wrench_from_model);
    //broadcastData<yarp::sig::Vector>(joint_foot_cartesian_wrench,port_joint_foot_cartesian_wrench);
    //broadcastData<yarp::sig::Vector>(joint_foot_cartesian_wrench_from_model,port_joint_foot_cartesian_wrench_from_model);

}


//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    run_mutex.lock();
    if( wbd_mode == NORMAL )
    {
        normal_run();
    }
    else if( wbd_mode == CALIBRATING )
    {
        calibration_run();
    }
    else
    {
        assert( wbd_mode == CALIBRATING_ON_DOUBLE_SUPPORT );
        calibration_on_double_support_run();
    }
    run_mutex.unlock();
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::normal_run()
{
    bool ret;
    //Get data from estimator and publish it

    //Get estimated torques
    assert(estimator->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE) == (int)all_torques.size());
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_TORQUE,all_torques.data());
    YARP_ASSERT(ret);

    //Get estimated external contacts
    ret = estimator->getEstimatedExternalForces(external_forces_list);
    YARP_ASSERT(ret);

    //Get estimated external ee wrenches
    getEndEffectorWrenches();

    //Send torques
    publishTorques();

    //Send external contacts
    publishContacts();

    //Send external wrench estimates
    publishEndEffectorWrench();

    //Send base information to iCubGui
    publishBaseToGui();

    //if in zmp test mode, publish the necessary information
    if( zmp_test_mode )
    {
        publishAnkleFootForceTorques();
    }

    //if normal mode, publish the
    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

    if( printCountdown == 0 ) {

        double avgTime, stdDev, avgTimeUsed, stdDevUsed, period;
        period = getRate();
        getEstPeriod(avgTime, stdDev);
        getEstUsed(avgTimeUsed, stdDevUsed);
        //if(avgTimeUsed>period) {
            //std::cout << "Performance information in cout" << std::endl;
            //std::cerr << "Performance information in cerr" << std::endl;
            //fprintf(stdout,"PERFORMANCE INFORMATION:\n");
            //printf("Expected period %lf ms.\nReal period: %3.1lf+/-%3.1lf ms.\n", period, avgTime, stdDev);
            //printf("Real duration of 'run' method: %3.1lf+/-%3.1lf ms.\n", avgTimeUsed, stdDevUsed);
            //if(avgTimeUsed<0.5*period)
            //    printf("Next time you could set a lower period to improve the wholeBodyDynamics performance.\n");
            //else if(avgTime>1.3*period)
            //    printf("The period you set was impossible to attain. Next time you could set a higher period.\n");
        //}
        /*
        std::cout << "Torques: " << std::endl;
        std::cout << all_torques.toString() << std::endl;
        std::cout << "Forces: " << std::endl;
        std::cout << external_forces_list.toString() << std::endl;
        std::cout << "Measured Force Torque Sensors: " << std::endl;
        yarp::sig::Vector ft_mes(6*6,0.0);
        ret = estimator->getEstimates(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,ft_mes.data());
        for(int i=0; i < 6; i++ )
        {
            std::cout << "FT sensor " << i << " : " << std::endl;
            std::cout << ft_mes.subVector(6*i,6*i+5).toString() << std::endl;
        }
        */
    }

}

std::string getCurrentDateAndTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
  std::string str(buffer);

  return str;
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::calibration_run()
{
    //std::cout << "wholeBodyDynamicsThread::calibration_run() " << samples_used_for_calibration << " / " << samples_requested_for_calibration << "  called" << std::endl;
    bool ret;
    YARP_ASSERT(tree_status.q.size() == 32);
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,tree_status.q.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_VEL,tree_status.dq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_ACC,tree_status.ddq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_IMU,tree_status.wbi_imu.data());
    YARP_ASSERT(ret);

    //std::cout << "wholeBodyDynamicsThread::calibration_run(): estimates obtained" << std::endl;

    //Setting imu proper acceleration from measure (assuming omega e domega = 0)
    //acceleration are measures 4:6 (check wbi documentation)
    if( assume_fixed_base_calibration )
    {
        double gravity = 9.8;
        tree_status.proper_ddp_imu[0] = 0.0;
        tree_status.proper_ddp_imu[1] = 0.0;
        tree_status.proper_ddp_imu[2] = 0.0;
        if( fixed_link_calibration == "root_link" )
        {
            tree_status.proper_ddp_imu[2] = gravity;
        }
        else if( fixed_link_calibration == "l_sole" || fixed_link_calibration == "r_sole" )
        {
            tree_status.proper_ddp_imu[0] = gravity;
        }
    }
    else
    {
        tree_status.proper_ddp_imu[0] = tree_status.wbi_imu[4];
        tree_status.proper_ddp_imu[1] = tree_status.wbi_imu[5];
        tree_status.proper_ddp_imu[2] = tree_status.wbi_imu[6];
    }
    tree_status.omega_imu[0] = 0.0*tree_status.wbi_imu[7];
    tree_status.omega_imu[1] = 0.0*tree_status.wbi_imu[8];
    tree_status.omega_imu[2] = 0.0*tree_status.wbi_imu[9];

    //Estimating sensors
    icub_model_calibration->setInertialMeasure(0.0*tree_status.omega_imu,0.0*tree_status.domega_imu,tree_status.proper_ddp_imu);
    icub_model_calibration->setAng(tree_status.q);
    icub_model_calibration->setDAng(0.0*tree_status.dq);
    icub_model_calibration->setD2Ang(0.0*tree_status.ddq);

    icub_model_calibration->kinematicRNEA();
    icub_model_calibration->dynamicRNEA();

    //std::cout << "wholeBodyDynamicsThread::calibration_run(): F/T estimates computed" << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_run() : imu proper acceleration " << tree_status.proper_ddp_imu.toString() << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_run() : q " << tree_status.q.toString() << std::endl;


    for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
        if( calibrate_ft_sensor[ft_sensor_id] ) {
            //Get sensor estimated from model
            icub_model_calibration->getSensorMeasurement(ft_sensor_id,tree_status.estimated_ft_sensors[ft_sensor_id]);
            //Get sensor measure
            estimator->getEstimate(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,ft_sensor_id,tree_status.measured_ft_sensors[ft_sensor_id].data());
            assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
            offset_buffer[ft_sensor_id] += tree_status.measured_ft_sensors[ft_sensor_id]-tree_status.estimated_ft_sensors[ft_sensor_id];
            //std::cout << "Estimated ft sensor " << ft_sensor_id << " : " << tree_status.estimated_ft_sensors[ft_sensor_id].toString() << std::endl;
            //std::cout << "Subchain mass : " << norm(tree_status.estimated_ft_sensors[ft_sensor_id].subVector(0,2))/norm( tree_status.proper_ddp_imu) << std::endl;
        }
    }

    samples_used_for_calibration++;


    if( samples_used_for_calibration >= samples_requested_for_calibration ) {
        //Calculating offset
        //std::cout.precision(20);
        std::cout << "[INFO] wholeBodyDynamicsThread: complete calibration at system time : " << getCurrentDateAndTime() << std::endl;
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            if( calibrate_ft_sensor[ft_sensor_id] ) {
                offset_buffer[ft_sensor_id] *= (1.0/(double)samples_used_for_calibration);
                assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
                estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_sensor_id),offset_buffer[ft_sensor_id].data());
                double ft_off[6];
                estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_sensor_id),ft_off);

                std::cout << "[INFO] wholeBodyDynamicsThread: new calibration for FT " << ft_sensor_id << " is " <<
                            ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5] << std::endl;
            }
        }


        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        this->disableCalibration();


        icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("root_link"));
        wbd_mode = NORMAL;
        calibration_mutex.unlock();
    }


    /*
    std::cout << "wholeBodyDynamicsThread::calibration_run() "
              <<  samples_used_for_calibration << " / "
              << samples_requested_for_calibration << "  finished" << std::endl;
    */
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::calibration_on_double_support_run()
{
    //std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run() " << samples_used_for_calibration << " / " << samples_requested_for_calibration << "  called" << std::endl;
    bool ret;
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,tree_status.q.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_VEL,tree_status.dq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_ACC,tree_status.ddq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_IMU,tree_status.wbi_imu.data());
    YARP_ASSERT(ret);

    //std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run(): estimates obtained" << std::endl;

    //Setting imu proper acceleration from measure (assuming omega e domega = 0)
    //acceleration are measures 4:6 (check wbi documentation)
    if( assume_fixed_base_calibration )
    {
        double gravity = 9.8;
        tree_status.proper_ddp_imu[0] = 0.0;
        tree_status.proper_ddp_imu[1] = 0.0;
        tree_status.proper_ddp_imu[2] = 0.0;
        if( fixed_link_calibration == "root_link" )
        {
            tree_status.proper_ddp_imu[2] = gravity;
        }
        else if( fixed_link_calibration == "l_sole" || fixed_link_calibration == "r_sole" )
        {
            tree_status.proper_ddp_imu[0] = gravity;
        }
    }
    else
    {
        tree_status.proper_ddp_imu[0] = tree_status.wbi_imu[4];
        tree_status.proper_ddp_imu[1] = tree_status.wbi_imu[5];
        tree_status.proper_ddp_imu[2] = tree_status.wbi_imu[6];
    }
    tree_status.omega_imu[0] = 0.0*tree_status.wbi_imu[7];
    tree_status.omega_imu[1] = 0.0*tree_status.wbi_imu[8];
    tree_status.omega_imu[2] = 0.0*tree_status.wbi_imu[9];

    //Estimating sensors
    icub_model_calibration->setInertialMeasure(0.0*tree_status.omega_imu,0.0*tree_status.domega_imu,tree_status.proper_ddp_imu);
    icub_model_calibration->setAng(tree_status.q);
    icub_model_calibration->setDAng(0.0*tree_status.dq);
    icub_model_calibration->setD2Ang(0.0*tree_status.ddq);

    icub_model_calibration->kinematicRNEA();
    icub_model_calibration->estimateDoubleSupportContactForce(left_foot_link_idyntree_id,right_foot_link_idyntree_id);
    icub_model_calibration->dynamicRNEA();

    //Get known terms of the Newton-Euler equation

    //std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run(): F/T estimates computed" << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run() : imu proper acceleration " << tree_status.proper_ddp_imu.toString() << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run() : q " << tree_status.q.toString() << std::endl;

    for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ )
    {
        if( calibrate_ft_sensor[ft_sensor_id] )
        {
            //Get sensor estimated from model
            icub_model_calibration->getSensorMeasurement(ft_sensor_id,tree_status.estimated_ft_sensors[ft_sensor_id]);

            //Get sensor measure
            estimator->getEstimate(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,ft_sensor_id,tree_status.measured_ft_sensors[ft_sensor_id].data());
            assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
            offset_buffer[ft_sensor_id] += tree_status.measured_ft_sensors[ft_sensor_id]-tree_status.estimated_ft_sensors[ft_sensor_id];
            //std::cout << "Estimated ft sensor " << ft_sensor_id << " : " << tree_status.estimated_ft_sensors[ft_sensor_id].toString() << std::endl;
            //std::cout << "Subchain mass : " << norm(tree_status.estimated_ft_sensors[ft_sensor_id].subVector(0,2))/norm( tree_status.proper_ddp_imu) << std::endl;
        }
    }

    samples_used_for_calibration++;


    if( samples_used_for_calibration >= samples_requested_for_calibration ) {
        //Calculating offset
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            if( calibrate_ft_sensor[ft_sensor_id] ) {
                offset_buffer[ft_sensor_id] *= (1.0/(double)samples_used_for_calibration);
                assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
                estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_sensor_id),offset_buffer[ft_sensor_id].data());
                double ft_off[6];
                estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_sensor_id),ft_off);

                std::cout << "wholeBodyDynamicsThread: new calibration for FT " << ft_sensor_id << " is " <<
                            ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5] << std::endl;
            }
        }


        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            offset_buffer[ft_sensor_id].zero();
            calibrate_ft_sensor[ft_sensor_id] = false;
        }

        icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("root_link"));
        wbd_mode = NORMAL;
        calibration_mutex.unlock();
    }

    /*
    std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run() "
              <<  samples_used_for_calibration << " / "
              << samples_requested_for_calibration << "  finished" << std::endl;
    */
}

//*****************************************************************************
void wholeBodyDynamicsThread::threadRelease()
{
    run_mutex.lock();

    std::cerr << "[INFO] Closing output torques ports\n";
    for(int output_torque_port_i = 0; output_torque_port_i < output_torque_ports.size(); output_torque_port_i++ )
    {
        output_torque_ports[output_torque_port_i].output_port.close();
    }

    std::cerr << "Closing contacts port\n";
    closePort(port_contacts);
    std::cerr << "Closing end effector wrenches port\n";
    closePort(port_external_cartesian_wrench_LA);
    closePort(port_external_cartesian_wrench_RA);
    closePort(port_external_cartesian_wrench_LL);
    closePort(port_external_cartesian_wrench_RL);
    closePort(port_external_wrench_LA);
    closePort(port_external_wrench_LL);
    closePort(port_external_wrench_RA);
    closePort(port_external_wrench_RL);
    std::cerr << "Closing iCubGui base port\n";
    closePort(port_icubgui_base);

    if(zmp_test_mode)
    {
        closePort(port_joint_ankle_cartesian_wrench);
        closePort(port_joint_ankle_cartesian_wrench_from_model);
        closePort(port_joint_foot_cartesian_wrench);
        closePort(port_joint_foot_cartesian_wrench_from_model);
        delete icub_model_zmp;
    }

    delete icub_model_calibration;

    run_mutex.unlock();
}

//*****************************************************************************
void wholeBodyDynamicsThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port->interrupt();
        _port->close();

        delete _port;
        _port = 0;
    }
}

//*****************************************************************************
template <class T> void wholeBodyDynamicsThread::broadcastData(T& _values, BufferedPort<T> *_port)
{
    if (_port && _port->getOutputCount()>0)
    {
        _port->setEnvelope(this->timestamp);
        _port->prepare()  = _values ;
        _port->write();
    }
}

//*****************************************************************************
void wholeBodyDynamicsThread::writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port)
{
    /** \todo TODO avoid (as much as feasible) dynamic memory allocation */
    Bottle a;
    a.addInt(_address);
    for(size_t i=0;i<_values.length();i++)
        a.addDouble(_values(i));
    _port->prepare() = a;
    _port->write();
}
