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

// Local includes
#include "wholeBodyDynamicsTree/wholeBodyDynamicsThread.h"
#include "wholeBodyDynamicsTree/wholeBodyDynamicsStatesInterface.h"

// Yarp includes
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/SVD.h>

// System includes
#include <cstring>
#include <ctime>

#include "ctrlLibRT/filters.h"


using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace wbi;
using namespace yarpWbi;
using namespace std;
using namespace iCub::skinDynLib;

RobotStatus::RobotStatus(int nrOfDOFs, int nrOfFTsensors)
{
    setNrOfDOFs(nrOfDOFs);
    setNrOfFTSensors(nrOfFTsensors);

    this->zero();
}

bool RobotStatus::zero()
{
    domega_imu.zero();
    omega_imu.zero();
    proper_ddp_imu.zero();
    wbi_imu.zero();
    qj.zero();
    dqj.zero();
    ddqj.zero();
    torquesj.zero();
    for(unsigned int i=0; i < estimated_ft_sensors.size(); i++ ) {
        estimated_ft_sensors[i].zero();
        measured_ft_sensors[i].zero();
        ft_sensors_offset[i].zero();
        model_ft_sensors[i].zero();
    }
    return true;
}

bool RobotStatus::setNrOfDOFs(int nrOfDOFs)
{
    domega_imu.resize(3);
    omega_imu.resize(3);
    proper_ddp_imu.resize(3);
    wbi_imu.resize(wbi::sensorTypeDescriptions[wbi::SENSOR_IMU].dataSize);
    qj.resize(nrOfDOFs);
    dqj.resize(nrOfDOFs);
    ddqj.resize(nrOfDOFs);
    torquesj.resize(nrOfDOFs);

    zero();
    return true;
}

bool RobotStatus::setNrOfFTSensors(int nrOfFTsensors)
{
    estimated_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    measured_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    ft_sensors_offset.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    model_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    zero();
    return true;
}

//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name,
                                                 string _robotName,
                                                 int _period,
                                                 yarpWholeBodySensors *_wbs,
                                                 yarp::os::Property & _yarp_options,
                                                 bool _assume_fixed_base_calibration,
                                                 std::string _fixed_link_calibration
                                                )
    :  RateThread(_period),
       moduleName(_name),
       robotName(_robotName),
       sensors(_wbs),
       yarp_options(_yarp_options),
       printCountdown(0),
       printPeriod(2000),
       samples_requested_for_calibration(200),
       max_samples_for_calibration(2000),
       samples_used_for_calibration(0),
       assume_fixed_base_calibration(_assume_fixed_base_calibration),
       fixed_link_calibration(_fixed_link_calibration)
    {
        // TODO FIXME move all this logic in threadInit

       yInfo() << "Launching wholeBodyDynamicsThread with name : " << _name << " and robotName " << _robotName << " and period " << _period;

       if( !_yarp_options.check("urdf") )
       {
            yError() << "wholeBodyDynamicsTree error: urdf not found in configuration files";
            return;
       }

    std::string urdf_file = _yarp_options.find("urdf").asString().c_str();
    yarp::os::ResourceFinder rf;
    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());



    std::vector<std::string> dof_serialization;

    // \todo TODO FIXME move IDList -> std::vector<std::string> conversion to wbiIdUtils
    IDList torque_estimation_list = _wbs->getSensorList(wbi::SENSOR_ENCODER);
    for(int dof=0; dof < (int)torque_estimation_list.size(); dof++)
    {
        ID wbi_id;
        torque_estimation_list.indexToID(dof,wbi_id);
        dof_serialization.push_back(wbi_id.toString());
    }

    std::vector<std::string> ft_serialization;
    IDList ft_sensor_list =  _wbs->getSensorList(wbi::SENSOR_FORCE_TORQUE);
    for(int ft=0; ft < (int)ft_sensor_list.size(); ft++)
    {
        ID wbi_id;
        ft_sensor_list.indexToID(ft,wbi_id);
        ft_serialization.push_back(wbi_id.toString());
    }

       if( assume_fixed_base_calibration ) {
           icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,fixed_link_calibration);
       } else {
           icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
       }

    //Resize buffer vectors
    all_torques.resize(_wbs->getSensorNumber(wbi::SENSOR_FORCE_TORQUE));

    iCubGuiBase.resize(6);
    FilteredInertialForGravityComp.resize(6);

    //Copied from old wholeBodyDynamics
    std::string robot_name = robotName;
    std::string local_name = moduleName;

    //Open ports
    port_contacts_input = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    port_contacts_input->open(string("/"+string(_name)+"/skin_contacts:i").c_str());

    port_contacts_output = new BufferedPort<skinContactList>;
    port_contacts_output->open(string("/"+local_name+"/contacts:o").c_str());

    //Open port for iCubGui
    port_icubgui_base = new BufferedPort<Vector>;
    port_icubgui_base->open(string("/"+local_name+"/base:o"));

    //Open port for filtered inertial
    port_filtered_inertial = new BufferedPort<Vector>;
    port_filtered_inertial->open(string("/"+local_name+"/filtered/inertial:o"));

    if( publish_filtered_ft )
    {
        //Open ports for filtered ft
        IDList ft_estimation_list = _wbs->getSensorList(wbi::SENSOR_FORCE_TORQUE);
        port_filtered_ft.resize(ft_estimation_list.size());
        for(int i=0; i < (int)ft_estimation_list.size(); i++ )
        {
            ID ft_id;
            ft_estimation_list.indexToID(i,ft_id);
            port_filtered_ft[i] = new BufferedPort<Vector>;
            port_filtered_ft[i]->open(string("/"+local_name+"/filtered/"+ft_id.toString()+":o"));
        }
    }

}

//*************************************************************************************************************************
wbi::ID wholeBodyDynamicsThread::convertFTiDynTreeToFTwbi(int ft_sensor_id)
{
    wbi::ID ret;
    sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).indexToID(ft_sensor_id,ret);
    return ret;
}

void checkFTSensorExist(std::string ft_sensor_name, wbi::IDList & all_fts, std::vector<int> & ft_id_list, iCub::iDynTree::TorqueEstimationTree * icub_model_calibration)
{
    if( all_fts.containsID(ft_sensor_name) )
    {
        int numeric_id;
        all_fts.idToIndex(ft_sensor_name,numeric_id);
        YARP_ASSERT(icub_model_calibration->getFTSensorIndex(ft_sensor_name) == numeric_id);
        ft_id_list.push_back(numeric_id);
    }
}


bool wholeBodyDynamicsThread::loadExternalWrenchesPortsConfigurations()
{
    // Load output external wrenches ports informations
    yarp::os::Bottle & output_wrench_bot = yarp_options.findGroup("WBD_OUTPUT_EXTERNAL_WRENCH_PORTS");
    if( output_wrench_bot.isNull() )
    {
        yWarning() << "WBD_OUTPUT_EXTERNAL_WRENCH_PORTS group not found in configuration";
        return true;
    }

    int nr_of_output_wrench_ports = output_wrench_bot.size() - 1;

    for(int output_wrench_port = 1; output_wrench_port < output_wrench_bot.size(); output_wrench_port++)
    {
        outputWrenchPortInformation wrench_port_struct;
        yarp::os::Bottle *wrench_port = output_wrench_bot.get(output_wrench_port).asList();
        if( wrench_port == NULL || wrench_port->isNull() || wrench_port->size() != 2
            || wrench_port->get(1).asList() == NULL
            || !(wrench_port->get(1).asList()->size() == 2 || wrench_port->get(1).asList()->size() == 3 ) )
        {
            yError() << "malformed WBD_OUTPUT_EXTERNAL_WRENCH_PORTS group  found configuration, exiting";
            if( wrench_port )
            {
                yError() << "malformed line " << wrench_port->toString();
            }
            else
            {
                yError() << "malformed line " << output_wrench_bot.get(output_wrench_port).toString();
                yError() << "malformed group " << output_wrench_bot.toString();
            }
            return false;
        }

        wrench_port_struct.port_name = wrench_port->get(0).asString();
        wrench_port_struct.link = wrench_port->get(1).asList()->get(0).asString();

        if( wrench_port->get(1).asList()->size() == 2 )
        {
            // Simple configuration, both the origin and the orientation of the
            // force belong to the same frame
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.origin_frame = wrench_port_struct.orientation_frame;
        }
        else
        {
            assert( wrench_port->get(1).asList()->size() == 3 );
            // Complex configuration: the first parameter is the frame of the point of expression,
            // the second parameter is the frame of orientation
            wrench_port_struct.origin_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(2).asString();
        }

        output_wrench_ports.push_back(wrench_port_struct);

    }

    // Load indeces for specified links and frame
    for(int i=0; i < output_wrench_ports.size(); i++ )
    {
        output_wrench_ports[i].link_index =
            icub_model_calibration->getFrameIndex(output_wrench_ports[i].link);
        if( output_wrench_ports[i].link_index < 0 )
        {
            yError() << "Link " << output_wrench_ports[i].link << " not found in the model.";
            return false;
        }

        output_wrench_ports[i].origin_frame_index =
            icub_model_calibration->getFrameIndex(output_wrench_ports[i].origin_frame);
        if( output_wrench_ports[i].origin_frame_index < 0 )
        {
            yError() << "Frame " << output_wrench_ports[i].origin_frame << " not found in the model.";
            return false;
        }


        output_wrench_ports[i].orientation_frame_index =
            icub_model_calibration->getFrameIndex(output_wrench_ports[i].orientation_frame);
        if( output_wrench_ports[i].orientation_frame_index < 0 )
        {
            yError() << "Frame " << output_wrench_ports[i].orientation_frame_index << " not found in the model.";
            return false;
        }
    }

    assert(nr_of_output_wrench_ports == output_torque_ports.size());

    return true;
}

bool wholeBodyDynamicsThread::openExternalWrenchesPorts()
{
    for(int i = 0; i < output_wrench_ports.size(); i++ )
    {
        std::string port_name = output_wrench_ports[i].port_name;
        output_wrench_ports[i].output_port = new BufferedPort<Vector>;
        output_wrench_ports[i].output_port->open(port_name);
        output_wrench_ports[i].output_vector.resize(6);
    }
}

bool wholeBodyDynamicsThread::closeExternalWrenchesPorts()
{
    for(int i = 0; i < output_wrench_ports.size(); i++ )
    {
        this->closePort(output_wrench_ports[i].output_port);
    }
}

bool wholeBodyDynamicsThread::loadEstimatedTorquesPortsConfigurations()
{
   // Load output torque ports informations
    yarp::os::Bottle & output_torques_bot = yarp_options.findGroup("WBD_OUTPUT_TORQUE_PORTS");
    if( output_torques_bot.isNull() )
    {
        yError() << "WBD_OUTPUT_TORQUE_PORTS group not found in wholeBodyDynamics configuration, exiting";
        return false;
    }

    int nr_of_output_torques_ports = output_torques_bot.size() - 1;

    IDList torque_list = sensors->getSensorList(wbi::SENSOR_ENCODER);
    for(int output_torque_port = 1; output_torque_port < output_torques_bot.size(); output_torque_port++)
    {
        outputTorquePortInformation torque_port_struct;
        yarp::os::Bottle * torque_port = output_torques_bot.get(output_torque_port).asList();
        if( torque_port == NULL || torque_port->isNull() || torque_port->size() != 2
            || torque_port->get(1).asList() == NULL || torque_port->get(1).asList()->size() != 2)
        {
            yError() << "malformed WBD_OUTPUT_TORQUE_PORTS group  found in wholeBodyDynamics configuration, exiting";
            if( torque_port )
            {
                std::cerr << "[ERR] malformed line " << torque_port->toString() << std::endl;
            }
            else
            {
                std::cerr << "[ERR] malformed line " << output_torques_bot.get(output_torque_port).toString() << std::endl;
                std::cerr << "[ERR] malformed group " << output_torques_bot.toString() << std::endl;
            }
            return false;
        }
        torque_port_struct.port_name = torque_port->get(0).asString();
        torque_port_struct.magic_number = torque_port->get(1).asList()->get(0).asInt();
        yarp::os::Bottle * torque_ids = torque_port->get(1).asList()->get(1).asList();
        for(int i = 0; i < torque_ids->size(); i++ )
        {
            std::string torque_wbi_id = torque_ids->get(i).asString();
            int torque_wbi_numeric_id;
            torque_list.idToIndex(torque_wbi_id,torque_wbi_numeric_id);
            torque_port_struct.wbi_numeric_ids_to_publish.push_back(torque_wbi_numeric_id);
        }
        assert(torque_ids->size() == torque_port_struct.wbi_numeric_ids_to_publish.size() );
        torque_port_struct.output_vector.resize(torque_port_struct.wbi_numeric_ids_to_publish.size());

        output_torque_ports.push_back(torque_port_struct);
    }

    assert(nr_of_output_torques_ports == output_torque_ports.size());

    return true;
}




//*************************************************************************************************************************
bool wholeBodyDynamicsThread::threadInit()
{
    bool ret = this->loadExternalWrenchesPortsConfigurations();

    ret = ret && this->loadEstimatedTorquesPortsConfigurations();

    if( ! ret ) return false;

    // Open estimator
    int periodInMilliseconds = (int)getRate();
    this->externalWrenchTorqueEstimator = new ExternalWrenchesAndTorquesEstimator(periodInMilliseconds,
                                                                                  (yarpWbi::yarpWholeBodySensors *)sensors,
                                                                                            port_contacts_input,
                                                                                             yarp_options);

    if( !this->externalWrenchTorqueEstimator->init() )
    {
        return false;
    }

    //Load configuration related to the controlboards for which we are estimating the torques
    IDList torque_estimation_list = sensors->getSensorList(wbi::SENSOR_ENCODER);
    ret = loadJointsControlBoardFromConfig(yarp_options,
                                           torque_estimation_list,
                                           torqueEstimationControlBoards.controlBoardNames,
                                           torqueEstimationControlBoards.controlBoardAxisList);

    if( ! ret || torque_estimation_list.size() != torqueEstimationControlBoards.controlBoardAxisList.size() )
    {
        yError() << "Error in loading controlboard information for the configuration file";
        return false;
    }
    else
    {
        yInfo() << "ControlBoards information correctly loaded from configuration file";
    }

    torqueEstimationControlBoards.deviceDrivers.resize(torqueEstimationControlBoards.controlBoardNames.size());
    torqueEstimationControlBoards.controlModeInterfaces.resize(torqueEstimationControlBoards.controlBoardNames.size());
    torqueEstimationControlBoards.interactionModeInterfaces.resize(torqueEstimationControlBoards.controlBoardNames.size());

    // Open calibration configuration
    calibration_support_link = "root_link";
    if( yarp_options.check("calibration_support_link") )
    {
        calibration_support_link = yarp_options.find("calibration_support_link").asString().c_str();
        yInfo() << "calibration_support_link is " << calibration_support_link;
    }


    //Calibration variables
    int nrOfAvailableFTSensors = sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).size();
    if( nrOfAvailableFTSensors != icub_model_calibration->getNrOfFTSensors() ) {
        yError() << "wholeBodyDynamicsThread::threadInit() error: number of FT sensors different between model (" <<
        icub_model_calibration->getNrOfFTSensors() << ") and interface (" << nrOfAvailableFTSensors << " ) ";
        return false;
    }

    offset_buffer.resize(nrOfAvailableFTSensors,yarp::sig::Vector(6,0.0));
    calibrate_ft_sensor.resize(nrOfAvailableFTSensors,false);
    tree_status.setNrOfDOFs(icub_model_calibration->getNrOfDOFs());
    tree_status.setNrOfFTSensors(nrOfAvailableFTSensors);

    //Get list of ft sensors for calibration shortcut
    wbi::IDList ft_list = sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE);

    if( yarp_options.check("enable_w0_dw0") )
    {
        YARP_ASSERT(false);

        yInfo() << "enable_w0_dw0 option found, enabling the use of IMU angular velocity/acceleration.";
        externalWrenchTorqueEstimator->setEnableOmegaDomegaIMU(true);
    }

    if( yarp_options.check("disable_w0_dw0") )
    {
        yInfo() << "disable_w0_dw0 option found, disabling the use of IMU angular velocity/acceleration.";
        externalWrenchTorqueEstimator->setEnableOmegaDomegaIMU(false);
    }

    if( yarp_options.check("min_taxel") )
    {
        int taxel_threshold = yarp_options.find("min_taxel").asInt();
        yInfo() << "min_taxel option found, ignoring skin contacts with less then "
                  << taxel_threshold << " active taxels will be ignored.";
        externalWrenchTorqueEstimator->setMinTaxel(taxel_threshold);
    }
    else
    {
        int taxel_threshold = 0;
        externalWrenchTorqueEstimator->setMinTaxel(0);
    }

    if( yarp_options.check("autoconnect") )
    {
        yInfo() << "autoconnect option found, enabling the autoconnection.";
        this->autoconnect = true;
    }

    this->publish_filtered_ft = false;
    if( yarp_options.check("output_clean_ft") )
    {
        yInfo() << "output_clean_ft option found, enabling output of filtered and without offset ft sensors";
        this->publish_filtered_ft = true;
    }


    //Find end effector ids
    int max_id = 100;

    root_link_idyntree_id = icub_model_calibration->getLinkIndex("root_link");
    //YARP_ASSERT(root_link_idyntree_id >= 0 && root_link_idyntree_id < max_id );
    left_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("l_foot");
    //YARP_ASSERT(left_foot_link_idyntree_id >= 0  && left_foot_link_idyntree_id < max_id);
    right_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("r_foot");
    //YARP_ASSERT(right_foot_link_idyntree_id >= 0 && right_foot_link_idyntree_id < max_id);

    //Open and connect all the ports
    for(int output_torque_port_i = 0; output_torque_port_i < (int)output_torque_ports.size(); output_torque_port_i++ )
    {
        std::string port_name = output_torque_ports[output_torque_port_i].port_name;
        std::string local_port = "/" + moduleName + "/" + port_name + "/Torques:o";
        std::string robot_port = "/" + robotName  + "/joint_vsens/" + port_name + ":i";

    }

    // Open external wrenches ports
    openExternalWrenchesPorts();

    // Create filters
    double cutoffInHz = 3.0;
    double periodInSeconds = getRate()*1e-3;
    filters = new wholeBodyDynamicsFilters(torque_estimation_list.size(),nrOfAvailableFTSensors,
                                          cutoffInHz,periodInSeconds);


    if( this->autoconnect )
    {
        yInfo() << "wholeBodyDynamicsThread: autoconnect option enabled, autoconnecting.";
        for(int output_torque_port_i = 0; output_torque_port_i < output_torque_ports.size(); output_torque_port_i++ )
        {
            std::string port_name = output_torque_ports[output_torque_port_i].port_name;
            std::string local_port = "/" + moduleName + "/" + port_name + "/Torques:o";
            std::string robot_port = "/" + robotName  + "/joint_vsens/" + port_name + ":i";
            output_torque_ports[output_torque_port_i].output_port = new BufferedPort<Bottle>;
            output_torque_ports[output_torque_port_i].output_port->open(local_port);
            if( autoconnect && Network::exists(robot_port) )
            {
                Network::connect(local_port,robot_port,"tcp",false);
            }
        }
    }


    //Start with calibration
    if( !calibrateOffset("all",samples_requested_for_calibration) )
    {
        yError() << "Initial wholeBodyDynamicsThread::calibrateOffset failed";
        return false;
    }

    yInfo() << "wholeBodyDynamicsThread::threadInit finished successfully.";

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
        std::cout << "[ERR] wholeBodyDynamicsThread::decodeCalibCode failed" << std::endl;
        return false;
    }

    //Resetting the offset for the sensor being calibrated, to get the raw values
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        if( calibrate_ft_sensor[ft_id] )
        {
            double ft_off[6];
            //estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);

            yInfo() << "wholeBodyDynamicsThread::calibrateOffset: current calibration for FT " << ft_id << " is " <<
                         ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5];
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;

            //estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
        }
    }

    //Changing the base of the calibration model to the root link
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));


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
        for(int i = 0; i < (int)arms_fts.size(); i++ )
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
        for(int i = 0; i < (int)legs_fts.size(); i++ )
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
    return true;
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
            //estimator->getEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
            memcpy(ft_off,tree_status.ft_sensors_offset[ft_id].data(),sizeof(double)*6);


            yInfo() << "wholeBodyDynamicsThread::calibrateOffset: current calibration for FT " << ft_id << " is " <<
                         ft_off[0] << " " << ft_off[1] << " " << ft_off[2] << " " << ft_off[3] << " " << ft_off[4] << " " << ft_off[5];
            ft_off[0] = ft_off[1] = ft_off[2] = ft_off[3] = ft_off[4] = ft_off[5] = 0.0;
            //estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,convertFTiDynTreeToFTwbi(ft_id),ft_off);
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
            yInfo() << "wholeBodyDynamicsThread::calibrateOffset: current calibration for FT " << ft_id << " is "
                    << tree_status.ft_sensors_offset[ft_id].toString();
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

    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ )
    {
        if( calibrate_ft_sensor[ft_id] )
        {

            yInfo() << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport: current calibration for FT " << ft_id << " is "
                    << tree_status.ft_sensors_offset[ft_id].toString();
        }
    }

    //Changing the base of the calibration model to the left foot
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("r_sole"));

    calibration_mutex.lock();
    yInfo() << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport " << calib_code  << " called successfully, starting calibration.";
    wbd_mode = CALIBRATING;
    run_mutex.unlock();

    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::resetOffset(const std::string calib_code)
{
    run_mutex.lock();
    yInfo() << "wholeBodyDynamicsThread::resetOffset called with code " << calib_code;

    if( !this->decodeCalibCode(calib_code) )
    {
        return false;
    }

    //Resetting the offset
    for(int ft_id = 0; ft_id < (int)calibrate_ft_sensor.size(); ft_id++ ) {
        if( calibrate_ft_sensor[ft_id] ) {
            tree_status.ft_sensors_offset[ft_id] = 0.0;
        }
    }

    yInfo() << "wholeBodyDynamicsThread::resetOffset " << calib_code  << " called successfully.";

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
    yInfo() << "wholeBodyDynamicsThread::waitCalibrationDone() returning: calibration finished with success";
    return true;
}

void KDLWrenchFromRawValues(double * buf, KDL::Wrench & f)
{
    for(int i = 0; i < 6; i++ )
    {
        f[i] = buf[i];
    }

    return;
}

void KDLWrenchToRawValues(const KDL::Wrench & f, double * buf)
{
    for(int i = 0; i < 6; i++ )
    {
        buf[i] = f[i];
    }
    return;
}

//****************************************************************************
void wholeBodyDynamicsThread::getExternalWrenches()
{
    for(int i=0; i < output_wrench_ports.size(); i++ )
    {
        int link_id = output_wrench_ports[i].link_index;
        int frame_origin_id = output_wrench_ports[i].origin_frame_index;
        int frame_orientation_id = output_wrench_ports[i].orientation_frame_index;

        KDL::Wrench f =
            externalWrenchTorqueEstimator->robot_estimation_model->getExternalForceTorqueKDL(link_id,frame_origin_id,frame_orientation_id);

        // We can do that just because the translational-angular serialization
        // is the same in KDL and wbi
        KDLWrenchToRawValues(f,output_wrench_ports[i].output_vector.data());
    }

    return;
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishTorques()
{
    //Converting torques from the serialization used in wholeBodyStates interface to the port used by the robot
    for(int output_torque_port_id = 0;
        output_torque_port_id < (int)output_torque_ports.size();
        output_torque_port_id++ )
    {
        for(int output_vector_index = 0;
            output_vector_index < (int)output_torque_ports[output_torque_port_id].wbi_numeric_ids_to_publish.size();
            output_vector_index++)
        {
            int torque_wbi_numeric_id = output_torque_ports[output_torque_port_id].wbi_numeric_ids_to_publish[output_vector_index];
            output_torque_ports[output_torque_port_id].output_vector[output_vector_index] = all_torques[torque_wbi_numeric_id];
        }

        writeTorque(output_torque_ports[output_torque_port_id].output_vector,
                    output_torque_ports[output_torque_port_id].magic_number,
                    (output_torque_ports[output_torque_port_id].output_port));
    }

}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishContacts()
{
    broadcastData<skinContactList>(external_forces_list, port_contacts_output);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishExternalWrenches()
{
    for(int i=0; i < output_wrench_ports.size(); i++ )
    {
        broadcastData<Vector>(output_wrench_ports[i].output_vector,
                              output_wrench_ports[i].output_port);
    }
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishBaseToGui()
{

    iCubGuiBase.zero();

    // Workaround: if not root_link or left_foot is defined, do not publish base information to the iCubGui
    if( left_foot_link_idyntree_id != -1 &&
        root_link_idyntree_id != -1 )
    {
        //For the icubGui, the world is the root frame when q == 0
        //So we have to find the transformation between the root now
        //and the root when q == 0
        icub_model_calibration->setAng(tree_status.qj);

        // {}^{leftFoot} H_{currentRoot}
        KDL::Frame H_leftFoot_currentRoot
            = icub_model_calibration->getPositionKDL(left_foot_link_idyntree_id,root_link_idyntree_id);

        tree_status.qj.zero();
        icub_model_calibration->setAng(tree_status.qj);

        //{}^world H_{leftFoot}
        KDL::Frame H_world_leftFoot
            = icub_model_calibration->getPositionKDL(root_link_idyntree_id,left_foot_link_idyntree_id);

        KDL::Frame H_world_currentRoot
            = H_world_leftFoot*H_leftFoot_currentRoot;

        //Set angular part
        double roll,pitch,yaw;
        H_world_currentRoot.M.GetRPY(roll,pitch,yaw);
        //H_world_currentRoot.M.Inverse().GetRPY(roll,pitch,yaw);

        const double RAD2DEG = 180.0/(3.1415);

        iCubGuiBase[0] = RAD2DEG*roll;
        iCubGuiBase[1] = RAD2DEG*pitch;
        iCubGuiBase[2] = RAD2DEG*yaw;

        //Set linear part (iCubGui wants the root offset in millimeters)
        const double METERS2MILLIMETERS = 1000.0;
        iCubGuiBase[3] = METERS2MILLIMETERS*H_world_currentRoot.p(0);
        iCubGuiBase[4] = METERS2MILLIMETERS*H_world_currentRoot.p(1);
        iCubGuiBase[5] = METERS2MILLIMETERS*H_world_currentRoot.p(2);

        //Add offset to avoid lower forces to be hided by the floor
        iCubGuiBase[5] = iCubGuiBase[5] + 1000.0;
    }

    broadcastData<yarp::sig::Vector>(iCubGuiBase,port_icubgui_base);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishFilteredInertialForGravityCompensator()
{
    //bool ret;
    FilteredInertialForGravityComp.zero();

    for(int i=0; i < 6; i++ )
    {
        FilteredInertialForGravityComp[0+i] = tree_status.wbi_imu[4+i];
    }

    broadcastData<yarp::sig::Vector>(FilteredInertialForGravityComp,port_filtered_inertial);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishFilteredFTWithoutOffset()
{
    if( publish_filtered_ft )
    {
        int nr_of_ft = sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).size();
        for(int ft =0; ft < nr_of_ft; ft++ )
        {
            broadcastData<yarp::sig::Vector>(tree_status.estimated_ft_sensors[ft],port_filtered_ft[ft]);
        }
    }
}

//***************************************************************************
void wholeBodyDynamicsThread::readRobotStatus()
{
    //Don't wait to get a sensor measure
    bool wait = false;
    //Don't get timestamps
    double * stamps = NULL;

    // Get joint encoders position, velocities and accelerations
    sensors->readSensors(wbi::SENSOR_ENCODER_POS, tree_status.qj.data(), stamps, wait);
    sensors->readSensors(wbi::SENSOR_ENCODER_SPEED, tree_status.dqj.data(), stamps, wait);
    sensors->readSensors(wbi::SENSOR_ENCODER_ACCELERATION, tree_status.ddqj.data(), stamps, wait);

    // Get 6-Axis F/T sensors measure
    const IDList & available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    for(int ft_numeric = 0; ft_numeric < (int)available_ft_sensors.size(); ft_numeric++ )
    {
        int ft_index = ft_numeric;
        if( sensors->readSensor(SENSOR_FORCE_TORQUE, ft_numeric, tree_status.measured_ft_sensors[ft_numeric].data(), stamps , wait) ) {
            // Add a low pass filter here? \todo TODO
            tree_status.estimated_ft_sensors[ft_numeric] = tree_status.measured_ft_sensors[ft_numeric] - tree_status.ft_sensors_offset[ft_numeric]; /// remove offset
        } else {
            yError() << "wholeBodyDynamics: Error in reading F/T sensors, exiting";
        }
    }

    // Get IMU measure (for now only one IMU is considered)
    const IDList & available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
    for(int imu_numeric = 0; imu_numeric < (int) 1; imu_numeric++ )
    {
        int imu_index = imu_numeric;
        assert((int)IMUs.size() > imu_index );
        assert((int)IMUs[imu_index].size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
        if( sensors->readSensor(SENSOR_IMU, imu_numeric, tree_status.wbi_imu.data(), stamps, wait) )
        {
            // fill imu values
            for(int i=0; i < 3; i++ )
            {
                tree_status.proper_ddp_imu[i] = tree_status.wbi_imu[i+4];
                tree_status.omega_imu[i]      = tree_status.wbi_imu[i+7];
            }

            tree_status.proper_ddp_imu = filters->imuLinearAccelerationFilter->filt(tree_status.proper_ddp_imu);
            tree_status.omega_imu      = filters->imuAngularVelocityFilter->filt(tree_status.omega_imu);

            filters->imuAngularAccelerationFiltElement.data = tree_status.omega_imu;
            filters->imuAngularAccelerationFiltElement.time = yarp::os::Time::now();
            tree_status.domega_imu     = filters->imuAngularAccelerationFilt->estimate(filters->imuAngularAccelerationFiltElement);

        } else {
            yError() << "wholeBodyDynamicsTree : Error in reading IMU";
        }
    }

}


//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    run_mutex.lock();

    readRobotStatus();

    if( wbd_mode == NORMAL )
    {
        estimation_run();
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
void wholeBodyDynamicsThread::estimation_run()
{
    bool ret;

    // Get sensors informations
    this->readRobotStatus();

    //
    externalWrenchTorqueEstimator->estimateExternalWrenchAndInternalJoints(this->tree_status);

    //Get estimated torques
    assert(estimator->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE) == (int)all_torques.size());
    all_torques = tree_status.torquesj;

    //Get estimated external contacts
    external_forces_list = externalWrenchTorqueEstimator->estimatedLastSkinDynContacts;

    //Get estimated external ee wrenches
    getExternalWrenches();

    //Send torques
    publishTorques();

    //Send external contacts
    publishContacts();

    //Send external wrench estimates
    publishExternalWrenches();

    //Send base information to iCubGui
    publishBaseToGui();

    //Send filtered force torque sensor measurment, if requested
    publishFilteredFTWithoutOffset();

    //if normal mode, publish the
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

    if( printCountdown == 0 ) {

        double avgTime, stdDev, avgTimeUsed, stdDevUsed, period;
        period = getRate();

        getEstPeriod(avgTime, stdDev);
        getEstUsed(avgTimeUsed, stdDevUsed);

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
    icub_model_calibration->setAng(tree_status.qj);
    icub_model_calibration->setDAng(0.0*tree_status.dqj);
    icub_model_calibration->setD2Ang(0.0*tree_status.ddqj);

    icub_model_calibration->kinematicRNEA();
    icub_model_calibration->dynamicRNEA();

    //std::cout << "wholeBodyDynamicsThread::calibration_run(): F/T estimates computed" << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_run() : imu proper acceleration " << tree_status.proper_ddp_imu.toString() << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_run() : q " << tree_status.q.toString() << std::endl;

    for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
        if( calibrate_ft_sensor[ft_sensor_id] ) {

            //Get sensor estimated from model
            icub_model_calibration->getSensorMeasurement(ft_sensor_id,tree_status.model_ft_sensors[ft_sensor_id]);

            //Get sensor measure
            assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
            offset_buffer[ft_sensor_id] += tree_status.measured_ft_sensors[ft_sensor_id]-tree_status.model_ft_sensors[ft_sensor_id];
            //std::cout << "Estimated ft sensor " << ft_sensor_id << " : " << tree_status.estimated_ft_sensors[ft_sensor_id].toString() << std::endl;
            //std::cout << "Subchain mass : " << norm(tree_status.estimated_ft_sensors[ft_sensor_id].subVector(0,2))/norm( tree_status.proper_ddp_imu) << std::endl;
        }
    }

    samples_used_for_calibration++;


    if( samples_used_for_calibration >= samples_requested_for_calibration ) {
        //Calculating offset
        //std::cout.precision(20);
        yInfo() << "wholeBodyDynamicsThread: complete calibration at system time : " << getCurrentDateAndTime();
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            if( calibrate_ft_sensor[ft_sensor_id] ) {
                offset_buffer[ft_sensor_id] *= (1.0/(double)samples_used_for_calibration);
                assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);

                tree_status.ft_sensors_offset[ft_sensor_id] = offset_buffer[ft_sensor_id];

                wbi::ID sensor_name;
                sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).indexToID(ft_sensor_id,sensor_name);

                yInfo() << "wholeBodyDynamicsThread: new calibration for FT " << sensor_name.toString() << " is "
                        << tree_status.ft_sensors_offset[ft_sensor_id].toString();
            }
        }


        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        this->disableCalibration();


        icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));
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
    icub_model_calibration->setAng(tree_status.qj);
    icub_model_calibration->setDAng(0.0*tree_status.dqj);
    icub_model_calibration->setD2Ang(0.0*tree_status.ddqj);

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

                tree_status.ft_sensors_offset[ft_sensor_id] = offset_buffer[ft_sensor_id];

                wbi::ID sensor_name;
                sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).indexToID(ft_sensor_id,sensor_name);

                yInfo() << "wholeBodyDynamicsThread: new calibration for FT " << sensor_name.toString() << " is "
                        << tree_status.ft_sensors_offset[ft_sensor_id].toString();

            }
        }


        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            offset_buffer[ft_sensor_id].zero();
            calibrate_ft_sensor[ft_sensor_id] = false;
        }

        icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));
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

    if( this->autoconnect )
    {
        yInfo() << "Switching all controlboards that use estimation to stiff interaction mode";
        ensureJointsAreNotUsingTorqueEstimates();
    }
    else
    {
        yInfo() << "Autoconnect option not enabled";
    }

    externalWrenchTorqueEstimator->fini();

    delete externalWrenchTorqueEstimator;

    yInfo() << "Closing output torques ports";
    for(int output_torque_port_i = 0; output_torque_port_i < output_torque_ports.size(); output_torque_port_i++ )
    {
        closePort(output_torque_ports[output_torque_port_i].output_port);
    }

    yInfo() << "Closing contacts port";
    closePort(port_contacts_input);
    closePort(port_contacts_output);
    yInfo() << "Closing end effector wrenches port";
    closeExternalWrenchesPorts();

    yInfo() << "Closing iCubGui base port";
    closePort(port_icubgui_base);
    closePort(port_filtered_inertial);

    if( publish_filtered_ft )
    {
        for(int i =0; i <  (int)sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).size(); i++ )
        {
            closePort(port_filtered_ft[i]);
        }
        port_filtered_ft.resize(0);
    }

    delete icub_model_calibration;

    delete filters;

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

//*****************************************************************************
bool wholeBodyDynamicsThread::ensureJointsAreNotUsingTorqueEstimates()
{
    //For all joints for which we are estimating the torques, ensure
    //that we are not doing nothing wrong:
    //switch for all joints the interaction mode to INTERACTION_STIFF
    //if the joint is currently using the CM_TORQUE control mode, switch
    //it to CM_POSITION
    if( !openControlBoards() )
    {
        yError("wholeBodyDynamicsThread: error in opening controlboards");
        return false;
    }

    int torque_list_size = sensors->getSensorList(wbi::SENSOR_ENCODER).size();
    for( int jnt = 0; jnt < torque_list_size; jnt++ )
    {
        int ctrlBrd = torqueEstimationControlBoards.controlBoardAxisList[jnt].first;
        int axis    =  torqueEstimationControlBoards.controlBoardAxisList[jnt].second;

        // Check control mode
        int ctrlMode = -1;
        bool ret;
        ret = torqueEstimationControlBoards.controlModeInterfaces[ctrlBrd]->getControlMode(axis,&ctrlMode);

        if( !ret || ctrlMode == -1 )
        {
            yError("wholeBodyDynamicsThread: error in calling getControlMode for controlboard %s of robot %s",
                   torqueEstimationControlBoards.controlBoardNames[ctrlBrd].c_str(),robotName.c_str());
        }

        // if the ctrlMode is torque, switch to position
        if( ctrlMode ==  VOCAB_CM_TORQUE)
        {
            ret = torqueEstimationControlBoards.controlModeInterfaces[ctrlBrd]->setControlMode(axis,VOCAB_CM_POSITION);

            if( !ret )
            {
                yError("wholeBodyDynamicsThread: error in calling setControlMode");
            }
        }

        // In any case, set the joint in interaction mode STIFF
        torqueEstimationControlBoards.interactionModeInterfaces[ctrlBrd]->setInteractionMode(axis,yarp::dev::VOCAB_IM_STIFF);

        if( !ret )
        {
            yError("wholeBodyDynamicsThread: error in calling setInteractionMode");
        }
    }

    closeControlBoards();
}

bool wholeBodyDynamicsThread::openControlBoards()
{
    for(int ctrlBrd = 0; ctrlBrd < torqueEstimationControlBoards.controlBoardNames.size(); ctrlBrd++ )
    {
        if( !openPolyDriver("wholeBodyDynamicsCloseControlBoards",
                             robotName,
                             torqueEstimationControlBoards.deviceDrivers[ctrlBrd],
                             torqueEstimationControlBoards.controlBoardNames[ctrlBrd]) )
        {
            yError() << "Error in opening " << torqueEstimationControlBoards.controlBoardNames[ctrlBrd]
                   << " of robot " << robotName;
            closeControlBoards();
            return false;
        }

        bool ret = torqueEstimationControlBoards.deviceDrivers[ctrlBrd]->view(torqueEstimationControlBoards.controlModeInterfaces[ctrlBrd]);
        ret = ret && torqueEstimationControlBoards.deviceDrivers[ctrlBrd]->view(torqueEstimationControlBoards.interactionModeInterfaces[ctrlBrd]);

        if( !ret )
        {
            yError() << "Error in opening " << torqueEstimationControlBoards.controlBoardNames[ctrlBrd]
                   << " of robot " << robotName;
            closeControlBoards();
            return false;
        }
    }

    return true;
}

bool wholeBodyDynamicsThread::closeControlBoards()
{
    for(int ctrlBrd = 0; ctrlBrd < torqueEstimationControlBoards.controlBoardNames.size(); ctrlBrd++ )
    {
        closePolyDriver(torqueEstimationControlBoards.deviceDrivers[ctrlBrd]);
        torqueEstimationControlBoards.controlModeInterfaces[ctrlBrd] = 0;
        torqueEstimationControlBoards.interactionModeInterfaces[ctrlBrd] = 0;
    }
}

wholeBodyDynamicsFilters::wholeBodyDynamicsFilters(int nrOfDOFs, int nrOfFTSensors, double cutoffInHz, double periodInSeconds)
{
    // Options

    ///< Window lengths of adaptive window filters
    int dqFiltWL            = 16;
    int d2qFiltWL           = 25;

    int imuAngularAccelerationFiltWL = 25;

    ///< Threshold of adaptive window filters
    double dqFiltTh            = 1.0;
    double d2qFiltTh           = 1.0;

    double imuAngularAccelerationFiltTh = 1.0;

    ///< Cut frequencies
    double tauJCutFrequency    =   cutoffInHz;

    double imuLinearAccelerationCutFrequency = cutoffInHz;
    double imuAngularVelocityCutFrequency    = cutoffInHz;
    double forcetorqueCutFrequency           = cutoffInHz;

    //

    ///< create derivative filters
    dqFilt = new iCub::ctrl::AWLinEstimator(dqFiltWL, dqFiltTh);
    d2qFilt = new iCub::ctrl::AWQuadEstimator(d2qFiltWL, d2qFiltTh);

    ///< create low pass filters
    yarp::sig::Vector dofZeros(nrOfDOFs,0.0);
    tauJFilt    = new iCub::ctrl::realTime::FirstOrderLowPassFilter(tauJCutFrequency, periodInSeconds, dofZeros);

    yarp::sig::Vector sixZeros(6,0.0);
    forcetorqueFilters.resize(nrOfFTSensors);
    for(int ft_numeric = 0; ft_numeric < nrOfFTSensors; ft_numeric++ )
    {
        forcetorqueFilters[ft_numeric] = new iCub::ctrl::realTime::FirstOrderLowPassFilter(forcetorqueCutFrequency,periodInSeconds,sixZeros); ///< low pass filter
    }

    yarp::sig::Vector threeZeros(6,0.0);
    imuLinearAccelerationFilter = new iCub::ctrl::realTime::FirstOrderLowPassFilter(imuLinearAccelerationCutFrequency,periodInSeconds,threeZeros);  ///< linear acceleration is filtered with a low pass filter
    imuAngularVelocityFilter = new iCub::ctrl::realTime::FirstOrderLowPassFilter(imuAngularVelocityCutFrequency,periodInSeconds,threeZeros);  ///< angular velocity is filtered with a low pass filter

     //Allocating a filter for angular acceleration estimation only for IMU used in iDynTree
    imuAngularAccelerationFilt = new iCub::ctrl::AWLinEstimator(imuAngularAccelerationFiltWL, imuAngularAccelerationFiltTh);
}

template <class T> void deleteObject(T** pp)
{
    T* p = *pp;
    if( p != 0 )
    {
        delete p;
        p = 0;
    }
}

wholeBodyDynamicsFilters::~wholeBodyDynamicsFilters()
{
    deleteObject(&dqFilt);
    deleteObject(&d2qFilt);
    deleteObject(&tauJFilt);

    for(int ft=0; ft < forcetorqueFilters.size(); ft++)
    {
        deleteObject(&(forcetorqueFilters[ft]));
    }

    deleteObject(&imuLinearAccelerationFilter);
    deleteObject(&imuAngularVelocityFilter);
}
