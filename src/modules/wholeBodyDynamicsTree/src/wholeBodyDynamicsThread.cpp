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

// iDynTree includes
#include "iCub/iDynTree/yarp_kdl.h"

// Yarp includes
#include <yarp/os/Time.h>
#include <yarp/os/LockGuard.h>
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



//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name,
                                                 string _robotName,
                                                 int _period,
                                                 yarpWholeBodySensors *_wbs,
                                                 yarp::os::Property & _yarp_options,
                                                 bool _assume_fixed_base_calibration,
                                                 std::string _fixed_link_calibration,
                                                 bool _assume_fixed_base_calibration_from_odometry
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
       fixed_link_calibration(_fixed_link_calibration),
       assume_fixed_base_calibration_from_odometry(_assume_fixed_base_calibration_from_odometry),
       run_mutex_acquired(false),
       odometry_enabled(false)
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
    if( _yarp_options.check("verbose") )
    {
        rf.setVerbose();
    }

    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());



    std::vector<std::string> dof_serialization;

    // \todo TODO FIXME move IDList -> std::vector<std::string> conversion to wbiIdUtils
    IDList torque_estimation_list = sensors->getSensorList(wbi::SENSOR_ENCODER);
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
           icub_model_world_base_position = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,fixed_link_calibration);
       }
       else if( assume_fixed_base_calibration_from_odometry )
       {
           icub_model_calibration_on_l_sole = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,"l_sole");
           icub_model_calibration_on_r_sole = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,"r_sole");
           icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
       }
       else
       {
           icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
           icub_model_world_base_position = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
       }

    iCubGuiBase.resize(6);
    FilteredInertialForGravityComp.resize(6);
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
        yAssert(icub_model_calibration->getFTSensorIndex(ft_sensor_name) == numeric_id);
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
    for(unsigned i=0; i < output_wrench_ports.size(); i++ )
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

        if( this->assume_fixed_base_calibration_from_odometry )
        {
            yAssert(icub_model_calibration->getFrameIndex(output_wrench_ports[i].origin_frame) == icub_model_calibration_on_l_sole->getFrameIndex(output_wrench_ports[i].origin_frame));
            yAssert(icub_model_calibration->getFrameIndex(output_wrench_ports[i].origin_frame) == icub_model_calibration_on_r_sole->getFrameIndex(output_wrench_ports[i].origin_frame))
        }


        if( output_wrench_ports[i].origin_frame_index < 0 )
        {
            yError() << "Frame " << output_wrench_ports[i].origin_frame << " not found in the model.";
            return false;
        }


        output_wrench_ports[i].orientation_frame_index =
            icub_model_calibration->getFrameIndex(output_wrench_ports[i].orientation_frame);

        if( this->assume_fixed_base_calibration_from_odometry )
        {
            yAssert(icub_model_calibration->getFrameIndex(output_wrench_ports[i].orientation_frame) == icub_model_calibration_on_l_sole->getFrameIndex(output_wrench_ports[i].orientation_frame));
            yAssert(icub_model_calibration->getFrameIndex(output_wrench_ports[i].orientation_frame) == icub_model_calibration_on_r_sole->getFrameIndex(output_wrench_ports[i].orientation_frame))
        }


        if( output_wrench_ports[i].orientation_frame_index < 0 )
        {
            yError() << "Frame " << output_wrench_ports[i].orientation_frame_index << " not found in the model.";
            return false;
        }
    }

    return true;
}

bool wholeBodyDynamicsThread::openExternalWrenchesPorts()
{
	bool ok = true;
    for(unsigned int i = 0; i < output_wrench_ports.size(); i++ )
    {
        std::string port_name = output_wrench_ports[i].port_name;
        output_wrench_ports[i].output_port = new BufferedPort<Vector>;
        ok = ok && output_wrench_ports[i].output_port->open(port_name);
        output_wrench_ports[i].output_vector.resize(6);
    }
	return ok;
}

bool wholeBodyDynamicsThread::closeExternalWrenchesPorts()
{
    for(unsigned int i = 0; i < output_wrench_ports.size(); i++ )
    {
        this->closePort(output_wrench_ports[i].output_port);
    }
	return true;
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

    //Open skin ports
    port_contacts_input = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    port_contacts_input->open(string("/"+moduleName+"/skin_contacts:i").c_str());

    port_contacts_output = new BufferedPort<skinContactList>;
    port_contacts_output->open(string("/"+moduleName+"/contacts:o").c_str());

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

    smooth_calibration = yarp_options.check("smooth_calibration");
    if( smooth_calibration )
    {
        smooth_calibration_period_in_ms = yarp_options.find("smooth_calibration").asDouble();
        yInfo() << "Smooth calibration option enabled, with switching period of  " << smooth_calibration_period_in_ms << " milliseconds";
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
    joint_status.setNrOfDOFs(icub_model_calibration->getNrOfDOFs());
    sensor_status.setNrOfFTSensors(nrOfAvailableFTSensors);
    zero_dof_elem_vector.resize(icub_model_calibration->getNrOfDOFs(),0.0);
    zero_three_elem_vector.resize(3,0.0);
    calibration_ddp.resize(3,0.0);

    //Get list of ft sensors for calibration shortcut
    wbi::IDList ft_list = sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE);

    if( yarp_options.check("enable_w0_dw0") )
    {
        yAssert(false);

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
        yInfo() << "min_taxel option found, ignoring skin contacts with less than "
                  << taxel_threshold << " active taxels will be ignored.";
        externalWrenchTorqueEstimator->setMinTaxel(taxel_threshold);
    }
    else
    {
        int taxel_threshold = 0;
        externalWrenchTorqueEstimator->setMinTaxel(0);
    }

    this->autoconnect = false;
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
    //yAssert(root_link_idyntree_id >= 0 && root_link_idyntree_id < max_id );
    left_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("l_foot");
    //yAssert(left_foot_link_idyntree_id >= 0  && left_foot_link_idyntree_id < max_id);
    right_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("r_foot");
    //yAssert(right_foot_link_idyntree_id >= 0 && right_foot_link_idyntree_id < max_id);
    joint_status.zero();

    if( assume_fixed_base_calibration )
    {
        icubgui_support_frame_idyntree_id = icub_model_calibration->getLinkIndex(fixed_link_calibration);

        if( icubgui_support_frame_idyntree_id < 0 )
        {
            yError() << "Warning: unknown fixed link " << fixed_link_calibration;
            return false;
        }
    }
    else
    {
        icubgui_support_frame_idyntree_id = left_foot_link_idyntree_id;
    }

    icub_model_calibration->setAng(joint_status.getJointPosYARP());
    //{}^world H_{leftFoot}
    initial_world_H_supportFrame
            = icub_model_calibration->getPositionKDL(root_link_idyntree_id,icubgui_support_frame_idyntree_id);

    //Open and connect all the ports
    for(int output_torque_port_i = 0; output_torque_port_i < (int)output_torque_ports.size(); output_torque_port_i++ )
    {
        std::string port_name = output_torque_ports[output_torque_port_i].port_name;
        std::string local_port = "/" + moduleName + "/" + port_name + "/Torques:o";
        std::string robot_port = "/" + robotName  + "/joint_vsens/" + port_name + ":i";

    }

    // Open external wrenches ports
    openExternalWrenchesPorts();

    //Open port for iCubGui
    port_icubgui_base = new BufferedPort<Vector>;
    port_icubgui_base->open(string("/"+moduleName+"/base:o"));

    //Open port for filtered inertial
    port_filtered_inertial = new BufferedPort<Vector>;
    port_filtered_inertial->open(string("/"+moduleName+"/filtered/inertial:o"));

    if( this->publish_filtered_ft )
    {
        //Open ports for filtered ft
        IDList ft_estimation_list = sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE);
        port_filtered_ft.resize(ft_estimation_list.size());
        for(int i=0; i < (int)ft_estimation_list.size(); i++ )
        {
            ID ft_id;
            ft_estimation_list.indexToID(i,ft_id);
            port_filtered_ft[i] = new BufferedPort<Vector>;
            bool ok = port_filtered_ft[i]->open(string("/"+moduleName+"/filtered/"+ft_id.toString()+":o"));

            if( !ok )
            {
                yError() << "Error in opening port " << string("/"+moduleName+"/filtered/"+ft_id.toString()+":o") << ", closing";
                return false;
            }
        }
    }

    // Create filters
    // backward compatibility :
    // support cutoff as a synonym of cutoff_imu
    if( !yarp_options.check("cutoff_imu") && yarp_options.check("cutoff") )
    {
        yWarning() << "deprecated option cutoff found, please update your configuration file to use cutoff_imu";
        yarp_options.put("cutoff_imu",yarp_options.find("cutoff"));
    }

    if( !yarp_options.check("cutoff_imu") )
    {
        yError() << "cutoff_imu option not found, wholeBodyDynamicsTree initialization failed.";
        yError() << "please add the cutoff_imu option to the wholeBodyDynamicsTree configuration file";
        return false;
    }

    if( !yarp_options.find("cutoff_imu").isDouble() )
    {
        yError() << "cutoff_imu option found but not a double, wholeBodyDynamicsTree initialization failed.";
        yError() << "the cutoff_imu option is required to be a double.";
        return false;
    }

    double cutoffInHzIMU = yarp_options.find("cutoff_imu").asDouble();

    if( cutoffInHzIMU <= 0.0 )
    {
        yError() << "cutoff_imu option found but equal to " << cutoffInHzIMU;
        yError() << "the cutoff_imu option is required to be a positive frequency in hertz.";
        return false;
    }

    bool enableFTFiltering = false;
    double cutoffInHzFT = -1.0;
    if( yarp_options.check("cutoff_ft") )
    {
        if( !(yarp_options.find("cutoff_ft").isDouble()) ||
            !(yarp_options.find("cutoff_ft").asDouble() > 0.0) )
        {
            yError() << "cutoff_ft option found but invalid";
            yError() << "the cutoff_ft option is required to be a positive frequency in hertz.";
            return false;
        }

        enableFTFiltering = true;
        cutoffInHzFT = yarp_options.find("cutoff_ft").asDouble();
    }
    else
    {
        yInfo() << "cutoff_ft option not found, disabling FT filtering";
    }

    bool enableVelAccFiltering = false;
    double cutoffInHzVelAcc = -1.0;
    if( yarp_options.check("cutoff_velacc") )
    {
        if( !(yarp_options.find("cutoff_velacc").isDouble()) ||
            !(yarp_options.find("cutoff_velacc").asDouble() > 0.0) )
        {
            yError() << "cutoff_velacc option found but invalid";
            yError() << "the cutoff_velacc option is required to be a positive frequency in hertz.";
            return false;
        }

        enableVelAccFiltering = true;
        cutoffInHzVelAcc = yarp_options.find("cutoff_velacc").asDouble();
    }
    else
    {
        yInfo() << "cutoff_velacc option not found, disabling joint velocities and accelerations filtering";
    }

    double periodInSeconds = getRate()*1e-3;
    filters = new wholeBodyDynamicsFilters(torque_estimation_list.size(),
                                           nrOfAvailableFTSensors,
                                           cutoffInHzIMU,
                                           periodInSeconds,
                                           enableFTFiltering,
                                           cutoffInHzFT,
                                           enableVelAccFiltering,
                                           cutoffInHzVelAcc);

    if( smooth_calibration )
    {
        offset_smoother = new OffsetSmoother(nrOfAvailableFTSensors,smooth_calibration_period_in_ms/1000.0);
    }
    else
    {
        offset_smoother = 0;
    }


    if( this->autoconnect )
    {
        yInfo() << "wholeBodyDynamicsThread: autoconnect option enabled, autoconnecting.";
    }

    {
        for(unsigned int output_torque_port_i = 0; output_torque_port_i < output_torque_ports.size(); output_torque_port_i++ )
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

    ///////////////////////////////////
    /// Odometry initialization
    ///////////////////////////////////
    initOdometry();

    //Start with calibration
    first_calibration = true;
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

    if( this->assume_fixed_base_calibration_from_odometry )
    {
        icub_model_calibration_on_l_sole->setFloatingBaseLink(icub_model_calibration_on_l_sole->getLinkIndex(calibration_support_link));
        icub_model_calibration_on_r_sole->setFloatingBaseLink(icub_model_calibration_on_r_sole->getLinkIndex(calibration_support_link));
    }

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
    yarp::os::LockGuard guard(run_mutex);

    samples_requested_for_calibration= samples_to_use;
    std::cout << "wholeBodyDynamicsThread::calibrateOffsetOnDoubleSupport called with code " << calib_code << std::endl;
    if( samples_requested_for_calibration <= 0 )
    {
        yError() << "wholeBodyDynamicsThread::calibrateOffsetOnDoubleSupport error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples.";
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
                    << sensor_status.ft_sensors_offset[ft_id].toString();
        }
    }

    calibration_mutex.lock();
    yInfo() << "wholeBodyDynamicsThread::calibrateOffset " << calib_code  << " called successfully, starting calibration.";
    wbd_mode = CALIBRATING_ON_DOUBLE_SUPPORT;

    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport(const std::string calib_code, int samples_to_use)
{
    run_mutex.lock();
    samples_requested_for_calibration= samples_to_use;
    yInfo() << "wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport called with code " << calib_code;
    if( samples_requested_for_calibration <= 0 )
    {
        yError() << "wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples.";
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
                    << sensor_status.ft_sensors_offset[ft_id].toString();
        }
    }

     //Changing the base of the calibration model to the left foot
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("l_sole"));

    if( this->assume_fixed_base_calibration_from_odometry )
    {
        icub_model_calibration_on_l_sole->setFloatingBaseLink(icub_model_calibration_on_l_sole->getLinkIndex("l_sole"));
        icub_model_calibration_on_r_sole->setFloatingBaseLink(icub_model_calibration_on_r_sole->getLinkIndex("l_sole"));
    }

    calibration_mutex.lock();
    yInfo() << "wholeBodyDynamicsThread::calibrateOffsetOnLeftFootSingleSupport " << calib_code  << " called successfully, starting calibration.";
    wbd_mode = CALIBRATING;
    run_mutex.unlock();

    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport(const std::string calib_code, int samples_to_use)
{
    run_mutex.lock();
    samples_requested_for_calibration= samples_to_use;
    yInfo() << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport called with code " << calib_code;
    if( samples_requested_for_calibration <= 0 )
    {
        yError() << "wholeBodyDynamicsThread::calibrateOffsetOnRightFootSingleSupport error: requested calibration with a negative (" << samples_requested_for_calibration << ") number of samples.";
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
                    << sensor_status.ft_sensors_offset[ft_id].toString();
        }
    }

    //Changing the base of the calibration model to the left foot
    icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex("r_sole"));

    if( this->assume_fixed_base_calibration_from_odometry )
    {
        icub_model_calibration_on_l_sole->setFloatingBaseLink(icub_model_calibration_on_l_sole->getLinkIndex("r_sole"));
        icub_model_calibration_on_r_sole->setFloatingBaseLink(icub_model_calibration_on_r_sole->getLinkIndex("r_sole"));
    }

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
            sensor_status.ft_sensors_offset[ft_id] = 0.0;
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

////////////////////////////////////////////////////////////
//// Simple Legged Odometry
////////////////////////////////////////////////////////////
bool wholeBodyDynamicsThread::initOdometry()
{
    yarp::os::Bottle & odometry_group = yarp_options.findGroup("SIMPLE_LEGGED_ODOMETRY");

    if( odometry_group.isNull()  )
    {
        yInfo() << " SIMPLE_LEGGED_ODOMETRY group not found, odometry disabled";
        this->odometry_enabled = false;
        this->com_streaming_enabled = false;
        this->frames_streaming_enabled = false;
        return true;
    }

    if( !odometry_group.check("initial_world_frame") ||
        !odometry_group.check("initial_fixed_link") ||
        !odometry_group.check("floating_base_frame") ||
        !odometry_group.find("initial_world_frame").isString() ||
        !odometry_group.find("initial_fixed_link").isString() ||
        !odometry_group.find("floating_base_frame").isString() )
    {
        yError() << " SIMPLE_LEGGED_ODOMETRY group found but malformed, exiting";
        this->odometry_enabled = false;
        this->com_streaming_enabled = false;
        this->frames_streaming_enabled = false;
        return false;
    }

    std::string initial_world_frame = odometry_group.find("initial_world_frame").asString();
    std::string initial_fixed_link = odometry_group.find("initial_fixed_link").asString();
    std::string floating_base_frame = odometry_group.find("floating_base_frame").asString();

    // Loading additional options

    // Check if com should be streamed
    this->com_streaming_enabled = odometry_group.check("stream_com");

    // Check if additional frames (besides the floating base) should be streamed
    if( odometry_group.check("additional_frames") &&
        odometry_group.find("additional_frames").isList() )
    {
        this->frames_streaming_enabled = true;
    }
    else
    {
        this->frames_streaming_enabled = false;
    }

    // Allocate model
    KDL::CoDyCo::UndirectedTree undirected_tree = this->icub_model_calibration->getKDLUndirectedTree();
    bool ok = this->odometry_helper.init(undirected_tree,
                                         initial_world_frame,
                                         initial_fixed_link);
    this->current_fixed_link_name = initial_fixed_link;
    externalWrenchTorqueEstimator->current_fixed_link_name = initial_fixed_link;

    // Get floating base frame index
    this->odometry_floating_base_frame_index = odometry_helper.getDynTree().getFrameIndex(floating_base_frame);

    ok = ok && (this->odometry_floating_base_frame_index >= 0 &&
                this->odometry_floating_base_frame_index < odometry_helper.getDynTree().getNrOfFrames());

    if( !ok )
    {
        yError() << "Odometry initialization failed, please check your parameters";
        this->odometry_enabled = false;
        this->com_streaming_enabled = false;
        this->frames_streaming_enabled = false;
        return false;
    }

    yInfo() << " SIMPLE_LEGGED_ODOMETRY initialized with initial world frame coincident with "
           << initial_world_frame << " and initial fixed link " << initial_fixed_link;

    this->odometry_enabled = true;
    world_H_floatingbase.resize(4,4);
    floatingbase_twist.resize(6,0.0);
    floatingbase_acctwist.resize(6,0.0);

    // Get id of frames to stream
    if( this->frames_streaming_enabled )
    {
        yarp::os::Bottle * frames_bot = odometry_group.find("additional_frames").asList();
        yDebug() << "frames_bot " << frames_bot->toString();
        if( frames_bot == NULL ||
            frames_bot->isNull() )
        {
            yError("additional_frames list malformed");
            return false;
        }

        for(size_t i=0; i < frames_bot->size(); i++ )
        {
            std::string frame_name = frames_bot->get(i).asString();
            int frame_index = odometry_helper.getDynTree().getFrameIndex(frame_name);
            if( frame_index < 0 )
            {
                yError("additional_frames: frame %s not found in the model",frame_name.c_str());
                return false;
            }

            frames_to_stream.push_back(frame_name);
            frames_to_stream_indices.push_back(frame_index);
        }

        buffer_bottles.resize(frames_to_stream.size());

        for(size_t i=0; i < buffer_bottles.size(); i++ )
        {
            yInfo("wholeBodyDynamicsTree: streaming world position of frame %s",frames_to_stream[i].c_str());
            buffer_bottles[i].addList();
        }

        buffer_transform_matrix.resize(4,4);
    }

    // Open ports
    port_floatingbasestate = new BufferedPort<Bottle>;
    port_floatingbasestate->open(string("/"+moduleName+"/floatingbasestate:o"));

    if( this->com_streaming_enabled )
    {
        port_com = new BufferedPort<Vector>;
        port_com->open(string("/"+moduleName+"/com:o"));
    }

    if( this->frames_streaming_enabled )
    {
        port_frames = new BufferedPort<Property>;
        port_frames->open(string("/"+moduleName+"/frames:o"));
    }

    return true;
}

void wholeBodyDynamicsThread::publishOdometry()
{
    if( this->odometry_enabled )
    {
        // Read joint position, velocity and accelerations into the odometry helper model
        // This could be avoided by using the same geometric model
        // for odometry, force/torque estimation and sensor force/torque calibration
        odometry_helper.setJointsState(joint_status.getJointPosKDL(),
                                       joint_status.getJointVelKDL(),
                                       joint_status.getJointAccKDL());

        // Get floating base position in the world
        KDL::Frame world_H_floatingbase_kdl = odometry_helper.getWorldFrameTransform(this->odometry_floating_base_frame_index);

        // Publish the floating base position on the port
        KDLtoYarp_position(world_H_floatingbase_kdl,this->world_H_floatingbase);

        yarp::os::Bottle & bot = port_floatingbasestate->prepare();
        bot.clear();
        bot.addList().read(this->world_H_floatingbase);
        bot.addList().read(this->floatingbase_twist);
        bot.addList().read(this->floatingbase_acctwist);

        port_floatingbasestate->write();


        if( this->com_streaming_enabled )
        {
            // Stream com in world frame
            KDL::Vector com = odometry_helper.getDynTree().getCOMKDL();

            yarp::sig::Vector & com_to_send = port_com->prepare();
            com_to_send.resize(3);

            KDLtoYarp(com,com_to_send);

            port_com->write();
        }

        if( this->frames_streaming_enabled )
        {
            // Stream frames in a property (highly inefficient! clean as soon as possible)
            Property& output = port_frames->prepare();

            for(int i =0; i < frames_to_stream.size(); i++ )
            {
                KDL::Frame frame_to_publish = odometry_helper.getWorldFrameTransform(frames_to_stream_indices[i]);

                KDLtoYarp_position(frame_to_publish,buffer_transform_matrix);

                buffer_bottles[i].get(0).asList()->read(buffer_transform_matrix);
                output.put(frames_to_stream[i].c_str(),buffer_bottles[i].get(0));
            }

            port_frames->write();
        }

        // save the current link considered as fixed by the odometry
        current_fixed_link_name = odometry_helper.getCurrentFixedLink();
    }
}

void wholeBodyDynamicsThread::closeOdometry()
{
    if( this->odometry_enabled )
    {
        closePort(port_floatingbasestate);
    }

    if( this->frames_streaming_enabled )
    {
        closePort(port_frames);
    }

    if( this->com_streaming_enabled )
    {
        closePort(port_com);
    }
}



bool wholeBodyDynamicsThread::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link)
{
    bool ok = true;
    run_mutex.lock();
    if( this->odometry_enabled )
    {
        // Not setting the angle position because it should be setted in the run

        ok = odometry_helper.reset(initial_world_frame,initial_fixed_link);
        if( ok )
        {
            yInfo() << "SIMPLE_LEGGED_ODOMETRY reset to world "
                    << initial_world_frame << "and fixed link " << initial_fixed_link << " was successfull";
        }
        else
        {
            yError() << "SIMPLE_LEGGED_ODOMETRY reset to world "
                    << initial_world_frame << "and fixed link " << initial_fixed_link << " failed";
        }
    }
    else
    {
        ok = false;
    }
    run_mutex.unlock();
    return ok;
}

bool wholeBodyDynamicsThread::changeFixedLinkSimpleLeggedOdometry(const string& new_fixed_link)
{
    bool ok = true;
    run_mutex.lock();
    if( this->odometry_enabled )
    {
        // Not setting the angle position because it should be setted in the run

        ok = odometry_helper.changeFixedLink(new_fixed_link);
        if( ok )
        {
            yInfo() << "SIMPLE_LEGGED_ODOMETRY fixed link successfully changed to " << new_fixed_link;
            this->current_fixed_link_name = new_fixed_link;
            externalWrenchTorqueEstimator->current_fixed_link_name = new_fixed_link;
        }
        else
        {
            yError() << "SIMPLE_LEGGED_ODOMETRY fixed link change to " << new_fixed_link << "failed";
        }
    }
    else
    {
        yWarning() << "SIMPLE_LEGGED_ODOMETRY is disabled, changing fixed link failed";
        ok = false;
    }
    run_mutex.unlock();
    return ok;
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

        KDL::Wrench f;

        if( !this->assume_fixed_base_calibration_from_odometry  )
        {
            f = externalWrenchTorqueEstimator->robot_estimation_model->getExternalForceTorqueKDL(link_id,frame_origin_id,frame_orientation_id);
        }

        if( this->assume_fixed_base_calibration_from_odometry )
        {
            if( this->current_fixed_link_name == "r_foot" )
            {
                f = externalWrenchTorqueEstimator->robot_estimation_model_on_r_sole->getExternalForceTorqueKDL(link_id,frame_origin_id,frame_orientation_id);
            }
            else if( this->current_fixed_link_name == "l_foot" )
            {
                f = externalWrenchTorqueEstimator->robot_estimation_model_on_l_sole->getExternalForceTorqueKDL(link_id,frame_origin_id,frame_orientation_id);
            }
            else
            {
                yAssert(false);
            }
        }

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
            if( torque_wbi_numeric_id >= joint_status.getJointTorquesYARP().size() || torque_wbi_numeric_id < 0 )
            {
                //std::cerr << "Warning: tryng to access element " << torque_wbi_numeric_id << " of vector of size " << joint_status.getJointTorquesYARP().size() << std::endl;
            }
            else
            {
                output_torque_ports[output_torque_port_id].output_vector[output_vector_index] = joint_status.getJointTorquesYARP()[torque_wbi_numeric_id];
            }
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

    KDL::Frame world_H_rootLink;
    bool world_H_rootLink_computed = false;

    // If the odometry is enabled, we send to the iCubGui the root_link position obtained throught the odometry
    // \todo TODO this port published ad hoc should be subsituted with a port
    // monitor so we can connect the "standard" floating base port to the base
    // port of the iCubGui
    if( this->odometry_enabled && root_link_idyntree_id != -1 )
    {
        world_H_rootLink = this->odometry_helper.getWorldFrameTransform(root_link_idyntree_id);
        world_H_rootLink_computed = true;
    }

    // If the odometry is not enabled, we use to publish to the gui the world
    // the fixed link specified by the assume_fixed option . If assume_fixed is not
    // passed, we just assume the `l_sole` link as fixed
    if( (icubgui_support_frame_idyntree_id != -1 &&
        root_link_idyntree_id != -1) && !this->odometry_enabled )
    {
        //For the icubGui, the world is the root frame when q == 0
        //So we have to find the transformation between the root now
        //and the root when q == 0
        icub_model_world_base_position->setAngKDL(joint_status.getJointPosKDL());

        // {}^{supportFrame} H_{currentRoot}
        KDL::Frame H_supportFrame_currentRoot
            = icub_model_world_base_position->getPositionKDL(icubgui_support_frame_idyntree_id,root_link_idyntree_id);

        world_H_rootLink
            = initial_world_H_supportFrame*H_supportFrame_currentRoot;

        world_H_rootLink_computed = true;
    }

    //Set angular part
    if( world_H_rootLink_computed )
    {
        double roll,pitch,yaw;
        world_H_rootLink.M.GetRPY(roll,pitch,yaw);

        const double RAD2DEG = 180.0/(M_PI);

        iCubGuiBase[0] = RAD2DEG*roll;
        iCubGuiBase[1] = RAD2DEG*pitch;
        iCubGuiBase[2] = RAD2DEG*yaw;

        //Set linear part (iCubGui wants the root offset in millimeters)
        const double METERS2MILLIMETERS = 1000.0;
        iCubGuiBase[3] = METERS2MILLIMETERS*world_H_rootLink.p(0);
        iCubGuiBase[4] = METERS2MILLIMETERS*world_H_rootLink.p(1);
        iCubGuiBase[5] = METERS2MILLIMETERS*world_H_rootLink.p(2);

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
        FilteredInertialForGravityComp[0+i] = sensor_status.wbi_imu[4+i];
    }

    broadcastData<yarp::sig::Vector>(FilteredInertialForGravityComp,port_filtered_inertial);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishFilteredFTWithoutOffset()
{
    if( publish_filtered_ft )
    {
        size_t nr_of_ft = sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).size();
        yAssert(nr_of_ft == port_filtered_ft.size());
        yAssert(nr_of_ft == sensor_status.estimated_ft_sensors.size());
        for(int ft =0; ft < nr_of_ft; ft++ )
        {
            broadcastData<yarp::sig::Vector>(sensor_status.estimated_ft_sensors[ft],port_filtered_ft[ft]);
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
    sensors->readSensors(wbi::SENSOR_ENCODER_POS, joint_status.getJointPosKDL().data.data(), stamps, wait);
    sensors->readSensors(wbi::SENSOR_ENCODER_SPEED, joint_status.getJointVelKDL().data.data(), stamps, wait);
    sensors->readSensors(wbi::SENSOR_ENCODER_ACCELERATION, joint_status.getJointAccKDL().data.data(), stamps, wait);

    // Update yarp vectors
    joint_status.updateYarpBuffers();

    // if the user requested to filter the encoder speed and acceleration, we filter them
    if( filters->enableVelAccFiltering )
    {
        joint_status.getJointVelYARP() = filters->jointVelFilter->filt(joint_status.getJointVelYARP());
        joint_status.getJointAccYARP() = filters->jointAccFilter->filt(joint_status.getJointAccYARP());

        // As we modified the yarp buffers, we need to update the KDL ones.
        joint_status.updateKDLBuffers();
    }

    // Get 6-Axis F/T sensors measure
    const IDList & available_ft_sensors = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    for(int ft_numeric = 0; ft_numeric < (int)available_ft_sensors.size(); ft_numeric++ )
    {
        int ft_index = ft_numeric;
        if( sensors->readSensor(SENSOR_FORCE_TORQUE,
                                ft_numeric,
                                sensor_status.measured_ft_sensors[ft_numeric].data(),
                                stamps ,
                                wait) )
        {
            /// remove offset
            sensor_status.estimated_ft_sensors[ft_numeric] =
                sensor_status.measured_ft_sensors[ft_numeric] - sensor_status.ft_sensors_offset[ft_numeric];

            // if requested, enable filtering
            if( filters->enableFTFiltering )
            {
                sensor_status.estimated_ft_sensors[ft_numeric] =
                    filters->forcetorqueFilters[ft_numeric]->filt(sensor_status.estimated_ft_sensors[ft_numeric]);
            }
        } else {
            yError() << "wholeBodyDynamics: Error in reading F/T sensors, exiting";
        }
    }

    // Get IMU measure (for now only one IMU is considered)
    const IDList & available_imu_sensors = sensors->getSensorList(SENSOR_IMU);
    for(int imu_numeric = 0; imu_numeric < (int) 1; imu_numeric++ )
    {
        int imu_index = imu_numeric;
        assert( sensor_status.wbi_imu.size() == sensorTypeDescriptions[SENSOR_IMU].dataSize );
        if( sensors->readSensor(SENSOR_IMU, imu_numeric, sensor_status.wbi_imu.data(), stamps, wait) )
        {
            // fill imu values
            for(int i=0; i < 3; i++ )
            {
                sensor_status.proper_ddp_imu[i] = sensor_status.wbi_imu[i+4];
                sensor_status.omega_imu[i]      = sensor_status.wbi_imu[i+7];
            }

            yAssert(sensor_status.proper_ddp_imu.size() == 3);
            yAssert(sensor_status.omega_imu.size() == 3);

            sensor_status.proper_ddp_imu = filters->imuLinearAccelerationFilter->filt(sensor_status.proper_ddp_imu);
            sensor_status.omega_imu      = filters->imuAngularVelocityFilter->filt(sensor_status.omega_imu);

            yAssert(sensor_status.proper_ddp_imu.size() == 3);
            yAssert(sensor_status.omega_imu.size() == 3);

            filters->imuAngularAccelerationFiltElement.data = sensor_status.omega_imu;
            filters->imuAngularAccelerationFiltElement.time = yarp::os::Time::now();
            sensor_status.domega_imu     = filters->imuAngularAccelerationFilt->estimate(filters->imuAngularAccelerationFiltElement);

        } else {
            yError() << "wholeBodyDynamicsTree : Error in reading IMU";
        }
    }

}


//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    if( this->run_mutex_acquired )
    {
        yError() << "wholeBodyDynamicsTree: run_mutex already acquired at the beginning of run method.";
        yError() << "    this could cause some problems, please report an issue at https://github.com/robotology/codyco-modules/issues/new";
    }

    run_mutex.lock();
    this->run_mutex_acquired = true;
    readRobotStatus();

    // If doing smooth calibration, continue to stream torques
    // even when doing calibration
    if( wbd_mode == NORMAL || (smooth_calibration && !first_calibration) )
    {
        estimation_run();
    }

    if( wbd_mode == CALIBRATING )
    {
        calibration_run();
    }

    if( wbd_mode == CALIBRATING_ON_DOUBLE_SUPPORT )
    {
        calibration_on_double_support_run();
    }

    this->run_mutex_acquired = false;
    run_mutex.unlock();
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::estimation_run()
{
    bool ret;

    // Update smoothed offset
    if( this->smooth_calibration )
    {
        double now = yarp::os::Time::now();
        for(unsigned int i=0; i < this->sensor_status.ft_sensors_offset.size(); i++ )
        {
            this->offset_smoother->updateOffset(now,i,sensor_status.ft_sensors_offset[i]);
        }
    }

    // Estimate external forces and internal torques (torques will be saved in the joint_status)
    externalWrenchTorqueEstimator->estimateExternalWrenchAndInternalJoints(this->joint_status,
                                                                           this->sensor_status);


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

    //Compute odometry
    publishOdometry();

    //Send base information to iCubGui
    publishBaseToGui();

    //Send filtered inertia for gravity compensation
    publishFilteredInertialForGravityCompensator();

    //Send filtered force torque sensor measurment, if requested
    publishFilteredFTWithoutOffset();

    //if normal mode, publish the
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

    if( printCountdown == 0 ) {
       
        double avgTime, stdDev, avgTimeUsed, stdDevUsed;

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
void wholeBodyDynamicsThread::setNewFTOffset(const int ft_sensor_id, const yarp::sig::Vector & new_offset)
{
    if( !smooth_calibration )
    {
        sensor_status.ft_sensors_offset[ft_sensor_id] = new_offset;
    }
    else
    {
        offset_smoother->setNewOffset(yarp::os::Time::now(),ft_sensor_id,new_offset,sensor_status.ft_sensors_offset[ft_sensor_id]);
    }
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
        calibration_ddp[0] = 0.0;
        calibration_ddp[1] = 0.0;
        calibration_ddp[2] = 0.0;
        if( fixed_link_calibration == "root_link" )
        {
            calibration_ddp[2] = gravity;
        }
        else if(    fixed_link_calibration == "l_sole"
                 || fixed_link_calibration == "r_sole"
                 || fixed_link_calibration == "r_foot_dh_frame"
                 || fixed_link_calibration == "l_foot_dh_frame" )
        {
            calibration_ddp[0] = gravity;
        }
    }
    else
    {
        calibration_ddp[0] = sensor_status.wbi_imu[4];
        calibration_ddp[1] = sensor_status.wbi_imu[5];
        calibration_ddp[2] = sensor_status.wbi_imu[6];
    }

    //Estimating sensors
    yAssert(sensor_status.omega_imu.size() == 3);
    yAssert(sensor_status.domega_imu.size() == 3);
    yAssert(sensor_status.proper_ddp_imu.size() == 3);

    icub_model_calibration->setInertialMeasure(zero_three_elem_vector,zero_three_elem_vector,calibration_ddp);
    icub_model_calibration->setAngKDL(joint_status.getJointPosKDL());
    icub_model_calibration->setDAng(zero_dof_elem_vector);
    icub_model_calibration->setD2Ang(zero_dof_elem_vector);

    icub_model_calibration->kinematicRNEA();
    icub_model_calibration->dynamicRNEA();

    //std::cout << "wholeBodyDynamicsThread::calibration_run(): F/T estimates computed" << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_run() : imu proper acceleration " << tree_status.proper_ddp_imu.toString() << std::endl;
    //std::cout << "wholeBodyDynamicsThread::calibration_run() : q " << tree_status.q.toString() << std::endl;

    for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
        if( calibrate_ft_sensor[ft_sensor_id] ) {

            //Get sensor estimated from model
            icub_model_calibration->getSensorMeasurement(ft_sensor_id,sensor_status.model_ft_sensors[ft_sensor_id]);

            //Get sensor measure
            assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
            offset_buffer[ft_sensor_id] += sensor_status.measured_ft_sensors[ft_sensor_id]-sensor_status.model_ft_sensors[ft_sensor_id];
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

                this->setNewFTOffset(ft_sensor_id,offset_buffer[ft_sensor_id]);

                wbi::ID sensor_name;
                sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).indexToID(ft_sensor_id,sensor_name);

                yInfo() << "wholeBodyDynamicsThread: new calibration for FT " << sensor_name.toString() << " is "
                        << sensor_status.ft_sensors_offset[ft_sensor_id].toString();
            }
        }


        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        this->disableCalibration();


        icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));
        first_calibration = false;
        wbd_mode = NORMAL;
        calibration_mutex.unlock();
    }


}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::calibration_on_double_support_run()
{
    //std::cout << "wholeBodyDynamicsThread::calibration_on_double_support_run(): estimates obtained" << std::endl;

    bool ok = true;

    //Setting imu proper acceleration from measure (assuming omega e domega = 0)
    //acceleration are measures 4:6 (check wbi documentation)
    if( assume_fixed_base_calibration )
    {
        double gravity = 9.8;
        calibration_ddp[0] = 0.0;
        calibration_ddp[1] = 0.0;
        calibration_ddp[2] = 0.0;
        if( fixed_link_calibration == "root_link" )
        {
            calibration_ddp[2] = gravity;
        }
        else if(    fixed_link_calibration == "l_sole"
                 || fixed_link_calibration == "r_sole"
                 || fixed_link_calibration == "r_foot_dh_frame"
                 || fixed_link_calibration == "l_foot_dh_frame" )
        {
            calibration_ddp[0] = gravity;
        }
    }
    else if( this->assume_fixed_base_calibration_from_odometry )
    {
        double gravity = 9.8;
        calibration_ddp[0] = 0.0;
        calibration_ddp[1] = 0.0;
        calibration_ddp[2] = 9.8;
    }
    else
    {
        calibration_ddp[0] = sensor_status.wbi_imu[4];
        calibration_ddp[1] = sensor_status.wbi_imu[5];
        calibration_ddp[2] = sensor_status.wbi_imu[6];
    }

    //Estimating sensors
    yAssert(sensor_status.omega_imu.size() == 3);
    yAssert(sensor_status.domega_imu.size() == 3);
    yAssert(sensor_status.proper_ddp_imu.size() == 3);

    if( !this->assume_fixed_base_calibration_from_odometry )
    {
        icub_model_calibration->setInertialMeasure(zero_three_elem_vector,zero_three_elem_vector,calibration_ddp);
        icub_model_calibration->setAngKDL(joint_status.getJointPosKDL());
        icub_model_calibration->setDAng(zero_dof_elem_vector);
        icub_model_calibration->setD2Ang(zero_dof_elem_vector);

        ok = ok && icub_model_calibration->kinematicRNEA();
        ok = ok && icub_model_calibration->estimateDoubleSupportContactForce(left_foot_link_idyntree_id,right_foot_link_idyntree_id);
        ok = ok && icub_model_calibration->dynamicRNEA();
    }
    else
    {
        if( this->current_fixed_link_name == "r_foot" )
        {
            icub_model_calibration_on_r_sole->setInertialMeasure(zero_three_elem_vector,zero_three_elem_vector,calibration_ddp);
            icub_model_calibration_on_r_sole->setAngKDL(joint_status.getJointPosKDL());
            icub_model_calibration_on_r_sole->setDAng(zero_dof_elem_vector);
            icub_model_calibration_on_r_sole->setD2Ang(zero_dof_elem_vector);

            ok = ok && icub_model_calibration_on_r_sole->kinematicRNEA();
            ok = ok && icub_model_calibration_on_r_sole->estimateDoubleSupportContactForce(left_foot_link_idyntree_id,right_foot_link_idyntree_id);
            ok = ok && icub_model_calibration_on_r_sole->dynamicRNEA();
        }

        if( this->current_fixed_link_name == "l_foot" )
        {
            icub_model_calibration_on_l_sole->setInertialMeasure(zero_three_elem_vector,zero_three_elem_vector,calibration_ddp);
            icub_model_calibration_on_l_sole->setAngKDL(joint_status.getJointPosKDL());
            icub_model_calibration_on_l_sole->setDAng(zero_dof_elem_vector);
            icub_model_calibration_on_l_sole->setD2Ang(zero_dof_elem_vector);

            ok = ok && icub_model_calibration_on_l_sole->kinematicRNEA();
            ok = ok && icub_model_calibration_on_l_sole->estimateDoubleSupportContactForce(left_foot_link_idyntree_id,right_foot_link_idyntree_id);
            ok = ok && icub_model_calibration_on_l_sole->dynamicRNEA();
        }
    }


    // todo check that the residual forze is zero

    if( !ok )
    {
        yError() << "wholeBodyDynamicsThread::calibration_on_double_support_run(): offset estimation failed";
    }

    //Get known terms of the Newton-Euler equation

    for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ )
    {
        if( calibrate_ft_sensor[ft_sensor_id] )
        {
            //Get sensor estimated from model
            if( !this->assume_fixed_base_calibration_from_odometry )
            {
                icub_model_calibration->getSensorMeasurement(ft_sensor_id,sensor_status.model_ft_sensors[ft_sensor_id]);
            }
            else
            {
                if( this->current_fixed_link_name == "r_foot" )
                {
                    icub_model_calibration_on_r_sole->getSensorMeasurement(ft_sensor_id,sensor_status.model_ft_sensors[ft_sensor_id]);
                }

                if( this->current_fixed_link_name == "l_foot" )
                {
                    icub_model_calibration_on_l_sole->getSensorMeasurement(ft_sensor_id,sensor_status.model_ft_sensors[ft_sensor_id]);
                }
            }

            //Get sensor measure
            assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
            offset_buffer[ft_sensor_id] += sensor_status.measured_ft_sensors[ft_sensor_id]-sensor_status.model_ft_sensors[ft_sensor_id];
        }
    }

    samples_used_for_calibration++;


    if( samples_used_for_calibration >= samples_requested_for_calibration ) {
        //Calculating offset
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            if( calibrate_ft_sensor[ft_sensor_id] ) {
                offset_buffer[ft_sensor_id] *= (1.0/(double)samples_used_for_calibration);
                assert((int)offset_buffer[ft_sensor_id].size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);

                this->setNewFTOffset(ft_sensor_id,offset_buffer[ft_sensor_id]);

                wbi::ID sensor_name;
                sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).indexToID(ft_sensor_id,sensor_name);

                yInfo() << "wholeBodyDynamicsThread: new calibration for FT " << sensor_name.toString() << " is "
                        << sensor_status.ft_sensors_offset[ft_sensor_id].toString();

            }
        }


        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        for(int ft_sensor_id=0; ft_sensor_id < (int)offset_buffer.size(); ft_sensor_id++ ) {
            offset_buffer[ft_sensor_id].zero();
            calibrate_ft_sensor[ft_sensor_id] = false;
        }

        icub_model_calibration->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));

        if( this->assume_fixed_base_calibration_from_odometry )
        {
            icub_model_calibration_on_l_sole->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));
            icub_model_calibration_on_r_sole->setFloatingBaseLink(icub_model_calibration->getLinkIndex(calibration_support_link));
        }

        wbd_mode = NORMAL;
        calibration_mutex.unlock();
    }

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
    for(unsigned int output_torque_port_i = 0; output_torque_port_i < output_torque_ports.size(); output_torque_port_i++ )
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

    yInfo() << "Closing filtered inertial port";
    closePort(port_filtered_inertial);

    if( publish_filtered_ft )
    {
        for(int i =0; i <  (int)sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE).size(); i++ )
        {
            closePort(port_filtered_ft[i]);
        }
        port_filtered_ft.resize(0);
    }

    yInfo() << "Deleting icub models used for calibration";
    delete icub_model_calibration;

    if( this->assume_fixed_base_calibration_from_odometry )
    {
        delete icub_model_calibration_on_l_sole;
        delete icub_model_calibration_on_r_sole;
    }

    if( !this->assume_fixed_base_calibration_from_odometry )
    {
        delete icub_model_world_base_position;
    }

    yInfo() << "Deleting filters";
    delete filters;

    if( smooth_calibration )
    {
        delete offset_smoother;
    }

    yInfo() << "Closing odometry class";
    closeOdometry();

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
void wholeBodyDynamicsThread::writeTorque(const Vector& _values, int _address, BufferedPort<Bottle> *_port)
{
    /** \todo TODO avoid (as much as feasible) dynamic memory allocation */
    Bottle & a = _port->prepare();
    a.clear();
    a.addInt(_address);
    for(size_t i=0;i<_values.length();i++)
        a.addDouble(_values(i));
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

    return true;
}

bool wholeBodyDynamicsThread::openControlBoards()
{
    for(unsigned int ctrlBrd = 0; ctrlBrd < torqueEstimationControlBoards.controlBoardNames.size(); ctrlBrd++ )
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
    bool ok = true;
    for(int ctrlBrd = 0; ctrlBrd < torqueEstimationControlBoards.controlBoardNames.size(); ctrlBrd++ )
    {
        ok = ok && closePolyDriver(torqueEstimationControlBoards.deviceDrivers[ctrlBrd]);
        torqueEstimationControlBoards.controlModeInterfaces[ctrlBrd] = 0;
        torqueEstimationControlBoards.interactionModeInterfaces[ctrlBrd] = 0;
    }
    return ok;
}

wholeBodyDynamicsFilters::wholeBodyDynamicsFilters(int nrOfDOFs,
                                                   int nrOfFTSensors,
                                                   double cutoffInHzIMU,
                                                   double periodInSeconds,
                                                   bool _enableFTFiltering,
                                                   double cutoffInHzFT,
                                                   bool _enableVelAccFiltering,
                                                   double cutoffInHzVelAcc):
                                                   enableFTFiltering(_enableFTFiltering),
                                                   enableVelAccFiltering(_enableVelAccFiltering)
{
    // Options

    ///< Window lengths of adaptive window filters
    int imuAngularAccelerationFiltWL = 25;

    ///< Threshold of adaptive window filters
    double imuAngularAccelerationFiltTh = 1.0;

    // FT
    yarp::sig::Vector sixZeros(6,0.0);
    forcetorqueFilters.resize(nrOfFTSensors);
    for(int ft_numeric = 0; ft_numeric < nrOfFTSensors; ft_numeric++ )
    {
        if( this->enableFTFiltering )
        {
            forcetorqueFilters[ft_numeric] =
                new iCub::ctrl::realTime::FirstOrderLowPassFilter(cutoffInHzFT,periodInSeconds,sixZeros);
        }
        else
        {
            forcetorqueFilters[ft_numeric] = 0;
        }
    }

    // IMU
    yarp::sig::Vector threeZeros(3,0.0);
    imuLinearAccelerationFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(cutoffInHzIMU,periodInSeconds,threeZeros);
    imuAngularVelocityFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(cutoffInHzIMU,periodInSeconds,threeZeros);

     //Allocating a filter for angular acceleration estimation only for IMU used in iDynTree
    imuAngularAccelerationFilt =
        new iCub::ctrl::AWLinEstimator(imuAngularAccelerationFiltWL, imuAngularAccelerationFiltTh);

    // Vel Acc
    yarp::sig::Vector dofsZeros(nrOfDOFs,0.0);
    if( this->enableVelAccFiltering )
    {
        jointVelFilter =
            new iCub::ctrl::realTime::FirstOrderLowPassFilter(cutoffInHzVelAcc,periodInSeconds,dofsZeros);
        jointAccFilter =
            new iCub::ctrl::realTime::FirstOrderLowPassFilter(cutoffInHzVelAcc,periodInSeconds,dofsZeros);
    }
    else
    {
        jointVelFilter = 0;
        jointAccFilter = 0;
    }
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
    for(unsigned int ft=0; ft < forcetorqueFilters.size(); ft++)
    {
        deleteObject(&(forcetorqueFilters[ft]));
    }

    deleteObject(&imuLinearAccelerationFilter);
    deleteObject(&imuAngularVelocityFilter);
    deleteObject(&jointVelFilter);
    deleteObject(&jointAccFilter);
}

OffsetSmoother::OffsetSmoother(int nrOfFTSensors, double smoothingTimeInSeconds)
{
    this->reset(nrOfFTSensors, smoothingTimeInSeconds);
}

void OffsetSmoother::reset(int nrOfFTSensors, double smoothingTimeInSeconds)
{
    this->smooth_calibration_period_in_seconds = smoothingTimeInSeconds;
    old_offset.resize(nrOfFTSensors,yarp::sig::Vector(6,0.0));
    new_offset.resize(nrOfFTSensors,yarp::sig::Vector(6,0.0));
    this->initial_smoothing_time.resize(nrOfFTSensors,0.0);
    this->is_smoothing.resize(nrOfFTSensors,false);

}

void OffsetSmoother::setNewOffset(double current_time, unsigned int ft_id, const Vector& _new_offset, const Vector& _old_offset)
{
    assert(ft_id < this->old_offset.size());
    old_offset[ft_id] = _old_offset;
    new_offset[ft_id] = _new_offset;
    this->initial_smoothing_time[ft_id] = current_time;
    this->is_smoothing[ft_id]           = true;
}


void OffsetSmoother::updateOffset(const double current_time,
                                  const unsigned int ft_id, yarp::sig::Vector & used_offset)
{
    if( this->is_smoothing[ft_id] )
    {
        // \todo TODO deal with double overflow
        double time_since_calibration_in_seconds = current_time-this->initial_smoothing_time[ft_id];
        if( time_since_calibration_in_seconds > smooth_calibration_period_in_seconds )
        {
            used_offset = new_offset[ft_id];
            this->is_smoothing[ft_id] = false;
        }
        double progress = time_since_calibration_in_seconds/smooth_calibration_period_in_seconds;
        used_offset.resize(6);
        for( int i =0; i < 6; i++ )
        {
            used_offset[i] = old_offset[ft_id][i] + progress*(new_offset[ft_id][i]-old_offset[ft_id][i]);
        }
    }
}


