/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
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

#include "wholeBodyDynamicsTree/wholeBodyDynamicsStatesInterface.h"

#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include <iCub/skinDynLib/common.h>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <string>
#include <iostream>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <kdl/frames_io.hpp>

#define INITIAL_TIMESTAMP -1000.0


using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY DYNAMICS ESTIMATOR
// *********************************************************************************************************************
// *********************************************************************************************************************
ExternalWrenchesAndTorquesEstimator::ExternalWrenchesAndTorquesEstimator(int _period,
                                                               yarpWholeBodySensors *_sensors,
                                                               yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * _port_skin_contacts,
                                                               yarp::os::Property & _wbi_yarp_conf
                                                              )
:  periodInMilliSeconds(_period),
   sensors(_sensors),
   port_skin_contacts(_port_skin_contacts),
   enable_omega_domega_IMU(false),
   min_taxel(0),
   wbi_yarp_conf(_wbi_yarp_conf),
   assume_fixed_base_from_odometry(false)
{

    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
    resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));


    ///< Skin timestamp
    last_reading_skin_contact_list_Stamp = -1000.0;

    if( _wbi_yarp_conf.check("fixed_base") )
    {
        assume_fixed_base = true;
        fixed_link = _wbi_yarp_conf.find("fixed_base").asString();
    }
    else
    {
        assume_fixed_base = false;
    }

    if( _wbi_yarp_conf.check("assume_fixed_from_odometry") )
    {
        this->assume_fixed_base_from_odometry = true;
    }
}

bool ExternalWrenchesAndTorquesEstimator::init()
{
    resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
    resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
    resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

    //Allocation model
    if(  !wbi_yarp_conf.check("urdf") && !wbi_yarp_conf.check("urdf_file") )
    {
        std::cerr << "yarpWholeBodyModel error: urdf not found in configuration files" << std::endl;
        return false;
    }

    std::string urdf_file;
    if( wbi_yarp_conf.check("urdf") )
    {
        urdf_file = wbi_yarp_conf.find("urdf").asString().c_str();
    }
    else
    {
        urdf_file = wbi_yarp_conf.find("urdf_file").asString().c_str();
    }

    yarp::os::ResourceFinder rf;
    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());

    std::vector<std::string> dof_serialization;
    IDList torque_estimation_list = sensors->getSensorList(SENSOR_ENCODER);
    for(int dof=0; dof < (int)torque_estimation_list.size(); dof++)
    {
        ID wbi_id;
        torque_estimation_list.indexToID(dof,wbi_id);
        dof_serialization.push_back(wbi_id.toString());
    }

    std::vector<std::string> ft_serialization;
    IDList ft_sensor_list = sensors->getSensorList(SENSOR_FORCE_TORQUE);
    for(int ft=0; ft < (int)ft_sensor_list.size(); ft++)
    {
        ID wbi_id;
        ft_sensor_list.indexToID(ft,wbi_id);
        ft_serialization.push_back(wbi_id.toString());
    }

    {
        std::cerr << "[DEBUG] Create TorqueEstimationTree with " << ft_serialization.size() << " ft sensors" << std::endl;
        if( !assume_fixed_base )
        {
            robot_estimation_model = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
        } else {
            robot_estimation_model = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,fixed_link);
        }

        if( this->assume_fixed_base_from_odometry )
        {
            robot_estimation_model_on_l_sole = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,"l_sole");
            robot_estimation_model_on_r_sole = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,"r_sole");
        }
    }
    //Load mapping from skinDynLib to iDynTree links from configuration files

    if( !this->wbi_yarp_conf.check("IDYNTREE_SKINDYNLIB_LINKS") )
    {
        std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: IDYNTREE_SKINDYNLIB_LINKS group not found in configuration files" << std::endl;
        return false;
    }

    yarp::os::Bottle & bot =  this->wbi_yarp_conf.findGroup("IDYNTREE_SKINDYNLIB_LINKS");
    for(int i=1; i < bot.size(); i++ )
    {
        yarp::os::Bottle * map_bot = bot.get(i).asList();
        if( map_bot->size() != 2 || map_bot->get(1).asList() == NULL ||
            map_bot->get(1).asList()->size() != 3 )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: IDYNTREE_SKINDYNLIB_LINKS group is malformed (" << map_bot->toString() << ")" << std::endl;
            return false;
        }
        std::string iDynTree_link_name = map_bot->get(0).asString();
        std::string iDynTree_skinFrame_name = map_bot->get(1).asList()->get(0).asString();
        int skinDynLib_body_part = map_bot->get(1).asList()->get(1).asInt();
        int skinDynLib_link_index = map_bot->get(1).asList()->get(2).asInt();
        bool ret_sdl = robot_estimation_model->addSkinDynLibAlias(iDynTree_link_name,iDynTree_skinFrame_name,skinDynLib_body_part,skinDynLib_link_index);
        if( this->assume_fixed_base_from_odometry )
        {
            robot_estimation_model_on_l_sole->addSkinDynLibAlias(iDynTree_link_name,iDynTree_skinFrame_name,skinDynLib_body_part,skinDynLib_link_index);
            robot_estimation_model_on_r_sole->addSkinDynLibAlias(iDynTree_link_name,iDynTree_skinFrame_name,skinDynLib_body_part,skinDynLib_link_index);
        }

        if( !ret_sdl )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: IDYNTREE_SKINDYNLIB_LINKS link " << iDynTree_link_name
                      << " and frame " << iDynTree_skinFrame_name << " and not found in urdf model" << std::endl;
            return false;
        }
    }
    std::cerr << std::endl << "[INFO] IDYNTREE_SKINDYNLIB_LINKS correctly loaded" << std::endl;


    //Load subtree information
    link2subtree.resize(robot_estimation_model->getNrOfLinks());

    if( !this->wbi_yarp_conf.check("WBD_SUBTREES") )
    {
        std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES group not found in configuration files" << std::endl;
        return false;
    }

    yarp::os::Bottle & wbd_subtrees_bot =  this->wbi_yarp_conf.findGroup("WBD_SUBTREES");


    for(int i=1; i < wbd_subtrees_bot.size(); i++ )
    {
        yarp::os::Bottle * subtree_bot = wbd_subtrees_bot.get(i).asList();
        if( subtree_bot->size() != 2
            || subtree_bot->get(1).asList() == NULL
            || subtree_bot->get(1).asList()->size() != 2
            || subtree_bot->get(1).asList()->get(0).asList() == NULL )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES group is malformed (" << subtree_bot->toString() << ")" << std::endl;
            return false;
        }


        TorqueEstimationSubtree subtree;

        std::string subtree_name = subtree_bot->get(0).asString();
        subtree.subtree_name = subtree_name;


        yarp::os::Bottle * subtree_links_bot = subtree_bot->get(1).asList()->get(0).asList();
        for(int l=0; l < subtree_links_bot->size(); l++ )
        {

            std::string link_name = subtree_links_bot->get(l).asString();
            int link_index = robot_estimation_model->getLinkIndex(link_name);
            if( link_index < 0 )
            {
                std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES link " << link_name << " not found in urdf model" << std::endl;
                return false;
            }
            subtree.links.push_back(link_name);
            subtree.links_numeric_ids.insert(link_index);

            int current_subtree = i-1;
            link2subtree[link_index] = current_subtree;

        }
        std::string default_contact_link_name = subtree_bot->get(1).asList()->get(1).asString();

        int default_contact_link_index = robot_estimation_model->getLinkIndex(default_contact_link_name);
        if( default_contact_link_index < 0 )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES link " << default_contact_link_name << " not found in urdf model" << std::endl;
            return false;
        }

        if( subtree.links_numeric_ids.find(default_contact_link_index) == subtree.links_numeric_ids.end() )
        {
            std::cerr << "[ERR] wholeBodyDynamicsStatesInterface error: WBD_SUBTREES link " << default_contact_link_name
                      << " was specified as default contact for subtree " << subtree.subtree_name
                      << " but it is not present in the subtree" << std::endl;
            return false;
        }

        subtree.default_contact_link = default_contact_link_index;

        torque_estimation_subtrees.push_back(subtree);
    }

    contacts_for_given_subtree.resize(torque_estimation_subtrees.size());

    std::cerr << "[DEBUG] robot_estimation_model->getSubTreeInternalDynamics().size() : " << robot_estimation_model->getSubTreeInternalDynamics().size() << std::endl;
    std::cerr << "[DEBUG] torque_estimation_subtrees.size(): " << torque_estimation_subtrees.size() << std::endl;
    YARP_ASSERT(robot_estimation_model->getSubTreeInternalDynamics().size() ==  torque_estimation_subtrees.size());

    std::cerr << "[INFO] WBD_SUBTREES correctly loaded with " << torque_estimation_subtrees.size() << "subtrees" << std::endl;

    IDList available_encoders = sensors->getSensorList(wbi::SENSOR_ENCODER);
    for(int i = 0; i < (int)available_encoders.size(); i++ )
    {
        ID enc;
        available_encoders.indexToID(i,enc);
        if(!(robot_estimation_model->getDOFIndex(enc.toString()) == i))
        {
            std::cerr << "[ERR] dof " << enc.toString() << " has ID " << robot_estimation_model->getDOFIndex(enc.toString())
                      << " in the dynamical model and id " << i << "in the wbi" << std::endl;
        }
        YARP_ASSERT((robot_estimation_model->getDOFIndex(enc.toString()) == i));
    }
    yDebug() << "ExternalWrenchesAndTorquesEstimator::init() terminated successfully";
    return true;
}

void ExternalWrenchesAndTorquesEstimator::estimateExternalWrenchAndInternalJoints(RobotJointStatus & joint_status, RobotSensorStatus & sensor_status)
{
    //Temporary workaround: wholeBodyDynamicsStatesInterface needs all the DOF present in the dynamical model
    if( sensors->getSensorNumber(wbi::SENSOR_ENCODER) != robot_estimation_model->getNrOfDOFs() )
    {
        IDList list = sensors->getSensorList(wbi::SENSOR_ENCODER);

        std::cerr << "Available encoders: " << list.toString() << std::endl;

           std::cerr << "yarpWholeBodyDynamicsEstimator::run() error: " <<
                  sensors->getSensorNumber(wbi::SENSOR_ENCODER) << " joint sensors are available, while  " <<
                  robot_estimation_model->getNrOfDOFs() << " joints are present in the model " << std::endl;
        assert(false);
        return;
    }

    ///< \todo improve robustness: what if a sensor dies or stop working? interface should warn the user
    {
        resizeAll(sensors->getSensorNumber(SENSOR_ENCODER));
        resizeFTs(sensors->getSensorNumber(SENSOR_FORCE_TORQUE));
        resizeIMUs(sensors->getSensorNumber(SENSOR_IMU));

        ///< Read encoders
        omega_used_IMU  = sensor_status.omega_imu;

        domega_used_IMU = sensor_status.domega_imu;

        ddp_used_IMU = sensor_status.proper_ddp_imu;

        ///< Read skin contacts
        readSkinContacts();

        ///< Estimate joint torque sensors from force/torque measurements
        estimateExternalForcesAndJointTorques(joint_status,sensor_status);

        ///< Filter obtained joint torque measures
        // \todo reintroduce the filter ?
        joint_status.setJointTorquesYARP(tauJ);// tauJFilt->filt(tauJ);  ///< low pass filter


    }

    return;
}

void ExternalWrenchesAndTorquesEstimator::readSkinContacts()
{
    skinContactList *scl = port_skin_contacts->read(false);
    if(scl)
    {
        //< \todo TODO check for envelope
        last_reading_skin_contact_list_Stamp = Time::now();
        if(scl->empty())   // if no skin contacts => leave the old contacts but reset pressure and contact list
        {
            //< \todo TODO this (using the last contacts if no contacts are detected) should be at subtree level, not at global level
            for(skinContactList::iterator it=skinContacts.begin(); it!=skinContacts.end(); it++)
            {
                it->setPressure(0.0);
                it->setActiveTaxels(0);
            }
            return;
        }

        //Probably source of crazy inefficiencies, here just to reach a working state as soon as possible \todo TODO
        map<BodyPart, skinContactList> contactsPerBp = scl->splitPerBodyPart();

        skinContacts.clear();

        // if there are more than 1 contact and less than 10 taxels are active then suppose zero moment
        for(map<BodyPart,skinContactList>::iterator it=contactsPerBp.begin(); it!=contactsPerBp.end(); it++)
        {
            for(skinContactList::iterator c=it->second.begin(); c!=it->second.end(); c++)
            {
                if( c->getActiveTaxels()<10 && it->second.size()>1)
                {
                    c->fixMoment();
                }

                //Insert a contact in skinContacts only if the number of taxel is greater than ActiveTaxels
                if( (int)c->getActiveTaxels() > min_taxel )
                {
                    skinContacts.insert(skinContacts.end(),*c);
                }
            }
            
        }

    }
    else if(Time::now()-last_reading_skin_contact_list_Stamp>SKIN_EVENTS_TIMEOUT && last_reading_skin_contact_list_Stamp!=0.0)
    {
        // if time is up, use default contact points \todo TODO 
        //std::cout << "Resetting skin contact for timeout" << std::endl;
        skinContacts.clear();
    }
  
    //std::cout << "skinContacts: " << skinContacts.toString() << std::endl;
    //std::cout << "dynContacts: " << dynContacts.toString() << std::endl;

    //At this point, in a way or the other skinContacts must have at least a valid contact for each subtree
    //If this is not true, we add a default contact for each subgraph
    dynContacts = skinContacts.toDynContactList();

    //std::cout << "dynContacts converted from skinContacts: " << dynContacts.toString() << std::endl;

    // std::cout << "dynContacts: " << std::endl;
    // std::cout << dynContacts.toString() << std::endl;

    for(int subtree=0; subtree < (int)contacts_for_given_subtree.size(); subtree++ )
    {
        contacts_for_given_subtree[subtree] = 0;
    }

    for(dynContactList::iterator it=dynContacts.begin();
        it != dynContacts.end(); it++ )
    {
        int skinDynLib_body_part = it->getBodyPart();
        int skinDynLib_link_index = it->getLinkNumber();
        int iDynTree_link_index = -1;
        int iDynTree_frame_index=  -1;
        bool skinDynLibID_found = robot_estimation_model->skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,iDynTree_link_index,iDynTree_frame_index);
        // \todo TODO FIXME properly address when you find an unexpcted contact id without crashing
        if( !skinDynLibID_found )
        {
            yError("wholeBodyDynamicsStatesInterface: unexpected contact from bodyPart %d link with local id %d",skinDynLib_body_part,skinDynLib_link_index);
        }
        else
        {
            std::string link_name;
            robot_estimation_model->getLinkName(iDynTree_link_index,link_name);
            //std::cout << "Found a contact from skin in link " << link_name << std::endl;
            contacts_for_given_subtree[link2subtree[iDynTree_link_index]]++;
        }
    }

    for(int subtree=0; subtree < (int)torque_estimation_subtrees.size(); subtree++ )
    {
        if( contacts_for_given_subtree[subtree] == 0 )
        {

            dynContact default_contact = this->getDefaultContact(subtree);
            dynContacts.push_back(default_contact);
        }
    }


}



dynContact ExternalWrenchesAndTorquesEstimator::getDefaultContact(const int subtree)
{
    int iDynTree_default_contact_link = torque_estimation_subtrees[subtree].default_contact_link;

    int skinDynLib_body_part = -1;
    int skinDynLib_link_index = -1;
    int iDynTree_default_contact_link_skinFrame;
    YARP_ASSERT(robot_estimation_model->getSkinDynLibAlias(iDynTree_default_contact_link,iDynTree_default_contact_link_skinFrame,skinDynLib_body_part,skinDynLib_link_index));
    YARP_ASSERT(skinDynLib_body_part != -1);
    YARP_ASSERT(skinDynLib_link_index != -1);
    dynContact return_value = dynContact();
    return_value.setBodyPart((iCub::skinDynLib::BodyPart)skinDynLib_body_part);
    return_value.setLinkNumber(skinDynLib_link_index);
    //std::cout << return_value.toString() << std::endl;
    return return_value;
}

void getEEWrench(const iCub::iDynTree::TorqueEstimationTree & icub_model,
                 const iCub::skinDynLib::dynContact & dyn_contact,
                 bool & contact_found,
                 yarp::sig::Vector & link_wrench,
                 yarp::sig::Vector & gripper_wrench,
                 int ee_frame_idyntree_id,
                 int link_idyntree_id)
{
    //std::cout << "getEEWRench " << std::endl;
    contact_found = true;
    KDL::Wrench f_gripper, f_link, f_contact;
    KDL::Vector COP;
    YarptoKDL(dyn_contact.getCoP(),COP);
    YarptoKDL(dyn_contact.getForceMoment(),f_contact);
    //std::cout << "f_contact " << f_contact << std::endl;
    KDL::Frame H_link_contact(COP);
    f_link = H_link_contact*f_contact;
    KDLtoYarp(f_link,link_wrench);
    yarp::sig::Matrix H_gripper_link_yarp = icub_model.getPosition(ee_frame_idyntree_id,link_idyntree_id);
    YARP_ASSERT(H_gripper_link_yarp.cols() == 4 &&
                H_gripper_link_yarp.rows() == 4);
    KDL::Frame H_gripper_link;
    YarptoKDL(H_gripper_link_yarp,H_gripper_link);
    f_gripper = H_gripper_link*f_link;
    KDLtoYarp(f_gripper,gripper_wrench);
}

void ExternalWrenchesAndTorquesEstimator::estimateExternalForcesAndJointTorques(RobotJointStatus & joint_status,
                                                                                RobotSensorStatus & sensor_status)
{
    //Assume that only a IMU is available

    /** \todo TODO check that serialization between wbi and iDynTree are the same */
    assert(omega_used_IMU.size() == 3);
    assert(domega_used_IMU.size() == 3);
    assert(ddp_used_IMU.size() == 3);

    if( !enable_omega_domega_IMU )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
    }

    double gravity = 9.8;

    if( assume_fixed_base )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
        ddp_used_IMU.zero();
        if( fixed_link == "root_link" )
        {
            ddp_used_IMU[2] = gravity;
        }
        else if(    fixed_link == "l_sole"
                 || fixed_link == "r_sole"
                 || fixed_link == "r_foot_dh_frame"
                 || fixed_link == "l_foot_dh_frame" )
        {
            ddp_used_IMU[0] = gravity;
        }
    }

    if( this->assume_fixed_base_from_odometry )
    {
        domega_used_IMU.zero();
        omega_used_IMU.zero();
        ddp_used_IMU.zero();
        ddp_used_IMU[2] = gravity;
    }

    assert((int)joint_status.getJointPosYARP().size() == robot_estimation_model->getNrOfDOFs());
    assert((int)joint_status.getJointVelYARP().size() == robot_estimation_model->getNrOfDOFs());
    assert((int)joint_status.getJointAccYARP().size() == robot_estimation_model->getNrOfDOFs());


    YARP_ASSERT(omega_used_IMU.size() == 3);
    YARP_ASSERT(domega_used_IMU.size() == 3);
    YARP_ASSERT(ddp_used_IMU.size() == 3);
    if( !assume_fixed_base_from_odometry )
    {
        bool ok = robot_estimation_model->setInertialMeasure(omega_used_IMU,domega_used_IMU,ddp_used_IMU);
        robot_estimation_model->setAng(joint_status.getJointPosYARP());
        robot_estimation_model->setDAng(joint_status.getJointVelYARP());
        robot_estimation_model->setD2Ang(joint_status.getJointAccYARP());

        for(int i=0; i < robot_estimation_model->getNrOfFTSensors(); i++ ) {
            assert(sensor_status.estimated_ft_sensors[i].size() == 6);
            ok  = ok && robot_estimation_model->setSensorMeasurement(i,sensor_status.estimated_ft_sensors[i]);
        }
        robot_estimation_model->setContacts(dynContacts);

        ok = ok && robot_estimation_model->kinematicRNEA();
        ok = ok && robot_estimation_model->estimateContactForcesFromSkin();
        ok = ok && robot_estimation_model->dynamicRNEA();
        ok = ok && robot_estimation_model->computePositions();

        if( !ok )
        {
            yWarning() << "wholeBodyDynamics: external forces computation and torque estimation failed";
        }

        estimatedLastDynContacts = robot_estimation_model->getContacts();
    }

    if( assume_fixed_base_from_odometry )
    {
        if( this->current_fixed_link_name == "r_foot" )
        {
            bool ok = robot_estimation_model_on_r_sole->setInertialMeasure(omega_used_IMU,domega_used_IMU,ddp_used_IMU);
            robot_estimation_model_on_r_sole->setAng(joint_status.getJointPosYARP());
            robot_estimation_model_on_r_sole->setDAng(joint_status.getJointVelYARP());
            robot_estimation_model_on_r_sole->setD2Ang(joint_status.getJointAccYARP());

            for(int i=0; i < robot_estimation_model_on_r_sole->getNrOfFTSensors(); i++ ) {
                assert(sensor_status.estimated_ft_sensors[i].size() == 6);
                ok  = ok && robot_estimation_model_on_r_sole->setSensorMeasurement(i,sensor_status.estimated_ft_sensors[i]);
            }
            robot_estimation_model_on_r_sole->setContacts(dynContacts);

            ok = ok && robot_estimation_model_on_r_sole->kinematicRNEA();
            ok = ok && robot_estimation_model_on_r_sole->estimateContactForcesFromSkin();
            ok = ok && robot_estimation_model_on_r_sole->dynamicRNEA();
            ok = ok && robot_estimation_model_on_r_sole->computePositions();

            if( !ok )
            {
                yError() << "wholeBodyDynamics: external forces computation and torque estimation failed";
            }

            estimatedLastDynContacts = robot_estimation_model_on_r_sole->getContacts();
        }

        if( this->current_fixed_link_name == "l_foot" )
        {
            bool ok = robot_estimation_model_on_l_sole->setInertialMeasure(omega_used_IMU,domega_used_IMU,ddp_used_IMU);
            robot_estimation_model_on_l_sole->setAng(joint_status.getJointPosYARP());
            robot_estimation_model_on_l_sole->setDAng(joint_status.getJointVelYARP());
            robot_estimation_model_on_l_sole->setD2Ang(joint_status.getJointAccYARP());

            for(int i=0; i < robot_estimation_model_on_l_sole->getNrOfFTSensors(); i++ ) {
                assert(sensor_status.estimated_ft_sensors[i].size() == 6);
                ok  = ok && robot_estimation_model_on_l_sole->setSensorMeasurement(i,sensor_status.estimated_ft_sensors[i]);
            }
            robot_estimation_model_on_l_sole->setContacts(dynContacts);

            ok = ok && robot_estimation_model_on_l_sole->kinematicRNEA();
            ok = ok && robot_estimation_model_on_l_sole->estimateContactForcesFromSkin();
            ok = ok && robot_estimation_model_on_l_sole->dynamicRNEA();
            ok = ok && robot_estimation_model_on_l_sole->computePositions();

            if( !ok )
            {
                yError() << "wholeBodyDynamics: external forces computation and torque estimation failed";
            }

            estimatedLastDynContacts = robot_estimation_model_on_l_sole->getContacts();
        }
    }

    //Create estimatedLastSkinDynContacts using original skinContacts list read from skinManager
    // for each dynContact find the related skinContact (if any) and set the wrench in it
    unsigned long cId;
    bool contactFound=false;

    estimatedLastSkinDynContacts = skinContacts;

    for(unsigned int i=0; i < estimatedLastDynContacts.size(); i++)
    {
        cId = estimatedLastDynContacts[i].getId();
        for(unsigned int j=0; j<estimatedLastSkinDynContacts.size(); j++)
        {
            if(cId == estimatedLastSkinDynContacts[j].getId())
            {
                estimatedLastSkinDynContacts[j].setForceMoment( estimatedLastDynContacts[i].getForceMoment() );
                contactFound = true;
                j = estimatedLastSkinDynContacts.size();    // break from the inside for loop
            }
        }

        // if there is no associated skin contact, create one
        if(!contactFound)
            estimatedLastSkinDynContacts.push_back(skinContact(estimatedLastDynContacts[i]));
        contactFound = false;

    }



    assert((int)tauJ.size() == robot_estimation_model->getNrOfDOFs());
    if( !this->assume_fixed_base_from_odometry )
    {
        tauJ = robot_estimation_model->getTorques();
    }

    if( this->assume_fixed_base_from_odometry )
    {
        if( this->current_fixed_link_name == "r_foot" )
        {
            tauJ = robot_estimation_model_on_r_sole->getTorques();
        }

        if( this->current_fixed_link_name == "l_foot" )
        {
            tauJ = robot_estimation_model_on_l_sole->getTorques();
        }
    }

}


void deleteFirstOrderFilterVector(std::vector<iCub::ctrl::FirstOrderLowPassFilter *> & vec)
{
    for(int i=0; i < (int)vec.size(); i++ ) {
        if( vec[i]!= 0 ) { delete vec[i]; vec[i]=0; }
    }
    vec.resize(0);
}

void ExternalWrenchesAndTorquesEstimator::fini()
{

    return;
}

void ExternalWrenchesAndTorquesEstimator::resizeAll(int n)
{
    q.resize(n,0.0);
    qStamps.resize(n,INITIAL_TIMESTAMP);
    tauJ.resize(n,0.0);
    tauJStamps.resize(n,INITIAL_TIMESTAMP);
}


void ExternalWrenchesAndTorquesEstimator::resizeFTs(int n)
{
    forcetorques.resize(n,Vector(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0));
    forcetorquesStamps.resize(n,INITIAL_TIMESTAMP);
}


void ExternalWrenchesAndTorquesEstimator::resizeIMUs(int n)
{
    IMUs.resize(n,Vector(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0));
    IMUStamps.resize(n,INITIAL_TIMESTAMP);
}

void copyVector(const yarp::sig::Vector & src, double * dest)
{
    memcpy(dest,src.data(),src.size()*sizeof(double));
}

bool ExternalWrenchesAndTorquesEstimator::setEnableOmegaDomegaIMU(bool _enabled_omega_domega_IMU)
{
    enable_omega_domega_IMU = _enabled_omega_domega_IMU;
    return true;
}

bool ExternalWrenchesAndTorquesEstimator::setMinTaxel(const int _min_taxel)
{
    if( _min_taxel < 0 )
    {
        return false;
    }
    min_taxel = _min_taxel;
    return true;
}
