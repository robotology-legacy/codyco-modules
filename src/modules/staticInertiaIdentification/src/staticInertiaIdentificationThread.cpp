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

#include "staticInertiaIdentificationThread.h"
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/math/SVD.h>

#include <string.h>

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <kdl_format_io/urdf_export.hpp>

using namespace yarp::math;
using namespace yarp::sig;
using namespace wbiIcub;
using namespace std;


//************************************************************************************************************************
staticInertiaIdentificationThread::staticInertiaIdentificationThread(string _name,
                                                                     string _robotName,
                                                                     int _period,
                                                                     icubWholeBodyStatesLocal *_wbs,
                                                                     const iCub::iDynTree::iCubTree_version_tag _icub_version,
                                                                     yarp::os::Property _opts,
                                                                     bool autoconnect
                                                                    )
    :  RateThread(_period),
       name(_name),
       robotName(_robotName),
       estimator(_wbs),
       printCountdown(0),
       printPeriod(_period),
       icub_version(_icub_version),
       opts(_opts),
       icub_tree_model(_icub_version),
       poseCount(0),
       currentPoseSampleCount(0),
       maxNrOfSamplesForPose(10),
       is_static_sample(false),
       is_sample_a_new_pose(false),
       is_external_contact_sample(false),
       estimation_disabled_for_external_contact(false),
       timestamp_of_last_external_contact(-1000.0),
       velocity_static_threshold(0.0),
       position_norm_threshold(0.0)
{

    std::cout << "Launching staticInertiaIdentificationThread with name : " << _name << " and robotName " << _robotName << " and period " << _period << std::endl;
    std::cout << "          thread options " << opts.toString() << std::endl;

}

//*************************************************************************************************************************
/*
wbi::LocalId staticInertiaIdentificationThread::convertFTiDynTreeToFTwbi(int ft_sensor_id)
{
    LocalId lid;
    assert(ft_sensor_id >= 0 && ft_sensor_id <= icub_model_calibration.getNrOfFTSensors());
    if( icub_version.feet_ft ) {
        return ICUB_MAIN_FOOT_FTS.globalToLocalId(ft_sensor_id);
    } else {
        return ICUB_MAIN_FTS.globalToLocalId(ft_sensor_id);
    }
}
*/

bool staticInertiaIdentificationThread::configureRegressorGenerator()
{
    std::vector<std::string> ft_names;
    if( icub_version.feet_ft ) {
        ft_names.push_back("l_arm_ft_sensor");
        ft_names.push_back("r_arm_ft_sensor");
        ft_names.push_back("l_leg_ft_sensor");
        ft_names.push_back("l_foot_ft_sensor");
        ft_names.push_back("r_leg_ft_sensor");
        ft_names.push_back("r_foot_ft_sensor");
    } else {
        ft_names.push_back("l_arm_ft_sensor");
        ft_names.push_back("r_arm_ft_sensor");
        ft_names.push_back("l_leg_ft_sensor");
        ft_names.push_back("r_leg_ft_sensor");
    }

    std::vector<std::string> fake_names;
    fake_names.push_back("torso");
    fake_names.push_back("imu_frame");
    fake_names.push_back("l_gripper");
    fake_names.push_back("r_gripper");
    fake_names.push_back("l_sole");
    fake_names.push_back("r_sole");
    fake_names.push_back("l_wrist_1");
    fake_names.push_back("r_wrist_1");

    std::string kinematic_base = "imu_frame";

    KDL::Tree icub_kdl_tree = icub_tree_model.getKDLTree();
    KDL::CoDyCo::UndirectedTree icub_kdl_undirected_tree = icub_tree_model.getKDLUndirectedTree();
    KDL::CoDyCo::TreeSerialization icub_serialization =icub_tree_model.getKDLUndirectedTree().getSerialization();
    KDL::CoDyCo::FTSensorList icub_ft_list(icub_kdl_undirected_tree,ft_names);
    bool consider_sensor_offset = true;
    bool verbose = true;

    icub_regressor_generator = new KDL::CoDyCo::Regressors::DynamicRegressorGenerator(icub_kdl_tree,
                                                     kinematic_base,
                                                     ft_names,
                                                     consider_sensor_offset,
                                                     fake_names,
                                                     icub_serialization,
                                                     verbose);

    return true;
}

bool staticInertiaIdentificationThread::configureGeneratedRegressor()
{

    yarp::os::Bottle & subtrees = opts.findGroup("INERTIA_IDENTIFICATION_SUBTREES");

    if( subtrees.isNull() )
    {
        std::cout << "staticInertiaIdentification : no subtrees found in configuration files, exiting" << std::endl;
        return false;
    }

    for(int i=1; i < subtrees.size(); i++ )
    {
        yarp::os::Bottle * subtree = subtrees.get(i).asList();

        std::vector< std::string > new_subtree;

        subtrees_names.push_back(subtree->get(0).asString().c_str());

        yarp::os::Bottle * subtree_leafs = subtree->get(1).asList();

        for(int j=0; j < subtree_leafs->size(); j++ )
        {
            new_subtree.push_back(subtree_leafs->get(j).asString().c_str());
        }

        list_of_subtrees.push_back(new_subtree);
    }
    std::cout << "staticInertiaIdentification : found " << subtrees_names.size() << " subtrees in configuration files " << std::endl;
    for(int i=0; i < (int)list_of_subtrees.size(); i++ )
    {
        std::cout << "\tsubtree " << i << " : " << subtrees_names[i] << std::endl;
        for(int j=0; j < (int)list_of_subtrees[i].size(); j++ )
        {
            std::cout << "\t\t leaf " << j << " : " << list_of_subtrees[i][j] << std::endl;
        }
    }

    //Add selected regressor to the generator
    for(int i=0; i < list_of_subtrees.size(); i++ )
    {
        int ret = icub_regressor_generator->addSubtreeRegressorRows(list_of_subtrees[i]);
        if( ret != 0 )
        {
            std::cout << "staticInertiaIdentification : fatal error in adding " << subtrees_names[i] << " subtree, exiting" << std::endl;

            return false;
        }
    }


    std::cout << "staticInertiaIdentification : computing identifiable subspace using gautier algorithm. " << std::endl;
    //Compute static base parameter subspace
    bool consider_only_static_pose = true;
    int n_samples = 100;
    YARP_ASSERT(icub_regressor_generator->computeNumericalIdentifiableSubspace(identificable_subspace_basis,
                                                                               consider_only_static_pose,
                                                                               n_samples) == 0);

    std::cout << "staticInertiaIdentification. Identified static base parameters subspace: " << std::endl;
    std::cout << icub_regressor_generator->analyseBaseSubspace(identificable_subspace_basis) << std::endl;

    return true;
}


bool staticInertiaIdentificationThread::configureParameterEstimator()
{
    parameter_estimator.setOutputSize(icub_regressor_generator->getNrOfOutputs());
    parameter_estimator.setParamSize(identificable_subspace_basis.cols());

    VectorXd sigma_oe(12);
    //sigma_oe << 0.0707119 ,  0.07460326,  0.11061799,  0.00253377,  0.00295331, 0.00281101, 0.0707119 ,  0.07460326,  0.11061799,  0.00253377,  0.00295331, 0.00281101;
    YARP_ASSERT(sigma_oe.size() == 12);
    sigma_oe *= 50;
    //parameter_estimator.setOutputErrorStandardDeviation(sigma_oe);

    return true;
}


//*************************************************************************************************************************
bool staticInertiaIdentificationThread::threadInit()
{
    //Calibration variables
    int nrOfAvailableFTSensors = estimator->getEstimateNumber(wbi::ESTIMATE_FORCE_TORQUE_SENSOR);

    if( nrOfAvailableFTSensors != icub_tree_model.getNrOfFTSensors() )
    {
        std::cout << "staticInertiaIdentificationThread::threadInit() error: number of FT sensors different between model (" <<
        icub_tree_model.getNrOfFTSensors() << ") and interface (" << nrOfAvailableFTSensors << " ) " << std::endl;
        return false;
    }

    int nrOfAvailableJointTorqueSensors = 0;

    robot_status = new KDL::CoDyCo::Regressors::DynamicSample(icub_tree_model.getNrOfDOFs(),
                                                              nrOfAvailableJointTorqueSensors,
                                                              icub_tree_model.getNrOfFTSensors());
    last_joint_position_used_for_identification.resize(icub_tree_model.getNrOfDOFs());

    wbi_imu.resize(13);

    velocity_static_threshold = opts.check("velocityThreshold",0.02,"Threshold on joint velocity norm to consider a sample static").asDouble();
    position_norm_threshold   = opts.check("positionThreshold",0.02,"Threshold on joint position difference norm to consider two sample different").asDouble();
    maxNrOfSamplesForPose = opts.check("maxNrOfSamplesForPose",100,"Maximum number of samples to consider for a given pose, to avoid overfitting").asInt();

    printPeriod = opts.check("printPeriod",printPeriod,"Period (in ms) for the debug print").asDouble();

    if( ! configureRegressorGenerator() )
    {
        std::cout << "staticInertiaIdentificationThread::threadInit() error: "
        << "problem in configuring regressor generator " << std::endl;
        return false;
    }

    if( ! configureGeneratedRegressor() )
    {
        std::cout << "staticInertiaIdentificationThread::threadInit() error: "
        << "problem in configuring generated regressor " << std::endl;
        return false;
    }

    if( ! configureParameterEstimator() )
    {
        std::cout << "staticInertiaIdentificationThread::threadInit() error: "
        << "problem in configuring parameter estimator " << std::endl;
        return false;
    }

    if( nrOfAvailableFTSensors != icub_regressor_generator->getNrOfWrenchSensors() )
    {
        std::cout << "staticInertiaIdentificationThread::threadInit() error: "
          << "number of FT sensors different between regressor_generator (" <<
          icub_regressor_generator->getNrOfWrenchSensors()
          << ") and interface (" << nrOfAvailableFTSensors << " ) " << std::endl;
        return false;
    }

    //Resize buffers
    ft_data_buffer.resize(6*nrOfAvailableFTSensors);
    regressor.resize(icub_regressor_generator->getNrOfOutputs(),icub_regressor_generator->getNrOfParameters());
    known_terms.resize(icub_regressor_generator->getNrOfOutputs());
    base_regressor.resize(icub_regressor_generator->getNrOfOutputs(),identificable_subspace_basis.cols());
    singular_value_buffer.resize(identificable_subspace_basis.cols());
    singular_vectors_buffer.resize(identificable_subspace_basis.cols(),identificable_subspace_basis.cols());

    //Open ports
    port_skin_contacts = new BufferedPort<skinContactList>;
    std::string local_name = opts.find("name").asString().c_str();
    port_skin_contacts->open(string("/"+local_name+"/skin_contacts:i").c_str());

    //Start in not estimating mode
    thread_state = NOT_ESTIMATING;

    printf("\n\n");
    return true;
}


//*************************************************************************************************************************
void staticInertiaIdentificationThread::run()
{
    run_mutex.lock();

    //Get joint data
    estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,robot_status->q.data.data());
    estimator->getEstimates(wbi::ESTIMATE_JOINT_VEL,robot_status->dq.data.data());
    estimator->getEstimates(wbi::ESTIMATE_JOINT_ACC,robot_status->ddq.data.data());

    //Get imu data
    estimator->getEstimates(wbi::ESTIMATE_IMU,wbi_imu.data());
    robot_status->base_acc.vel[0] = wbi_imu[4];
    robot_status->base_acc.vel[1] = wbi_imu[5];
    robot_status->base_acc.vel[2] = wbi_imu[6];

    // \todo TODO for now support only static estimation
    SetToZero(robot_status->base_acc.rot);
    SetToZero(robot_status->base_vel);

    //Get FT sensors data
    estimator->getEstimates(wbi::ESTIMATE_FORCE_TORQUE_SENSOR,ft_data_buffer.data());

    for(int i=0; i < robot_status->getNrOfWrenchSensors(); i++ )
    {
        memcpy(robot_status->wrench_sensors_measures[i].force.data,ft_data_buffer.data()+6*i,3*sizeof(double));
        memcpy(robot_status->wrench_sensors_measures[i].torque.data,ft_data_buffer.data()+6*i+3,3*sizeof(double));
    }

    //Feed the data read in the regressor generator
    icub_regressor_generator->setRobotStateAndSensors(*robot_status);

    //Compute regressor and relative known terms
    icub_regressor_generator->computeRegressor(regressor,known_terms);

    //Compute base regressor
    base_regressor = regressor*identificable_subspace_basis;

    //Compute flags
    is_static_sample = isSampleStatic(*robot_status);
    is_sample_a_new_pose = isSamplePositionDifferent(*robot_status,last_joint_position_used_for_identification);
    is_external_contact_sample = isSampleCorruptedByExternalContact();

    if( useCurrentSampleForIdentification() )
    {
        parameter_estimator.feedSample(base_regressor,known_terms);

        //Store last pose used
        last_joint_position_used_for_identification = robot_status->q;

        // Reset sample counter for this pose and increase poseCounter
        if( is_sample_a_new_pose )
        {


            poseCount++;
            currentPoseSampleCount = 0;
        }

        currentPoseSampleCount++;
    }

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();

    //std::cout << "printCountdown : " << printCountdown << std::endl;

    if( printCountdown == 0 )
    {
        printStatus();
        printCountdown = 0;
    }

    run_mutex.unlock();
}

bool staticInertiaIdentificationThread::useCurrentSampleForIdentification()
{
    if(    thread_state == ESTIMATING
        && is_static_sample
        && ( is_sample_a_new_pose || currentPoseSampleCount < maxNrOfSamplesForPose )
        && !is_external_contact_sample
      )
    {
        return true;
    }
    else
    {
        return false;
    }

}
bool staticInertiaIdentificationThread::isSampleStatic(const KDL::CoDyCo::Regressors::DynamicSample & sample)
{
    if( sample.dq.data.norm() > velocity_static_threshold )
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool staticInertiaIdentificationThread::isSamplePositionDifferent(const KDL::CoDyCo::Regressors::DynamicSample & sample,
                                                                  const KDL::JntArray & previous_position)
{
    if( (sample.q.data-previous_position.data).norm() > position_norm_threshold )
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool staticInertiaIdentificationThread::isSampleCorruptedByExternalContact()
{
    skinContactList *scl = port_skin_contacts->read(false);
    if( !scl || scl->empty() )
    {
        if( estimation_disabled_for_external_contact )
        {
            if( fabs(yarp::os::Time::now()-timestamp_of_last_external_contact) < 1.0 )
            {
                return true;
            } else {
                estimation_disabled_for_external_contact = false;
                return false;
            }
        }
        return false;
    }

    //Contact found, disabling estimation for a second
    estimation_disabled_for_external_contact = true;
    timestamp_of_last_external_contact = yarp::os::Time::now();

    return true;
}


void staticInertiaIdentificationThread::printStatus()
{
    std::cout <<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< std::endl;
        std::cout << "Thread status: ";
        if( thread_state == ESTIMATING )
        {
            std::cout << "Estimating " << std::endl;
        }
        else
        {
            std::cout << "Not estimating" << std::endl;
        }
        if( is_static_sample )
        {
            std::cout << "Robot is still. " << std::endl;
        }
        else
        {
            std::cout << "Robot is moving. " << std::endl;
        }

        if( is_sample_a_new_pose )
        {
            std::cout << "Robot is in a new pose. " << std::endl;
        }
        else
        {
            std::cout << "Robot is not in a new pose. " << std::endl;
        }
        if( is_external_contact_sample )
        {
            std::cout << "The robot is in contact with something with the skin, temporarly disabling identification" << std::endl;
        }

        std::cout << "Pose acquired    : " << poseCount << std::endl;
        std::cout << "Samples acquired : " << parameter_estimator.getNrOfSamples() << std::endl;
        std::cout << "Samples acquired for current pose : " << currentPoseSampleCount << std::endl;


        if( parameter_estimator.getNrOfSamples() > 5 )
        {
            std::cout << "Paramer estimation " << std::endl;
            parameter_estimator.getSingularValues(singular_value_buffer);
            parameter_estimator.getSingularVectors(singular_vectors_buffer);
            parameter_estimator.updateParameterEstimate();
            std::cout << "Estimated parameters" << std::endl
                      <<  parameter_estimator.getParameterEstimate() << std::endl;
            //std::cout << "Estimated parameters" << std::endl;
            //std::cout << icub_regressor_generator->getDescriptionOfParameters(identificable_subspace_basis*parameter_estimator.getParameterEstimate());

            std::cout << "Singular values: " << std::endl
                      << singular_value_buffer << std::endl;
            std::cout << "Condition number: " << sqrt(singular_value_buffer[0]/singular_value_buffer[singular_value_buffer.rows()-1]) << std::endl;
            //std::cout << "SVD V Matrix : " << std::endl;
            //std::cout << singular_vectors_buffer << std::endl;
            //double essential_parameters_threshold;
        }
        else
        {
            std::cout << "Too few samples to print estimation status. " << std::endl;
        }
        std::cout << "dq norm " << robot_status->dq.data.norm() << std::endl;
        std::cout << "ddq norm " << robot_status->ddq.data.norm() << std::endl;
        std::cout << "difference in q norm " << (last_joint_position_used_for_identification.data-robot_status->q.data).norm() << std::endl;
}

//*****************************************************************************
void staticInertiaIdentificationThread::threadRelease()
{
    run_mutex.lock();
    delete icub_regressor_generator; icub_regressor_generator = 0;
    delete robot_status; robot_status = 0;
    closePort(port_skin_contacts);
    run_mutex.unlock();
}

//*****************************************************************************
bool staticInertiaIdentificationThread::startEstimation()
{
    run_mutex.lock();
    thread_state = ESTIMATING;
    run_mutex.unlock();
    return true;
}

//*****************************************************************************
bool staticInertiaIdentificationThread::stopEstimation()
{
    run_mutex.lock();
    thread_state = NOT_ESTIMATING;
    run_mutex.unlock();
    return true;
}

//*****************************************************************************
bool staticInertiaIdentificationThread::saveURDF(const std::string & urdf_filename, const std::string & robotName)
{
    std::cout << "staticInertiaIdentificationThread::saveURDF called with " <<
                 urdf_filename << " " << robotName << std::endl;
    Eigen::VectorXd model_regressor_parameters(icub_regressor_generator->getNrOfParameters());
    Eigen::VectorXd estimated_base_parameters(parameter_estimator.getParamSize());
    run_mutex.lock();
    //We have to compose the existing parameters with the one already in the model,
    //considering that we are only estimating the base parameters for the given regressor
    estimated_base_parameters = parameter_estimator.getParameterEstimate();

    icub_regressor_generator->getModelParameters(model_regressor_parameters);
    run_mutex.unlock();

    //Create a new urdf data structure
    boost::shared_ptr<urdf::ModelInterface> icub_ptr(new urdf::ModelInterface);

    int nr_of_base_parameters = identificable_subspace_basis.cols();
    Eigen::MatrixXd not_identifiable_subspace_projector =
        Eigen::MatrixXd::Identity(nr_of_base_parameters,nr_of_base_parameters)
        - identificable_subspace_basis*identificable_subspace_basis.transpose();

    Eigen::VectorXd estimated_parameters =
         identificable_subspace_basis*estimated_base_parameters
         + not_identifiable_subspace_projector*model_regressor_parameters;

    KDL::CoDyCo::UndirectedTree identified_undirect_tree_model;

    icub_regressor_generator->getUpdatedModel(estimated_parameters,identified_undirect_tree_model);

    KDL::Tree identified_tree_model = identified_undirect_tree_model.getTree();

    bool success = kdl_format_io::treeToUrdfModel(identified_tree_model,robotName,*icub_ptr);

    if( !success )
    {
        std::cout << "staticInertiaIdentification saveURDF: error in KDL - URDF conversion. " << std::endl;
        return false;
    }

    TiXmlDocument* xml_doc = urdf::exportURDF(icub_ptr);
    success = xml_doc->SaveFile(urdf_filename);

    if( !success )
    {
        std::cout << "staticInertiaIdentification saveURDF: error in saving " << urdf_filename << " URDF file. " << std::endl;
        return false;
    }

    return success;
}


//*****************************************************************************
void staticInertiaIdentificationThread::closePort(Contactable *_port)
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
/*
template <class T> void staticInertiaIdentificationThread::broadcastData(T& _values, BufferedPort<T> *_port)
{
    if (_port && _port->getOutputCount()>0)
    {
        _port->setEnvelope(this->timestamp);
        _port->prepare()  = _values ;
        _port->write();
    }
}*/
