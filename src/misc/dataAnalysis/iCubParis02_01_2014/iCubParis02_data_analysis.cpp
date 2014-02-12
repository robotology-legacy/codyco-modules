#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>
#include <dirl/dataset/DynamicDatasetFile.hpp>
#include <dirl/dynamicRegressorGenerator.hpp>
#include <cstdlib>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/regressor_utils.hpp>

#include <iostream>
#include <fstream>
#include <boost/concept_check.hpp>

#include <yarp/os/Property.h>

#include "multitaskRecursiveLinearEstimator.h"
#include "multitaskSVDLinearEstimator.h"


using namespace KDL;
using namespace KDL::CoDyCo;
using namespace KDL::CoDyCo::Regressors;

#define MODEL_DOFS 32
#define DATASET_DOFS 41

#define LEFT_ELBOW_TORQUE_INDEX 2
#define RIGHT_ELBOW_TORQUE_INDEX 8

#define RIGHT_SHOULDER_PITCH_TORQUE_INDEX 6

#define LEFT_ARM_FT_INDEX 0
#define RIGHT_ARM_FT_INDEX 1

#define RIGHT_ELBOW_GAIN 3.5290

#define RIGHT_SHOULDER_PITCH_GAIN 2.4191

/** Tolerance for considering two values equal */
const double ZERO_TOL = 1e-5;



bool convertJointsFromDatasetToModel(const KDL::JntArray & q_dataset, KDL::JntArray & q_model, bool verbose = false)
{
    if( q_dataset.rows() != DATASET_DOFS ) {
        if( verbose ) std::cout << "Q from dataset has size " << q_dataset.rows() << std::endl;
    }

    assert(q_dataset.rows() == DATASET_DOFS);

    assert(q_model.rows() == MODEL_DOFS);
    //Joints in dataset are : Torso (3), head(6), left arm (16), right_arm (16)
    //Joints in the mode are : Torso (3), head(3), left arm (7), right arm (7), left_leg (6), right_leg(6)
    
    SetToZero(q_model);
    
    //Torso should be inverted
    q_model(0) = q_dataset(2);
    q_model(1) = q_dataset(1);
    q_model(2) = q_dataset(0);
    
    //Copyng head
    for(int i = 0; i < 3; i++ ) {
        q_model(3+i) = q_dataset(3+i);
    }
    
    //Copyng left arm
    for(int i = 0; i < 7; i++ ) {
        q_model(3+3+i) = q_dataset(3+6+i);
    }
    
    //Copyng right arm 
    for(int i = 0; i < 7; i++ ) {
        q_model(3+3+7+i) = q_dataset(3+6+16+i);
    }
    
   
    
}

bool toYarp(const KDL::Wrench & ft, yarp::sig::Vector & ft_yrp)
{
    if( ft_yrp.size() != 6 ) { ft_yrp.resize(6); }
    for(int i=0; i < 6; i++ ) {
        ft_yrp[i] = ft[i];
    }
}

bool toKDL(const yarp::sig::Vector & ft_yrp, KDL::Wrench & ft)
{
    for(int i=0; i < 6; i++ ) {
        ft[i] = ft_yrp[i];
    }
}

bool toYarp(const Eigen::VectorXd & vec_eigen, yarp::sig::Vector & vec_yrp)
{
    if( vec_yrp.size() != vec_eigen.size() ) { vec_yrp.resize(vec_eigen.size()); }
    if( memcpy(vec_yrp.data(),vec_eigen.data(),sizeof(double)*vec_eigen.size()) != NULL ) {
        return true;
    } else {
        return false;
    }
}

bool toEigen(const yarp::sig::Vector & vec_yrp, Eigen::VectorXd & vec_eigen)
{
 if( vec_yrp.size() != vec_eigen.size() ) { vec_eigen.resize(vec_yrp.size()); }
    if( memcpy(vec_eigen.data(),vec_yrp.data(),sizeof(double)*vec_eigen.size()) != NULL ) {
        return true;
    } else {
        return false;
    }
}


int main(int argc, char ** argv)
{
    yarp::os::Property opt;
    
    opt.fromCommand(argc,argv);
   
    if( !opt.check("dataset") ) {
        std::cout << "iCubParis02_data_analysis: " << std::endl;
        std::cout << "usage: ./iCubParis02_data_analysis --dataset dataset.csv --results results.csv" << std::endl;
        std::cout << "additional options: --n_samples n the number of random regressors to generate for the numerical identifable subspace computation" << std::endl;
        std::cout << "additional options: --cut_freq the cut frequency to use for filtering torques and ft measures (in Hz, default 3 Hz)" << std::endl;
        std::cout << "additional options: --sampleTime the sample time of the data (in s, default: 0.01 s)" << std::endl;
        std::cout << "additional options: --lambda regularization parameter" << std::endl;
        std::cout << "additional options: --skin number of initial samples to skip" << std::endl;
        std::cout << "additional options: --verbose print debug information" << std::endl;
        std::cout << "additional options: --parameters param.csv output a file of trajectoreis of the estimated parameters" << std::endl;
        return 0;
    }
    
    int n_samples;
    if(!opt.check("n_samples")) {
        n_samples = 1000;
    } else {
        n_samples = opt.find("n_samples").asInt();
    }
    
    double sampleTime;
    if(!opt.check("sampleTime")) {
        sampleTime = 0.01;
    } else {
        sampleTime = opt.find("sampleTime").asDouble();
    }
    
        
    double cut_freq;
    if(!opt.check("cut_freq")) {
        cut_freq = 3;
    } else {
        cut_freq = opt.find("cut_freq").asDouble();
    }
    
    double lambda;
    if(!opt.check("lambda")) {
        lambda = 1;
    } else {
        lambda = opt.find("lambda").asDouble();
    }
    
    int skip;
    if(!opt.check("skip")) {
        skip = 1;
    } else {
        skip = opt.find("skip").asDouble();
    }
    
    bool verbose;
    if(!opt.check("verbose")) {
        verbose = false;
    } else {
        verbose = true;
    }
    
    std::string dataset_file_name(opt.find("dataset").asString());
    std::string results_file_name(opt.find("results").asString());
    bool output_param = false;
    std::string params_file_name;
    
    if(  opt.check("parameters") ) {
        params_file_name = opt.find("parameters").asString().c_str();
        output_param = true;
    }
    
    //Open result file
    std::ofstream results_file;
    results_file.open(results_file_name.c_str());
    
    std::ofstream param_file;
    
    if( output_param ) {
        param_file.open(params_file_name.c_str());
    }
    
    std::vector<std::string> l_arm_subtree, r_arm_subtree;
    
    l_arm_subtree.push_back("l_arm");
    r_arm_subtree.push_back("r_arm");
    
    
    iCub::iDynTree::iCubTree_version_tag tag;
    //iCubParis02 is a v2 robot
    tag.head_version = 2;
    tag.legs_version = 2;
    tag.arms_version = 2;
    
    iCub::iDynTree::iCubTree icub_tree_model(tag);
    
    //Can be l_elbow or r_elbow
    std::string elbow_joint_name = "r_elbow";
    
    std::string shoulder_pitch_joint_name = "r_shoulder_pitch";
    
    //Can be l_arm_ft_sensor or r_arm_ft_sensor 
    std::string ft_sensor_name = "r_arm_ft_sensor";
    
    //Can be RIGHT_ELBOW_TORQUE_INDEX or LEFT_ELBOW_TORQUE_INDEX
    const int elbow_torque_global_id = RIGHT_ELBOW_TORQUE_INDEX;
    
    const int shoulder_pitch_global_id = RIGHT_SHOULDER_PITCH_TORQUE_INDEX;
    
    //Can be r_arm_subtreee or l_arm_subtree
    std::vector<std::string> arm_subtree = r_arm_subtree;
    
    //We add only a FT sensor for regressor, so it should be 0
    const int ft_regressor_generator_sensor_index = 0;
    
    //Can be RIGHT_ARM_FT_INDEX or LEFT_ARM_FT_INDEX
    const int ft_dataset_sensor_index = RIGHT_ARM_FT_INDEX;
    
    //Can be l_upper_arm or r_upper_arm
    std::string dynamic_base = "r_upper_arm";
    
    //Can be right or left (gain between raw outputs)
    const double elbow_torque_gain = RIGHT_ELBOW_GAIN; 
    
    const double shoulder_pitch_torque_gain = RIGHT_SHOULDER_PITCH_GAIN;
    
    std::vector<std::string> ft_names;
    ft_names.push_back(ft_sensor_name);
    
    std::vector<std::string> fake_names;
    fake_names.push_back("torso");
    fake_names.push_back("imu_frame");
    fake_names.push_back("l_gripper");
    fake_names.push_back("r_gripper");    
    fake_names.push_back("l_sole");
    fake_names.push_back("r_sole");
    fake_names.push_back("l_wrist_1");
    fake_names.push_back("r_wrist_1");
    
    std::string root_link_name = "root_link";
    
    KDL::Tree icub_kdl_tree = icub_tree_model.getKDLTree();
    KDL::CoDyCo::UndirectedTree icub_kdl_undirected_tree = icub_tree_model.getKDLUndirectedTree();
    KDL::CoDyCo::TreeSerialization icub_serialization =icub_tree_model.getKDLUndirectedTree().getSerialization();
    
    KDL::CoDyCo::FTSensorList icub_ft_list(icub_kdl_undirected_tree,ft_names);
    
    DynamicRegressorGenerator ft_regressor_generator(icub_kdl_tree,
                                                  root_link_name,
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_serialization,true);
    
    ft_regressor_generator.changeDynamicBase(dynamic_base);
    
    DynamicRegressorGenerator torque_regressor_generator(icub_kdl_tree,
                                                  root_link_name,
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_serialization,true);
    
   DynamicRegressorGenerator shoulder_pitch_torque_regressor_generator(icub_kdl_tree,
                                                  root_link_name,
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_serialization,true);
    
    torque_regressor_generator.changeDynamicBase(dynamic_base);

    
    DynamicRegressorGenerator reverse_torque_regressor_generator(icub_kdl_tree,
                                                  root_link_name,
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_serialization,true);
    
    reverse_torque_regressor_generator.changeDynamicBase(dynamic_base);


    
    //Adding three regressors to the generator:
    //    (1) 6 for ft sensors (with offset)
    //    (2) 1 for forward torque regressor
    //    (3) 1 for backward torque regressor
    int ret = ft_regressor_generator.addSubtreeRegressorRows(arm_subtree);
    assert(ret == 0);
    
    ret = torque_regressor_generator.addTorqueRegressorRows(elbow_joint_name);
    assert(ret == 0);
    
    ret = shoulder_pitch_torque_regressor_generator.addTorqueRegressorRows(shoulder_pitch_joint_name);
    assert(ret == 0);
    
    reverse_torque_regressor_generator.addTorqueRegressorRows(elbow_joint_name,true,arm_subtree);
    assert(ret == 0);
    
    //Pointer to filters
    iCub::ctrl::FirstOrderLowPassFilter * tau_filt;    
    iCub::ctrl::FirstOrderLowPassFilter * tau_filt_shoulder_pitch;
    iCub::ctrl::FirstOrderLowPassFilter * ft_filt;
    iCub::ctrl::AWLinEstimator * lin_estimator;
    iCub::ctrl::AWPolyEstimator * quad_estimator;
    
    //Loading dataset
    DynamicDatasetFile test_dataset;
  
    test_dataset.loadFromFile(dataset_file_name);
    
    KDL::JntArray q, dq, ddq;
    yarp::sig::Vector q_yarp, dq_yarp, ddq_yarp;
    q.resize(ft_regressor_generator.getNrOfDOFs());
    SetToZero(q);
    q_yarp.resize(q.rows(),0.0);
    dq_yarp = q_yarp;
    ddq_yarp = q_yarp;
    
    dq = q;
    ddq = q;
        
    const double g = 9.806;
    KDL::Twist gravity(KDL::Vector(0.0,0.0,g),KDL::Vector(0.0,0.0,0.0));
    
    KDL::Twist zero_gravity = KDL::Twist::Zero();
    KDL::JntArray zero_q = q;
    SetToZero(zero_q);
    
    Eigen::MatrixXd ft_regressor_wrong_frame(ft_regressor_generator.getNrOfOutputs(),ft_regressor_generator.getNrOfParameters());
    Eigen::VectorXd ft_kt_wrong_frame(ft_regressor_generator.getNrOfOutputs());
    
    Eigen::MatrixXd ft_regressor(ft_regressor_generator.getNrOfOutputs(),ft_regressor_generator.getNrOfParameters());
    Eigen::VectorXd ft_kt(ft_regressor_generator.getNrOfOutputs());
    
    Eigen::MatrixXd tau3_regressor(ft_regressor_generator.getNrOfOutputs()/2,ft_regressor_generator.getNrOfParameters());
    Eigen::VectorXd tau3_kt(ft_regressor_generator.getNrOfOutputs()/2);
    
    Eigen::MatrixXd torque_regressor(torque_regressor_generator.getNrOfOutputs(),torque_regressor_generator.getNrOfParameters());
  
    Eigen::VectorXd torque_kt(torque_regressor_generator.getNrOfOutputs());
    
    Eigen::MatrixXd reverse_torque_regressor(reverse_torque_regressor_generator.getNrOfOutputs(),reverse_torque_regressor_generator.getNrOfParameters());
    Eigen::VectorXd reverse_torque_kt(reverse_torque_regressor_generator.getNrOfOutputs());
    
    Eigen::MatrixXd shoulder_pitch_torque_regressor(shoulder_pitch_torque_regressor_generator.getNrOfOutputs(),reverse_torque_regressor_generator.getNrOfParameters());
    Eigen::VectorXd shoulder_pitch_torque_kt(shoulder_pitch_torque_regressor_generator.getNrOfOutputs());
     
    
    Eigen::MatrixXd base_parameters_subspace; 
    Eigen::MatrixXd nullspace_projector_base_parameters_subspace;
    
    Eigen::MatrixXd static_base_parameters_subspace;
    
    Eigen::MatrixXd base_parameters_subspace_torque;
    
    Eigen::MatrixXd static_base_parameters_subspace_torque;
    
    int ret_value = 0;
    
    ret_value = ft_regressor_generator.computeNumericalIdentifiableSubspace(base_parameters_subspace,false,n_samples);
    assert( ret_value == 0 );
    
    Eigen::MatrixXd Btranspose = base_parameters_subspace.transpose();
    nullspace_projector_base_parameters_subspace = Eigen::MatrixXd::Identity(base_parameters_subspace.rows(),base_parameters_subspace.rows()) - base_parameters_subspace*Btranspose;
    
    ret_value = ft_regressor_generator.computeNumericalIdentifiableSubspace(static_base_parameters_subspace,true,n_samples);
    assert( ret_value == 0 );
    
    ret_value = torque_regressor_generator.computeNumericalIdentifiableSubspace(base_parameters_subspace_torque,false,n_samples);
    assert( ret_value == 0 );
    
    ret_value = torque_regressor_generator.computeNumericalIdentifiableSubspace(static_base_parameters_subspace_torque,true,n_samples);
    assert( ret_value == 0 );
    
    /*
    base_parameters_subspace = Eigen::MatrixXd::Identity(ft_regressor_generator.getNrOfParameters(),ft_regressor_generator.getNrOfParameters()-1);
    static_base_parameters_subspace = Eigen::MatrixXd::Identity(ft_regressor_generator.getNrOfParameters(),ft_regressor_generator.getNrOfParameters()-2);

    base_parameters_subspace_torque = Eigen::MatrixXd::Identity(ft_regressor_generator.getNrOfParameters(),ft_regressor_generator.getNrOfParameters()-3);
    static_base_parameters_subspace_torque = Eigen::MatrixXd::Identity(ft_regressor_generator.getNrOfParameters(),ft_regressor_generator.getNrOfParameters()-4);
    */
    
    int static_base_parameters = static_base_parameters_subspace.cols();
    int base_parameters = base_parameters_subspace.cols();
    
    int static_base_parameters_torque = static_base_parameters_subspace_torque.cols();
    int base_parameters_torque = base_parameters_subspace_torque.cols();
    
    //Augmented regressor for accounting for trque sensor offset 
    Eigen::MatrixXd base_torque_regressor_augmented(torque_regressor_generator.getNrOfOutputs(),base_parameters_torque+1);
    base_torque_regressor_augmented.rightCols<1>()(0) = 1;
    if( verbose )std::cout << "Base torque regressor augmented " << base_torque_regressor_augmented << std::endl;
   
    
    Eigen::MatrixXd static_base_torque_regressor_augmented(torque_regressor_generator.getNrOfOutputs(),static_base_parameters_torque+1);
    static_base_torque_regressor_augmented.rightCols<1>()(0) = 1;
    
    Eigen::VectorXd output_stddev(6);
    /*
    for(int r=0; r < 3; r++ ) { output_stddev[r] = 10.0; }
    for(int r=0; r < 3; r++ ) { output_stddev[3+r] = 1.0; }
    */
    output_stddev << 0.0707119 ,  0.07460326,  0.11061799,  0.00253377,  0.00295331, 0.00281101;

    
    multiTaskRecursiveLinearEstimator estimator_static(static_base_parameters,ft_regressor_generator.getNrOfOutputs(),lambda);
    estimator_static.setOutputErrorStandardDeviation(output_stddev);
    multiTaskRecursiveLinearEstimator estimator_dynamic(base_parameters,ft_regressor_generator.getNrOfOutputs(),lambda);
    estimator_dynamic.setOutputErrorStandardDeviation(output_stddev);
    
    multiTaskRecursiveLinearEstimator estimator_static_torque(static_base_parameters_torque+1,torque_regressor_generator.getNrOfOutputs(),lambda);
    multiTaskRecursiveLinearEstimator estimator_dynamic_torque(base_parameters_torque+1,torque_regressor_generator.getNrOfOutputs(),lambda);

    //Print header in results
    results_file << "measured_torque,filtered_torque,torque_bwd_estimation_with_ft_parameters_dynamic,torque_bwd_estimation_with_ft_parameters_static,torque_bwd_estimation_with_torque_parameters_dynamic,torque_bwd_estimation_with_torque_parameters_static,torque_bwd_estimation_with_cad_parameters,q_elbow,dq_elbow,ddq_elbow"
                << ",force_z_measured,force_z_filtered,force_z_estimated_with_cad,force_z_estimated_with_ft_param,torque_fwd_with_ft_parameters_static,torque_fwd_with_ft_parameters_dynamic,torque_fwd_cad_parameters,torque_fwd_ft_projection,torque_fwd_t_projection,shoulder_pitch_torque_measured,shoulder_pitch_torque_estimated_by_cad,shoulder_pitch_torque_estimated_by_ft_param,shoulder_pitch_torque_estimated_by_ft_cad"  << std::endl;
    
    KDL::Wrench ft_sample;
    yarp::sig::Vector ft_sample_raw(6,0.0), ft_sample_filtered(6,0.0);
    double torque_sample;
    yarp::sig::Vector torque_sample_raw(1,0.0), torque_sample_filtered(1,0.0);
    double torque_sample_shoulder_pitch;
    yarp::sig::Vector torque_shoulder_pitch_raw(1,0.0), torque_shoulder_pitch_sample_filtered(1,0.0);
    
    
    if( output_param ) {
        //Print outputs
        params_file_name;
    }
    
    //Get cad parameters
    Eigen::VectorXd cad_parameters(ft_regressor_generator.getNrOfParameters());
    cad_parameters.setZero();
    
    KDL::CoDyCo::inertialParametersVectorLoopFakeLinks(icub_kdl_undirected_tree,cad_parameters,fake_names);
    
    //Structures for offset estimation
    multiTaskSVDLinearEstimator estimator_static_offset(6,6);

    int sample_nr;
    int nmbr_of_samples_for_offset_calibration = 30;
    int nmbr_of_samples_for_offset_calibration_obtained = 0;
    Eigen::VectorXd offset = Eigen::VectorXd(6);
    Eigen::MatrixXd regressor_offset;
    Eigen::VectorXd offset_kt;
    
    //Diagnostic of velocities
    double q_elbow, dq_elbow, ddq_elbow;
    int elbow_id = icub_tree_model.getDOFIndex(elbow_joint_name);
    
    for(sample_nr=skip; sample_nr < test_dataset.getNrOfSamples(); sample_nr++ ) {
        if( verbose ) { std::cout << "Looping on sample: " << sample_nr << std::endl; }
        
        
        //Joints in dataset are : Torso (3), head(6), left arm (16), right_arm (16)
        //Depending on the dataset we use different parts, here we use only the  right arm, the torso and the head (which however should be still)
        DynamicSample sample;
        bool ret = test_dataset.getSample(sample_nr,sample);
        if( !ret ) return -1;
        assert(sample.getNrOfDOFs() == DATASET_DOFS);
        
        assert( sample.getJointPosition().rows() == DATASET_DOFS);
       
        convertJointsFromDatasetToModel(sample.getJointPosition(),q);
               
        //Filter values
        ft_sample = sample.getWrenchMeasure(ft_dataset_sensor_index);
        
        if( verbose ) {
            std::cout << "FT sample from dataset" << std::endl;
            std::cout << ft_sample << std::endl;
            std::cout << "Force norm: "<<  ft_sample.force.Norm() << std::endl;
        }
        
        torque_sample = elbow_torque_gain*sample.getTorqueMeasure(elbow_torque_global_id);
        
        torque_sample_shoulder_pitch = shoulder_pitch_torque_gain*sample.getTorqueMeasure(shoulder_pitch_global_id);
        
        toYarp(ft_sample,ft_sample_raw);
        torque_sample_raw[0] = torque_sample;
        torque_shoulder_pitch_raw[0] = torque_sample_shoulder_pitch;
        
        if( sample_nr == skip ) {
            tau_filt = new iCub::ctrl::FirstOrderLowPassFilter(cut_freq,sampleTime,torque_sample_raw);
            tau_filt_shoulder_pitch = new iCub::ctrl::FirstOrderLowPassFilter(cut_freq,sampleTime,torque_shoulder_pitch_raw);
            ft_filt = new iCub::ctrl::FirstOrderLowPassFilter(cut_freq,sampleTime,ft_sample_raw);
            lin_estimator = new iCub::ctrl::AWLinEstimator(16,0.02);
            quad_estimator = new iCub::ctrl::AWQuadEstimator(50,0.02);
        } else {
            torque_sample_filtered = tau_filt->filt(torque_sample_raw);
            torque_shoulder_pitch_sample_filtered = tau_filt_shoulder_pitch->filt(torque_shoulder_pitch_raw);
            ft_sample_filtered = ft_filt->filt(ft_sample_raw);
            toKDL(ft_sample_filtered,ft_sample);
            torque_sample = torque_sample_filtered[0];
            torque_sample_shoulder_pitch = torque_shoulder_pitch_sample_filtered[0];
        }
        
        if( verbose ) {
            std::cout << "FT sample filtered" << std::endl;
            std::cout << ft_sample << std::endl;
            std::cout << "Force norm: "<<  ft_sample.force.Norm() << std::endl;
        }
        
        //Obtain velocities
        iCub::ctrl::AWPolyElement el;
        toYarp(q.data,el.data);
        el.time = sample.getTimestamp();
        toEigen(lin_estimator->estimate(el),dq.data);
        toEigen(quad_estimator->estimate(el),ddq.data);
        
        q_elbow = q(elbow_id);
        dq_elbow = dq(elbow_id);
        ddq_elbow = ddq(elbow_id);
        
        /*
        std::cout << "FT sample " << std::endl;
        std::cout << ft_sample << std::endl;
        */
        
        ft_regressor_generator.setRobotState(q,dq,ddq,gravity);
        ft_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample);
        ft_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),torque_sample);
        ft_regressor_generator.computeRegressor(ft_regressor_wrong_frame,ft_kt_wrong_frame);

        //Change frame of reference to the ft_regressor and ft_kt
        //They should be on the r_upper_arm (parent of the sensor) and we want it in the sensor reference frame
        KDL::Frame H_sensor_parent = icub_ft_list.ft_sensors_vector[0]->getH_parent_sensor().Inverse();
        Eigen::Matrix<double,6,6> H_sens_parent = KDL::CoDyCo::WrenchTransformationMatrix(H_sensor_parent); 
        ft_regressor = H_sens_parent*ft_regressor_wrong_frame;
        ft_kt = H_sens_parent*ft_kt_wrong_frame;
        
        
        if( verbose ) {
            std::cout << "Known term for forcetorque based regression (wrong frame)" << std::endl;
            std::cout << ft_kt_wrong_frame;
            std::cout << "Known term for forcetorque based regression" << std::endl;
            std::cout << ft_kt << std::endl;
            std::cout << H_sensor_parent << std::endl;
            std::cout << H_sens_parent << std::endl;
            std::cout << "Force norm: " << ft_kt.head(3).norm() << std::endl;
        }
        
        
        //tau3_regressor = ft_regressor.
        
        torque_regressor_generator.setRobotState(q,dq,ddq,gravity);
        torque_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample);
        torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),torque_sample);
        torque_regressor_generator.computeRegressor(torque_regressor,torque_kt);
        
        reverse_torque_regressor_generator.setRobotState(q,dq,ddq,gravity);
        reverse_torque_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample);
        reverse_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),torque_sample);
        reverse_torque_regressor_generator.computeRegressor(reverse_torque_regressor,reverse_torque_kt);
        
        
        shoulder_pitch_torque_regressor_generator.setRobotState(q,dq,ddq,gravity);
        shoulder_pitch_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(shoulder_pitch_joint_name),torque_sample_shoulder_pitch);
        shoulder_pitch_torque_regressor_generator.computeRegressor(shoulder_pitch_torque_regressor,shoulder_pitch_torque_kt);
        
        //Compute offset
        if( nmbr_of_samples_for_offset_calibration_obtained < nmbr_of_samples_for_offset_calibration ) {
            regressor_offset = ft_regressor.rightCols<6>(); 
            offset_kt = ft_kt - ft_regressor*cad_parameters;
            
            estimator_static_offset.feedSample(regressor_offset,offset_kt);
            nmbr_of_samples_for_offset_calibration_obtained++;
            
            if( nmbr_of_samples_for_offset_calibration_obtained == nmbr_of_samples_for_offset_calibration ) {
                estimator_static_offset.updateParameterEstimate();
                offset = estimator_static_offset.getParameterEstimate();
                assert(offset.size() == 6);
                cad_parameters.tail<6>() = offset;
            }
        }
        
        
        //Predict variables
        double measured_torque = torque_sample_raw[0];
        double filtered_torque = torque_kt[0];
        
        double shoulder_pitch_torque_measured = shoulder_pitch_torque_kt[0];
        double shoulder_pitch_torque_cad = (shoulder_pitch_torque_regressor*cad_parameters)[0];
        Eigen::VectorXd cad_and_estimated_parameters = base_parameters_subspace*estimator_dynamic.getParameterEstimate() + nullspace_projector_base_parameters_subspace*cad_parameters;
        double shoulder_pitch_torque_ft = (shoulder_pitch_torque_regressor*base_parameters_subspace*estimator_dynamic.getParameterEstimate())[0];
        double shoulder_pitch_torque_ft_cad = (shoulder_pitch_torque_regressor*cad_and_estimated_parameters)[0];
        
        /*
        std::cout << "Sample " << sample_nr << std::endl;
        std::cout << "estimator_static.getParameterEstimate(): " << std::endl << estimator_static.getParameterEstimate() << std::endl;
        std::cout << "ft_regressor*static_base_parameters_subspace" << std::endl << ft_regressor*static_base_parameters_subspace << std::endl;
        std::cout << "ft_kt" << std::endl << ft_kt << std::endl;
        */
        
        
        //std::cout << "Augmenting one" << std::endl;
        assert(base_torque_regressor_augmented.cols()-1 == base_parameters_subspace_torque.cols());
        assert(base_torque_regressor_augmented.leftCols(base_torque_regressor_augmented.cols()-1).cols() == base_parameters_subspace_torque.cols());
        base_torque_regressor_augmented.leftCols(base_torque_regressor_augmented.cols()-1) = torque_regressor*base_parameters_subspace_torque;
    
        //std::cout << "Augmenting two" << std::endl;
        assert(static_base_torque_regressor_augmented.cols()-1 == static_base_parameters_subspace_torque.cols());
        static_base_torque_regressor_augmented.leftCols(static_base_torque_regressor_augmented.cols()-1) = torque_regressor*static_base_parameters_subspace_torque;
        //std::cout << "finishing augmentation" << std::endl;

        
        double torque_bwd_estimation_with_ft_parameters_dynamic = (torque_regressor*base_parameters_subspace*estimator_dynamic.getParameterEstimate())[0];
        double torque_bwd_estimation_with_ft_parameters_static = (torque_regressor*static_base_parameters_subspace*estimator_static.getParameterEstimate())[0];

        double torque_bwd_estimation_with_torque_parameters_dynamic = (base_torque_regressor_augmented*estimator_dynamic_torque.getParameterEstimate())[0];
        double torque_bwd_estimation_with_torque_parameters_static = (static_base_torque_regressor_augmented*estimator_static_torque.getParameterEstimate())[0];

        double torque_bwd_estimation_with_cad_parameters = (torque_regressor*cad_parameters)[0];

        //Projecting torque from sensor
        Eigen::MatrixXd dummy_reverse_regressor = reverse_torque_regressor;
        Eigen::VectorXd dummy_reverse_kt = reverse_torque_kt;
        reverse_torque_regressor_generator.setRobotState(q,zero_q,zero_q,zero_gravity);
        reverse_torque_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample);
        reverse_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),0);
        reverse_torque_regressor_generator.computeRegressor(dummy_reverse_regressor,dummy_reverse_kt);
        double projected_ft = dummy_reverse_kt(0);
        
        if( verbose ) {
            std::cout << "Dummy reverse regressor: " << std::endl;
            std::cout << dummy_reverse_regressor << std::endl;
        }
        
        /*
        KDL::Wrench ft_sample_without_offset_static;
        Eigen::VectorXd<double, 6> estimated_offset_static = estimator_static.getParameterEstimate().tail<6>();
        for(int i=0; i < 6; i++ ) { ft_sample_without_offset_static(i) = ft_sample(i) - estimated_offset_static(i); }
        Eigen::MatrixXd dummy_reverse_regressor_static = reverse_torque_regressor;
        Eigen::VectorXd dummy_reverse_kt_static = reverse_torque_kt;
        reverse_torque_regressor_generator.setRobotState(q,zero_q,zero_q,zero_gravity);
        reverse_torque_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample_without_offset_static);
        reverse_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),0);
        reverse_torque_regressor_generator.computeRegressor(dummy_reverse_regressor_static,dummy_reverse_kt_static);
        double projected_ft_static = dummy_reverse_kt_static(0);
        
        
        
        KDL::Wrench ft_sample_without_offset_dynamic;
        Eigen::VectorXd<double, 6> estimated_offset_dynamic = estimator_dynamic.getParameterEstimate().tail<6>();
        for(int i=0; i < 6; i++ ) { ft_sample_without_offset_dynamic(i) = ft_sample(i) - estimated_offset_dynamic(i); }
        Eigen::MatrixXd dummy_reverse_regressor_dynamic = reverse_torque_regressor;
        Eigen::VectorXd dummy_reverse_kt_dynamic = reverse_torque_kt;
        reverse_torque_regressor_generator.setRobotState(q,zero_q,zero_q,zero_gravity);
        reverse_torque_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample_without_offset_dynamic);
        reverse_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),0);
        reverse_torque_regressor_generator.computeRegressor(dummy_reverse_regressor_dynamic,dummy_reverse_kt_dynamic);
        double projected_ft_dynamic = dummy_reverse_kt_dynamic(0);
        
        KDL::Wrench ft_sample_without_offset_cad;
        Eigen::VectorXd<double, 6> estimated_offset_cad = offset_kt;
        for(int i=0; i < 6; i++ ) { ft_sample_without_offset_cad(i) = ft_sample(i) + estimated_offset_cad(i); }
        Eigen::MatrixXd dummy_reverse_regressor_cad = reverse_torque_regressor;
        Eigen::VectorXd dummy_reverse_kt_cad = reverse_torque_kt;
        reverse_torque_regressor_generator.setRobotState(q,zero_q,zero_q,zero_gravity);
        reverse_torque_regressor_generator.setFTSensorMeasurement(ft_regressor_generator_sensor_index,ft_sample_without_offset_cad);
        reverse_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex(elbow_joint_name),0);
        reverse_torque_regressor_generator.computeRegressor(dummy_reverse_regressor_cad,dummy_reverse_kt_cad);
        double projected_ft_cad = dummy_reverse_kt_cad(0);
        */
        
        double torque_fwd_ft_projection = -projected_ft;
        double torque_fwd_t_projection = ft_sample[4];
        double torque_fwd_with_ft_parameters_dynamic = (reverse_torque_regressor*base_parameters_subspace*estimator_dynamic.getParameterEstimate())[0] - projected_ft;
        double torque_fwd_with_ft_parameters_static = (reverse_torque_regressor*static_base_parameters_subspace*estimator_static.getParameterEstimate())[0] - projected_ft;
        double torque_fwd_cad_parameters = (reverse_torque_regressor*cad_parameters)[0] - projected_ft;
        
        
        
        double force_z_filtered = ft_kt[5];
        if( verbose ) std::cout << "base_parameters_subspace.cols(): " << base_parameters_subspace.cols() << std::endl;
        if( verbose ) std::cout << "cad_parameters.size(): " << cad_parameters.size() << std::endl;
        if( verbose ) std::cout << "ft_regressor.cols(): " << ft_regressor.cols() << std::endl;
        if( verbose ) std::cout << "ft_regressor_generator.getNrOfParameters() : " << ft_regressor_generator.getNrOfParameters() << std::endl;
        assert( ft_regressor.cols() == base_parameters_subspace.rows() );
        assert( ft_regressor.cols() == cad_parameters.size() );
        assert( ft_regressor.cols() == base_parameters_subspace.rows() );
        assert( base_parameters_subspace.cols() == estimator_dynamic.getParamSize() );
        assert( ft_regressor.cols() == cad_parameters.size() );
        Eigen::VectorXd res = (ft_regressor*cad_parameters);
        if( verbose )std::cout << "Calculating force_z_estimated_with_cad " << std::endl;
        
        double force_z_estimated_with_cad = res(5);
        if( verbose ) std::cout << "Calculated force_z_estimated_with_cad " << std::endl;
        
        if( verbose )std::cout << "Calculating force_z_estimated_with_ft_param " << std::endl;
        res = (ft_regressor*base_parameters_subspace*estimator_dynamic.getParameterEstimate());
        double force_z_estimated_with_ft_param = res(5);
        if( verbose )std::cout << "Calculating force_z_estimated_with_ft_param " << std::endl;

        
        //Compute parameter estimation
        /*
                std::cout << "sample nr : " << sample_nr << std::endl;
        std::cout << "ft_kt " << ft_kt << std::endl;
        */
        estimator_dynamic.feedSampleAndUpdate(ft_regressor*base_parameters_subspace,ft_kt);
        
        //Compute parameter estimation
        /*
        std::cout << "sample nr : " << sample_nr << std::endl;
        std::cout << "regr" << ft_regressor*static_base_parameters_subspace << std::endl;
        std::cout << "ft_kt " << ft_kt << std::endl;
        */
        
        estimator_static.feedSampleAndUpdate(ft_regressor*static_base_parameters_subspace,ft_kt);
                
        estimator_dynamic_torque.feedSampleAndUpdate(base_torque_regressor_augmented,torque_kt);
      
        
        estimator_static_torque.feedSampleAndUpdate(static_base_torque_regressor_augmented,torque_kt);

        //std::cout << "Static parameters estimated " << estimator_static.getParameterEstimate() << std::endl;

        double force_z_raw = ft_sample(2);
      
        results_file << measured_torque << "," << filtered_torque << "," << torque_bwd_estimation_with_ft_parameters_dynamic 
                     << "," << torque_bwd_estimation_with_ft_parameters_static << "," << torque_bwd_estimation_with_torque_parameters_dynamic << "," << 
                     torque_bwd_estimation_with_torque_parameters_static << "," << torque_bwd_estimation_with_cad_parameters  << "," <<
                     q_elbow << "," << dq_elbow << "," << ddq_elbow << "," << force_z_raw << "," << force_z_filtered << "," << force_z_estimated_with_cad << "," <<  force_z_estimated_with_ft_param << 
                     "," << torque_fwd_with_ft_parameters_static << "," << torque_fwd_with_ft_parameters_dynamic << "," << torque_fwd_cad_parameters << "," << torque_fwd_ft_projection << "," << torque_fwd_t_projection 
                     << "," << shoulder_pitch_torque_measured << "," << shoulder_pitch_torque_cad << "," << shoulder_pitch_torque_ft << "," << shoulder_pitch_torque_ft_cad << std::endl;
                     
      
        /*
        if( output_param ) {
            Eigen::VectorXd params1 = estimator_dynamic.getParameterEstimate();
            Eigen::VectorXd params2 =  estimator_static.getParameterEstimate();
            for(int param_nr=0; param_nr < params1.size()+params2.size(); param_nr++ ) {
                if( param_nr < params1.size() ) {
                    param_file << params1[param_nr];
                } else {
                    param_file << params2[param_nr-params1.size()];
                }
                if( param_nr != params1.size()+params2.size()-1 ) {
                    param_file << ",";
                } else {
                    param_file << std::endl;
                }
            }
        }*/
        
        if( output_param ) {
            Eigen::VectorXd params1 = estimator_dynamic.getParameterEstimate();
            for(int param_nr=0; param_nr < params1.size(); param_nr++ ) {
                if( param_nr < params1.size() ) {
                    param_file << params1[param_nr];
                } 
                if( param_nr != params1.size()-1 ) {
                    param_file << ",";
                } else {
                    param_file << std::endl;
                }
            }
        }
                     
                     
        if( sample_nr % 1000 == 0 ) { std::cout << "Processing sample " << sample_nr << " with real_torque " << elbow_torque_gain*sample.getTorqueMeasure(elbow_torque_global_id) <<  std::endl;
                                      std::cout << "Static parameters estimated " << estimator_dynamic.getParameterEstimate() << std::endl;
        }

       
    }
   
    results_file.close();
    
    if( output_param ) {
        param_file.close();
    }
    
    std::cout << "iCubParis02_analysis: saved result to " << std::endl;
    std::cout << "iCubParis02_analysis: got " <<sample_nr << " samples "  << std::endl;
    
    return 0;
}