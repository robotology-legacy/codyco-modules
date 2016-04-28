#include "QuaternionEKF.h"

REGISTERIMPL(QuaternionEKF);

using namespace yarp::os;
using namespace yarp::math;

QuaternionEKF::QuaternionEKF() : m_className("QuaternionEKF")
{}

bool QuaternionEKF::init(ResourceFinder &rf, wbi::iWholeBodySensors *wbs)
{
    eulerAngles.assign(3, 0.0);
    //TODO: Maybe instead of passing the resourceFinder I should pass the already parsed groups?
    // Module Parameters

    //TODO: The class name should be addressed better. I had thought of a private member of this class enforced through IEstimator, but its quality of Interface shouldn't allow for private variables.
    // Read filter parameters
    if ( !readEstimatorParams(rf, this->m_quaternionEKFParams) )
    {
        yError("[QuaternionEKF::init()] Failed ");
        return false;
    } else {
        yInfo("[QuaternionEKF::init()]QuaternionEKF estimator parsed parameters from configuration file correctly. ");
    }

    // Open publisher ports
    //TODO: These publisher ports should be addressed automatically by some publishing interface as stated in issue ...
    // enum ORIENTATION_ESTIMATE_PORT_QUATERNION
    publisherPortStruct orientationEstimatePort;
    if ( !orientationEstimatePort.configurePort(this->m_className, std::string("filteredOrientation")) )
    {
        yError("[QuaternionEKF::init()] Estimates port in quaternions could not be configured");
        return false;
    } else {
        m_outputPortsList.push_back(orientationEstimatePort);
    }

    // Open publisher port for estimate in euler angles
    // enum ORIENTATION_ESTIMATE_PORT_EULER
    publisherPortStruct orientationEstimateEulerPort;
    if ( !orientationEstimateEulerPort.configurePort(this->m_className, std::string("filteredOrientationEuler")) )
    {
        yError("[QuaternionEKF::init] Estimate port in euler could not be configured");
        return false;
    } else {
        m_outputPortsList.push_back(orientationEstimateEulerPort);
    }

    // Open publisher port for raw accelerometer data
    // enum RAW_ACCELEROMETER_DATA_PORT
    publisherPortStruct rawAccDataPort;
    if ( !rawAccDataPort.configurePort(this->m_className, std::string("rawAccMeas")) )
    {
        yError("[QuaternionEKF::init] Raw accelerometer port could not be configured");
        return false;
    } else {
        m_outputPortsList.push_back(rawAccDataPort);
    }

    // Open publisher port for raw gyroscope data
    // enum RAW_GYROSCOPE_DATA_PORT
    publisherPortStruct rawGyroDataPort;
    if ( !rawGyroDataPort.configurePort(this->m_className, std::string("rawGyroMeas")) )
    {
        yError("[QuaternionEKF::init] Raw gyroscope data port could not be configured");
        return false;
    } else {
        m_outputPortsList.push_back(rawGyroDataPort);
    }
    
    // Open publisher port for floating base rotation matrix estimate
    // enum FLOATING_BASE_ROTATION_PORT
    publisherPortStruct floatingBaseRotPort;
    if ( !floatingBaseRotPort.configurePort(this->m_className, std::string("floatingBaseRotationMatrix")) )
    {
        yError("[QuaternionEKF::init] Floating base rotation matrix estimate port could not be configured");
        return false;
    } else {
        m_outputPortsList.push_back(floatingBaseRotPort);
    }

    //FIXME: Temporary, while yarpWholeBodySensors is finished.
    // Open sensor ports
    sensorMeasPort = new yarp::os::Port;
    readerPortStruct sensorDataPort;
    std::string srcPort = std::string("/" + this->m_quaternionEKFParams.robotPrefix + "/right_leg/inertialMTB");
    if ( !sensorDataPort.configurePort(this->m_className, std::string("rightFootMTBreader"), srcPort, sensorMeasPort) )
    {
        yError("[QuaternionEKF::init] Could not connect to source port");
        return false;
    } else {
        m_inputPortsList.push_back(sensorDataPort);
    }
    
    std::string srcPortFloatingBasePose = "/LeggedOdometry/floatingbasestate:o";
//    std::cerr << "[QuaternionEKF] Checking existance of floating base port ... " << std::endl;
//    if ( yarp::os::Network::exists(srcPortFloatingBasePose.c_str()) )
//    {
        // Create reader for the result of LeggedOdometry
        floatingBasePoseExt = new yarp::os::Port;
        readerPortStruct floatingBasePoseExtPort;
        if ( !floatingBasePoseExtPort.configurePort(this->m_className, std::string("floatingBasePose"), srcPortFloatingBasePose, floatingBasePoseExt) )
        {
            yError("[QuaternionEKF::init] Could not connect to %s ", srcPortFloatingBasePose.c_str());
            return false;
        } else {
            m_inputPortsList.push_back(floatingBasePoseExtPort);
        }
//    } else {
//        yError("[QuaternionEKF::init] /LeggedOdometry/floatingbasestate:o does not exist! ");
//    }
    


    // Initialize rf-dependent private variables
    m_prior_mu_vec.resize(m_quaternionEKFParams.stateSize);
    m_posterior_state.resize(m_quaternionEKFParams.stateSize);

    // Create system model
    createSystemModel();

    // Create measurement model
    createMeasurementModel();

    // Setting priors
    setPriors();

    // Create filter
    createFilter();

    // Initialize measurement object
    measurements.linAcc.resize(3,0.0);
    measurements.angVel.resize(3,0.0);

    // Should this module provide the floating base attitude in the world defined by QuaternionEKF?
    if (m_quaternionEKFParams.floatingBaseAttitude)
    {
        m_floatingBaseEstimate = new wholeBodyEstimator::floatingBase;

        // Retrieve wbiProperties
//        std::string wbiConfFile;
//        yarp::os::Property yarpWbiOptions;
//        wbi::IDList RobotDynamicModelJoints;
//
//        if (!rf.check("wbi_conf_file"))
//        {
//            yError("[QuaternionEKF::init] WBI configuration file name not specified in config file of this module.");
//            return false;
//        } else
//        {
//            wbiConfFile = rf.findFile("wbi_conf_file");
//            if ( !yarpWbiOptions.fromConfigFile(wbiConfFile) )
//            {
//                yError("[QuaternionEKF::init] File %s does not exist and could not be read", wbiConfFile.c_str());
//                return false;
//            }
//        }
//        // Configure yarpWholeBodySensors
//        // Get model joints list
//        std::string modelJointsListName = rf.check("joints_list",
//                                                   yarp::os::Value("ROBOT_DYNAMIC_MODEL_JOINTS"),
//                                                   "Name of the list of joint used for the current robot").asString().c_str();
//        if( !yarpWbi::loadIdListFromConfig(modelJointsListName,yarpWbiOptions,RobotDynamicModelJoints) )
//        {
//            yError("[QuaternionEKF::configure] Impossible to load wbiId joint list with name %s\n",modelJointsListName.c_str());
//            return false;
//        }

        // Retrieve rot_from_ft_to_acc matrix
        MatrixWrapper::Matrix mat_rot_from_ft_to_acc(3,3);
        unsigned int k = 0;
        for (unsigned int i = 0; i < 3; i++)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                mat_rot_from_ft_to_acc(i+1,j+1) = m_quaternionEKFParams.rot_from_ft_to_acc_bottle->get(k).asDouble();
                k++;
            }
        }
        m_floatingBaseEstimate->configure(rf, mat_rot_from_ft_to_acc, wbs);
    }

    yInfo("[QuaternionEKF::init] QUATERNIONEKF is running... \n");

    return true;
}

void QuaternionEKF::run()
{
    // Read sensor data
//    std::cerr << "[QuaternionEKF] Reading sensor data ... " << std::endl;
    if ( !readSensorData(measurements) )
    {
        yWarning("[QuaternionEKF::run] SENSOR DATA COULD NOT BE READ!");
    } else {
        //yInfo("[QuaternionEKF::run] Parsed sensor data: \n Acc [m/s^2]: \t%s \n Ang Vel [deg/s]: \t%s \n",  (measurements.linAcc).toString().c_str(), (measurements.angVel).toString().c_str());
    }

    // Copy ang velocity data from a yarp vector into a ColumnVector
    MatrixWrapper::ColumnVector input(measurements.angVel.data(),m_quaternionEKFParams.inputSize);
    // Copy accelerometer data from a yarp vector into a ColumnVector
    MatrixWrapper::ColumnVector measurement(measurements.linAcc.data(),m_quaternionEKFParams.measurementSize);

    // Noise gaussian
    // System Noise Mean
    //TODO: [NOT SURE] This mean changes!!!
    MatrixWrapper::ColumnVector sys_noise_mu(m_quaternionEKFParams.stateSize);
    sys_noise_mu = 0.0;

    /**************** System Noise Covariance *********************************************************/
    MatrixWrapper::Matrix Xi(m_quaternionEKFParams.stateSize, m_quaternionEKFParams.inputSize);
    XiOperator(m_posterior_state, &Xi);
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_quaternionEKFParams.stateSize);
    sys_noise_cov = 0.0;
    // NOTE m_sigma_gyro must be small, ||ek|| = 10e-3 rad/sec
    MatrixWrapper::Matrix Sigma_gyro(m_quaternionEKFParams.inputSize,m_quaternionEKFParams.inputSize);
    Sigma_gyro = 0.0;
    Sigma_gyro(1,1) = Sigma_gyro(2,2) = Sigma_gyro(3,3) = m_quaternionEKFParams.sigmaGyro;
    MatrixWrapper::Matrix tmp = Xi*Sigma_gyro*Xi.transpose();
    //NOTE: on 30-07-2015 I commented the following lines because making this matrix symmetric this way does not make much sense from a theoretical point of view. I'd rather add a term such as alpha*I_4x4
    //         MatrixWrapper::SymmetricMatrix tmpSym(m_state_size);
    //         tmp.convertToSymmetricMatrix(tmpSym);
    sys_noise_cov = (MatrixWrapper::SymmetricMatrix) tmp*pow(m_quaternionEKFParams.period/(1000.0*2.0),2);
    //NOTE: Next line is setting system noise covariance matrix to a constant diagonal matrix
    //         sys_noise_cov = 0.0; sys_noise_cov(1,1) = sys_noise_cov (2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = 0.000001;
    /**************** ENDS System Noise Covariance *******************************************************/

    //std::cout << "System covariance matrix will be: " << std::endl << sys_noise_cov << std::endl;

    //FIXME: Remove the line below as the mean pretty much never changes and remains zero
    //m_sysPdf->AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf->AdditiveNoiseSigmaSet(sys_noise_cov);

    if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
    {
        yError("[QuaternionEKF::run] Update step of the Kalman Filter could not be performed\n");
    }

    // Get the posterior of the updated filter. Result of all the system model and meaurement information
    BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
    // Posterior Expectation
    m_posterior_state = posterior->ExpectedValueGet();
    MatrixWrapper::Quaternion expectedValueQuat(m_posterior_state);
    // Posterior Covariance
    MatrixWrapper::SymmetricMatrix covariance(m_quaternionEKFParams.stateSize);
    covariance = posterior->CovarianceGet();
    //std::cout << "[QuaternionEKF::run] Posterior Mean: " << expectedValueQuat << std::endl;
    //std::cout << "Posterior Covariance: " << posterior->CovarianceGet() << std::endl;
    MatrixWrapper::Quaternion tmpQuat(expectedValueQuat);
    //tmpQuat.getEulerAngles(std::string("xyz"), eulerAngles);
    expectedValueQuat.conjugate(tmpQuat);
    tmpQuat.getEulerAngles(std::string("xyz"), eulerAngles);
    //std::cout << "[QuaternionEKF::run] Posterior Mean in Euler Angles: " << (180/PI)*eulerAngles  << std::endl;
    // Publish results to port
    yarp::sig::Vector tmpVec(m_quaternionEKFParams.stateSize);
    for (unsigned int i=1; i<m_posterior_state.size()+1; i++) {
        tmpVec(i-1) = m_posterior_state(i);
    }
    // Publish Euler Angles estimate to port
    //TODO: Check why I need to do this multiplication in this particular way.
    yarp::sig::Vector tmpEuler(3);
    for (unsigned int i=1; i<eulerAngles.rows()+1; i++)
        tmpEuler(i-1) = eulerAngles(i)*(180/PI);
    // Writing to port the full estimated orientation in Euler angles (xyz order)
    yarp::sig::Vector& tmpPortEuler = m_outputPortsList[ORIENTATION_ESTIMATE_PORT_EULER].outputPort->prepare();
    tmpPortEuler = tmpEuler;
    m_outputPortsList[ORIENTATION_ESTIMATE_PORT_EULER].outputPort->write();
    // Writing to port the full estimated quaternion
    yarp::sig::Vector& tmpPortRef = m_outputPortsList[ORIENTATION_ESTIMATE_PORT_QUATERNION].outputPort->prepare();
    tmpPortRef = tmpVec;
    m_outputPortsList[ORIENTATION_ESTIMATE_PORT_QUATERNION].outputPort->write();



    /**
     *  Estimate floating base attitude
     */
    // rot_from_world_to_sensor which is the result of the estimate a.k.a. tmpQuat in previous lines.
    MatrixWrapper::Matrix rot_from_world_to_sensor(3,3);
    rot_from_world_to_sensor = 0;
    // Intentionally setting the yaw to zero as this is not yet properly estimated from magnetometer measurements.
    rot_from_world_to_sensor.eulerToRotation(eulerAngles(1), eulerAngles(2), 0);
    // When using Eigen I need to transpose this matrix 
    rot_from_world_to_sensor = rot_from_world_to_sensor.transpose();
    // Output matrix rot_from_floatingBase_to_world
//    std::cerr << "[QuaternionEKF::run] Rotation from world to sensor: " << std::endl << rot_from_world_to_sensor << std::endl;
    MatrixWrapper::Matrix rot_from_floatingBase_to_world(3,3);
    rot_from_floatingBase_to_world = 0;
    if ( m_quaternionEKFParams.floatingBaseAttitude )
    {
        m_floatingBaseEstimate->compute_Rot_from_floatingBase_to_world(rot_from_world_to_sensor, rot_from_floatingBase_to_world);
    }
    

    /**
     *  Streaming of measurements
     */

     if ( this->m_quaternionEKFParams.streamMeasurements )
     {
         yarp::sig::Vector& tmpRawAccPortRef = m_outputPortsList[RAW_ACCELEROMETER_DATA_PORT].outputPort->prepare();
         tmpRawAccPortRef = measurements.linAcc;
         m_outputPortsList[RAW_ACCELEROMETER_DATA_PORT].outputPort->write();

         yarp::sig::Vector& tmpRawGyroPortRef = m_outputPortsList[RAW_GYROSCOPE_DATA_PORT].outputPort->prepare();
         tmpRawGyroPortRef = measurements.angVel;
         m_outputPortsList[RAW_GYROSCOPE_DATA_PORT].outputPort->write();
     }
    
    /**
     *  Retrieving floating base position estimated by LeggedOdometry if available
     *  Currently, this is actually the position of l_sole from the floating base (root) expressed in root.
     */
    
    yarp::os::Bottle floatingBasePositionBottle;
    yarp::sig::Vector floatingBasePosition(3);
    floatingBasePosition = 0;

//    std::cerr << "[QuaternionEKF] Checking existance of floating base port ... " << std::endl;
//    if ( yarp::os::Network::exists("/LeggedOdometry/floatingbasestate:o") )
//    {
//        floatingBasePoseExt->read(floatingBasePositionBottle);
        // Copying floating base position vector into floatingBasePosition
//        floatingBasePosition(0) = floatingBasePositionBottle.get(0).asList()->get(0).asDouble();
//        floatingBasePosition(1) = floatingBasePositionBottle.get(0).asList()->get(1).asDouble();
//        floatingBasePosition(2) = floatingBasePositionBottle.get(0).asList()->get(2).asDouble();
//        yInfo("floating base position vector: %s", floatingBasePosition.toString().c_str());
//    }
    
    
    /**
     *  Streaming floating base attitude
     */
    
    if ( m_quaternionEKFParams.floatingBaseAttitude )
    {
        // Creating roto-translation matrix
        MatrixWrapper::Matrix rotoTrans_from_floatingBase_to_world(4,4);
        rotoTrans_from_floatingBase_to_world = 0;
        // Copying rotational part
        rotoTrans_from_floatingBase_to_world.setSubMatrix(rot_from_floatingBase_to_world, 1, 3, 1, 3);
        // Copying translational part
        MatrixWrapper::ColumnVector pos_from_floatingBase_to_foot_in_floatingBase(floatingBasePosition.data(),3);
//        std::cerr << "[QuaternionEKF] pos from floating base to foot in floating base: " << std::endl << pos_from_floatingBase_to_foot_in_floatingBase << std::endl;
        MatrixWrapper::ColumnVector tmpPosition = rot_from_floatingBase_to_world*pos_from_floatingBase_to_foot_in_floatingBase;
//        std::cerr << "[QuaternionEKF] pos from floating base to foot in world" << std::endl << tmpPosition << std::endl;
        
        // Position will contain the homogeneous position vector from floating base to l_sole expressed in floating base.
        MatrixWrapper::ColumnVector position(4);
        position = 0; // all rows 1. After setting the first three values, the last one will remain 1.
        position(1) = tmpPosition(1);
        position(2) = tmpPosition(2);
        position(3) = tmpPosition(3);
        //TODO: Add to this vector the position vector from <world> to <l_sole> expressed in <world> should be roughly [0 0 distance_from_l_sole_to_accelerometer]
        MatrixWrapper::ColumnVector pos_from_world_to_lsole_in_world(4); pos_from_world_to_lsole_in_world = 0;
        pos_from_world_to_lsole_in_world(3) = 0.01;
        pos_from_world_to_lsole_in_world(4) = 1;
        MatrixWrapper::ColumnVector pos_from_world_to_floatingBase_in_world = pos_from_world_to_lsole_in_world - position;
//        std::cerr << "[Quaternion] Position from world to floating base in world: " << pos_from_world_to_floatingBase_in_world << std::endl;
        rotoTrans_from_floatingBase_to_world.setColumn(pos_from_world_to_floatingBase_in_world, 4);
        
        
        yarp::sig::Vector tmpRotMatVec(16);
        // Streaming columnwise
        unsigned int k = 0;
        for (unsigned int j=0; j<4; j++)
        {
            for (unsigned int i=0; i<4; i++)
            {
                tmpRotMatVec(k) = rotoTrans_from_floatingBase_to_world(i+1,j+1);
                k++;
            }
        }
        yarp::sig::Vector &tmpFloatingBaseRotation = m_outputPortsList[FLOATING_BASE_ROTATION_PORT].outputPort->prepare();
        tmpFloatingBaseRotation = tmpRotMatVec;
        m_outputPortsList[FLOATING_BASE_ROTATION_PORT].outputPort->write();
    }
}

void QuaternionEKF::release()
{
    yInfo("[QuaternionEKF::release] Destroying QuaternionEKF");
    yInfo("Destroying QuaternionEKF");
    if (m_quaternionEKFParams.floatingBaseAttitude)
    {
        yInfo("[QuaternionEKF::~QuaternionEKF] Deleting m_floatingBaseEstimate");
        delete m_floatingBaseEstimate;
        m_floatingBaseEstimate = 0;
        yInfo("m_floatingBaseEstimate deleted");

        yInfo("[QuaternionEKF::~QuaternionEKF] Deleting m_quaternionEKFParams.rot_from_ft_to_acc_bottle");
        delete m_quaternionEKFParams.rot_from_ft_to_acc_bottle;
        m_quaternionEKFParams.rot_from_ft_to_acc_bottle = 0;
        yDebug("[QuaternionEKF::~QuaternionEKF] Deallocated m_quaternionEKFParams.rot_from_ft_to_acc_bottle");
    }
    if (m_filter)
    {
        delete m_filter;
        m_filter = NULL;
        yDebug("[QuaternionEKF::~QuaternionEKF] m_filter deleted");
    }
    if (m_measPdf)
    {
        delete m_measPdf;
        m_measPdf = NULL;
        yDebug("[QuaternionEKF::~QuaternionEKF] m_measPdf deleted");
    }
    if (m_measurement_uncertainty)
    {
        delete m_measurement_uncertainty;
        m_measurement_uncertainty = NULL;
        yDebug("[QuaternionEKF::~QuaternionEKF] m_measurement_uncertainty deleted");
    }
    if (m_meas_model)
    {
        delete m_meas_model;
        m_meas_model = NULL;
        yDebug("[QuaternionEKF::~QuaternionEKF] m_meas_model deleted");
    }
    if (m_prior)
    {
        delete m_prior;
        m_prior = NULL;
        yDebug("[QuaternionEKF::~QuaternionEKF] m_prior deleted");
    }
    if (m_sys_model)
    {
        delete m_sys_model;
        m_sys_model = NULL;
        yDebug("[QuaternionEKF::~QuaternionEKF] m_sys_model deleted");
    }
    
    // Closing ports
//    for (std::vector<publisherPortStruct>::iterator it = m_outputPortsList.begin() ; it != m_outputPortsList.end() ; it++)
//    {
//        yInfo("[QuaternionEKF::release] Closing ports...");
//        publisherPortStruct tmpPort = *m_outputPortsList.pop_back();
//        tmpPort.close();
//    }
    
    for (unsigned i = 0; i < m_outputPortsList.size(); ++i)
    {
        yInfo("[QuaternionEKF::release] ... closing output port");
        m_outputPortsList[i].close();
    }
    
    for (unsigned i = 0; i < m_inputPortsList.size(); ++i)
    {
        yInfo("[QuaternionEKF::release] ... closing input port");
        m_inputPortsList[i].close();
    }
    
    yInfo("[QuaternionEKF::~QuaternionEKF] QuaternionEKF destroyed correctly");


}

bool QuaternionEKF::readEstimatorParams(yarp::os::ResourceFinder &rf, quaternionEKFParams &estimatorParams)
{
    yarp::os::Bottle botParams;
    botParams = rf.findGroup("QuaternionEKF");
    yInfo("QuaternionEKF params are: %s ", botParams.toString().c_str());
    if ( botParams.isNull() )
    {
        yError("[QuaternionEKF::readEstimatorParams] No parameters were read from QuaternionEKF group");
        return false;
    } else {
        estimatorParams.stateSize = botParams.find("state_size").asInt();
        estimatorParams.inputSize = botParams.find("input_size").asInt();
        estimatorParams.measurementSize = botParams.find("measurement_size").asInt();
        estimatorParams.muSystemNoise = botParams.find("mu_system_noise").asDouble();
        estimatorParams.sigmaSystemNoise = botParams.find("sigma_system_noise").asDouble();
        estimatorParams.sigmaMeasurementNoise = botParams.find("sigma_measurement_noise").asDouble();
        estimatorParams.sigmaGyro = botParams.find("sigma_gyro_noise").asDouble();
        estimatorParams.piorMu = botParams.find("prior_mu_state").asDouble();
        estimatorParams.priorCovariance = botParams.find("prior_cov_state").asDouble();
        estimatorParams.muGyroNoise = botParams.find("mu_gyro_noise").asDouble();
        estimatorParams.floatingBaseAttitude = botParams.find("floating_base_attitude").asBool();
        estimatorParams.rot_from_ft_to_acc_bottle = new yarp::os::Bottle(*botParams.find("rot_from_ft_to_acc").asList());
    }

    botParams.clear();
    botParams = rf.findGroup("module_parameters");
    if ( botParams.isNull() )
    {
        yError("[QuaternionEKF::readEstimatorParams] No parameters were read from 'module_params' group. period is needed!");
        return false;
    } else {
        estimatorParams.period = botParams.find("period").asInt();
        estimatorParams.robotPrefix = botParams.find("robot").asString();
        estimatorParams.streamMeasurements = botParams.find("stream_measurements").asBool();
    }
    return true;
}

void QuaternionEKF::createSystemModel()
{
    // System Noise Mean
    MatrixWrapper::ColumnVector sys_noise_mu(m_quaternionEKFParams.stateSize);
    sys_noise_mu(1) = sys_noise_mu(2) = sys_noise_mu(3) = sys_noise_mu(4) = 0.0;

    // System Noise Covariance
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_quaternionEKFParams.stateSize);
    sys_noise_cov = 0.0;
    sys_noise_cov(1,1) = sys_noise_cov(2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = m_quaternionEKFParams.sigmaSystemNoise;

    // Setting System noise uncertainty
    m_sysPdf = new BFL::nonLinearAnalyticConditionalGaussian(m_quaternionEKFParams.stateSize);
    m_sysPdf->AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf->AdditiveNoiseSigmaSet(sys_noise_cov);
    m_sysPdf->setPeriod(m_quaternionEKFParams.period);
    // Creating the model
    m_sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(m_sysPdf);

}

void QuaternionEKF::createMeasurementModel()
{
    // Creating measurement model for linear measurement model
    // Measurement noise distribution
    // Measurement noise mean
    MatrixWrapper::ColumnVector meas_noise_mu(m_quaternionEKFParams.measurementSize);
    meas_noise_mu = 0.0;                // Set all to zero
    meas_noise_mu(3) = 0.0;
    // Measurement noise covariance
    MatrixWrapper::SymmetricMatrix meas_noise_cov(m_quaternionEKFParams.measurementSize);
    meas_noise_cov = 0.0;
    meas_noise_cov(1,1) = meas_noise_cov(2,2) = meas_noise_cov(3,3) = m_quaternionEKFParams.sigmaMeasurementNoise;
    // Measurement noise uncertainty
    //TODO: Remember to delete this object at the end
    m_measurement_uncertainty = new BFL::Gaussian(meas_noise_mu, meas_noise_cov);
    // Probability density function (PDF) for the measurement
    m_measPdf = new BFL::nonLinearMeasurementGaussianPdf(*m_measurement_uncertainty);
    //  Measurement model from the measurement PDF
    m_meas_model = new BFL::AnalyticMeasurementModelGaussianUncertainty(m_measPdf);

}

void QuaternionEKF::setPriors()
{
    // Setting prior. This is equivalent to a zero rotation
    MatrixWrapper::ColumnVector prior_mu(m_quaternionEKFParams.stateSize);
    prior_mu = 0.0;
    prior_mu(1) = 1.0;
    m_prior_mu_vec = prior_mu;
    m_posterior_state = prior_mu;
    MatrixWrapper::SymmetricMatrix prior_cov(4);
    prior_cov = 0.0;
    prior_cov(1,1) = prior_cov(2,2) = prior_cov(3,3) = prior_cov(4,4) = m_quaternionEKFParams.priorCovariance;
    yInfo("[QuaternionEKF::setPriors] Priors will be: ");
    std::cout << "[QuaternionEKF::setPriors] State prior:" << std::endl << prior_mu << std::endl;
    std::cout << "[QuaternionEKF::setPriors] Covariance prior: " << std::endl << prior_cov << std::endl;
    m_prior = new BFL::Gaussian(prior_mu, prior_cov);

}

void QuaternionEKF::createFilter()
{
    m_filter = new BFL::ExtendedKalmanFilter(m_prior);
}

bool QuaternionEKF::readSensorData(measurementsStruct &meas)
{
    //FIXME: TEMPORARY WHILE WHOLEBODYSENSORS IS FINISHED
    if ( extractMTBDatafromPort(MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID, meas) )
        return true;
    else {
        yError("[QuaternionEKF::readSensorData] QuaternionEKF was not able to read sensor data.");
        return false;
    }

}

bool QuaternionEKF::extractMTBDatafromPort(int boardNum, measurementsStruct &measurements)
{
    yarp::sig::Vector fullMeasurement;
    yarp::os::Bottle measurementBottle;
    int indexSubVector = 0;
    if ( !sensorMeasPort->read(fullMeasurement) ) {
        yError("[QuaternionEKF::extractMTBDatafromPort] There was an error trying to read from the MTB port");
        return false;
    } else {
        //yInfo("[QuaternionEKF::extractMTBDatafromPort] Raw meas: %s", fullMeasurement.toString().c_str());
        /******************* searching for multiple instances of the sensor  **************************/
        double* tmp;
        tmp = fullMeasurement.data();
        double *it = tmp + 2; // First two elements of the vector can be skipped
        while (it < tmp + fullMeasurement.size()) {
            it = std::find(it, it + (fullMeasurement.size() - indexSubVector), boardNum);
            if (it < tmp + fullMeasurement.size()) {
                indexSubVector = (int)(it - tmp) + 1;

                // Parse sensor data
                int trueindexSubVector = indexSubVector - 1;
                if ( tmp[trueindexSubVector]  == boardNum ) {
                    //  If sensor from board "boardNum" is an accelerometer
                    if ( tmp[trueindexSubVector + 1] == 1.0) {
                        measurements.linAcc(0) = tmp[trueindexSubVector + 3];
                        measurements.linAcc(1) = tmp[trueindexSubVector + 4];
                        measurements.linAcc(2) = tmp[trueindexSubVector + 5];
                    } else {
                        // If sensor from board "boardNum" is a gyroscope
                        if ( tmp[trueindexSubVector + 1] == 2.0 ) {
                            measurements.angVel(0) = tmp[trueindexSubVector + 3];
                            measurements.angVel(1) = tmp[trueindexSubVector + 4];
                            measurements.angVel(2) = tmp[trueindexSubVector + 5];
                        }
                    }
                }
                it = it + MTB_PORT_DATA_PACKAGE_OFFSET; //Move to the next position in the vector where a package data is expected
            }
        }
        /**********************************************************************************/
        measurements.linAcc = measurements.linAcc*CONVERSION_FACTOR_ACC;
        if (yarp::math::norm(measurements.linAcc) > 11.0) {
            yWarning("[QuaternionEKF::extractMTBDatafromPort]  WARNING!!! Gravity's norm is too big!");
        }
        // This change of signs is just to rotate the gyro reference frame to match the accelerometer's ref frame given the way it's been installed on the foot.
        measurements.angVel(1) = measurements.angVel(1);
        measurements.angVel(2) = measurements.angVel(2);
        measurements.angVel = PI/180*CONVERSION_FACTOR_GYRO*measurements.angVel;
        if (yarp::math::norm(measurements.angVel) > 100.0) {
            yWarning("[QuaternionEKF::extractMTBDatafromPort]  WARNING!!! [QuaternionEKF::extractMTBDatafromPort] Ang vel's norm is too big!");
        }
    }
    return true;
}

void QuaternionEKF::XiOperator ( MatrixWrapper::ColumnVector quat, MatrixWrapper::Matrix* Xi )
{
    //     In  Matlab language this would be:
    //     Xi = [       -qk(2:4,:)'           ;
    //           qk(1)*eye(3) + S(qk(2:4,:)) ];
    // NOTE When testing Eigen I changed the following line from 1.0 to 0.0. Couldn't understand why I would have put 1.0 for instance
    (*Xi) = 0.0;
    MatrixWrapper::ColumnVector omg(3);
    omg(1) = quat(2);    omg(2) = quat(3);    omg(3) = quat(4);
    (*Xi)(1,1) = -quat(2);    (*Xi)(1,2) = -quat(3);    (*Xi)(1,3) = -quat(4);
    MatrixWrapper::Matrix eye(3,3);
    eye.toIdentity();
    MatrixWrapper::Matrix S(3,3);
    SOperator(omg, &S);
    MatrixWrapper::Matrix tmpAdd = eye*quat(1) + S;
    Xi->setSubMatrix(tmpAdd,2,4,1,3);
}

void QuaternionEKF::SOperator ( MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix* S )
{
    (*S)(1,1) = 0.0;    (*S)(1,2) = -omg(3); (*S)(1,3) = omg(2);
    (*S)(2,1) = omg(3); (*S)(2,2) = 0.0    ; (*S)(2,3) = -omg(1);
    (*S)(3,1) = -omg(2);(*S)(3,2) = omg(1) ; (*S)(3,3) = 0.0;
}

QuaternionEKF::~QuaternionEKF()
{}
