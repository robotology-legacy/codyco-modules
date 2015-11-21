#include "QuaternionEKF.h"

REGISTERIMPL(QuaternionEKF);

using namespace yarp::os;

QuaternionEKF::QuaternionEKF() : m_className("QuaternionEKF")
{ }

QuaternionEKF::~QuaternionEKF() {}

bool QuaternionEKF::init(ResourceFinder &rf, wbi::iWholeBodySensors *wbs)
{
    // Read filter parameters
    // The class name should be addressed better. I had thought of a private member of this class enforced through IEstimator, but its quality of Interface shouldn't allow for private variables.
    if ( !readEstimatorParams(rf, this->m_quaternionEKFParams) )
    {
        yError("[QuaternionEKF::init()] Failed ");
        return false;
    } else {
        yInfo("QuaternionEKF estimator parsed parameters from configuration file correctly.");
    }
    
    // Open publisher ports
    // TODO These publisher ports should be addressed automatically by some publishing interface as stated in issue
    publisherPortStruct orientationEstimatePort;
    if ( !orientationEstimatePort.configurePort(this->m_className, std::string("filteredOrientation")) )
    {
        yError("[QuaternionEKF::init()] Estimates port in quaternions could not be configured");
        return false;
    } else {
        m_portsList.push_back(orientationEstimatePort);
    }
    
    // Open publisher port for estimate in euler angles
    publisherPortStruct orientationEstimateEulerPort;
    if ( !orientationEstimateEulerPort.configurePort(this->m_className, std::string("filteredOrientationEuler")) )
    {
        yError("[QuaternionEKF::init()] Estimate port in euler could not be configured");
        return false;
    } else {
        m_portsList.push_back(orientationEstimateEulerPort);
    }
    
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
        
    yInfo("QuaternionEKF is running...");
    
    return true;
}

void QuaternionEKF::run()
{
    
    
}

void QuaternionEKF::release()
{
    
}

bool QuaternionEKF::readEstimatorParams(yarp::os::ResourceFinder &rf, quaternionEKFParams &estimatorParams)
{
    yarp::os::Bottle botParams;
    botParams = rf.findGroup("QuaternionEKF");
    if ( botParams.isNull() )
    {
        yError("[QuaternionEKF::readEstimatorParams] No parameters were read from QuaternionEKF group");
        return false;
    } else {
        estimatorParams.stateSize = botParams.find("STATE_SIZE").asInt();
        estimatorParams.inputSize = botParams.find("INPUT_SIZE").asInt();
        estimatorParams.measurementSize = botParams.find("MEASUREMENT_SIZE").asInt();
        estimatorParams.muSystemNoise = botParams.find("MU_SYSTEM_NOISE").asDouble();
        estimatorParams.sigmaSystemNoise = botParams.find("SIGMA_SYSTEM_NOISE").asDouble();
        estimatorParams.sigmaMeasurementNoise = botParams.find("SIGMA_MEASUREMENT_NOISE").asDouble();
        estimatorParams.sigmaGyro = botParams.find("SIGMA_GYRO_NOISE").asDouble();
        estimatorParams.piorMu = botParams.find("PRIOR_MU_STATE").asDouble();
        estimatorParams.priorCovariance = botParams.find("PRIOR_COV_STATE").asDouble();
        estimatorParams.muGyroNoise = botParams.find("MU_GYRO_NOISE").asDouble();
    }
    
    botParams.clear();
    botParams = rf.findGroup("MODULE_PARAMS");
    if ( botParams.isNull() )
    {
        yError("[QuaternionEKF::readEstimatorParams] No parameters were read from MODULE_PARAMS group. period is needed!");
        return false;
    } else {
        estimatorParams.period = botParams.find("period").asInt();
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
    yInfo("Priors will be: ");
    std::cout << "State prior:" << std::endl << prior_mu << std::endl;
    std::cout << "Covariance prior: " << std::endl << prior_cov << std::endl;
    m_prior = new BFL::Gaussian(prior_mu, prior_cov);

}

void QuaternionEKF::createFilter()
{
    m_filter = new BFL::ExtendedKalmanFilter(m_prior);
}