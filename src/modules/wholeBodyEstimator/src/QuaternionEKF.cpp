#include "QuaternionEKF.h"

REGISTERIMPL(QuaternionEKF);

using namespace yarp::os;
using namespace yarp::math;

QuaternionEKF::QuaternionEKF() : m_className("QuaternionEKF")
{}

QuaternionEKF::~QuaternionEKF()
{}

bool QuaternionEKF::init(ResourceFinder &rf, wbi::iWholeBodySensors *wbs)
{
    // NOTE Maybe instead of passing the resourceFinder I should pass the already parsed groups?
    // Module Parameters
    
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
        yError("[QuaternionEKF::init()] Estimate port in euler could not be configured");
        return false;
    } else {
        m_outputPortsList.push_back(orientationEstimateEulerPort);
    }
    
    // TEMPORARY
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
    yInfo("QUATERNIONEKF is running...");
    
    return true;
}

void QuaternionEKF::run()
{
    // Read sensor data
    if ( !readSensorData(measurements) )
    {
        yWarning("QuaternionEKF::run() SENSOR DATA COULD NOT BE READ!");
    } else {
        yInfo("[QuaternionEKF::run] Parsed sensor data: \n Acc [m/s^2]: \t%s \n Ang Vel [deg/s]: \t%s \n",  (measurements.linAcc).toString().c_str(), (measurements.angVel).toString().c_str());
    }
    
    // Copy ang velocity data from a yarp vector into a ColumnVector
    MatrixWrapper::ColumnVector input(measurements.angVel.data(),m_quaternionEKFParams.inputSize);
    // Copy accelerometer data from a yarp vector into a ColumnVector
    MatrixWrapper::ColumnVector measurement(measurements.linAcc.data(),m_quaternionEKFParams.measurementSize);

    // Noise gaussian
    // System Noise Mean
    // TODO [NOT SURE] This mean changes!!!
    MatrixWrapper::ColumnVector sys_noise_mu(m_quaternionEKFParams.stateSize);
    sys_noise_mu = 0.0;
    
    /**************** System Noise Covariance ********************************************************************************/
    MatrixWrapper::Matrix Xi(m_quaternionEKFParams.stateSize, m_quaternionEKFParams.inputSize);
    XiOperator(m_posterior_state, &Xi);
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_quaternionEKFParams.stateSize);
    sys_noise_cov = 0.0;
    // NOTE m_sigma_gyro must be small, ||ek|| = 10e-3 rad/sec
    MatrixWrapper::Matrix Sigma_gyro(m_quaternionEKFParams.inputSize,m_quaternionEKFParams.inputSize);
    Sigma_gyro = 0.0;
    Sigma_gyro(1,1) = Sigma_gyro(2,2) = Sigma_gyro(3,3) = m_quaternionEKFParams.sigmaGyro;
    MatrixWrapper::Matrix tmp = Xi*Sigma_gyro*Xi.transpose();
    // NOTE on 30-07-2015 I commented the following lines because making this matrix symmetric this way does not make much sense from a theoretical point of view. I'd rather add a term such as alpha*I_4x4
    //         MatrixWrapper::SymmetricMatrix tmpSym(m_state_size);
    //         tmp.convertToSymmetricMatrix(tmpSym);
    sys_noise_cov = (MatrixWrapper::SymmetricMatrix) tmp*pow(m_quaternionEKFParams.period/(1000.0*2.0),2);
    // NOTE Next line is setting system noise covariance matrix to a constant diagonal matrix
    //         sys_noise_cov = 0.0; sys_noise_cov(1,1) = sys_noise_cov (2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = 0.000001;
    /****************END System Noise Covariance *********************************************************************************/
    
    //std::cout << "System covariance matrix will be: " << std::endl << sys_noise_cov << std::endl;
    
    m_sysPdf->AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf->AdditiveNoiseSigmaSet(sys_noise_cov);

    if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
    {
        yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    }
    
    // Get the posterior of the updated filter. Result of all the system model and meaurement information
    BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
    // Posterior Expectation
    m_posterior_state = posterior->ExpectedValueGet();
    MatrixWrapper::Quaternion expectedValueQuat(m_posterior_state);
    // Posterior Covariance
    MatrixWrapper::SymmetricMatrix covariance(m_quaternionEKFParams.stateSize);
    covariance = posterior->CovarianceGet();
    std::cout << "Posterior Mean: " << expectedValueQuat << std::endl;
    //std::cout << "Posterior Covariance: " << posterior->CovarianceGet() << std::endl;
    MatrixWrapper::ColumnVector eulerAngles(3);
    //MatrixWrapper::Quaternion tmpQuat(expectedValueQuat);
    //tmpQuat.getEulerAngles(std::string("xyz"), eulerAngles);
    expectedValueQuat.getEulerAngles(std::string("xyz"), eulerAngles);
    std::cout << "Posterior Mean in Euler Angles: " << (180/PI)*eulerAngles  << std::endl;
    // Publish results to port
    yarp::sig::Vector tmpVec(m_quaternionEKFParams.stateSize);
    for (unsigned int i=1; i<m_posterior_state.size()+1; i++) {
        tmpVec(i-1) = m_posterior_state(i);
    }
    // Publish Euler Angles estimation to port
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

    //TODO RAW GYRO AND ACCELEROMETER DATA COULD BE STREAMED TOO
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
    botParams = rf.findGroup("MODULE_PARAMETERS");
    if ( botParams.isNull() )
    {
        yError("[QuaternionEKF::readEstimatorParams] No parameters were read from MODULE_PARAMS group. period is needed!");
        return false;
    } else {
        estimatorParams.period = botParams.find("period").asInt();
        estimatorParams.robotPrefix = botParams.find("robot").asString();
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

bool QuaternionEKF::readSensorData(measurementsStruct &meas)
{
    // TEMPORARY WHILE WHOLEBODYSENSORS IS FINISHED
    if ( extractMTBDatafromPort(MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID, meas) )
        return true;
    else {
        yError("QuaternionEKF was not able to read sensor data.");
        return false;
    }
    
}

bool QuaternionEKF::extractMTBDatafromPort(int boardNum, measurementsStruct &measurements)
{
    yarp::sig::Vector fullMeasurement;
    yarp::os::Bottle measurementBottle;
    int indexSubVector = 0;
    if ( !sensorMeasPort->read(fullMeasurement) ) {
        yError("[quaternionEKFThread::extractMTBDatafromPort] There was an error trying to read from the MTB port");
        return false;
    } else {
            //yInfo("[quaternionEKFThread::extractMTBDatafromPort] Raw meas: %s", fullMeasurement.toString().c_str());
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
            yWarning("WARNING!!! [quaternionEKFThread::run] Gravity's norm is too big!");
        }
        //TODO Check that this conversion factor is correct (doubts on the PI/180). See what was done in quaternionEKF
        measurements.angVel = PI/180*CONVERSION_FACTOR_GYRO*measurements.angVel;
        if (yarp::math::norm(measurements.angVel) > 100.0) {
            yWarning("WARNING!!! [quaternionEKFThread::run] Ang vel's norm is too big!");
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

