/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Jorhabib Eljaik
 * email:  jorhabib.eljaik@iit.it
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

#include "quaternionEKFThread.h"

#include <iCub/ctrl/filters.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace filter;
using namespace iCub::ctrl;
using namespace yarp::math;

quaternionEKFThread::quaternionEKFThread ( int period,
                                           std::string moduleName,
                                           std::string robotName,
                                           bool autoconnect,
                                           bool usingxsens,
                                           bool usingEKF,
                                           bool inWorldRefFrame,
                                           double gravityVec,
                                           bool usingSkin,
                                           std::string sensorPort,
                                           bool debugGyro,
                                           bool debugAcc,
                                           bool verbose,
                                           yarp::os::Property &filterParams,
                                           yarp::os::BufferedPort<yarp::sig::Vector>* gyroMeasPort,
                                           yarp::os::BufferedPort<yarp::sig::Vector>* gyroMeasPort2
                                         )
    : RateThread ( period ),
      m_period ( period ),
      m_moduleName ( moduleName ),
      m_robotName ( robotName ),
      m_autoconnect ( autoconnect ),
      m_usingxsens ( usingxsens ),
      m_usingEKF ( usingEKF ),
      m_inWorldRefFrame ( inWorldRefFrame ),
      m_gravityVec ( gravityVec ),
      m_usingSkin ( usingSkin ),
      m_sensorPort ( sensorPort ),
      m_debugGyro ( debugGyro ),
      m_debugAcc ( debugAcc ),
      m_verbose ( verbose ),
      m_filterParams( filterParams ),
      m_gyroMeasPort ( gyroMeasPort ),
      m_gyroMeasPort2 ( gyroMeasPort2 ),
      m_sysPdf( STATEDIM ),
      m_prior_mu_vec( STATEDIM ),
      m_waitingTime( 0.0 ),
      m_using2acc( false )
{
    //TODO Initialize m_gyroMeasPort according to gyroMeasPort
    // NOTE This is assuming that the accelerometer readings are coming from the MTB boards' accelerometers as done in iCubGenova01
    if (!m_sensorPort.compare("/icub/left_foot_inertial/analog:o") && m_using2acc)
        m_sensorPort2 = "/icub/right_foot_inertial/analog:o";

}

void quaternionEKFThread::run()
{
    // Get Input and measurement from XSens or iCubGenova01's sensors
    if ( m_usingxsens && !m_usingSkin) {
        bool reading = true;
        imu_measurement = m_gyroMeasPort->read(reading);
        if (m_using2acc) {
            imu_measurement2 = m_gyroMeasPort2->read(reading);
        }
    }
    
    if (m_verbose && (!m_usingSkin || m_usingxsens)) {
        cout << "Full imu_measurement vec: " << endl;
        cout << imu_measurement->toString().c_str() << endl;
    }
    
    yarp::sig::Vector imu_linAcc(3);
    yarp::sig::Vector imu_angVel(3);
    yarp::sig::Vector realOrientation(3);
    
    // Get input(gyro) and measurement(acc) from MTB port
    if (m_usingSkin && m_usingEKF && !m_usingxsens) {
        if( !extractMTBDatafromPort(MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID, imu_linAcc, imu_angVel) ) {
            yError("[quaternionEKFThread::run] Sensor data could not be parsed from MTB port");
        } else {
            if (m_verbose)
                yInfo("[quaternionEKFThread::run] Parsed sensor data: \n Acc [m/s^2]: \t%s \n Ang Vel [deg/s]: \t%s \n",  imu_linAcc.toString().c_str(), imu_angVel.toString().c_str());
        }
    }

    if (m_usingEKF) {
        // XSens orientation
        if (m_usingxsens & !m_usingSkin)
            realOrientation = imu_measurement->subVector(0,2);
        // Extract linear acceleration in m/s^2
        if (m_usingxsens && !m_usingSkin) {
            imu_linAcc = imu_measurement->subVector(3,5);
            // NOTE The raw angular speed read from the IMU is in deg/s. In this module we will transform
            // it to rad/s
            imu_angVel = PI/180*imu_measurement->subVector(6,8);
        }
        // Copy ang velocity data from a yarp vector into a ColumnVector
        MatrixWrapper::ColumnVector input(imu_angVel.data(),m_input_size);
        if (m_verbose)
            cout << "VEL INPUT IS: " << input << endl;
        // Copy accelerometer data from a yarp vector into a ColumnVector
        MatrixWrapper::ColumnVector measurement(imu_linAcc.data(),m_measurement_size);
        if (m_verbose)
            cout << "ACC INPUT IS: " << measurement << endl;

        // Noise gaussian
        // System Noise Mean
        // TODO [NOT SURE] This mean changes!!!
        MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
        sys_noise_mu = 0.0;

        /**************** System Noise Covariance ********************************************************************************/
        MatrixWrapper::Matrix Xi(m_state_size, m_input_size);
        XiOperator(m_posterior_state, &Xi);
        MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
        sys_noise_cov = 0.0;
        // NOTE m_sigma_gyro must be small, ||ek|| = 10e-3 rad/sec
        MatrixWrapper::Matrix Sigma_gyro(m_input_size,m_input_size);
        Sigma_gyro = 0.0;
        Sigma_gyro(1,1) = Sigma_gyro(2,2) = Sigma_gyro(3,3) = m_sigma_gyro;
        MatrixWrapper::Matrix tmp = Xi*Sigma_gyro*Xi.transpose();
        // NOTE on 30-07-2015 I commented the following lines because making this matrix symmetric this way does not make much sense from a theoretical point of view. I'd rather add a term such as alpha*I_4x4
//         MatrixWrapper::SymmetricMatrix tmpSym(m_state_size);
//         tmp.convertToSymmetricMatrix(tmpSym);
//         cout << "Symm Matrix: " << tmpSym << endl;
        sys_noise_cov = (MatrixWrapper::SymmetricMatrix) tmp*pow(m_period/(1000.0*2.0),2);
        // NOTE Next line is setting system noise covariance matrix to a constant diagonal matrix
//         sys_noise_cov = 0.0; sys_noise_cov(1,1) = sys_noise_cov (2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = 0.000001;
        /****************END System Noise Covariance *********************************************************************************/

        if (m_verbose)
            cout << "System covariance matrix will be: " << sys_noise_cov << endl;

        m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
        m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);

        double elapsedTime = yarp::os::Time::now() - m_waitingTime;

    //     double intpart = 0.0;

    //     // NOTE Let's include the measurement roughly every ten seconds
    //     if (modf(elapsedTime/10, &intpart) < 0.001) {
    //         if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
    //             yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    //     } else {
    //             if(!m_filter->Update(m_sys_model, input))
    //                 yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    //     }
        
        // NOTE THE NEXT TWO LINES ARE THE ONES I ACTUALLY NEED TO USE!!! DON'T FORGET TO UNCOMMENT AFTER DEBUGGING
//         if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
//             yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
        // NOTE Testing just the model equations
        if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement));
//         if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
//             yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");

        // Get the posterior of the updated filter. Result of all the system model and meaurement information
        BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
        // Posterior Expectation
        m_posterior_state = posterior->ExpectedValueGet();
        MatrixWrapper::Quaternion expectedValueQuat(m_posterior_state);
        // Posterior Covariance
        MatrixWrapper::SymmetricMatrix covariance(m_state_size);
        covariance = posterior->CovarianceGet();
        if (m_verbose) {
            cout << "Posterior Mean: " << expectedValueQuat << endl;
            cout << "Posterior Covariance: " << posterior->CovarianceGet() << endl;
        }
        MatrixWrapper::ColumnVector eulerAngles(3);
        MatrixWrapper::Quaternion tmpQuat;
        // NOTE When comparing results with the XSens sensor, this parameters should be set to TRUE
        // as the orientation estimated by the XSens IMU is in the Earth reference frame, thus the conjugate
        // of our estimate should be written to the port.
//         if (m_inWorldRefFrame) {
//             expectedValueQuat.conjugate(tmpQuat);
//         } else { 
//             tmpQuat = expectedValueQuat;
//         }
        expectedValueQuat.conjugate(tmpQuat);
        tmpQuat.getEulerAngles(string("xyz"), eulerAngles);
        if (m_verbose)
            cout << "Posterior Mean in Euler Angles: " << (180/PI)*eulerAngles  << endl;
        // Publish results to port
        yarp::sig::Vector tmpVec(m_state_size);
        for (unsigned int i=1; i<m_posterior_state.size()+1; i++) {
            tmpVec(i-1) = m_posterior_state(i);
        }
        // Publish Euler Angles estimation to port
        yarp::sig::Vector tmpEuler(3);
        for (unsigned int i=1; i<eulerAngles.rows()+1; i++)
            tmpEuler(i-1) = eulerAngles(i)*(180/PI);
        // Writing to port the full estimated orientation in Euler angles (xyz order)
        yarp::sig::Vector& tmpPortEuler = m_publisherFilteredOrientationEulerPort->prepare();
        tmpPortEuler = tmpEuler;
        m_publisherFilteredOrientationEulerPort->write();
        // Writing to port the full estimated quaternion
        yarp::sig::Vector& tmpPortRef = m_publisherFilteredOrientationPort->prepare();
        tmpPortRef = tmpVec;
        m_publisherFilteredOrientationPort->write();

        if (m_usingSkin && m_debugGyro) {
            yarp::sig::Vector &tmpGyroMeas = m_publisherGyroDebug->prepare();
            if (imu_angVel.data() != NULL) {
                tmpGyroMeas = imu_angVel;
                m_publisherGyroDebug->write();
            } else {
                yError("[quaternionEKFThread::run] ang velocity reading was empty");
            }
        }

        if (m_usingSkin && m_debugAcc) {
            yarp::sig::Vector &tmpAccMeas = m_publisherAccDebug->prepare();
            if (imu_linAcc.data() != NULL) {
                tmpAccMeas = imu_linAcc;
                m_publisherAccDebug->write();
            } else {
                yError("[quaternionEKFThread::run] Accelerometer reading was empty");
            }
        }

        //  Publish XSens orientation just for debugging
        if (m_usingxsens) {
            yarp::sig::Vector& tmpXSensEuler = m_publisherXSensEuler->prepare();
            if (realOrientation.data()!=NULL) {
                tmpXSensEuler = realOrientation;
                m_publisherXSensEuler->write();
            }
        }

        if (m_verbose) {
            cout << "Elapsed time: " << elapsedTime << endl;
            cout << " " << endl;
        }
    }

    // Direct filter computation with one or two accelerometers as specified by m_using2acc (iCubGenova01 specific)
    // Accelerometer only!!
    if (!m_usingEKF && !m_usingxsens) {
        yarp::sig::Vector output(12), output2(12);
        output.zero(); output2.zero();
        if(m_using2acc) {
            // Filtering accelerometers
            (*imu_measurement) = lowPassFilter->filt((*imu_measurement));
            (*imu_measurement2) = lowPassFilter->filt((*imu_measurement2));
            // Read and filter the two accelerometers
            m_directComputation->computeOrientation(imu_measurement, output);
            m_directComputation->computeOrientation(imu_measurement2, output2);
            // NOTE Storing only the mean of the roll!! 
            output(0) = (output(0) + output2(0))/2;
            // Streaming orientation
            yarp::sig::Vector& tmpPortEuler = m_publisherFilteredOrientationEulerPort->prepare();
            tmpPortEuler = output;
            m_publisherFilteredOrientationEulerPort->write();
        } else {
            (*imu_measurement) = lowPassFilter->filt((*imu_measurement));
            m_directComputation->computeOrientation(imu_measurement, output);
            // Low pass filtering
//             output = lowPassFilter->filt(output);
    //         cout << "Roll, Pitch, Yaw " << output(0) << " " << output(1) << " " << output(2) << endl;
            yarp::sig::Vector& tmpPortEuler = m_publisherFilteredOrientationEulerPort->prepare();
            tmpPortEuler = output;
            m_publisherFilteredOrientationEulerPort->write();
        }
    }
}

bool quaternionEKFThread::threadInit()
{
    if (!m_filterParams.isNull() && m_usingEKF) {
        m_state_size = m_filterParams.find("STATE_SIZE").asInt();
        m_input_size = m_filterParams.find("INPUT_SIZE").asInt();
        m_measurement_size = m_filterParams.find("MEASUREMENT_SIZE").asInt();
        m_prior_state_cov = m_filterParams.find("PRIOR_COV_STATE").asDouble();
        m_mu_system_noise = m_filterParams.find("MU_SYSTEM_NOISE").asDouble();
        m_sigma_system_noise = m_filterParams.find("SIGMA_SYSTEM_NOISE").asDouble();
        m_sigma_measurement_noise = m_filterParams.find("SIGMA_MEASUREMENT_NOISE").asDouble();
        m_sigma_gyro = m_filterParams.find("SIGMA_GYRO_NOISE").asDouble();
        m_prior_mu = m_filterParams.find("PRIOR_MU_STATE").asDouble();
        m_prior_cov = m_filterParams.find("PRIOR_COV_STATE").asDouble();
        m_mu_gyro_noise = m_filterParams.find("MU_GYRO_NOISE").asDouble();
        m_smoother = m_filterParams.find("smoother").asBool();
        m_external_imu = m_filterParams.find("externalimu").asBool();
    } else {
        if (!m_filterParams.isNull() && !m_usingEKF) {
            cout << "Real part of initial quat orientation" << m_filterParams.find("lsole_qreal_sensor").asDouble() << endl;
            m_quat_lsole_sensor = new MatrixWrapper::Quaternion(m_filterParams.find("lsole_qreal_sensor").asDouble(),
                                                                m_filterParams.find("lsole_qvec1_sensor").asDouble(),
                                                                m_filterParams.find("lsole_qvec2_sensor").asDouble(),
                                                                m_filterParams.find("lsole_qvec3_sensor").asDouble());
            m_lowPass_cutoffFreq = m_filterParams.find("cutoff_freq").asDouble();
            m_using2acc = m_filterParams.find("using2acc").asBool();
        } else {
            yError(" [quaternionEKFThread::threadInit] Filter parameters from configuration file could not be extracted");
            return false;
        }
    }

    //NOTE If using direct atan2 computation
    if (!m_usingEKF && !m_usingxsens) {
        m_directComputation = new directFilterComputation(*m_quat_lsole_sensor);
        double periodInSeconds = getRate()*1e-3;
        yarp::sig::Vector dofZeros(3,0.0);
        lowPassFilter = new iCub::ctrl::FirstOrderLowPassFilter(m_lowPass_cutoffFreq, periodInSeconds, dofZeros);
    }

    // IMU Measurement vector
    // If we are using a USB-plugged external XSens or we're connected to iCub's XSens
    if (m_usingxsens || !m_sensorPort.compare("/" + m_robotName + "/inertial") || !m_sensorPort.compare("/externalXSens/data:o"))
        imu_measurement = new yarp::sig::Vector(12);
    else {
            if (!m_sensorPort.compare("/icub/right_hand_inertial/analog:o") || !m_sensorPort.compare("/icub/left_hand_inertial/analog:o")) {
                imu_measurement = new yarp::sig::Vector(6);
            } else {
                if (!m_sensorPort.compare("/icub/right_foot_inertial/analog:o") || !m_sensorPort.compare("/icub/left_foot_inertial/analog:o")) {
                    imu_measurement = new yarp::sig::Vector(3);
                    if (m_using2acc)
                        imu_measurement2 = new yarp::sig::Vector(3);
                }
            }
    }

    // Open publisher port for estimate in quaternion
    m_publisherFilteredOrientationPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    m_publisherFilteredOrientationPort->open(string("/" + m_moduleName + "/filteredOrientation:o").c_str());

    // Open publisher port for estimate in euler angles
    m_publisherFilteredOrientationEulerPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    m_publisherFilteredOrientationEulerPort->open(string("/" + m_moduleName + "/filteredOrientationEuler:o").c_str());
    
    if (m_usingSkin && m_debugGyro) {
        m_publisherGyroDebug = new yarp::os::BufferedPort<yarp::sig::Vector>;
        m_publisherGyroDebug->open(string("/" + m_moduleName + "/rawGyroMeas:o").c_str());
    }
    if (m_usingSkin && m_debugAcc) {
        m_publisherAccDebug = new yarp::os::BufferedPort<yarp::sig::Vector>;
        m_publisherAccDebug->open(string("/" + m_moduleName + "/rawAccMeas:o").c_str());
    }

    if (m_usingxsens) {
        m_publisherXSensEuler = new yarp::os::BufferedPort<yarp::sig::Vector>;
        m_publisherXSensEuler->open(string("/xsens/euler:o").c_str());
    }

    if(m_usingEKF) {
        // System Noise Mean
        MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
        sys_noise_mu(1) = sys_noise_mu(2) = sys_noise_mu(3) = sys_noise_mu(4) = 0.0;

        // System Noise Covariance
        MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
        sys_noise_cov = 0.0;
        sys_noise_cov(1,1) = sys_noise_cov(2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = m_sigma_system_noise;

        // Setting System noise uncertainty
        m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
        m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);
        m_sysPdf.setPeriod(m_period);
        // Creating the model
        m_sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(&m_sysPdf);

        // Creating measurement model for linear measurement model
        // Measurement noise distribution
        // Measurement noise mean
        MatrixWrapper::ColumnVector meas_noise_mu(m_measurement_size);
        meas_noise_mu = 0.0;                // Set all to zero
        meas_noise_mu(3) = 0.0;
        // Measurement noise covariance
        MatrixWrapper::SymmetricMatrix meas_noise_cov(m_measurement_size);
        meas_noise_cov = 0.0;
        meas_noise_cov(1,1) = meas_noise_cov(2,2) = meas_noise_cov(3,3) = m_sigma_measurement_noise;
        // Measurement noise uncertainty
        m_measurement_uncertainty = new BFL::Gaussian(meas_noise_mu, meas_noise_cov);
        // Probability density function (PDF) for the measurement
        m_measPdf = new BFL::nonLinearMeasurementGaussianPdf(*m_measurement_uncertainty);
        //  Measurement model from the measurement PDF
        m_meas_model = new BFL::AnalyticMeasurementModelGaussianUncertainty(m_measPdf);
        // Setting prior. This is equivalent to a zero rotation
        MatrixWrapper::ColumnVector prior_mu(m_state_size);
        prior_mu = 0.0;
        prior_mu(1) = 1.0;
        m_prior_mu_vec = prior_mu;
        m_posterior_state = prior_mu;
        MatrixWrapper::SymmetricMatrix prior_cov(4);
        prior_cov = 0.0;
        prior_cov(1,1) = prior_cov(2,2) = prior_cov(3,3) = prior_cov(4,4) = m_prior_cov;
        cout << "Priors will be: " << endl;
        cout << "State prior: " << prior_mu << endl;
        cout << "Covariance prior: " << prior_cov << endl;
        m_prior = new BFL::Gaussian(prior_mu, prior_cov);

        // Construction of the filter
        m_filter = new BFL::ExtendedKalmanFilter(m_prior);
    }

    // Sensor ports
    // This port was opened by the module.
    std::string gyroMeasPortName = string("/" + m_moduleName + "/imu:i");

    // NOTE If using acccelerometer and gyro in the foot
    if ( m_usingSkin && m_usingEKF && !m_usingxsens && !m_using2acc) {
        std::string srcTmp = string("/" + m_robotName + "/right_leg/inertialMTB");
        if ( !m_sensorPort.compare(srcTmp) && m_usingSkin) {
            // NOTE Here I need to create a port that reads a bottle because the dimensions of this port can't be known a priori, since its size will depend on the amount of sensors that have been specified in the skin configuration file.
            m_imuSkinPortIn.open(string("/" + m_moduleName + "/imuSkin:i"));
            if (!yarp::os::Network::connect(srcTmp,m_imuSkinPortIn.getName())) {
                yError("[quaternionEKFThread::threadInit] Could not connect imuSkin port to the module");
                return false;
            }
        } else {
            yError("[quaternionEKFThread::threadInit] Seems like you have specified in the configuration file of this module that you are using the accelerometer and gyroscope connected to the right foot but the sensor port name specified does not correspond to icub/right_leg/inertialMTB");
            return false;
        }
    } else {
            // NOTE If using external XSens or Direct Method (atan2)  with feet accelerometers (iCubGenova01 specific)
            if ( m_usingxsens && !m_usingSkin ) {
                yarp::os::ConstString src = m_sensorPort;
                cout << "[quaternionEKFThread::threadInit()] Sensor SRC port name is: " << src << endl;
                if(!yarp::os::Network::connect(src, gyroMeasPortName,"tcp")){
                    yError(" [quaternionEKFThread::threadInit()] Connection with %s was not possible!\n \
                    Is the robotInterface running? or XSens IMU connected?\n \
                    Tip: Check also that the variable sensorPortName has been properly set in quaternionEKFModule.ini\n \
                    Also, when using an external USB-plugged XSens launch yarpdev --device inertial --subdevice xsensmtx --name /externalXSens/data:o ", m_sensorPort.c_str());
                    return false;
                }
                if (m_using2acc && !m_usingxsens && !m_usingSkin) {
                    std::string gyroMeasPortName2 = string("/" + m_moduleName + "/imu2:i");
                    yarp::os::ConstString src2 = m_sensorPort2;
                    cout << "[quaternionEKFThread::threadInit()]  src2 is: " << src2 << endl;
                    if(!yarp::os::Network::connect(src2, gyroMeasPortName2, "tcp")) {
                        yError("[quaternionEKFThread::threadInit()] Connection with %s was not possible. Is the robotInterface running? or the right leg accelerometer available?", gyroMeasPortName2.c_str());
                        return false;
                    }
                }
            }
      }

    cout << "Thread waiting two seconds before starting..." <<  endl;
    yarp::os::Time::delay(2);

    m_waitingTime = yarp::os::Time::now();
    cout << "Thread is running ... " << endl;
    return true;
}

void quaternionEKFThread::XiOperator ( MatrixWrapper::ColumnVector quat, MatrixWrapper::Matrix* Xi )
{
//     In  Matlab language this would be:
//     Xi = [       -qk(2:4,:)'           ;
//           qk(1)*eye(3) + S(qk(2:4,:)) ];
    (*Xi) = 1.0;
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

void quaternionEKFThread::SOperator ( MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix* S )
{
    (*S)(1,1) = 0.0;    (*S)(1,2) = -omg(3); (*S)(1,3) = omg(2);
    (*S)(2,1) = omg(3); (*S)(2,2) = 0.0    ; (*S)(2,3) = -omg(1);
    (*S)(3,1) = -omg(2);(*S)(3,2) = omg(1) ; (*S)(3,3) = 0.0;
}

bool quaternionEKFThread::extractMTBDatafromPort ( int boardNum, Vector& linAccOutput, Vector& gyroMeasOutput )
{
    int indexSubVector = 0;
    if ( !m_imuSkinPortIn.read(m_MTBmeas) ) {
        yError("[quaternionEKFThread::extractMTBDatafromPort] There was an error trying to read from the MTB port");
        return false;
    } else {
        if (m_verbose)
            yInfo("[quaternionEKFThread::extractMTBDatafromPort] Raw meas: %s", m_MTBmeas.toString().c_str());
        /******************* searching for multiple instances of the sensor  **************************/
        double* tmp;
        tmp = m_MTBmeas.data();
        double *it = tmp + 2; // First two elements of the vector can be skipped
        while (it < tmp + m_MTBmeas.size()) {
            it = std::find(it, it + (m_MTBmeas.size() - indexSubVector), boardNum);
            if (it < tmp + m_MTBmeas.size()) {
                indexSubVector = (int)(it - tmp) + 1;

                // Parse sensor data
                int trueindexSubVector = indexSubVector - 1;
                if ( tmp[trueindexSubVector]  == boardNum ) {
                    //  If sensor from board "boardNum" is an accelerometer
                    if ( tmp[trueindexSubVector + 1] == 1.0) {
                        linAccOutput(0) = tmp[trueindexSubVector + 3];
                        linAccOutput(1) = tmp[trueindexSubVector + 4];
                        linAccOutput(2) = tmp[trueindexSubVector + 5];
                    } else {
                        // If sensor from board "boardNum" is a gyroscope
                        if ( tmp[trueindexSubVector + 1] == 2.0 ) {
                            gyroMeasOutput(0) = tmp[trueindexSubVector + 3];
                            gyroMeasOutput(1) = tmp[trueindexSubVector + 4];
                            gyroMeasOutput(2) = tmp[trueindexSubVector + 5];
                        }
                    }
                }
                it = it + MTB_PORT_DATA_PACKAGE_OFFSET; //Move to the next position in the vector where a package data is expected
            }
        }
        /**********************************************************************************/
        linAccOutput = CONVERSION_FACTOR_ACC*linAccOutput;
        if (yarp::math::norm(linAccOutput) > 11.0) {
            yError("WARNING!!! [quaternionEKFThread::run] Gravity's norm is too big!");
        }
        gyroMeasOutput = PI/180*CONVERSION_FACTOR_GYRO*gyroMeasOutput;
        if (yarp::math::norm(gyroMeasOutput) > 100.0) {
            yError("WARNING!!! [quaternionEKFThread::run] Ang vel's norm is too big!");
        }
    }
    return true;
}

void quaternionEKFThread::threadRelease()
{
    if (!m_usingEKF) {
        if (m_directComputation) {
            cout << "deleting m_directComputation " << endl;
            delete m_directComputation;
            m_directComputation = NULL;
            cout << "m_directComputation deleted" << endl;
        }
        if (lowPassFilter) {
            cout << "deleting lowPassFilter" << endl;
            delete lowPassFilter;
            lowPassFilter = NULL;
            cout << "lowPassFilter deleted "<< endl;
        }
    }
    if (m_publisherFilteredOrientationEulerPort) {
        cout << "deleting m_publisherFilteredOrientationEulerPort" << endl;
        m_publisherFilteredOrientationEulerPort->interrupt();
        delete m_publisherFilteredOrientationEulerPort;
        m_publisherFilteredOrientationEulerPort = NULL;
        cout << "m_publisherFilteredOrientationEulerPort deleted" << endl;
    }
    if (m_publisherFilteredOrientationPort) {
        cout << "deleting m_publisherFilteredOrientationPort" << endl;
        m_publisherFilteredOrientationPort->interrupt();
        delete m_publisherFilteredOrientationPort;
        m_publisherFilteredOrientationPort = NULL;
        cout << "m_publisherFilteredOrientationPort deleted" << endl;
    }
    if (m_publisherAccDebug && m_usingSkin) {
        cout << "deleting m_publisherAccDebug" << endl;
        m_publisherAccDebug->interrupt();
        delete m_publisherAccDebug;
        m_publisherAccDebug = NULL;
        cout << "m_publisherAccDebug deleted" << endl;
    }
    if (m_publisherGyroDebug && m_usingSkin) {
        cout << "deleting m_publisherAccDebug" << endl;
        m_publisherGyroDebug->interrupt();
        delete m_publisherGyroDebug;
        m_publisherGyroDebug = NULL;
        cout << "m_publisherGyroDebug deleted" << endl;
    }
    if (m_usingxsens) {
        cout << "deleting m_publisherXSensEuler " << endl;
        m_publisherXSensEuler->interrupt();
        delete m_publisherXSensEuler;
        m_publisherXSensEuler = NULL;
        cout << "m_publisherXSensEuler deleted" << endl;
    }
    if (m_usingEKF) {
        if (m_sys_model) {
            cout << "deleting m_sys_model" << endl;
            delete m_sys_model;
            m_sys_model = NULL;
            cout << "m_sys_model deleted" << endl;
        }
        if (m_measurement_uncertainty) {
            cout << "deleting m_measurement_uncertainty " << endl;
            delete m_measurement_uncertainty;
            m_measurement_uncertainty = NULL;
            cout << "m_measurement_uncertainty deleted" << endl;
        }
        if (m_measPdf) {
            cout << "deleting m_measPdf"  << endl;
            delete m_measPdf;
            m_measPdf = NULL;
            cout << "m_measPdf deleted" << endl;
        }
        if (m_meas_model) {
            cout << "deleting m_meas_model" << endl;
            delete m_meas_model;
            m_meas_model = NULL;
            cout << "m_meas_model deleted" << endl;
        }
        if (m_prior) {
            cout << "deleting m_prior" << endl;
            delete m_prior;
            m_prior = NULL;
            cout << "m_prior deleted" << endl;
        }
        if (m_filter) {
            cout << "deleting m_filter" << endl;
            delete m_filter;
            m_filter = NULL;
            cout << "m_filter deleted" << endl;
        }
    }
    if (imu_measurement && !m_usingSkin) {
        cout << "deleting imu_measurement" << endl;
        delete imu_measurement;
        imu_measurement = NULL;
        cout << "imu_measurement deleted" << endl;
    }
    if (m_using2acc) {
        cout << "deleting imu_measurement2" << endl;
        delete imu_measurement2;
        imu_measurement2 = NULL;
        cout << "imu_measurement2 deleted" << endl;
    }
}

