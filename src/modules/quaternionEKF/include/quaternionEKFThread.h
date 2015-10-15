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

#ifndef __QUATERNIONEKFTHREAD_H__
#define __QUATERNIONEKFTHREAD_H__

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>

#include <iomanip> //setw
#include <algorithm> //std::find
#include "nonLinearAnalyticConditionalGaussian.h"
#include "nonLinearMeasurementGaussianPdf.h"
#include "dataDumperParser.h"
#include "directFilterComputation.h"
#include <iCub/ctrl/filters.h>
#include <yarp/math/Math.h>

//TODO The path to the original data file must be retrieved by the ResourceFinder.
#define DATAFILE "/home/jorhabib/Software/extended-kalman-filter/EKF_Quaternion_DynWalking2015/orocos_bfl/data/dumper/icub/inertial/data.log"
//TODO This should come from the configuration file
#define STATEDIM 4
//TODO In case you wanna add a different group in the configuration file
#define FILTER_GROUP_PARAMS_NAME "EKFPARAMS"
#define GRAVITY_ACC 9.81
#define PI 3.141592654
// Accelerometer conversionf actor in m/s^2
#define CONVERSION_FACTOR_ACC 5.9855e-04
// Gyroscope conversion factor in deg/sec
#define CONVERSION_FACTOR_GYRO 7.6274e-03
#define MTB_RIGHT_FOOT_ACC_PLUS_GYRO_1_ID 32.0
#define MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID 33.0 // The board to which the skin is connected
#define MTB_RIGHT_HAND_ACC_PLUS_GYRO_1_ID 25.0
#define MTB_PORT_DATA_PACKAGE_OFFSET 6


namespace filter{
class quaternionEKFThread: public yarp::os::RateThread
{
    // Ports for sensor readings
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_port_input;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_gyroMeasPort;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_gyroMeasPort2;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_publisherFilteredOrientationPort;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_publisherFilteredOrientationEulerPort;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_publisherXSensEuler;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_publisherGyroDebug;
    yarp::os::BufferedPort<yarp::sig::Vector>   *m_publisherAccDebug;
    yarp::os::Port                               m_imuSkinPortIn;
    int                                          m_period; // Period in ms
    std::string                                  m_moduleName;
    std::string                                  m_robotName;
    std::string                                  m_sensorPort;
    std::string                                  m_sensorPort2;
    bool                                         m_autoconnect;
    bool                                         m_usingxsens;
    bool                                         m_usingEKF;
    bool                                         m_usingSkin;
    bool                                         m_debugGyro;
    bool                                         m_debugAcc;
    bool                                         m_verbose;
    bool                                         m_inWorldRefFrame;
    double                                       m_gravityVec;
    yarp::os::Property                           m_filterParams;
    dataDumperParser                            *m_parser;
    // currentData struct defined in dataDumperParser.h
    currentData                                  m_currentData;
    BFL::nonLinearAnalyticConditionalGaussian    m_sysPdf;
    BFL::AnalyticSystemModelGaussianUncertainty *m_sys_model;
    BFL::Gaussian                               *m_measurement_uncertainty;
    BFL::nonLinearMeasurementGaussianPdf        *m_measPdf;
    BFL::AnalyticMeasurementModelGaussianUncertainty* m_meas_model;
    MatrixWrapper::ColumnVector                  m_posterior_state;
    // filter parameters read from configuration file
    // TODO These should be put in some structure
    int                                          m_state_size;
    int                                          m_input_size;
    int                                          m_measurement_size;
    double                                       m_prior_state_cov;
    double                                       m_mu_system_noise;
    double                                       m_sigma_system_noise;
    double                                       m_sigma_measurement_noise;
    double                                       m_sigma_gyro;
    double                                       m_mu_gyro_noise;
    bool                                         m_smoother;
    bool                                         m_external_imu;
    // Priors
    BFL::Gaussian                               *m_prior;
    double                                       m_prior_cov;
    double                                       m_prior_mu;
    MatrixWrapper::ColumnVector                  m_prior_mu_vec;
    // Filter
    BFL::ExtendedKalmanFilter                   *m_filter;
    // Others
    double                                       m_waitingTime;
    yarp::sig::Vector                           *imu_measurement;
    yarp::sig::Vector                           *imu_measurement2;
    yarp::os::Bottle                             m_imuSkinBottle;
    yarp::sig::Vector                            m_MTBmeas;
    directFilterComputation                     *m_directComputation;
    MatrixWrapper::Quaternion                   *m_quat_lsole_sensor;
    double                                       m_lowPass_cutoffFreq;
    bool                                         m_using2acc;
    // Low Pass Filter
    iCub::ctrl::FirstOrderLowPassFilter         *lowPassFilter;

public:
  quaternionEKFThread ( int                             period,
                        std::string                     moduleName,
                        std::string                     robotName,
                        bool                            autoconnect,
                        bool                            usingxsens,
                        bool                            usingEKF,
                        bool                            inWorldRefFrame,
                        double                          gravityVec,
                        bool                            usingSkin,
                        std::string                     sensorPort,
                        bool                            debugGyro,
                        bool                            debugAcc,
                        bool                            verbose,
                        yarp::os::Property              &filterParams,
                        yarp::os::BufferedPort<yarp::sig::Vector>* m_gyroMeasPort,
                        yarp::os::BufferedPort<yarp::sig::Vector>* m_gyroMeasPort2
                      );
  bool threadInit();
  void run();
  void threadRelease();
  // TODO Temporarily will put this method here but it should be in QuaternionWrapper
  void XiOperator(MatrixWrapper::ColumnVector quat, MatrixWrapper::Matrix* Xi);
  // TODO Temporarily putting this method here. Should be put in MatrixWrapper somewhere
  void SOperator(MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix* S);
   
  /** \brief Filters and extracts a specific MTB data from an MTB port.
   * 
   *    MTB port example: /icub/rigth_leg/inertialMTB
   *  \param[in]  sensorType ID number of the MTB "position" in iCub's body.
   *  \param[out] linAccOutput Extracted/Parsed accelerometer measurement from MTB port reading.
   *  \param[out] gyroMeasOutput Extracted/Parsed gyroscope measurement from MTB port reading.
   */
  bool extractMTBDatafromPort(int sensorType, yarp::sig::Vector &linAccOutput, yarp::sig::Vector &gyroMeasOutput);
  };
}

#endif
