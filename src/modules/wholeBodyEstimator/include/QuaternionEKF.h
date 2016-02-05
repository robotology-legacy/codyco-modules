/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
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

#ifndef QUATERNIONEKF_H_
#define QUATERNIONEKF_H_

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "nonLinearAnalyticConditionalGaussian.h"
#include "nonLinearMeasurementGaussianPdf.h"
#include "floatingBase.h"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include "IEstimator.h"

// TEMPORARY
/**
 *  ID number of the first board in the right foot with accelerometer plus gyro as specified in:
 *
 *  @return 32.0
 */
#define MTB_RIGHT_FOOT_ACC_PLUS_GYRO_1_ID 32.0
/**
 *  ID number of the second board in the right foot with accelerometer plus gyro as specified in:
 *
 *  @return 33.0
 *  @note: This is the board to which the palm skin is currently connected.
 */
#define MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID 33.0 // The board to which the skin is connected
/**
 *  ID number of the MTB board in the palm of iCubGenova01 with accelerometer plys gyro as specified in:
 *
 *  @return 25.0
 */
#define MTB_RIGHT_HAND_ACC_PLUS_GYRO_1_ID 25.0
/**
 *  Position in the measurement vector as read from the port where a package data is expected.
 *  Recall that before the actual data, there are some other numbers and identifiers that could change.
 *
 *  @return 6
 */
#define MTB_PORT_DATA_PACKAGE_OFFSET 6
// Accelerometer conversion factor in m/s^2
/**
 *  MTB accelerometer conversion factor.
 *
 *  @return 5.9855e-04
 */
#define CONVERSION_FACTOR_ACC 5.9855e-04
// Gyroscope conversion factor in deg/sec
/**
 *  MTB gyroscope conversion factor.
 *
 *  @return 7.6274e-03
 */
#define CONVERSION_FACTOR_GYRO 7.6274e-03
#define PI 3.141592654

/**
 *  Structure containing this class parameters necessary for the Extended Kalman Filter.
 */
struct quaternionEKFParams
{
    unsigned int period;
    std::string robotPrefix;
    unsigned int stateSize;
    unsigned int inputSize;
    unsigned int measurementSize;
    double muSystemNoise;
    double sigmaSystemNoise;
    double sigmaMeasurementNoise;
    double sigmaGyro;
    double piorMu;
    double priorCovariance;
    double muGyroNoise;
    /**
     *  Enable the creation and opening of ports streaming raw accelerometer and gyroscope data, when these are directly read from the port. 
     */
    bool streamMeasurements;
    /**
     *  Enables the floating base attitude estimate
     */
    bool floatingBaseAttitude;
    /**
     *  Rotation matrix from FT sensor to accelerometer. This is temporary while added to the URDF of the robot
     */
    yarp::os::Bottle * rot_from_ft_to_acc_bottle;
};

enum outputPorts {
    ORIENTATION_ESTIMATE_PORT_QUATERNION,
    ORIENTATION_ESTIMATE_PORT_EULER,
    RAW_ACCELEROMETER_DATA_PORT,
    RAW_GYROSCOPE_DATA_PORT,
    FLOATING_BASE_ROTATION_PORT
};

/**
 *  Structure holding information about a publisher port and a method to open and publish.
 */
struct publisherPortStruct
{
    std::string estimatorName;
    std::string portName;
    yarp::sig::Vector outputData;
    yarp::os::BufferedPort<yarp::sig::Vector> * outputPort;
    /**
     *  Opens a publisher port and provides methods to configure and publish data.
     *
     *  @param className Current estimator class name.
     *  @param pName     Publisher port name. e.g. "rawAccelerometerData". Notice that no ":o" must be added at the end.
     *
     *  @return True if port was successfully opened, false otherwise.
     */
    bool configurePort(std::string className, std::string pName)
    {
        estimatorName = className;
        portName = pName;
        outputPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
        std::string fullPortName = std::string("/" + estimatorName + "/" + portName + ":o").c_str();
        if ( !outputPort->open( fullPortName ) )
        {
            yError ("Could not open output port %s ", fullPortName.c_str());
            return false;
        }
        return true;
    }
    //TODO: Finish the implementation of this method.
    /**
     *  Method used to retrieve and publish data.
     *
     *  @return True when succesfully written, false otherwise. 
     */
    bool publishData(){ return true; }
    
    bool close()
    {
        yInfo("[QuaternionEKF::publisherPortStruct] Closing port");
        this->outputPort->close();
        delete this->outputPort;
        this->outputPort = NULL;

        return true;
    }
};

/** 
 *  Structure holding information of a reader port and methods to configure it and connect to an input port.
 *  @param className name of the current class.
 *  @param pName Port name.
 *  @param srcPort Source port name.
 */
struct readerPortStruct
{
    std::string      estimatorName;
    std::string      portName;
    std::string      fullPortName;
    yarp::os::Port * inPort;
    /**
     *  Opens a reader port and connects it to its source.
     *
     *  @param className    Current estimator class name.
     *  @param pName        Reader port name.
     *  @param srcPort      Source port name.
     *  @param inputPort    Pointer to input port. This should be a private variable of the calling class. 
     *
     *  @return true when ports creation and connections are successful.
     */
    bool configurePort(std::string className, std::string pName, std::string srcPort, yarp::os::Port * inputPort)
    {
        inPort = inputPort;
        estimatorName = className;
        portName = pName;
        fullPortName = std::string("/" + estimatorName + "/" + portName + ":i").c_str();
        if ( !inputPort->open(fullPortName) ) {
            yError ("Could not open input port %s ", fullPortName.c_str());
            return false;
        } else {
            if ( !yarp::os::Network::connect(srcPort, fullPortName) )
            {
                yError ("Could not connect to port %s", srcPort.c_str() );
                return false;
            }
        }
        return true;
    }
    
    bool close()
    {
        yInfo("[QuaternionEKF::readerPort] Closing port");
        inPort->close();
        delete inPort;
        inPort = NULL;
        
        return true;
    }

};
/**
 *  Structure holding IMU-like information, namely linear acceleration, angular velocity and real orientation when available in body frame.
 */
struct measurementsStruct
{
    yarp::sig::Vector linAcc;
    yarp::sig::Vector angVel;
    // Assuming real orientation is given in Eulers
    yarp::sig::Vector realOrientation;
};

class QuaternionEKF : public IEstimator
{
    // Every estimator must register itself first here this way
    REGISTER(QuaternionEKF)
public:
    QuaternionEKF();
    virtual ~QuaternionEKF();
    /**
     Implemented the init() method from IEstimator. More documentation in the corresponding class.
     In particular this method reads the estimator parameters from configuration file, opens publisher 
     and resder ports, initializes filter variables, creates system and measurement model, sets prios 
     and instantiates an Extended Kalman Filter.
     
     - parameter yarp: rf reference to resource finder.
     - parameter wbi:  wbs pointer to object of type iWholeBodySensors.
     
     - returns: Returns true if successful, false otherwise.
     */
    bool init(yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors* wbs);
    /**
     *  Documentation in IEstimator class. 
     *  In this implementation the following is done:
     *  - Sensor data is read each time step
     *  - Updates system noise covariance.
     *  - Calls the prediction and update steps of the Kalman filter (EKF).
     *  - Retrieves posterior mean and covariance of the EKF.
     *  - Publishes estimates results through the ports configured in the init method (quaternion and euler).
     *  - Optionally streams read gyro and accelerometer data.
     */
    void run();
    void release();
    //TODO: This method should also be enforced through IEstimator
    /**
     *  Reads the filter parameters specified under the group CLASSNAME.
     *
     *  @param rf              It is assumed that rf has already been configured and populated.
     *  @param estimatorParams Filled with the parsed module parameters (output).
     *
     *  @return True if parameters for this class were found. False otherwise.
     *  @note This method should be ported to the estimators interface.
     */
    bool readEstimatorParams(yarp::os::ResourceFinder &rf, quaternionEKFParams &estimatorParams);
    /**
     *  Sets initial system noise mean and covariance and instantiates the model of type BFL::AnalyticSystemModelGaussianUncertainty.
     */
    void createSystemModel();
    /**
     *  Sets initial measurement noise mean and covariance and uses it to create an initial uncertainty PDF which is then used to 
        create a measurement model of type BFL::AnalyticMeasurementModelGaussianUncertainty.
     */
    void createMeasurementModel();
    /**
     *  Sets prior means and convariances.
     */
    void setPriors();
    /**
     *  Instantiates an Extended Kalman Filter of type BFL::ExtendedKalmanFilter.
     */
    void createFilter();
    //FIXME: This should not exist at all. yarpWholeBodySensors should be able to read this after proper initialization.
    /**
     *  Temporary fix while yarpWholeBodySensors parses acceleromenters and gyros from URDF and provides this measurement directly through the interface.
     *  Basically calls extractMTBDatafromPort.
     *
     *  @param m This object will contain raw angular velocity, linear acceleration and external estimated orientation -if provided- (output).
     *
     *  @return True if parsing from sensor reading port is successful, false otherwise.
     *  @note This method will be soon deprecated. Waiting for newest version of yarpWholeBodySensors.
     */
    bool readSensorData(measurementsStruct &m);
    /**
     *  Given that the following variables are somewhere defined: MTB_PORT_DATA_PACKAGE_OFFSET, CONVERSION_FACTOR_ACC, CONVERSION_FACTOR_GYRO.
        This method parses the measurement as streamed by the inertial unit and separates them into linear acceleration, angular velocity and orientation -if provided- (output).
     *
     *  @param boardNum     Currently specified in the header of this class.
     *  @param measurements Parsed data (output).
     *
     *  @return True when
     */
    bool extractMTBDatafromPort(int boardNum, measurementsStruct &measurements);
    //TODO: Temporary
    void XiOperator(MatrixWrapper::ColumnVector quat, MatrixWrapper::Matrix* Xi);
    void SOperator(MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix* S);

private:
    quaternionEKFParams m_quaternionEKFParams;
    std::string m_className;
    // Publisher ports list
    std::vector<publisherPortStruct> m_outputPortsList;
    std::vector<readerPortStruct> m_inputPortsList;
    // System model variables
    BFL::nonLinearAnalyticConditionalGaussian * m_sysPdf;
    BFL::AnalyticSystemModelGaussianUncertainty * m_sys_model;
    BFL::Gaussian * m_measurement_uncertainty;
    BFL::nonLinearMeasurementGaussianPdf * m_measPdf;
    BFL::AnalyticMeasurementModelGaussianUncertainty * m_meas_model;
    BFL::Gaussian * m_prior;
    BFL::ExtendedKalmanFilter * m_filter;
    MatrixWrapper::ColumnVector m_prior_mu_vec;
    MatrixWrapper::ColumnVector m_posterior_state;
    //FIXME This should be temporary
    yarp::os::Port * sensorMeasPort;
    measurementsStruct measurements;
    wholeBodyEstimator::floatingBase * m_floatingBaseEstimate;
    // Resulting Euler angles
    MatrixWrapper::ColumnVector eulerAngles;
    
};


#endif /* quaternionEKF.h */
