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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include "IEstimator.h"

struct quaternionEKFParams
{
    int period;
    int stateSize;
    int inputSize;
    int measurementSize;
    double muSystemNoise;
    double sigmaSystemNoise;
    double sigmaMeasurementNoise;
    double sigmaGyro;
    double piorMu;
    double priorCovariance;
    double muGyroNoise;
};

struct publisherPortStruct
{
    std::string estimatorName;
    std::string portName;
    yarp::sig::Vector outputData;
    yarp::os::BufferedPort<yarp::sig::Vector> * outputPort;
    bool configurePort(std::string className, std::string pName)
    {
        estimatorName = className;
        portName = pName;
        outputPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
        outputPort->open(std::string("/" + estimatorName + "/" + portName + ":o").c_str());
    }
};

class QuaternionEKF : public IEstimator
{
    // Every estimator must register itself first here this way
    REGISTER(QuaternionEKF)
public:
    QuaternionEKF();
    virtual ~QuaternionEKF();
    bool init(yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors* wbs);
    void run();
    void release();
    // TODO Probably this method should also be enforced through IEstimator
    bool readEstimatorParams(yarp::os::ResourceFinder &rf, quaternionEKFParams &estimatorParams);
    void createSystemModel();
    void createMeasurementModel();
    void setPriors();
    void createFilter();
private:
    quaternionEKFParams m_quaternionEKFParams;
    std::string m_className;
    // Publisher ports list
    std::vector<publisherPortStruct> m_portsList;
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
};


#endif /* quaternionEKF.h */
