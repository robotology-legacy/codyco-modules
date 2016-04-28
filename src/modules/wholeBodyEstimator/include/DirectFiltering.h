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

#ifndef DIRECTFILTERING_H_
#define DIRECTFILTERING_H_

#include <bfl/wrappers/matrix/matrix_wrapper.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include "IEstimator.h"
#include "constants.h"
#include "portsInterface.h"

struct directFilteringParams
{
    std::string robotPrefix;
    bool streamMeasurements;
};

class DirectFiltering : public IEstimator
{
    REGISTER(DirectFiltering)
    
public:
    DirectFiltering();
    ~DirectFiltering();
    bool init(yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors *wbs);
    void run();
    void release();
    void computeOrientation(yarp::sig::Vector* sensorReading, yarp::sig::Vector& output);
    void computeTilt(yarp::sig::Vector* sensorReading, yarp::sig::Vector& output);
    void setWorldOrientation(MatrixWrapper::Quaternion& worldOrientation);
    bool readEstimatorParams(yarp::os::ResourceFinder &rf, directFilteringParams &params);
private:
    MatrixWrapper::Matrix                       m_lsole_R_acclsensor;
    MatrixWrapper::Matrix                       m_world_R_lsole;
    // Measurements port
    yarp::os::Port                            * sensorMeasPort;
    yarp::os::BufferedPort<yarp::sig::Vector> * outputPort;
    directFilteringParams                       m_params;
    readerPort                                  sensorDataPort;
    publisherPort                               m_estimatePort;
    publisherPort                               m_tiltPort;
    std::string                                 m_className;
    measurementsStruct                          m_meas;
};

#endif /*directFiltering*/
