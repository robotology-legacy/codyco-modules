/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 *  Authors: Naveen Kuppuswamy
 *  email: naveen.kuppuswamy@iit.it
 * 
 *  The development of this software was supported by the FP7 EU projects
 *  CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 *  http://www.codyco.eu
 * 
 *  Permission is granted to copy, distribute, and/or modify this program
 *  under the terms of the GNU General Public License, version 2 or any
 *  later version published by the Free Software Foundation.
 * 
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details
 *  
 * 
 */

#include "directFilterComputation.h"
#include <math.h>

using namespace filter;

directFilterComputation::directFilterComputation()
{

}

directFilterComputation::directFilterComputation ( MatrixWrapper::Quaternion lsole_Rq_acclsensor )
{
    // Store rotation from sensor to left sole frame
    lsole_Rq_acclsensor.getRotation(m_lsole_R_acclsensor);

}

directFilterComputation::~directFilterComputation()
{

}

void directFilterComputation::computeOrientation ( yarp::sig::Vector* sensorReading, yarp::sig::Vector& output )
{
    MatrixWrapper::ColumnVector input(sensorReading->data(), sensorReading->length());
    // Rotate sensor reading (expressed in the sensor frame) to the foot reference frame
    input = m_lsole_R_acclsensor*input;
    // Roll assuming xyz rotation
    double phi_xyz = atan2(input(2), input(3));
    // Pitch assuming xyz rotation
    double theta_xyz = (-input(1))/(sqrt(pow(input(2),2) + pow(input(3),2)));
    output(0) = phi_xyz;
    output(1) = theta_xyz;
    output(2) = 0.0;
}

void directFilterComputation::setWorldOrientation ( MatrixWrapper::Quaternion& worldOrientation )
{

}

