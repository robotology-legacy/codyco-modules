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

#ifndef DIRECTFILTERCOMPUTATION_H
#define DIRECTFILTERCOMPUTATION_H

#include <yarp/sig/Vector.h> 
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <yarp/sig/Vector.h>

namespace filter{
class directFilterComputation
{
public:
    directFilterComputation();
    directFilterComputation(MatrixWrapper::Quaternion lsole_Rq_acclsensor);
    ~directFilterComputation();
    computeOrientation(yarp::sig::Vector sensorReading, yarp::sig::Vector& output);
    setWorldOrientation(MatrixWrapper::Quaternion& worldOrientation);
private:
    MatrixWrapper::Matrix lsole_R_acclsensor;
    MatrixWrapper::Matrix world_R_lsole;

};
}
#endif // DIRECTFILTERCOMPUTATION_H
