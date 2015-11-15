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

#include <yarp/os/ResourceFinder.h>
#include "IEstimator.h"

class QuaternionEKF : public IEstimator
{
    // Every estimator must register itself first here this way
    REGISTER(QuaternionEKF)
    
    bool init(yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors* wbs);
    void run();
    void release();
};

#endif /* quaternionEKF.h */
