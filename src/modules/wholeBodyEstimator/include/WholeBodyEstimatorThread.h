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

#ifndef WHOLEBODYESTIMATORTHREAD_H_
#define WHOLEBODYESTIMATORTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Mutex.h>

#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include <iDynTree/Estimation/robotStatus.h>

#include <iomanip> //setw
#include <algorithm> //std::find for parsing lines
#include <yarp/math/Math.h>

#include "IEstimator.h"
#include "LeggedOdometry.h"

class WholeBodyEstimatorThread: public yarp::os::RateThread
{
private:
    IEstimator* m_quaternionEKFInstance;
    IEstimator* m_floatingBaseLeggedOdometry;

    // Variables for LeggedOdometry
    wbi::iWholeBodySensors*  m_wbs;
    yarp::os::ResourceFinder m_rfCopy;
    iDynTree::RobotJointStatus * m_joint_status;

    yarp::os::Mutex run_mutex;
    bool m_run_mutex_acquired;

public:
    WholeBodyEstimatorThread (yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors* wbs, int period);
//    void readRobotStatus();
    bool threadInit();
    void run();
    void threadRelease();
};

#endif
