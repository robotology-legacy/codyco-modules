/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
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

#ifndef ICUB_WALKING_IK_THREAD_H
#define ICUB_WALKING_IK_THREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Semaphore.h>

#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarpWholeBodyInterface/yarpWholeBodyStates.h>

#include "SplineInterpolator.h"
#include "UtilityFunctions.h"
#include "IK.h"
#include "floatingBaseOdometry.h"
#include "paramsStructures.h"

class iCubWalkingIKThread: public yarp::os::RateThread {
private:
    int m_period;
    std::string m_walkingPatternFile;
    walkingParams m_walkingParams;
    odometryParams m_odometryParams;
    inverseKinematicsParams m_inverseKinematicsParams;
    yarp::os::ResourceFinder m_rf;
    yarpWbi::yarpWholeBodyModel* m_wbm;
    yarpWbi::yarpWholeBodyStates* m_wbs;
    std::string m_outputDir;
    floatingBaseOdometry* m_odometry;
    yarp::os::Port m_rpc_port;
public:
    yarp::os::Semaphore  thread_mutex;
    bool planner_flag;
    
    iCubWalkingIKThread (int period,
                         yarpWbi::yarpWholeBodyModel* wbm,
                         yarpWbi::yarpWholeBodyStates* wbs,
                         walkingParams params,
                         odometryParams &odometry_params,
                         inverseKinematicsParams &inverseKin_params,
                         yarp::os::ResourceFinder& rf,
                         std::string walkingPatternFile,
                         std::string outputDir);
    virtual ~iCubWalkingIKThread();
    bool threadInit();
    void run();
    void threadRelease();
    void generateFeetTrajectories(std::string walkingPatternFile,
                                  walkingParams params);
    void inverseKinematics(walkingParams params);
    
    /**
     *  Computes a distance vector that goes from ref_frame to the center of both feet at the current configuration
     *
     *  @param v         Output KDL vector
     *  @param ref_frame Reference frame to use for the computation
     *
     *  @return true if everything goes well, false otherwise
     */
    bool computeCenterBetweenFeet(KDL::Vector &v, std::string ref_frame);
};

#endif
