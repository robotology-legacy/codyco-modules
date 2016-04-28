/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Authors: Silvio Traversaro, Jorhabib Eljaik
 * email: silvio(dot)traversaro(at)iit(dot)it, jorhabib(dot)eljaik(at)iit(dot)it
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
#ifndef LEGGEDODOMETRY_H_
#define LEGGEDODOMETRY_H_


#include "IEstimator.h"
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Contactable.h>

#include <iCub/iDynTree/DynTree.h>
#include <iCub/iDynTree/TorqueEstimationTree.h>
#include <iDynTree/Estimation/simpleLeggedOdometry.h>
#include <iDynTree/Estimation/robotStatus.h>
#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>
#include <iCub/iDynTree/yarp_kdl.h>
#include <wbi/wbi.h>

#include <yarp/os/LogStream.h>

using namespace wbi;

class LeggedOdometry : public IEstimator
{
    REGISTER(LeggedOdometry)
private:
    iDynTree::simpleLeggedOdometry odometry_helper;
    int odometry_floating_base_frame_index;
    yarp::sig::Matrix world_H_floatingbase;
    yarp::sig::Vector floatingbase_twist;
    yarp::sig::Vector floatingbase_acctwist;
    bool odometry_enabled;
    bool frames_streaming_enabled;
    yarp::os::BufferedPort<yarp::os::Bottle> * port_floatingbasestate;
    yarp::os::BufferedPort<yarp::os::Property> * port_frames;
    /**
     *  Vector containing the indices of the frames to be streamed, after checking they are actually present. These frame have been specified via configuration file of the wholeBodyEstimator under the group LeggedOdometry.
     */
    std::vector<int> frames_to_stream_indices;
    /**
     *  Vector containing the names of the frames to be streamed, after checking they are actually present.
     */
    std::vector<std::string> frames_to_stream;
    std::vector<yarp::os::Bottle> buffer_bottles;
    yarp::sig::Matrix buffer_transform_matrix;
    bool com_streaming_enabled;
    yarp::os::BufferedPort<yarp::sig::Vector> * port_com;
    std::string current_fixed_link_name;

    iCub::iDynTree::DynTree * icub_model;
    wbi::iWholeBodySensors * m_sensors;
    iDynTree::RobotJointStatus  * m_joint_status;

    std::string m_module_name;
    std::string m_className;
public:
    LeggedOdometry();

    virtual ~LeggedOdometry();

    /**
     See documentation of IEstimator. Does the same job as initOdometry() in wholeBodyDynamicsTree
     
     - parameter rf: Reference to resource finder assuming it has been initialized.
     - parameter wbs:  Pointer to a whole body sensors object that should have been initialized.
     
     - returns: True when initialization is successful, false otherwise.
     */
    bool init(yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors *wbs);

    /**
     *  More info in the documentation of the IEstimator class. Called by wholeBodyEstimatorThread each time step and does the job of publishOdometry() in wholeBodyDynamicsTree.
     */
    void run();

    /**
     *  Same as closeOdometry from wholeBodyDynamicsTree.
     */
    void release();

    /**
     *  Closes a single port properly.
     */
    void closePort(yarp::os::Contactable *_port);

    /** 
     *  Updates joint_status
     */
    void readRobotStatus();
};

#endif /* LeggedOdometry */
