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

#ifndef FLOATING_BASE_ODOMETRY_H
#define FLOATING_BASE_ODOMETRY_H

#include <iDynTree/Estimation/simpleLeggedOdometryKDL.h>
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/LogStream.h>
#include <iDynTree/Estimation/robotStatus.h>
#include <iCub/iDynTree/yarp_kdl.h>

//TODO: Include this class in yarpWholeBodyModel
class floatingBaseOdometry {
private:
    yarpWbi::yarpWholeBodyModel * m_wbm;
    iDynTree::simpleLeggedOdometry m_floatingBase;
    iCub::iDynTree::DynTree * m_robot_model;
    std::string m_current_fixed_link_name;
    int m_floating_base_frame_index;
    yarp::sig::Matrix m_world_H_floatingBase;
    iDynTree::RobotJointStatus * m_joint_status;
    int fixed_link;
public:
    floatingBaseOdometry(yarpWbi::yarpWholeBodyModel * wbm);
    virtual ~floatingBaseOdometry();
    bool init(std::string initial_world_frame_position = "l_sole",
              std::string initial_fixed_link = "r_sole",
              std::string floating_base_frame_index = "root_link",
              KDL::Vector initial_world_offset = KDL::Vector());
    void update(double* joints_configuration, bool switch_foot);
    void get_world_H_floatingbase(wbi::Frame &m);
    bool changeFixedFoot();
};

#endif