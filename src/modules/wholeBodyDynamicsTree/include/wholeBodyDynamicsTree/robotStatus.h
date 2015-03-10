/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef _WHOLE_BODY_DYNAMICS_ROBOT_STATUS_H_
#define _WHOLE_BODY_DYNAMICS_ROBOT_STATUS_H_


class RobotStatus
{
public:
    yarp::sig::Vector qj;
    yarp::sig::Vector dqj;
    yarp::sig::Vector ddqj;
    yarp::sig::Vector torquesj;

    yarp::sig::Vector omega_imu;
    yarp::sig::Vector domega_imu;
    yarp::sig::Vector proper_ddp_imu;
    yarp::sig::Vector wbi_imu;

    std::vector<yarp::sig::Vector> measured_ft_sensors;
    std::vector<yarp::sig::Vector> estimated_ft_sensors;
    std::vector<yarp::sig::Vector> ft_sensors_offset;
    std::vector<yarp::sig::Vector> model_ft_sensors;

    RobotStatus(int nrOfDOFs=0, int nrOfFTSensors=0);
    bool setNrOfDOFs(int nrOfDOFs);
    bool setNrOfFTSensors(int nrOfFTSensors);
    bool zero();
};

#endif