/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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

/**
\defgroup torqueGraspManager torqueGraspManager

@ingroup codyco_module

This is a modified version of the Joint Grasping Demo developed by IIT
and ISR (available at http://wiki.icub.org/brain/group__demoGraspManager.html)
made to work on the top of the wholeBody torque balancing provided by the torqueBalancing module.

The following documentation matches the one of the demoGraspManager, so it is left untouched.
The main difference is that for hand cartesian movements the module does not use directly
the iCub cartesian interface (http://wiki.icub.org/brain/icub_cartesian_interface.html) but
it just uses the cartesian solver to get the desired joint position, that are then send to
the torqueBalancing module.

Copyright (C) 2010 RobotCub Consortium

Author: Ugo Pattacini, Francesco Romano

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This

This module collects the 3-d object positions estimated by the
particle filter and sends data to the head and arm controllers
in order to gaze at the target, reach for it and eventually
grasp it.
It relies on the YARP ICartesianControl interface to control
both arms and on the YARP IGazeControl interface to control the
gaze.

Furthermore, there exists a second modality that enables to
estimate the 3-d object position using stereo vision that needs
to be calibrated in advance relying on a feed-forward neural
network.

\section lib_sec Libraries
- ctrlLib.
- iKin.
- YARP libraries.

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
The robot interface is assumed to be operative; in particular,
the ICartesianControl interface must be available. The
\ref iKinGazeCtrl must be running.

\section portsc_sec Ports Created

- \e /torqueGraspManager/trackTarget:i receives the 3-d
  position to track.

- \e /torqueGraspManager/imdTargetLeft:i receives the
  blobs list as produced by the \ref motionCUT module for the
  left eye.

- \e /torqueGraspManager/imdTargetRight:i receives the
  blobs list as produced by the \ref motionCUT module for the
  right eye.

- \e /torqueGraspManager/cmdFace:o sends out commands to
  the face expression high level interface in order to give an
  emotional representation of the current robot state.

- \e /torqueGraspManager/gui:o sends out info to update target
  within the \ref icub_gui.

- \e /torqueGraspManagerr/rpc remote procedure
    call. Recognized remote commands:
    -'quit' quit the module

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
The configuration file passed through the option \e --from
should look like as follows:

\code
[general]
// the robot name to connect to
robot           icub
// the thread period [ms]
thread_period   30
// left arm switch
left_arm        on
// right arm switch
right_arm       on
// arm trajectory execution time [s]
traj_time       2.0
// reaching tolerance [m]
reach_tol       0.01
// eye used
eye             left
// homes limbs if target detection timeout expires [s]
idle_tmo        5.0
// enable the use of stereo vision calibrated by NN
use_network off
// NN configuration file
network         network.ini

[torso]
// joint switch (min **) (max **) [deg]; 'min', 'max' optional
pitch on  (max 30.0)
roll off
yaw on

[left_arm]
// the offset [m] to be added to the desired position
reach_offset        0.0 -0.15 -0.05
// the offset [m] for grasping
grasp_offset        0.0 0.0 -0.05
// perturbation given as standard deviation [m]
grasp_sigma 0.01 0.01 0.01
// hand orientation to be kept [axis-angle rep.]
hand_orientation 0.064485 0.707066 0.704201 3.140572
// enable impedance velocity mode
impedance_velocity_mode off
impedance_stiffness 0.5 0.5 0.5 0.2 0.1
impedance_damping 60.0 60.0 60.0 20.0 0.0

[right_arm]
reach_offset        0.0 0.15 -0.05
grasp_offset        0.0 0.0 -0.05
grasp_sigma         0.01 0.01 0.01
hand_orientation    -0.012968 -0.721210 0.692595 2.917075
impedance_velocity_mode off
impedance_stiffness 0.5 0.5 0.5 0.2 0.1
impedance_damping 60.0 60.0 60.0 20.0 0.0

[home_arm]
// home position [deg]
poss    -30.0 30.0 0.0  45.0 0.0  0.0  0.0
// velocities to reach home positions [deg/s]
vels    10.0  10.0 10.0 10.0 10.0 10.0 10.0

[arm_selection]
// hysteresis range added around plane y=0 [m]
hysteresis_thres 0.1

[grasp]
// ball radius [m] for still target detection
sphere_radius   0.05
// timeout [s] for still target detection
sphere_tmo      3.0
// timeout [s] to open hand after closure
release_tmo     3.0
// open hand positions [deg]
open_hand       0.0 0.0 0.0   0.0   0.0 0.0 0.0   0.0   0.0
// close hand positions [deg]
close_hand      0.0 80.0 12.0 18.0 27.0 50.0 20.0  50.0 135.0
// velocities to reach hand positions [deg/s]
vels_hand       10.0 10.0  10.0 10.0 10.0 10.0 10.0 10.0  10.0
\endcode

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini, Alessandro Roncone, Francesco Romano
*/

#include <string>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "ManagerModule.h"
#include "Utilities.h"

using namespace yarp::os;

YARP_DECLARE_PLUGINS(icubmod)

int main(int argc, char *argv[])
{
    Network yarp;
    if (!Network::checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_PLUGINS(icubmod)

    codyco::myReport rep;

    ResourceFinder rf;
    rf.setVerbose(true);
    //???: setMonitor?
    rf.setMonitor(&rep);
    rf.setDefaultContext("torqueGraspManager");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    codyco::ManagerModule mod;
    mod.setName("/torqueGraspManager");

    return mod.runModule(rf);
}


