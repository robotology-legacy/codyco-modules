/*
 * Copyright (C) 2014-2015 Fondazione Istituto Italiano di Tecnologia -
                                      Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
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
@ingroup codyco_module

\defgroup wholeBodyDynamicsTree wholeBodyDynamicsTree

Estimates the external wrenches and internal torques
of a robot by fusing the information coming from
the dynamic model and the measurements
coming from the sensors of the robot.

\author Silvio Traversaro

\section intro_sec Description

This module estimates the external wrenches acting at any link of robot,
through a model based compensation
of the 6-axis force/torque (FT) sensor's measurements, which are
acquired through an input YARP port and provides them to an
output YARP ports.

The estimation is perfomed relying on rigid body dynamics using CAD
parameters.

The intrinsic offsets of the sensors, which are due to the stresses
generated during mounting, are defined by the initial FT sensor data.
Another source of offsets is the thermal drift of the sensors, to recalibrate
the sensors you can send (assuming that the robot is attached to a fixed base)
the `calib all` command to the RPC port of wholeBodyDynamicsTree
(see Ports Created section to get more information on how to access the RPC port).

The model of the sensor measurements considers a fixed base, with z-axis
pointing upwards. The estimation of the external wrench applied at the
end-effector of the limb has the same orientation of the fixed base frame.
For further information about the use of this module and of the iCub force control interface,
please refer to the force control page:

http://wiki.icub.org/wiki/Force_Control

\section lib_sec Libraries
- YARP library.
- iDynTree library.
- yarpWholeBodyInterface library

\section parameters_sec Parameters

--robot \e name
- The parameter \e name identifies the robot name. If not specified
  \e icub is assumed.

--period \e period
  The parameter \e period identifies the rate the estimation thread will work. If not
  specified \e 10ms is assumed.

--cutoff \e cutoff
  The parameter cutoff specifies the cutoff frequency (in Hz) of the low pass filters used in
  wholeBodyDynamicsTree, a typical values is 3.0 Hz.


\section portsa_sec Ports Accessed
Coming soon.

\section portsc_sec Ports Created
- \e /${name}/rpc:i
  This port enable to communicate via rpc with the module.
  To access it through the command line, open a terminal and write:
    yarp rpc /${name}/rpc:i
  Where ${name} is the name of the module (usually wholeBodyDynamics or wholeBodyDynamicsTree)
  After that, you can type `help` to get an overview of the available functions.
  If your robot is attached to the fixed base, the:
   > calib all
  command will be used to calibrate the FT sensors.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files

Configuration files of wholeBodyDynamicsTree load several groups,
to separate concerns about the different functionalities of the wholeBodyDynamicsTree.

\subsection `SIMPLE_LEGGED_ODOMETRY` group

  | Parameter name | Type | Units | Default Value | Required | Description | Notes |
  |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
  | initial_world_frame | string | - | - | Yes | Name of the frame of the model that is supposed to be coincident with the world/inertial at start | - |
  | initial_fixed_link  | string | - | - | Yes | Name of the link that is assumed to be fixed at start | - |
  | floating_base_frame  | string | - | - | Yes | Name of the frame assume to be the floating base | - |
  | stream_com           | -      | - | - | No  | If present, open a port /${name}/com:o where you stream the COM position in world coordinates | - |
  | additional_frames    | list of strings | - | - | No  | If present, open a port /${name}/frames:o where a Bottle of states of additional frames is streamed | - |

Consider that this values are just initialization values, but you can always
reset/change fixed link of the simple legged odometry using the RPC port.

It the odometry is correctly configured, the /${name}/floatingbasestate:o port
will stream a bottle with three lists, that are:
 * the 4x4 Transform yarp::sig::Matrix representing the world_H_floatingbase transform
(i.e. the matrix that transforms position homogenous vectors expressed in the floatingbase frame to
 vector expressed in the world frame)
 * The 6x1 yarp::sig::Vector of the coordinates of the base link twist

\section tested_os_sec Tested OS
Linux and OS X.

\section example_sec Example
If you properly setted the YARP_ROBOT_NAME enviromental variable,
and if you provide a proper wholeBodyDynamicsTree.ini configuration file
you can simply launch wholeBodyDynamicsTree using the following command:

\code
wholeBodyDynamicsTree --autoconnect
\endcode


\author Silvio Traversaro

*/


#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include "wholeBodyDynamicsTree/wholeBodyDynamicsModule.h"

#define DEFAULT_YARP_CONTEXT "wholeBodyDynamicsTree"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;



int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("wholeBodyDynamicsTree.ini");         //default config file name.
    rf.setDefaultContext(DEFAULT_YARP_CONTEXT); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo()<< "Possible parameters";
        yInfo()<< "\t--context          :Where to find an user defined .ini e.g. /" << DEFAULT_YARP_CONTEXT << "conf" ;
        yInfo()<< "\t--from             :Name of the file .ini user for configuration." ;
        yInfo()<< "\t--wbi_conf_file    :Name of the configuration file used for yarpWholeBodyInterface ." ;
        yInfo()<< "\t--torque_estimation_joint_list :Name of the wbi::IDList of joint to use in torqueEstimation."
            << "\t                                This list should be found in the wholeBodyDynamicsTree configuration file"
            << "\t                                or in the yarpWholeBodyInterface configuration file. Default: ROBOT_DYNAMIC_MODEL_JOINTS" ;

        yInfo()<< "\t--robot            :Robot name, overload the setting contained in the wbi_conf_file configuration file."                                                ;
        yInfo()<< "\t--rate             :Period (in ms) used by the module. Default set to 10ms."                                                      ;
        yInfo()<< "\t--name             :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyDynamicsTree."    ;
        yInfo()<< "\t--enable_w0_dw0/disable_w0_dw0    :Enable/disable use of angular velocity and acceleration measured from the IMU (default: disabled)." ;
        yInfo()<< "\t--autoconnect      :Autoconnect torques port for low-level torque feedback. " ;
        yInfo()<< "\t--assume_fixed     :Use a link as a kinematic root in estimation and calibration (assuming a constant gravity). Possible options: (root_link, l_foot_dh_frame, r_foot_dh_frame).";
        yInfo()<< "\t--assume_fixed_from_odometry :Use the fixed link from odometry for assume a constant gravity in estimation and calibration";
        yInfo()<< "                                 ***NOTE: with this option only the calibration on two feet is supported. Furthermore the link should not be switched during calibration.";
        yInfo()<< "                                 Furthermore the only supported fixed link for odometry are r_foot and l_foot***";
        yInfo()<< "\t--output_clean_ft  :Output the measure of the FT sensors without offset in set of ports." ;
        yInfo()<< "\t--min_taxel  threshold   :Filter input skin contacts: if the activated taxels are lower than the threshold, ignore the contact (default: 1)." ;
        yInfo()<< "\t--smooth_calibration switch_period : Perform a smooth calibration (i.e.: don't stop estimating torques during calibration, and then smoothly change the ft offsets)";
        yInfo()<< "\t                                     the switch_period express the period (in ms) used for offset interpolation.";
        yInfo()<< "\t--cutoff           :cutoff frequency (in Hz) of the low pass filters used in wholeBodyDynamicsTree.";
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    wholeBodyDynamicsModule module;
    return module.runModule(rf);
}
