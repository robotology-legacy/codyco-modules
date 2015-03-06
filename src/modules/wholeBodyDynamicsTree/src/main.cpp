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
generated during mounting, are defined by the first FT data. In the
future it will also be given the possibility to set the offsets of
the sensors.
The model of the sensor measurements considers a fixed base, with z-axis
pointing upwards. The estimation of the external wrench applied at the
end-effector of the limb has the same orientation of the fixed base frame.
For further information about the use of this module and of the iCub force control interface, please refer to the force control page:
http://wiki.icub.org/wiki/Force_Control

\section lib_sec Libraries
- YARP library.
- iDynTree library.
- yarpWholeBodyInterface library

\section parameters_sec Parameters

--robot \e name
- The parameter \e name identifies the robot name. If not specified
  \e icub is assumed.

--rate \e r
- The parameter \e r identifies the rate the estimation thread will work. If not
  specified \e 10ms is assumed.


\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created


\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None

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



#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <wholeBodyDynamicsTree/wholeBodyDynamicsModule.h>

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
        cout<< "Possible parameters"                                                                                                                                          << endl << endl;
        cout<< "\t--context          :Where to find an user defined .ini e.g. /" << DEFAULT_YARP_CONTEXT << "conf"                                   <<endl;
        cout<< "\t--from             :Name of the file .ini user for configuration."                                                                                       <<endl;
        cout<< "\t--wbi_conf_file    :Name of the configuration file used for yarpWholeBodyInterface ." << endl;
        cout<< "\t--torque_estimation_joint_list :Name of the wbi::IDList of joint to use in torqueEstimation." << endl
            << "\t                                This list should be found in the wholeBodyDynamicsTree configuration file" << endl
            << "\t                                or in the yarpWholeBodyInterface configuration file. Default: ROBOT_DYNAMIC_MODEL_JOINTS" << endl;

        cout<< "\t--robot            :Robot name, overload the setting contained in the wbi_conf_file configuration file."                                                                                  <<endl;
        cout<< "\t--rate             :Period (in ms) used by the module. Default set to 10ms."                                                                                        <<endl;
        cout<< "\t--name             :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyDynamicsTree."                                      <<endl;
        cout<< "\t--enable_w0_dw0/disable_w0_dw0    :Enable/disable use of angular velocity and acceleration measured from the IMU (default: disabled)." << endl;
        cout<< "\t--autoconnect      :Autoconnect torques port for low-level torque feedback. " << endl;
        cout<< "\t--assume_fixed     :Use a link as a kinematic root in estimation (assuming a constant gravity). Possible options: (root_link, l_sole, r_sole)." <<endl;
        cout<< "\t--assume_fixed_base_calibration :Use the root link as a kinematic root  in calibration (assuming constant gravity)." <<endl;
        cout<< "\t--output_clean_ft  :Output the measure of the FT sensors without offset in set of ports." << endl;
        cout<< "\t--min_taxel  threshold   :Filter input skin contacts: if the activated taxels are lower than the threshold, ignore the contact (default: 1)." << endl;
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
