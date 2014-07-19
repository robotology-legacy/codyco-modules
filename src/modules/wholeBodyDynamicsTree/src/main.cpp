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

void iCubPartVersionOptionsPrint()
{
    cout<< "\t--headV1/headV2    :Version of the head."  <<endl;
    cout<< "\t--legsV1/legsV2    :Version of the legs."  <<endl;
    cout<< "\t--feetV1/feetV2    :Version of the feet."  <<endl;
    #ifdef CODYCO_USES_URDFDOM
    cout<< "\t--urdf             :URDF file to load."    <<endl;
    #endif
}

int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("default.ini");         //default config file name.
    rf.setDefaultContext(DEFAULT_YARP_CONTEXT); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<< "Possible parameters"                                                                                                                                          << endl << endl;
        cout<< "\t--context          :Where to find an user defined .ini file within $ICUB_ROOT/app e.g. /" << DEFAULT_YARP_CONTEXT << "conf"                                   <<endl;
        cout<< "\t--from             :Name of the file.ini to be used for calibration."                                                                                       <<endl;
        cout<< "\t--rate             :Period used by the module. Default set to 10ms."                                                                                        <<endl;
        cout<< "\t--robot            :Robot name. Set to icub by default."                                                                                  <<endl;
        cout<< "\t--name             :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyDynamicsTree."                                      <<endl;
        iCubPartVersionOptionsPrint();
        cout<< "\t--enable_w0_dw0/disable_w0_dw0    :Enable/disable use of angular velocity and acceleration measured from the IMU (default: disabled)." << endl;
        cout<< "\t--autoconnect      :Autoconnect torques port for low-level torque feedback. " << endl;
        cout<< "\t--assume_fixed     :Use a link as a kinematic root in estimation (assuming a constant gravity). Possible options: (root_link, l_sole, r_sole)." <<endl;
        cout<< "\t--assume_calibration_fixed_root :Use the root link as a kinematic root  in calibration (assuming constant gravity)." <<endl;
        cout<< "\t--min_taxel  threshold   :Filter input skin contacts: if the activated taxels are lower than the threshold, ignore the contact (default: 1)." << endl;
        cout<< "\t--zmp_test_left/--zmp_test_right : Enable debug port outputs for robot single standing on left or right foot. " << endl;
        cout<< "\t\t this option will open the following ports: " << endl;
        cout<< "\t\t\t /local_name/joint_ankle_cartesian_wrench:o 6 element vector: force torque transmitted from link *_ankle_1 "
            << "\t\t\t\t to link *_foot expressed on the orientation frame of *_sole and with torque expressed with respect to the origin of frame of link *_sole" << endl;
        cout<< "\t\t\t /local_name/joint_ankle_cartesian_wrench_from_model:o 6 element vector: as before, but calculated from model " << std::endl;
        cout<< "\t\t\t /local_name/joint_foot_cartesian_wrench:o 6 element vector: force torque transmitted from link *_foot "
            << "\t\t\t\t to the enviroment expressed on the orientation frame of *_sole and torque expressed with respect to the origin of frame of link *_sole" << endl;
        cout<< "\t\t\t /local_name/joint_foot_cartesian_wrench_from_model:o 6 element vector: as before, but calculated from model " << std::endl;
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
