/*
 * Copyright (C) 2013 CODYCO Project
 * Author: Serena Ivaldi
 * email:  serena.ivaldi@isir.upmc.fr
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 @ingroup codyco_modules
 
 \defgroup reachComBalance reachComBalance
 
 A module for providing an interface for performing reaching actions on the robot while balancing. The movement control is based on ISIR controller.
 
 \section lib_sec Libraries
 - YARP libraries.
 - iCub libraries.
 - CODYCO libraries: modHelp
  
 \section example_sec Example
 
 In a terminal, simply launch
 
 \code
 reachComBalance --from default.ini
 \endcode
 
 then to send commands open a rpc port:
 
 \code
 yarp rpc --client /myPort
 \endcode
 
 and connect it to the command port
 
 \code
 yarp connect /myPort /reachComBalance/rpc:i
 \endcode
 
 finally type commands in the rpc port, like:
 
 \code
 calibration
 goto right_arm -0.3 0.1 0.05
 \endcode
 
 \section tested_os_sec Tested OS
 Windows
 
 \author Serena Ivaldi
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <modHelp/modHelp.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "reachComBalance.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace modHelp;
using namespace std;

// necessary for cartesian interfaces
YARP_DECLARE_DEVICES(icubmod)


//---------------------------------------------------------
//                  MAIN
//---------------------------------------------------------

int main (int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)
    
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }
    
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("default.ini");         
    rf.setDefaultContext("ISIR_control/conf"); 
    rf.configure("ICUB_ROOT",argc,argv);
    
    if (rf.check("help"))
    {
        cout<< "Possible parameters"                                                                                                                                          << endl << endl;
        cout<< "\t--context          :Where to find an user defined .ini file "                                   <<endl;
        cout<< "\t--from             :Name of the file.ini to be used for calibration."                                                                                       <<endl;
        cout<< "\t--rate             :Period used by the module. Default set to 10ms."                                                                                        <<endl;
        cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default."                                                                                  <<endl;
//        cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. balancerModule."                                      <<endl;
//        cout<< "\t--display_only     :[on/off] Flag whether to display computed zmp on iCubGui or on terminal if verbose is on."                                              <<endl;
//        cout<< "\t--no_sens          :[on/off] Flag for precenting the module from reading additional sesnors (the ones not available in iCubSim)"                            <<endl;
//        
//        cout<< "\t--q0LL_both             :Initial position for the left  leg"                                                                                                      <<endl;
//        cout<< "\t--q0RL_both             :Initial position for the right leg"                                                                                                      <<endl;
//        cout<< "\t--q0TO_both             :Initial position for the torso    "                                                                                                      <<endl;
//        
//        cout<< "\t--verbose          :[on/off] Prints relevant data for debugging."                                                                                           <<endl;
//        cout<< "\t--torso            :[on/off] Enables balancing using the torso to compensate for whole body angular momentum."                                              <<endl;
//        cout<< "\t--springs          :[on/off] Uses right gains when springs have been set at the joints level."                                                              <<endl;
//        cout<< "\t--ankle_sens       :[on/off] Uses F/T sensors at the ankles."                                                                                               <<endl;
//        cout<< "\t--wbs_name         :Name of the wholeBodyDynamics module that is to be used by the module."                                                                 <<endl;
//        
//        cout<< "\t--sat_vel          :Maximum velocity (saturation level by scaling)."                                                                                        <<endl;
//        
//        cout<< "\t--pi_a_d_right     :Desired ZMP position while in right  foot support (expressed in * right * foot coordinates)."                                           <<endl;
//        cout<< "\t--pi_a_d_both      :Desired ZMP position while in double foot support (expressed in * right * foot coordinates)."                                           <<endl;
//        cout<< "\t--pi_c_d_left      :Desired ZMP position while in left   foot support (expressed in * left  * foot coordinates)."                                           <<endl;
//        
//        cout<< "\t--Kp_x             :Proportional gain for x direction of zmp controller."                                                                                   <<endl;
//        cout<< "\t--Kd_x             :Derivative gain for x direction of zmp controller."                                                                                     <<endl;
//        cout<< "\t--Kp_y             :Proportional gain for y direction of zmp controller."                                                                                   <<endl;
//        cout<< "\t--Kd_y             :Derivative gain for y direction of zmp controller."                                                                                     <<endl;
//        
//        cout<< "\t--Kp               :Proportional gain for the right to left foot displacement controller."                                                                  <<endl;
//        cout<< "\t--Kd               :Proportional gain for the right to left foor orientation controller."                                                                   <<endl;
//        
//        cout<< "\t--com_position     :specify a port to be opened in order to read the com_position in the root reference frame."                                             <<endl;
//        cout<< "\t--com_jacobian     :specify a port to be opened in order to read the com_jacobian w.r.t the root reference frame."                                          <<endl;
//        
//        cout<< "\t--r2l_error        :specify a port to output the tracking errors on the right/left foot position and orientation."                                          <<endl;
//        cout<< "\t--com_error        :specify a port to output the tracking errors for the COM."                                                                              <<endl;
//        
//        cout<< "\t--w0RL             :specify the wrench (force/torque) offset for the right leg F/T sensor."                                                                 <<endl;
//        cout<< "\t--w0LL             :specify the wrench (force/torque) offset for the left  leg F/T sensor."                                                                 <<endl;
//        
//        cout<< "\t--njRL             :specify number of joints in the right leg."                                                                                             <<endl;
//        cout<< "\t--njLL             :specify number of joints in the  left leg."                                                                                             <<endl;
//        cout<< "\t--njTO             :specify number of joints in the     torso."                                                                                             <<endl;
//        
//        cout<< "\t--r2l_jointsSwgLeg :(0/1 vector) specify the joints of the swing   leg dedicated to the right to left foot displacement controller."                          <<endl;
//        cout<< "\t--r2l_jointsSupLeg :(0/1 vector) specify the joints of the support leg dedicated to the right to left foot displacement controller."                          <<endl;
//        cout<< "\t--com_jointsTorso  :(0/1 vector) specify the joints of the     torso dedicated to the COM controller."                                                      <<endl;
        
        return 0;
    }
    
    //Creating the module
    ISIR_Balancer balancerModule;
    balancerModule.runModule(rf);
    
    return 0;
}
