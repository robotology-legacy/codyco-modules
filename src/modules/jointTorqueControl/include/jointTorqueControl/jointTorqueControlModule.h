/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Daniele Pucci
 * email:  daniele.pucci@iit.it
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
*
@ingroup codyco_module
\defgroup icub_jointTorqueControl linearRegressorsAdaptiveControl

This module implements the whole-body torque control.

\section intro_sec Background

the robot's motors are controlled by using 
the PWM technique, i.e. the tension applied to each motor is a square wave of constant frequency and amplitude, 
and the duty cycle of this wave is used as control input. 

	V(t) ^           Tdc
             |         /-----/ 
          Vb | _  _  _  _____             ____ _
             |         |     |           |
             |         |     |           |
             |         |     |           |
             |_________|_____|___________|_______ 
                       /-----------------/       t
                                  T 

   T: period of the square wave;
   V: tension applied to the motor;
  Vb: (positive) amplitude of the square wave;
 Tdc: duty cycle time;

The relationship between the link's torque tao and the tension V applied to the motor is assumed to be 

	V  = kt*tao + kv*dq + kc*sign(dq),    
	
with kt, kv, kc three constants, and dq the link's velocity. Since the tension V(t) is a high-frequency square 
wave, we can assume that the above relationship holds for V = Vm, where Vm stands for the "mean tension value" 
over the time period T. By direct calculations, one can verify that 

	Vm = (Tdc/T)*Vb. 
	
Also, discontinuities are always challenging in practice. So, it is best to smooth the sign function. 
Among an infinite possible choices, we choose the hyperbolic function instead of sign(.). Then one has

	    Vm  = kt*tao + kv*dq + kc*tanh(ks*dq).      		    			 (1)   

The model (1) can be improved by modeling eventual parameters' asymmetries with respect to the joint velocity dq. 
In particular, the parameters kv and kc may depend on the sign of dq, and have different values depending on this 
sign. Then, an improved model is:

	Vm  = kt*tao + [kvp*s(dq) + kvn*s(-dq)]*dq + [kcp*s(dq) + kcn*s(-dq)]*tanh(ks*dq),       (2)    

where the function s(x) is the step function, i.e.
		  _
		 |  1, x >= 0,
	s(x) =  <
	     |_ 0, x < 0.
		 
As stated, Eq. (1) constitutes the relation between the tension applied to the motor and the link torque. Then, to 
generate a desired torque taoD coming from an higher control loop, it suffices to evaluate (1) with tao = taoD. In 
practice,however, it is a best practice to add a lower loop to generate tao so that tao will converge to taoD, i.e
        
	    et := tao - taoD,                                              			(3a)
	    tao = taoD - kp*et - ki*integral(et).                          			(3b)

\section intro_sec To do and warning list

a) Syncronization between aJ and taoD;
b) Anti wind-up and associated parameters;
c) Observer and a.p.;
d) Filtering parameters for velocity estimation and torque measurement;
	    
\section lib_sec Libraries
YARP.

\section parameters_sec Parameters


\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS Linux.

\section example_sec Example Instantiation of the Module
skinDriftCompensation --context graspingDemo/conf --from skinDriftCompensationRight.ini


\author Daniele Pucci 

Copyright (C) 2013 CoDyCo Project

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at ICUB_HOME/main/src/modules/skinDriftCompensation/include/iCub/skinDriftCompensation/SkinDriftCompensation.h.
**/


#ifndef __LOCOMOTION_PLANNER_MODULE_H__
#define __LOCOMOTION_PLANNER_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include <paramHelp/paramHelperClient.h>
#include <paramHelp/paramHelperServer.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <jointTorqueControl/jointTorqueControlThread.h>
 
using namespace std;
using namespace yarp::os; 
using namespace paramHelp;
using namespace wbi;

namespace jointTorqueControl
{

class jointTorqueControlModule: public RFModule, public CommandObserver
{
    /* module parameters */
    string  moduleName;
    string  robotName;
    string  locoCtrlName;
    string  fileName;
    int     period;

    Port                        rpcPort;	// a port to handle rpc messages
    jointTorqueControlThread*   ctrlThread;     // locomotion control thread
    ParamHelperClient*          torqueCtrl;     // helper class for communicating with the locomotion controller
    ParamHelperServer*          paramHelper;    // helper class for communication
    wholeBodyInterface*         robotInterface; // interface to communicate with the robot

public:
    jointTorqueControlModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(){ return 0.1; }
    bool updateModule();

    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

};

}

#endif
//empty line to make gcc happy

