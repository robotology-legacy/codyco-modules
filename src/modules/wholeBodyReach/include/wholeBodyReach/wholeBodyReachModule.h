/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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
 * @ingroup codyco_module
 * \defgroup codyco_wholeBodyReach wholeBodyReach
 *
 * This module implements a whole-body inverse-dynamics controller to achieve a reaching behavior.
 * The robot is supposed to reach with one hand while (possibly) using the other arm to make 
 * contact with the environment to allow reaching farther in front of it.
 *
 * \section tasks_sec Stack Of Tasks
 * Currently the controlled tasks in order of priority are:
 * - contact constraints at both feet (12 dofs)
 * - com (or momentum, depending on the control algorithm) (3 or 6 dofs)
 * - right hand (3 dofs)
 * - left forearm (3 dofs)
 * - joint posture (25 dofs)
 *
 * The control algorithm that gave the best results in simulation is COM_SOT (5), which controls
 * the CoM position, but not the angular momentum.
 *
 * \section config_sec Configuring the module
 * The module uses a large number of configuration parameters, which can be found in the file
 * conf/defaultSim.ini. More details on the module's parameters can be found by connecting to
 * the RPC port of the module (the port's name is "/moduleName/rpc") and typing "help".
 *
 * For sure the user needs to modify the parameter "urdf", which provides the complete path
 * to the urdf model of the robot.
 
 \section example_sec Example Instantiation of the Module
 First of all start the module specifying the configuration file:
     wholeBodyReach --from defaultSim.ini
 Then connect to the RPC port of the module:
     yarp rpc /wbr/rpc
 Then start the module by giving the start command on the RPC port.
     start
 At start all the desired positions are set to the measured positions, so the robot should not move.
 When the module is started the robot is supposed to be standing on both feet, with the feet parallel
 to each other. After the controller is started you can modify the reference positions of the different
 tasks. You can this in two ways; you can directly set the reference position of a task, e.g.:
     set xd com 0.04 -0.07 0.45
 Alternatively you can use the "go down" and "go up" commands, which change at the same time the desired 
 positions of all the tasks (actually at the moment it is only for CoM, right hand and joint posture).
     go down
 The desired positions associated to the "go down" commands can be changed by configuration file
 or by rpc port through the parameters "go down com", "go down hand", "go down posture". 
 The same applies for the "go up" command.
 
 \section scope_sec Monitoring the behavior of the controller
 A bunch of configuration files for plotting some useful quantities using yarpscope
 can be found in the conf folder. In particular:
 - com_scope: plot the reference and measured position of the CoM and angular momentum.
 - hand_forearm_scope: plot reference and measured positions of right hand and left forearm.
 - force_inequalities: plot the desired ZMPs, ratio of tangential and normal forces, ratio of normal 
 moments and forces; namely it plots all the bounded quantities related to the contact forces. This 
 is useful to monitor the robustness of the desired contact forces.
 - joint_limits: plot the measured joint angles normalized between 0 and 1, useful to monitor when some
 joints are getting close to their limits.
 - joint_vels: plot the measured joint velocities in deg/sec
 
 \section parameters_sec Parameters
 List of the module commands and descriptions:
 - start: Start the controller
 - stop: Stop the controller
 - help: Get instructions about how to communicate with this module
 - quit: Stop the controller and quit the module
 - reset profiler: Reset the profiling statistics
 - go down: Move CoM and hand references to go down
 - go up: Move CoM and hand references to go back up
 - grasp: Close the fingers to grasp
 
 List of the configuration parameters:
 - name: Name of the instance of the module
 - period: Period of the control loop (ms)
 - robot: Name of the robot
 
 List of the streaming parameters:
 - support phase: Contact support phase, 0: double, 1: triple
 - xd com: Desired 3d position of the center of mass
 - xd forearm: Desired position/orientation of the forearm
 - xd hand: Desired position/orientation of the grasping hand
 - qd: Desired joint angles
 - H_w2b: Estimated rototranslation matrix between world and robot base reference frames
 
 List of the rpc parameters:
 - ctrl alg: Id of the control algorithm (MOMENTUM_SOT=0, NULLSPACE_PROJ=1, COM_POSTURE=2, MOMENTUM_POSTURE=3, MOMENTUM_SOT_SAFE=4, COM_SOT=5)
 - kp momentum: Proportional gain for the momentum control
 - kp forearm: Proportional gain for the forearm control
 - kp hand: Proportional gain for the hand control
 - kp posture: Proportional gain for the joint posture control
 - ki posture: Integral gain for the joint posture control
 - kp constraints: Proportional gain for correcting constraint drifts
 - kd momentum: Derivative gain for the momentum control
 - kd forearm: Derivative gain for the forearm control
 - kd hand: Derivative gain for the hand control
 - kd posture: Derivative gain for the joint posture control
 - tt momentum: Trajectory time for the momentum minimum jerk trajectory generator
 - tt forearm: Trajectory time for the forearm minimum jerk trajectory generator
 - tt hand: Trajectory time for the hand minimum jerk trajectory generator
 - tt posture: Trajectory time for the posture minimum jerk trajectory generator
 - dyn damp: Numerical damping used to regularize the dynamics resolutions
 - constr damp: Numerical damping used to regularize the constraint resolutions
 - task damp: Numerical damping used to regularize the task resolutions
 - use nullspace base: 0: use nullspace projectors, 1: use nullspace basis (not tested yet!)
 - q max: Joint upper bounds [deg]
 - q min: Joint lower bounds [deg]
 - joint lim min dist: Minimum distance to maintain from the joint limits [deg]
 - dq max: Max joint velocities [deg/s]
 - ddq max: Max joint accelerations [deg/s^2]
 - joint lim dt: Timestep to predict future joint positions to compute joint acceleration limits [s]
 - force friction: Friciton coefficient for tangential forces
 - moment friction: Friciton coefficient for normal moments
 - wrench weights: Weights used to penalize 6d wrenches
 - go down com: Desired com position associated to the go-down command
 - go down hand: Desired hand position associated to the go-down command
 - go down q: Desired joint positions associated to the go-down command
 - go up com: Desired com position associated to the go-up command
 - go up hand: Desired hand position associated to the go-up command
 - go up q: Desired joint positions associated to the go-up command
 - integrate EoM: If true the desired joint torques are not sent to the robot/simulator but are locally integrated
 - integrator dt: Timestep used by the numerical integrator
 - integrator damp: Numerical damping used by the numerical integrator to solve the constraints
 
 \section lib_sec Libraries
 YARP, Eigen, iCub, wbi, paramHelp.
 
\section tested_os_sec Tested OS Mac, Linux.
 
 \author Andrea Del Prete
 
 Copyright (C) 2014 CoDyCo Project
 
 CopyPolicy: Released under the terms of the GNU GPL v2.0.
 
 This file can be edited at CODYCO_MODULE_HOME/src/modules/wholeBodyReach/include/wholeBodyRead/wholeBodyReachModule.h.
 **/

#ifndef WHOLE_BODY_REACH_MODULE_H__
#define WHOLE_BODY_REACH_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include <paramHelp/paramHelperServer.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <wholeBodyReach/wholeBodyReachThread.h>
 
using namespace std;
using namespace yarp::os; 
using namespace paramHelp;
using namespace wbi;

namespace wholeBodyReach
{

class WholeBodyReachModule: public RFModule, public CommandObserver
{
    /* module parameters */
	string  moduleName;
	string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

	Port                rpcPort;		// a port to handle rpc messages
	WholeBodyReachThread*   ctrlThread;     // wholeBodyReach control thread
    ParamHelperServer*  paramHelper;    // helper class for rpc set/get commands and streaming data
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    WholeBodyReachModule();

	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports 
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod(){ return 30;  }
	bool updateModule();

    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

};

}

#endif
//empty line to make gcc happy

