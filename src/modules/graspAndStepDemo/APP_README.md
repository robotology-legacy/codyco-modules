graspAndStep Application (CoDyCo Y2 demo)
=========================================

The graspAndStep demo can be started/stopped using the `graspAndStep.xml.template` yarpmanager application.
This document is a short companion to explain the logic behind this connection.

Main Components launched
---------------------------

### graspAndStepDemo.lua
This is the main coordinator of the demo. It coordinates the demo, using the FSM
described in the README.md file in this directory.

### RedBallGraspDemo
This is a component (forked from the `icub-main` `demoGraspManager`) that
tracks an external target (provided by the `pf3dTracker` module) using the
robot gaze and then tries to grasp it. The main modification in this version
over the demoGraspManager one is that this version has been modified to send
the desired joint positions of the upper part of the iCub robot to the `torqueBalancing`
module instead of sending directly Cartesian level commands to the iCub cartesianController.

### torqueBalancing
This module implements a prioritized controller that takes in input:
 * the desired trajectory of the Center of Mass of the robot
 * the active contacts
 * the desired impedance equilibrium joint position.

To see how this different task are simultaneously achieved, please check torqueBalancing documentation.

### pf3dTracker
`pf3dTracker` is a canonical `icub-main` module used for getting the position of an external
object using the robot vision. It provides the information of the position of the object to grasp
to the `RedBallGraspDemo`.

### CodycoCoordinatorY2
This helper module (to be renamed) acts as a broker between `RedBallGraspDemo`,
`graspAndStepDemo.lua` and the
`torqueBalancing`. Namely, it performs trajectory generation on the setpoint
in position and com desired values provided by `RedBallGraspDemo` and `torqueBalancing`.
For some joints (mainly wrist joints) that are not handled by the `torqueBalancing` but
are used for kinematic inversion in `RedBallGraspDemo`, it supports sending that desired
position directly to the robot controlboards, using the `IPositionDirect` controlmode.

Helper Components launched
---------------------------
### yarpview
`yarpview` is used to visualize the output of `pf3dTracker` superimposed with the robot vision.

### iSpeak
`iSpeak` is used to make the robot speak, used for debugging states of the FSM.

### eventRepeater
`eventRepeater` is used to provide a nice RPC interface to the user for raising events in the FSM, mainly for debug. 
