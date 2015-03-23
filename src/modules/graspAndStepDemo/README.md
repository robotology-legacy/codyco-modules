graspAndStepDemo
================

This lua component coordinates the graspAndStepDemo, in which the robot
takes a step to grasp an object outside its workspace.

To get a better understanding of the patterns used in this component,
it can be useful to read the paper [Coordinating Robotic Tasks and Systems with rFSM Statecharts](http://joser.unibg.it/index.php?journal=joser&page=article&op=view&path[]=52) .

Structure
---------

The component is executed from the main script `steppingDemo.lua`.

This script executes periodically the following:
* a `Coordinator`, implemented throught a `rFSM` finite state machine.
  This is loaded from the `fsm_stepping.lua` script, and is executed by the `steppingDemo.lua`.
* a `Configurator`, implemented throught the `entry`, `doo` and `exit` functions
  associated to the finite state machine states. Depending on the state, several actions
  can be performed by the configurator, tipically setting desireded values for controllers
  or trajectory generators.
* a `Monitor`, implemented in the `steppingMonitor` class contained
  in the `steppingMonitor.lua` file. This class raise events to the FSM based on the enviroment.
  For example it reads the external forces from the wholeBodyDynamics and raises an appropriate
  event if the external force on the left foot is below a given threshold.
* events can be raised by the Monitor class or can be readed from other modules from the `events:i` port.

Events
------

| Event name | Description |  Usual origin |
|:-----------:|:-------------:|:-----------:|
| `e_grasping_enabled`  | The `**GraspDemo` module is running and the grasping behaviour is **active** | Sent by the `**GraspDemo` module on `events:i` port |
| `e_grasping_disabled` | The `**GraspDemo` module is running and the grasping behaviour is **not active** | Sent by the `**GraspDemo` module on `events:i` port |
| `e_weight_on_left_foot` | The overall external force acting on the `l_foot` link is greater than or equal to the `force_threshold` `steppingDemo` parameter | Raised internally by the `steppingMonitor` class |
| `e_no_weight_on_left_foot` | The overall external force acting on the `l_foot` link is lower than the `force_threshold` `steppingDemo` parameter | Raised internally by the `steppingMonitor` class |
| `e_weight_on_right_foot` | The overall external force acting on the `r_foot` link is greater than or equal to the `force_threshold` `steppingDemo`  parameter | Raised internally by the `steppingMonitor` class |
| `e_no_weight_on_right_foot` | The overall external force acting on the `r_foot` link is lower than to the `force_threshold` `steppingDemo` parameter | Raised internally by the `steppingMonitor` class |
| `e_right_step_requested`    | The `**GraspDemo` module requested a right step to enlarge its working space |  Sent by the `**GraspDemo` module on `events:i` port |
| `e_left_step_requested`    | The `**GraspDemo` module requested a left step to enlarge its working space |  Sent by the `**GraspDemo` module on `events:i` port |
| `e_right_step_completed`    | The left step have been completed | - |
| `e_left_step_completed`    | The right step have been completed | - |
| `e_left_leg_swing_motiondone` | The swing motion of the left leg have been completed | - |
| `e_right_leg_swing_motiondone` | The swing motion of the right leg have been completed | - |
