##Module description

This module implements a torque control balancing strategy.
It computes the interaction forces at the feet in order to stabilise a desired centroidal dynamics, which ensures the tracking of a desired center-of-mass trajectory.
A cost function penalizing high joint torques - that generate the feet forces - is added to the control framework.

For details see [iCub whole-body control through force regulation on rigid non-coplanar contacts](http://journal.frontiersin.org/article/10.3389/frobt.2015.00006/abstract)

###Launch procedure
The procedure to run the torque balancing module is still quite elaborate.
Users willing to use the module should follow this list.

- Bring the robot in a suitable home position (e.g. `$ yarpmotorgui --from robotMotorGuiBalancingTwoFeet.ini` and then pressing the 'Home All' button)
- (Robot Only) Launch `wholeBodyDynamicsTree` with the following parameters: `--autoconnect --assume_fixed l_foot_dh_frame`
- (Robot Only) Execute the [sensors calibration script](https://github.com/robotology/codyco-modules/blob/master/src/scripts/twoFeetStandingIdleAndCalib.sh): `$ twoFeetStandingIdleAndCalib.sh`
- Launch `torqueBalancing`
- Connect with the rpc module to `/torqueBalancing/rpc` and type `start`


##Module details
###Configuration file

####General section
- `name`: module name. Ports will be opened with this name. Default to `torqueBalancing`
- `robot`: name of the robot to connect to
- `period`: controller period in milliseconds. Default is 10ms (100Hz)
- `dynSmooth`: smoothing time for constraints switching (removal or adding)
- `modulePeriod`: module-thread period in seconds. Currently this thread is used only to send debug data. Default to 0.25s (250ms)
- `wbi_config_file`: name (or full path, see ResourceFinder documentation) to the whole body interface initialization file
- `wbi_joint_list`: name of the torque controlled joint list.

####Gains
#####Center of Mass task
- `comIntLimit`: integral limit on the CoM PID. One single positive value.
- `comKp`: proportional gains. 3 values
- `comKd`: derivative gains. 3 values
- `comKi`: integral gains. 3 values

#####Other parameters
- `kw`: gain used for the desired rate of change of the momentum. One value
- `kImp`: gains for the impedance (low level) task. The number must match the number of torque controlled joints.
- `tsat`: saturations to be applied to the output torques. The number must match the number of torque controlled joints. It must be a positive value.

###Monitored variables
The following variables are monitored, that is, they are streamed (in a big vector) in the `monitor:o` port:

- 'Desired CoM acceleration', that is the output of the CoM PID. 3 values
- 'CoM Error': 3 values
- 'CoM integral error': 3 values. The integral of the CoM error
- 'Feet forces': the computed feet forces. 12 values (left and right feet)
- 'Torques': the computed (and sent to the actuators) torques.

### Interacting with the module
#### RPC
The opened RPC port allows to send the following commands:

- `start`: starts the module
- `stop`: stops the module
- `quit`: quits the module

It also allows to change the value of the gains.

When the user sends a `start` command the actual CoM position and the joint positions are set as a reference for the CoM PID and for the impedance task.

#### CoM reference
It is possible to send a `CoM` reference by connecting to the streaming port `comDes:i`. The port expects 9 elements: 3 values for the  desired CoM  position, 3 values for the  desired CoM velocity and 3 values for the  desired CoM acceleration

#### Joint reference
It is possible to send the impedance resting position as a reference to the streaming port `qdes:i`. This port expects the same number of element as the torque controlled joints. References are in **radians**

### Module architecture
The module is composed of the following parts:

- The module itself. It is responsible to setup the application (read configuration files, etc.) and of handling RPC communication and monitoring the variables.
- The controller. It implements the "math" described in the paper. 
- References generators. These elements implements a PID-like controller.

#### Note on reference generators
The implementation of the reference generator is agnostic of the underlining physical signal. To get the feedback they use a generic interface (currently implemented to retrieve position and velocity of an end-effector and of the CoM).

#### Citing this contribution
In case you want to cite the content of this module please refer to [iCub whole-body control through force regulation on rigid non-coplanar contacts](http://journal.frontiersin.org/article/10.3389/frobt.2015.00006/abstract) and use the following bibtex entry:

```
 @article{Nori_etal2015,
 author="Nori, F. and Traversaro, S. and Eljaik, J. and Romano, F. and Del Prete, A. and Pucci, D.",
 title="iCub whole-body control through force regulation on rigid non-coplanar contacts",
 year="2015",
 journal="Frontiers in {R}obotics and {A}{I}",
 volume="1"
 }
```
