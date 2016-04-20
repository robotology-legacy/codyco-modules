## Module Description

This modules generates in principle joint trajectories for a yarp-based humanoid robot whose URDF description is readily available, using a traditional [ZMP-based approach]()<sup>1</sup>. The module must be provided with the center of mass trajectory the robot is supposed to follow, and timed foothold patterns for the left and right foot.

## Procedure
1. Set the environmental variable `YARP_ROBOT_NAME`.
2. Prepare your Matlab environment accordingly before launching the patternGenerator script in Matlab as described [here](https://github.com/robotology/codyco-modules/tree/newModule/icubWalkingIK/src/modules/icubWalkingIK/app/scripts).
3. Launch the `patternGenerator` script in Matlab after setting the desired walking parameters in the file `walkingParams.txt`. The result will be three files that will be installed in the corresponding [robot directory](http://www.yarp.it/yarp_data_dirs.html#datafiles_contextsrobots). In addition, also the walkingParams.txt file will be installed.
4. Write a configuration file for the [`yarpWholeBodyInterface`](https://github.com/robotology/yarp-wholebodyinterface) with a list of the parts that you want to include for your robot For instance, in the configuration file [`yarpWholeBodyInterface_icubWalkingIK.ini`](https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface_icubWalkingIK.ini) you will find the list [`ROBOT_WALKING_IK`](https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface_icubWalkingIK.ini#L86) containing only the two legs and torso of the robot. The name of this configuration file should be [**specified**](https://github.com/robotology/codyco-modules/blob/newModule/icubWalkingIK/src/modules/icubWalkingIK/app/robots/icubGazeboSim/iCubWalkingIKModule.ini.in#L9) within the configuration file of `icubWalkingIK`. 
5. When opening the configuration file of `icubWalkingIK` you might notice a key whose value corresponds to the robot-specific installation directory. This value has been automatically filled by CMake when configuring and compiling `codyco-modules`. 
6. Write a configuration file for the [`yarpWholeBodyInterface`](https://github.com/robotology/yarp-wholebodyinterface) containing the list of parts that you want to take into account in the computations. For instance, for iCub, we can use the file [`yarpWholeBodyInterface_icubWalkingIK.ini`](https://github.com/robotology/codyco-modules/blob/newModule/icubWalkingIK/src/modules/icubWalkingIK/app/robots/icubGazeboSim/iCubWalkingIKModule.ini.in). By default, the configuration file of icubWalkingIK contains this name. In the next section you can find a table with a brief description of each parameter in the configuration file. 
7. The module responds to the `rpc` command `run`. Therefore, launch the module `icubWlakingIK` from terminal or your preferred IDE, then open another window and type `yarp rpc /icubWalkingIK/rpc`. Enter `run` and the module will perform inverse kinematics on the different specified targets (COM and both feet end-effectors). After a few seconds it will generate three files: `test_ik_pg.csv`, `real_com_traj.csv`, `com_l_sole.csv`, `real_left_foot.csv` and `real_right_foot.csv`. 
8. Finally, to split the previously generated files into files that the module [`walkPlayer`](https://github.com/robotology/codyco-modules/tree/master/src/modules/walkPlayer) can interpreted, launch the next two Matlab scripts: [`mainGenerateFilesForWalkPlayer`](https://github.com/robotology/codyco-modules/blob/newModule/icubWalkingIK/src/modules/icubWalkingIK/app/scripts/mainGenerateFilesForWalkPlayer.m) and [`mainGenerateFilesForTorqueBalancing`](https://github.com/robotology/codyco-modules/blob/newModule/icubWalkingIK/src/modules/icubWalkingIK/app/scripts/mainGenerateFilesForTorqueBalancing.m).
9. If you want to see the results using the simulator, launch the `walkPlayer` with the following parameters:
`walkPlayer --robot icubGazeboSim --period 10 --refSpeedMinJerk 0 --filename icub_walk_seq --execute --torqueBalancingSequence torqueBalancing`

## Configuration file 
In the following table you can find a description of the current parameters in the configuration file of this module and a brief description.

| Parameter                      | Description                                                                             |
| -------------------------------|:---------------------------------------------------------------------------------------:|
|name                            |Prefix of the ports opened by this module [icubWalkingIK]                                |
|robot                           |icub for the real robot, icubGazeboSim for the simulator                                 |
|period                          |Period of the thread in ms [10]                                                          |
|patternFile                     |COM pattern file name as generated by the patternGenerator script [comTraj_iCubGenova01] |
|outputDir                       |Robot-specific install directory. Automatically filled by CMake.                         |
|stream_floating_base_pose       |Assuming the robot is walking, this will stream the floating base when true [false]      |
|                                |                                                                                         |
|wbi_config_file                 |Name of the configuration file used by yarpWholeBodyInterface                            |
|wbi_joints_list                 |Name of the robot parts list in the yarpWholeBodyInterface config file.                  |
|                                |                                                                                         |
|**[odometry_params]**           |Odometry params group                                                                    |
|initial_world_reference_frame   |Name of the frame to be used as initial world reference frame [l_sole]                   |
|initial_fixed_frame             |Currently this must be equal to initial_world_reference_frame [l_sole]                   |
|floating_base                   |Name of the link to be used as floating_base                                             |
|offset_from_world_reference     |Offset from the previously defined world reference frame                                 |
|world_between_feet              |When true, this flag allows to have the world ref. frame between both feet.              |
|                                |                                                                                         |
|**[inverse_kinematics_params]** |Inverse Kinematics params group                                                          |
|step_tolerance                  |Tolerance used to decide convergence of the inverse kinematics algorithm                 |
|lambda                          |Parameter involved in the inverse kinematics algorithm. No need to change.               |
|max_iter                        |Maximun number of iterations per end-effector inverse kinematics                         |
|trials_initial_IK               |Maximun number of iterations used at the beginning to get a more accurate initial COM.   |




<sup>1</sup> This should link to the corresponding paper. Waiting for publication. 
