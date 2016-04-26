Fill-in the `walkingParams.txt` file with the desired walking parameters. Then launch `tableCart` to obtain the desired ZMP-based COM trajectory for your robot as well as feet patterns. Currently suporting only straight walking trajectories. First, remember to setup your startup.m file in Matlab or environmental variables as described in the following section. 

## Example of a startup.m for these scripts to work correctly
```matlab
% iCubWalkingIK
display(['[WARNING] The env. variables ICUB_WALK_ROOT, CODYCO_SUPERBUILD_ROOT'... 
          'CODYCO_SUPERBUILD_DIR, YARP_DATA_DIRS, YARP_ROBOT_NAME and INSTALLED_ROBOT_DIRS have been set']);
setenv('ICUB_WALK_ROOT','/Users/jeljaik/Code/icub_walk_pg_ik');
setenv('CODYCO_SUPERBUILD_ROOT','/Users/jeljaik/Code/codyco-superbuild');
setenv('CODYCO_SUPERBUILD_DIR','/Users/jeljaik/Code/codyco-superbuild/build');
setenv('YARP_DATA_DIRS','/Users/jeljaik/Code/codyco-superbuild/build/install/share/codyco');
setenv('YARP_ROBOT_NAME','icubGazeboSim');
setenv('INSTALLED_ROBOT_DIRS', [getenv('CODYCO_SUPERBUILD_ROOT') '/build/install/share/codyco/robots/' getenv('YARP_ROBOT_NAME')]); 
```
