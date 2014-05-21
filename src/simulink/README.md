WBI Toolbox (WBI-T) - Simulink Wrapper for Whole Body Control
-------------------------------------------------------------

**NEWS: The WBI Toolbox will be presented in the workshop MATLAB/Simulink for Robotics Education and Research during ICRA 2014 with a live demo on the Gazebo simulator!! Check out the whole program of the workshop here: http://goo.gl/L76DbM**

This document contains basic instructions on how to install this toolbox, *tips and tricks* to do so and a walkthrough to get you started using it. Simulink blocks consist of S-functions (http://goo.gl/1GuHVd) which allow C/C++ user specific code compiled as Matlab Executable (MEX) files, thus extending the capabilities of the Simulink environment. In other words, MEX files have been created linking YARP, iCub, **iDynTree** (a more efficient and generic YARP-based robot dynamics library than its predecessor iDyn - http://goo.gl/BnGzKr) and CoDyCo, wrapping the **Whole Body Interface** described in http://goo.gl/dBWO3k. The following video shows a quick introduction to the way it works. For further information read the sections below.


<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=h2T4BtuDiJg
" target="_blank"><img src="http://img.youtube.com/vi/h2T4BtuDiJg/0.jpg" 
alt="Overview of the Simulink library for Whole Body Control" width="480" height="360" border="10" /></a>
</p>



###### Main Goal ######
> The library should allow non-programming experts or those researchers just getting acquainted with Whole Body Control to more easily deploy controllers either on simulation or a real YARP-based robotic platform, as well as to analyze their performance and take advantage of the innumerable MATLAB and Simulink toolboxes. We like to call it "rapid controller prototyping" after which a proper YARP module should be made for hard real time performance and final deployment.


###### Requirements
* Matlab V. 7.1+ and Simulink (Tested with Matlab R2013a, 7.14 R2012a/b, 7.12 R2011a).
* YARP (compiled as shared library -IMPORTANT-).
* CoDyCo. 
* iCub.
* Gazebo Simulator + `gazebo_yarp_plugins` (if you wish to use Gazebo instead of iCub_SIM).
* Operating Systems supported: Linux, MAC OS X, Windows.

**Note: The following instructions are for Linux distributions, but it works similarly on the other operating systems.**

###### Installation
Before going ahead with the compilation of the library, make sure that you have MATLAB and Simulink properly installed and running. Then, check that the MEX compiler for MATLAB is setup and working. For this you can try compiling some of the MATLAB C code examples as described in [http://www.mathworks.com/help/matlab/ref/mex.html#btz1tb5-12]. In the following steps assume that `$CODYCO_DIR` points to the `/build` directory of your CoDyCo installation and `$CODYCO_ROOT` to the corresponding `/src` directory. In case you are using the simulator, make sure that the iCub models are being loaded and the `gazebo_yarp_plugins` properly working. This is easy to verify as you need only to launch a `yarpserver`, then Gazebo and load the desired model, be it iCub (fixed) or iCub. If the robot does not fall under the effect of gravity, it means the plugins are working and you can go ahead with the installation of the Toolbox.

- **Compiling the WBI Toolbox.** When configuring the CMakeLists for CoDyCo make sure to enable the `CODYCO_USES_WBI_TOOLBOX` flag by doing
```bash
    cd $CODYCO_DIR
    ccmake ../
```
In the UI look for *CODYCO_USES_WBI_TOOLBOX* and press enter to turn it ON/OFF. Then as usual type c to configure until no stars (*) show up and g to generate. Finally, to compile type `make`.

~~**Soft Real Time.** For the time being, this block has been taken from the Matlab File Exchange [http://goo.gl/8LMWGD] and it has to be compiled from within MATLAB by changing its current directory to `${CODYCO_ROOT}/simulink/controllers/RealTimeSlower` and typing `mex sfun_time.c`. This will create a mex file according to your operating system and architecture, e.g. for a 32bits Linux-based OS you will get sfun_time.mexglx. This mex file will be used by the example models included in `b${CODYCO_ROOT}/simulink/controllers/` to slow down the simulation for a user-specified rate. It is recommended to define the rate in this block with a variable such as **Ts** and mask your final model where the user can later define the rate.~~ This block will be soon deprecated and replaced by the ySynchronizer block. It is still present in the library and the user does not need to compile it manually.


###### Before Using the Simulink Library
- **Prepare the MATLAB Environment.** When starting MATLAB it is recommended to add to its path the location of our mex files, i.e. `${CODYCO_DIR}/lib` as well as that for the controllers and the toolbox itself, i.e. `${CODYCO_ROOT}/src/simulink/controllers` by doing
```bash
    addpath([getenv(CODYCO_DIR) /lib])
    addpath([getenv(CODYCO_ROOT) /src/simulink/controllers])
```
You can also create a .m file with these two lines and launch MATLAB from terminal:
```bash
    matlab -r yourStartupFile
```

As an alternative you can also run the MATLAB script `startup_wbitoolbox.m` in `${CODYCO_ROOT}/simulink` which will try to add the previous directories automatically.

Depending on what you would like to do, remember that you can change the Simulink simulation settings by going to Simulation > Configuration Parameters > Solver and changing the `Stop time` to your desired value, or `inf` if you want the simulation to run 'forever'. Also, since this toolbox is very oriented to real implementation, the `Solver Options` in the very same window should be changed to `Fixed Step` Type and `discrete(no continuous states)` as the Solver.

- **Test the Library.** In `$CODYCO_ROOT/src/simulink/controllers` you can find some models for testing (more on this in the readme of the aforementioned directory). In order to test that the library is working correctly and properly linking YARP you can try running a `yarpserver`, after which you can go to the controllers directory in MATLAB and open yarpwrite.mdl. Before starting the simulation, give a name to the YARP port where you want to write by double clicking the block and editing the mask that pops up. 

- **For MAC OS X Users.** It has been reported that on MAC OS you need to define the place where you want MATLAB to find at runtime dynamic libraries for YARP, in case you have compiled YARP in a directory different from the default one. This can be added in `${MATLAB_ROOT}/bin/.matlab7rc.sh`. 
```bash
    chmod +w .matlab7rc.sh
    LDPATH_SUFFIX = 'YOUR_ENV_DYLD_LIBRARY_PATH'
    chmod -w .matlab7rc.sh
```
- **Additional notes.** In case Matlab has trouble finding a specific library, a workaround is to launch it preloading the variable `LD_PRELOAD` (or `DYLD_INSERT_LIBRARIES` on Mac OS X) with the location of the missing library.

###### Using the Simulink Library
Internally, the toolbox uses YARP's ResourceFinder (http://goo.gl/4zAS6r). When you first pull this repository you will download some default .ini files for each robot for which we have used the Toolbox (i.e. iCubGenova01, iCubGenova03, icubGazeboSim). These .ini files can be found in `${CODYCO_ROOT}/src/simulink/libraries/wbInterface/conf/wbit`. If you want to use this toolbox with a different robot, you can create a new .ini file in which you set the following parameters:

- **robot**:     robot name (i.e. icubGazeboSim, iCubGenova01, etc).
- **local**:     prefix of the YARP ports that the WBI will open.
- **headV**:     [int] head version of your robot.
- **legsV**:     [int] legs version of your robot.
- **feetFT**:    [bool] Is the robot endowed with force/torque sensors for its feet?
- **uses_urdf**: [bool] Is your robot fixed to root or standing on the floor? (for icubGazeboSim this would mean whether you are using the `iCub (fixed)` or `iCub` models)
- **urdf**:      When using the icubGazeboSim you need to specify the exact location of the urdf model of the robot as found in `/icub-model-generator/generated/`. These models can be downloaded from the repository https://github.com/robotology-playground/icub-model-generator. This step is still succeptible to changes in the near future.

You will find a few controllers and models that have already been used with the iCub simulator and the real robot as found in `${CODYCO_ROOT}/src/simulink/controllers`. The latest whole-body torque controller for iCub's center of mass can be found in `${CODYCO_ROOT}/src/simulink/controllers/torqueBalancing/controller.mdl`

###### Tested OS
Linux, Windows, MAC OS X

###### To Do List
- [ ] Yarp read should read bottles!
- [x] ~~Debug incompatibilities with Gazebo (at the c++ whole body interface level)~~ - with Francesco Romano
- [x] ~~Compile the Soft Real Time mex as another module of the library. Possibly make our own.~~
- [x] ~~Modify YarpRead module so that you can specify the port you wanna read from and where you want it to connect. Connection should be done inside the block.~~
- [x] ~~Restructure code for wbInterface~~
- [x] ~~Expose computeMass() and generalizedBiasForces()~~
- [x] ~~Debug computeMass() and generalizedBiasForces()~~
- [ ] Documentation (Functions, etc)
- [ ] ZMP block.
- [x] ~~Check minimum jerk generator.~~
- [x] ~~Reproduce COM Controller as a Force Controlled version.~~
- [x] ~~Documentation (Installation)~~
- [x] ~~How to properly get dynamic libraries linked at runtime on MAC OS X.~~
- [x] ~~Divide blocks into subgroups (actuators, estimators, etc) and put them all together as a real Simulink Library :D~~
- [x] ~~Create icons for each block in the library.~~
 
