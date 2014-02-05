Simulink Library for Whole Body Control on Humanoid Robots 
----------------------------------------------------------

This document describes basic instructions on how to use this library, *tips and tricks* to do so and a walkthrough to get you started using it. Simulink blocks consist of S-functions (http://goo.gl/1GuHVd) which allow C/C++ user specific code compiled as Matlab Executable (MEX) files, thus extending the capabilities of the Simulink environment. In other words, MEX files have been created linking YARP, iCub and **iDynTree** (a more efficient and generic YARP-based robot dynamics library than its predecessor iDyn - http://goo.gl/BnGzKr) and wrapping the **Whole Body Interface** described in http://goo.gl/dBWO3k. Soft-Real Time is ensured by slowing down the simulation to respect a user specified rate.


###### Main Goal ######
> The library should allow non-programming experts or those researchers just getting acquainted with Whole Body Control to more easily deploy controllers either on simulation or the real platform, as well as to analyze their performance and take advantage of the innumerable MATLAB and Simulink toolboxes. We like to call it "rapid controller prototyping" after which a proper YARP module should be made for hard real time performance and final deployment.

###### Requirements
* Matlab V. 7.1+ (Tested with Matlab 7.14 R2012a, 7.12 R2011a)
* Yarp
* CoDyCo 
* iCub
* Gazebo Simulator + yarp plugins (if you wish to use Gazebo instead of iCub_SIM)

###### Installation
Before going ahead with the compillation of the library, make sure that you have Matlab and Simulink properly installed and running. Then, check that the MEX compiler for MATLAB work.
1. **Compiling the Simulink Library.** When configuring the CMakeLists for CoDyCo make sure to enable to SIMULINK_LIBRARY flag. 
```bash
    cd $CODYCO_DIR
    ccmake ../
```
In the GUI look for the SIMULINK_LIBRARY FLAG and press enter to turn it ON/OFF. Then as usual type c to configure until no stars (*) show up and finally g to generate. Finally to compile type
```bash
    make
```
2. **hex # 8864**


###### Before Using

###### To Do List
- [ ] ...
- [ ] ...
- [ ] ...


When starting MATLAB remember to add to its path the mex files location and also the models location if you please by doing:
addpath('${CODYCO_DIR}\lib\Release\')
addpath('${CODYCO_ROOT}\simulink\controllers\')

Where CODYCO_DIR points to the location of the CODYCO build directory and CODYCO_ROOT to CODYCO src directory.
