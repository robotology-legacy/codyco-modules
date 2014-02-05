Simulink Library for Whole Body Control 
----------------------------------------------------------

This document describes basic instructions on how to use this library, *tips and tricks* to do so and a walkthrough to get you started using it. Simulink blocks consist of S-functions (http://goo.gl/1GuHVd) which allow C/C++ user specific code compiled as Matlab Executable (MEX) files, thus extending the capabilities of the Simulink environment. In other words, MEX files have been created linking YARP, iCub and **iDynTree** (a more efficient and generic YARP-based robot dynamics library than its predecessor iDyn - http://goo.gl/BnGzKr) and wrapping the **Whole Body Interface** described in http://goo.gl/dBWO3k. Soft-Real Time is ensured by slowing down the simulation to respect a user specified rate. The following video shows a quick introduction to the way it works. For further information read the sections below.


<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=h2T4BtuDiJg
" target="_blank"><img src="http://img.youtube.com/vi/h2T4BtuDiJg/0.jpg" 
alt="Overview of the Simulink library for Whole Body Control" width="480" height="360" border="10" /></a>
</p>



###### Main Goal ######
> The library should allow non-programming experts or those researchers just getting acquainted with Whole Body Control to more easily deploy controllers either on simulation or the real platform, as well as to analyze their performance and take advantage of the innumerable MATLAB and Simulink toolboxes. We like to call it "rapid controller prototyping" after which a proper YARP module should be made for hard real time performance and final deployment.

###### Requirements
* Matlab V. 7.1+ (Tested with Matlab 7.14 R2012a, 7.12 R2011a)
* Yarp
* CoDyCo 
* iCub
* Gazebo Simulator + yarp plugins (if you wish to use Gazebo instead of iCub_SIM)

###### Installation
Before going ahead with the compilation of the library, make sure that you have MATLAB and Simulink properly installed and running. Then, check that the MEX compiler for MATLAB is setup and working. For this you can try compiling some of MATLAB C code examples as described in [http://www.mathworks.com/help/matlab/ref/mex.html#btz1tb5-12].

- **Compiling the Simulink Library.** When configuring the CMakeLists for CoDyCo make sure to enable the SIMULINK_LIBRARY flag by doing
```bash
    cd $CODYCO_DIR
    ccmake ../
```
In the UI look for *SIMULINK_LIBRARY* and press enter to turn it ON/OFF. Then as usual type c to configure until no stars (*) show up and finally g to generate. Finally to compile type `make`.

- **Soft Real Time.** For the time being, this block has been taken from the Matlab File Exchange [http://goo.gl/8LMWGD] and it has to be compiled from within MATLAB by changing its current directory to `${CODYCO_ROOT}/simulink/controllers/RealTimeSlower` and typing `mex sfun_time.c`. This will create a mex file according to your operating system and architecture, e.g. for a 32bits Linux-based OS you will get sfun_time.mexglx. This mex file will be used by the example models included in `${CODYCO_ROOT}/simulink/controllers/` to slow down the simulation for a user-specified rate.

- **Preparing the MATLAB Environment.** When starting MATLAB remember for comodity 

###### Before Using

###### Using the Simulink Library

###### Tested OS
Linux, Windows, MAC OS

###### To Do List
- [ ] Documentation.
- [ ] Compile the Soft Real Time mex as another module of the library. Possibly make our own.
- [ ] ...


When starting MATLAB remember to add to its path the mex files location and also the models location if you please by doing:
addpath('${CODYCO_DIR}\lib\Release\')
addpath('${CODYCO_ROOT}\simulink\controllers\')

Where CODYCO_DIR points to the location of the CODYCO build directory and CODYCO_ROOT to CODYCO src directory.
