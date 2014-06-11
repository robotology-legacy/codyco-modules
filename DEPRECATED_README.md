CoDyCo
======
[![Build Status](https://travis-ci.org/robotology/codyco.png?branch=master)](https://travis-ci.org/robotology/codyco)

Whole-body Compliant Dynamical Contacts in Cognitive Humanoids


The CoDyCo project is a four-years long project that started in March
2013. At the end of each year a scenario will be used to validate on the
iCub  the theoretical advances of the project.

More info at http://codyco.eu/

Code documentation automatically generated: http://wiki.icub.org/codyco/html/index.html

Installation
------------

Before installing CoDyCo software it is necessary to install some dependencies:

###Yarp/iCub software 
You can follow the instructions on: http://wiki.icub.org/wiki/ICub_Software_Installation .
You should install at least Yarp version 2.3.22 and iCub version 1.1.13 . 

**The repository is being migrated to yarp version 2.4 (current yarp master branch).**

*Modules _comStepper_ and _wholeBodyDynamicsTree_ are already migrated to version 2.4, so it is necessary to install yarp from the repository to build them.* 

###Eigen
You can follow the instructions on: http://eigen.tuxfamily.org .
For example, on OS X you can simply use brew to install Eigen:
```bash
brew install eigen
```

while on Debian/Ubuntu you can use apt-get :
```bash
sudo apt-get install libeigen3-dev
```

The minimum version of Eigen required by codyco is 3.2.0 . 


###kdl
It is possible to install orocos_kdl easily from the orocos github repository. The master branch of the repository is required only for proper windows installation, while for Linux also older versions, as the one provided by ROS releases are ok):
```bash
git clone https://github.com/orocos/orocos_kinematics_dynamics
cd orocos_kinematics_dynamics
cd orocos_kdl
mkdir build 
cd build
ccmake ../
make
sudo make install
```

For any further information you can check http://www.orocos.org/kdl .

    
###kdl_codyco
It is possible to install kdl_codyco easily from GitHub.
```bash
git clone https://github.com/traversaro/kdl_codyco.git
cd kdl_codyco
mkdir build
cd build
ccmake ../
make
sudo make install
```

###kdl_format_io
To enable URDF support in iDynTree and wholeBodyInterface, you need to install the kdl_format_io library and all its dependecies, following the instructions in [kdl_format_io README](https://github.com/traversaro/kdl_format_io).

After the instalation, you should compile codyco enabling the CODYCO_USES_URDFDOM CMake flag. If a dependency is not found the CODYCO_USES_URDFDOM option is automatically disabled, so pay attention to properly install all the dependencies. 

##CoDyCo
Finally, after installing all dependencies, it is possible to install codyco (this repository itself) from GitHub.
```bash
git clone https://github.com/robotology/codyco.git
cd codyco
mkdir build
cd build
ccmake ../
make
sudo make install
```
