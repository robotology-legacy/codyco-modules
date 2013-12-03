CoDyCo
======
[![Build Status](https://travis-ci.org/robotology/codyco.png?branch=master)](https://travis-ci.org/robotology/codyco)

Whole-body Compliant Dynamical Contacts in Cognitive Humanoids


The CoDyCo project is a four-years long project that started in March
2013. At the end of each year a scenario will be used to validate on the
iCub  the theoretical advances of the project.

More info at http://codyco.eu/

Code documentation automatically generated: http://wiki.icub.org/codyco/dox/html/index.html

Installation
------------

Before installing CoDyCo software it is necessary to install some dependencies:

###Yarp/iCub software 
You can follow the instructions on: http://wiki.icub.org/wiki/ICub_Software_Installation .
You should install at least Yarp version 2.3.22 and iCub version 1.1.13 .

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

###kdl
It is possible to install orocos_kdl easily from the orocos github repository:
```bash
git clone https://github.com/orocos/orocos_kinematics_dynamics
cd orocos_kinematics_dynamics/orocos_kdl
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
    
