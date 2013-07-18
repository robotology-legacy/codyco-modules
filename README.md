CoDyCo
======


Whole-body Compliant Dynamical Contacts in Cognitive Humanoids


The CoDyCo project is a four-years long project and starts in March
2013. At the end of each year a scenario will be used to validate on the
iCub  the theoretical advances of the project.

More info at http://codyco.eu/

Code documentation automatically generated: http://wiki.icub.org/codyco/dox/html/index.html

Installation
------------

Before installing CoDyCo software it is necessary to install some dependencies:

###Yarp/iCub software 
You can follow the instructions on: http://wiki.icub.org/wiki/ICub_Software_Installation .

###Eigen
You can follow the instructions on: http://eigen.tuxfamily.org .

###KDL
It is possible to install KDL easily from the CoDyCo repository:
```bash
cd $CODYCO_ROOT/extern/orocos_kinematics_dynamics/orocos_kdl
mkdir build
cd build
ccmake ../
make
sudo make install
```
For any further information you can check http://www.orocos.org/kdl .
    
