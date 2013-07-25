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

###kdl
It is possible to install Orocos-KDL easily from the Orocos Git repository repository:
```bash
git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics/orocos_kdl
mkdir build 
cd build
ccmake ../
make
sudo make install
```
Please note that for compiling the plain version of KDL, you have to use the CMakeLists.txt in orocos_kinematics_dynamics/orocos_kdl,
while the CMakeLists.txt in orocos_kinematics_dynamics contains ROS-specific commands.

For any further information you can check http://www.orocos.org/kdl .
    
###kdl_codyco
It is possible to install kdl_codyco easily from GitHub.
```bash
git clone https://github.com/pegua/kdl_codyco.git
cd kdl_codyco
cd build
ccmake ../
make
sudo make install
```
    
