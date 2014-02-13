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

The minimum version of Eigen required by codyco is 3.0.5

**The module motorFrictionIdentification requires at least Eigen version 3.1.0**


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

    
Fix for OS X Mavericks (10.9) compilation and build errors
------------
Starting from Mac OSX 10.9 (Mavericks) the default C++ standard library changed to `libc++` (the LLVM library).
This switch has caused (and still causes) a lot of issues because the new library is more stringent on non-standard C++.
For example Orocos KDL uses non-standard C++ in the `TreeNode` class thus it is not possible to build it with `libc++` (see [related Github issue](https://github.com/orocos/orocos_kinematics_dynamics/pull/4)).

You can (and must) build KDL by explicitly specify the old GNU C++ library.
You can do this by toggling the advanced CCMake configuration (`t` key) and by adding in both `CMAKE_CXX_FLAGS` and `CMAKE_EXE_LINKER_FLAGS` the following string: `-stdlib=libstdc++`.

One issue still remains. It is not clear if two applications can link (shared linking, not static linking) at the same time the GNU and LLVM C++ library. For sure Homebrew does not allow it.
You can check which shared libraries an application (or another shared library) links by issuing the following commands:
```bash
otool -L library_name
````
E.g.
```bash
$ otool -L libYARP_OS.dylib 
local/lib/libYARP_OS.dylib:
    libYARP_OS.1.dylib (compatibility version 1.0.0, current version 2.3.60)
    /usr/local/lib/libACE.dylib (compatibility version 0.0.0, current version 0.0.0)
    /usr/lib/libstdc++.6.dylib (compatibility version 7.0.0, current version 60.0.0)
    /usr/lib/libSystem.B.dylib (compatibility version 1.0.0, current version 1197.1.1)
````