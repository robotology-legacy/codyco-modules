/*
 * Copyright (C)2013  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * Last Modified by: Jorhabib Eljaik
 * email:  marco.randazzo@iit.it, jorhabib.eljaik@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include "constants.h"
#include "scriptModule.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

int main(int argc, char *argv[])
{
    ResourceFinder rf = ResourceFinder::getResourceFinderSingleton();
    rf.setVerbose(true);
    rf.setDefaultConfigFile("walkPlayer.ini");
    rf.setDefaultContext("walkPlayer");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name               <moduleName>: set new module name" << endl;
        cout << "\t--robot              <robotname>:  robot name"          << endl;
        cout << "\t--filename          <filename>:   to specifiy to use two files (left and leg separate). _left.txt and _right.txt automatically appended"  << endl;
        cout << "\t--execute            activate the iPid->setReference() control"  << endl;
        cout << "\t--period             <period>: the period in ms of the internal thread (default 5)"  << endl;
        cout << "\t--speed              <factor>: speed factor (default 1.0 normal, 0.5 double speed, 2.0 half speed etc)"  << endl;
        cout << "\t--refSpeedMinJerk    [0] Reference speed value used by the minimun jerk controllers. " << endl;
        cout << "\t--minJerkLimit       [0] (int) Limit of the trajectory points after which position direct commands are sent " << endl;
        cout <<"\t--torqueBalancingSequence [torqueBalancing] Prefix of the sequences for torque balancing. Overwrites the execute flag value. This option has higher priority and should simply stream trajectories used by the torqueBalancing module." << endl;
//         cout << "\t--ankleImpedanceAt   <ankleImpedanceStartTime> time at which one of the ankles starts optimal impedance control mode. " <<endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        cout << "ERROR: yarp.checkNetwork() failed."  << endl;
        return -1;
    }

    scriptModule mod;

    return mod.runModule(rf);
}



