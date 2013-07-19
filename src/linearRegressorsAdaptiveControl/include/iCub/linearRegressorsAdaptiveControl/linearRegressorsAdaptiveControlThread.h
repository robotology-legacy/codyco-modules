
/*
 * Copyright (C) 2013 Italian Institute of Technology CoDyCo Project
 * Authors: Daniele Pucci and Silvio Traversaro
 * email:   daniele.pucci@iit.it and silvio.traversaro@iit.it
 * website: www.codyco.eu
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

#include <iostream>
#include <string>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/Network.h>	//temporary

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wbi;

namespace iCub{

namespace linearRegressorsAdaptiveControl{

class linearRegressorsAdaptiveControlThread : public RateThread
{
public:
	/* class methods */

	linearRegressorsAdaptiveControlThread(ResourceFinder* rf, string robotName, wholeBodyInterface* robot, Intint period);
	bool threadInit();
	void threadRelease();
	void run();

private:

	/* class constants */
	const int PERIOD;

	/* class variables */

	wholeBodyInterface* robot;

	ResourceFinder* rf;

	// input parameters
	string robotName;

	/* ports */
	BufferedPort<Vector> qfPort;

    /* Mathematical variables */
    Vector q, dq, ddq;

};

} //namespace iCub

} //namespace skinCalib
