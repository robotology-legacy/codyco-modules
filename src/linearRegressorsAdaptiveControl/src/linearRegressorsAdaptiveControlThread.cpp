
/*
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete
 * email:   andrea.delprete@iit.it
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
#include <yarp/os/Time.h>
#include "math.h"
#include "iCub/linearRegressorsAdaptiveControl/linearRegressorsAdaptiveControlThread.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::linearRegressorsAdaptiveControl;


linearRegressorsAdaptiveControlThread::linearRegressorsAdaptiveControlThread(ResourceFinder* rf, string robotName, bool rightArm, int period)
									   : rf(rf), rightArm(rightArm), RateThread(period),
									   PERIOD(period), robotName(robotName)
{}

bool SkinCalibThread::threadInit()
{
    fprintf(stderr, "THREAD INIT\n\n");

   /* initialize variables and create data-structures if needed */


	// open the output port
	string compensatedTactileDataPortName;
	if(rightArm){
		compensatedTactileDataPortName		= "/"+ robotName+ "/skin/right_hand_comp";
	}else{
		compensatedTactileDataPortName		= "/"+ robotName+ "/skin/left_hand_comp";
	}
	if (!skinPort.open(compensatedTactileDataPortName.c_str())) {
		cout << "Unable to open port " << compensatedTactileDataPortName << endl;
		return false;  // unable to open
	}

	// open the analog sensor interface for the skin
	Property options;
	options.put("robot",  robotName.c_str());
	if(rightArm){
		options.put("part",   "righthand");         //skin part that you want to control
		options.put("local",  "/skinComp/right");
		options.put("remote",  ("/"+robotName+"/skin/right_hand").c_str());
	}else{
		options.put("part",   "lefthand");          //skin part that you want to control
		options.put("local",  "/skinComp/left");
		options.put("remote",  ("/"+robotName+"/skin/left_hand").c_str());
	}
	options.put("device", "analogsensorclient");	//important! It’s different from remote_controlboard that you use to control motors!
	// create a new device driver
	tactileSensorDevice = new PolyDriver(options);
	if (!tactileSensorDevice->isValid()){
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		return false;
	}
	// open the sensor interface
	bool ok = tactileSensorDevice->view(tactileSensor);
	if (!ok) {
		printf("Problems acquiring interfaces\n");
		return false;
	}

	SKIN_DIM = tactileSensor->getChannels();
	if(SKIN_DIM==0){
		fprintf(stderr, "Error while reading the number of channels of the tactile sensor device\n");
		SKIN_DIM = 192;
	}

	return true;
}

void SkinCalibThread::run(){

}


void SkinCalibThread::threadRelease()
{
	/* close device driver and port */

}
