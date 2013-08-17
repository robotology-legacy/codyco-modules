
#ifndef __LOCOMOTION_MODULE_H__
#define __LOCOMOTION_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include <paramHelp\paramHelp.h>
#include <wbiy\wbiy.h>
#include <locomotion\locomotionThread.h>
 
using namespace std;
using namespace yarp::os; 
using namespace paramHelp;
using namespace wbi;

namespace locomotion
{

class LocomotionModule: public RFModule
{
    /* module parameters */
	string  moduleName;
	string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

	Port                rpcPort;		// a port to handle rpc messages
	LocomotionThread*   ctrlThread;     // locomotion control thread
    ParamHelper*        paramHelper;    // helper class for rpc set/get commands
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    LocomotionModule();

	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports 
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod(){ return period*1e-3;  }
	bool updateModule();

};

}

#endif
//empty line to make gcc happy

