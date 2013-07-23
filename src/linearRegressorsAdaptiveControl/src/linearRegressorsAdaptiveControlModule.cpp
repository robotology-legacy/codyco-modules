
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

#include <sstream>			// string stream
#include "iCub/linearRegressorsAdaptiveControl/linearRegressorsAdaptiveControlModule.h"

using namespace iCub::linearRegressorsAdaptiveControl;

// module default values
const int linearRegressorsAdaptiveControlModule::PERIOD_DEFAULT = 50;
const int linearRegressorsAdaptiveControlModule::JOINT = 3+3+7+3;
const string linearRegressorsAdaptiveControlModule::MODULE_NAME_DEFAULT = "linearRegressorsAdaptiveControlModule";
const string linearRegressorsAdaptiveControlModule::ROBOT_NAME_DEFAULT = "icub";
const string linearRegressorsAdaptiveControlModule::RPC_PORT_DEFAULT = "/rpc";

// the order of the command(s) in this list MUST correspond to the order of the enum linearRegressorsAdaptiveControlModule::linearRegressorsAdaptiveControlModuleCommand
const string linearRegressorsAdaptiveControlModule::COMMAND_LIST[] = {"help", "quit","setGamma","setKappa","setLambda"};

// the order in COMMAND_DESC must correspond to the order in COMMAND_LIST
const string linearRegressorsAdaptiveControlModule::COMMAND_DESC[]  = {
	"get this list",
	"quit the module",
    "set the Gamma gain (matrix with equal values on the diagonal)",
    "set the Kamma gain (matrix with equal values on the diagonal)",
    "set the Lambda gain (matrix with equal values on the diagonal)"
    };

bool linearRegressorsAdaptiveControlModule::configure(yarp::os::ResourceFinder &rf)
{
	/* Process all parameters from both command-line and .ini file */

	/* get the module name which will form the stem of all module port names */
	moduleName			= rf.check("name", Value(MODULE_NAME_DEFAULT.c_str()), "module name (string)").asString();
	robotName			= rf.check("robot", Value(ROBOT_NAME_DEFAULT.c_str()), "name of the robot (string)").asString();
	/* before continuing, set the module name before getting any other parameters,
	* specifically the port names which are dependent on the module name*/
	setName(moduleName.c_str());

    
	/* get some other values from the configuration file */
	int period			= (int)rf.check("period", Value(PERIOD_DEFAULT),
	   "Calling thread period in ms (positive int)").asInt();
    int controlled_joint = (int)rf.check("joint", Value(PERIOD_DEFAULT),
	   "Calling thread period in ms (positive int)").asInt();

	/*
	* attach a port of the same name as the module (prefixed with a /) to the module
	* so that messages received from the port are redirected to the respond method
	*/
	handlerPortName = "/";
	handlerPortName += getName(rf.check("handlerPort", Value(RPC_PORT_DEFAULT.c_str())).asString());
	if (!handlerPort.open(handlerPortName.c_str())) {
		cout << getName() << ": Unable to open port " << handlerPortName << endl;
		return false;
	}
	attach(handlerPort);                  // attach to port

    //Create the dynamical model
    iCubTree_version_tag version;
    version.head_version = 2;
    version.legs_version = 2;
    
    iCubTree_serialization_tag serial_version =  SKINDYNLIB_SERIALIZATION;
    
    icub_dynamical_model = new iCubTree(version,serial_version);
    
    controlled_DOFs.clear();
    controlled_DOFs.resize(icub_dynamical_model->getNrOfDOFs(),false);
    
    controlled_DOFs[controlled_joint] = true;
    
    icub_interface = new icubWholeBodyInterface();

	/* create the thread and pass pointers to the module parameters */
	controlThread = new linearRegressorsAdaptiveControlThread(&rf, robotName, icub_interface, icub_dynamical_model, controlled_DOFs, period);
	/* now start the thread to do the work */
	controlThread->start(); // this calls threadInit() and it if returns true, it then calls run()

	return true ;      // let the RFModule know everything went well
					  // so that it will then run the module
}


bool linearRegressorsAdaptiveControlModule::interruptModule()
{
	handlerPort.interrupt();

	return true;
}


bool linearRegressorsAdaptiveControlModule::close()
{
	/* stop the thread */
	if(controlThread)
		controlThread->stop();

	handlerPort.close();

	return true;
}


bool linearRegressorsAdaptiveControlModule::respond(const Bottle& command, Bottle& reply)
{
	stringstream temp;
	string helpMessage =  string(getName().c_str()) + " commands are: ";
	reply.clear();

	linearRegressorsAdaptiveControlCommand com;
	if(!identifyCommand(command, com)){
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		return true;
	}

	switch( com ){
		case quit:
			reply.addString("quitting");
			return false;
        
        case setGamma:
            reply.addString("setGamma command received");
            return controlThread->setGain(gamma_gain,command.get(1).asDouble());
        
        case setKappa:
            reply.addString("setKappa command received");
            return controlThread->setGain(kappa_gain,command.get(1).asDouble());
        
        case setLambda:
            reply.addString("setLambda command received");
            return controlThread->setGain(lambda_gain,command.get(1).asDouble());

		case help:
			reply.addString("many");				// print every string added to the bottle on a new line
			reply.addString(helpMessage.c_str());
			for(unsigned int i=0; i< COMMANDS_COUNT; i++){
				reply.addString( ("- "+COMMAND_LIST[i]+": "+COMMAND_DESC[i]).c_str() );
			}
			return true;

		default:
			reply.addString("ERROR: This command is known but it is not managed in the code.");
			return true;
	}

	reply.addString( (COMMAND_LIST[com]+" command received.").c_str());

	return true;
}


/**
  * Identify the command in the bottle and return the correspondent enum value.
  */
bool linearRegressorsAdaptiveControlModule::identifyCommand(Bottle commandBot, linearRegressorsAdaptiveControlModule::linearRegressorsAdaptiveControlCommand &com){
	for(unsigned int i=0; i<COMMANDS_COUNT; i++){
		stringstream stream(COMMAND_LIST[i]);
		string word;
		int j=0;
		bool found = true;

		while(stream>>word){
			if (commandBot.get(j).asString() != word.c_str()){
				found=false;
				break;
			}
			j++;
		}
		if(found){
			com = (linearRegressorsAdaptiveControlCommand)i;
			return true;
		}
	}

	return false;
}

bool linearRegressorsAdaptiveControlModule::updateModule(){ return true;}
double linearRegressorsAdaptiveControlModule::getPeriod(){ return 0.1;}



