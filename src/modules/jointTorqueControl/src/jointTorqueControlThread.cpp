/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Daniele Pucci
 * email:  daniele.pucci@iit.it
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

#include <jointTorqueControl/jointTorqueControlThread.h>
#include <jointTorqueControl/jointTorqueControlConstants.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/Property.h>


using namespace jointTorqueControl;


jointTorqueControlThread::jointTorqueControlThread(int period, string _name, string _robotName, ParamHelperServer *_ph, ParamHelperClient *_lc, wholeBodyInterface *_wbi, string _filename)
    : RateThread(period), name(_name), robotName(_robotName), paramHelper(_ph), torqueCtrl(_lc), robot(_wbi), filename(_filename)
{
    cout <<"Instantiating the thread: "<< filename << endl;
    mustStop = false;
    codyco_root = "CODYCO_ROOT";
    status = CONTROL_OFF;
    
    yarp::os::Property configFile;
    
    configFile.fromConfigFile("default.ini");
    
//     Bottle *pointerToFile = configFile.find("ActiveJoints").asList(); 
//     fromListToVector(pointerToFile, aj);
//     
//     pointerToFile = configFile.find("kt").asList(); 
//     fromListToVector(pointerToFile, kt);
//     
//     pointerToFile = configFile.find("kvp").asList(); 
//     fromListToVector(pointerToFile, kvp);
//     
//     pointerToFile = configFile.find("kvn").asList(); 
//     fromListToVector(pointerToFile, kvn);
//     
//     pointerToFile = configFile.find("kcp").asList(); 
//     fromListToVector(pointerToFile, kcp);
//     
//     pointerToFile = configFile.find("kcn").asList(); 
//     fromListToVector(pointerToFile, kcn);
//     
//     pointerToFile = configFile.find("ki").asList(); 
//     fromListToVector(pointerToFile, ki);
//     
//     pointerToFile = configFile.find("kp").asList(); 
//     fromListToVector(pointerToFile, kp);
//     
//     pointerToFile = configFile.find("ks").asList(); 
//     fromListToVector(pointerToFile, ks);
//     
//     pointerToFile = configFile.find("Vmax").asList(); 
//     fromListToVector(pointerToFile, Vmax);
	
	tau 			= VectorNd::Constant(0.0); 
	etau 			= VectorNd::Constant(0.0); 
	tauD 			= VectorNd::Constant(0.0); 
	tauM 			= VectorNd::Constant(0.0); 
	integralState 	= VectorNd::Constant(0.0); 
	Vm 				= VectorNd::Constant(0.0); 
	DT				= 0;
       
}

//*************************************************************************************************************************
bool jointTorqueControlThread::threadInit()
{
    const string partialLocation = "jointTorqueControl/conf/data/timestamp10/randomStandingPoses_iCubGenova01_";
    filename = string(get_env_var(codyco_root) + partialLocation + filename + ".txt");
    cout<<filename<<endl;

    // link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_AJ,		aj.data()));    // constant size
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KT,		kt.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVP,	kvp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVN,	kvn.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCP,	kcp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCN,	kcn.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KI,		ki.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP,		kp.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KS,		ks.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VMAX,	Vmax.data()));
	
    // link controller input streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAUD,	tauD.data()));
	
    // link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VM,		Vm.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TAU,	tau.data()));

    // Register callbacks for some module parameters

    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));
    
    return true;
}

//*************************************************************************************************************************
void jointTorqueControlThread::run(){
	if(status == CONTROL_ON){
		// Read joint velocities
		// dq = 
		
		// Read torques
		// tauM =
		// Receive desired torques
				
		for (int i=0; i < N_DOF; i++){
			if (aj(i) == 1) {
				// etau = tauM(i) - tauD(i);
				// integralState(i) = integralState(i) - DT*etau
				// tau(i) = tauD(i) - kp(i)*etau -ki(i)*integralState(i);
				// Vm(i) = kt(i)*tao(i) + [kvp(i)*s(dq) + kvn(i)*s(-dq)]*dq + [kcp(i)*s(dq) + kcn(i)*s(-dq)]*tanh(ks(i)*dq),
			}
		}
    }
}

//*************************************************************************************************************************
void jointTorqueControlThread::startSending()
{
    status = CONTROL_ON;       //sets thread status to ON
}
//*************************************************************************************************************************
void jointTorqueControlThread::stopSending()
{
    status = CONTROL_OFF;
}

//*************************************************************************************************************************
void jointTorqueControlThread::threadRelease()
{

}

//*************************************************************************************************************************
void jointTorqueControlThread::parameterUpdated(const ParamDescription &pd)
{
    //switch(pd.id)
    //{
    //default:
    sendMsg("A callback is registered but not managed for the parameter "+pd.name, MSG_WARNING);
    //}
}

//*************************************************************************************************************************
void jointTorqueControlThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
        startSending();
        sendMsg("Starting the planner.", MSG_INFO); break;
    case COMMAND_ID_STOP:
        stopSending();
        sendMsg("Stopping the planner.", MSG_INFO); break;
    default:
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
string jointTorqueControlThread::readParamsFile(ifstream& fp)
{
    string lineStr;
    getline(fp,lineStr);
    return(lineStr);
}
//*************************************************************************************************************************
string jointTorqueControlThread::get_env_var( string const & key ) {
    char * val;
    val = getenv( key.c_str() );
    std::string retval = "";
    if (val != NULL) {
        retval = val;
    }
    return retval;
}
//*************************************************************************************************************************
void jointTorqueControlThread::sendMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[jointTorqueControlThread] %s\n", s.c_str());
}

void jointTorqueControlThread::fromListToVector(Bottle * pointerToList, VectorNd& vector) {
	    for (int i=0; i < pointerToList->size(); i++)
            {
                vector(i) = pointerToList->get(i).asDouble();
            }
    return;
}
