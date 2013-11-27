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

#ifndef LOCOMOTION_PLANNER_THREAD
#define LOCOMOTION_PLANNER_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include <yarp/os/Property.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <Eigen/Core>                               // import most common Eigen types

#include <wbi/wbi.h>
#include <paramHelp/paramHelperServer.h>
#include <paramHelp/paramHelperClient.h>
#include <jointTorqueControl/jointTorqueControlConstants.h>


using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;

namespace jointTorqueControl
{

enum ControlStatus {CONTROL_ON, CONTROL_OFF};
/** Locomotion planner thread: this thread sends the desired position trajectory of the COM,
    swinging foot and joint posture.**/

class jointTorqueControlThread: public RateThread, public ParamObserver, public CommandObserver
{
    string              name;
    string              robotName;
    ParamHelperServer   *paramHelper;
    ParamHelperClient   *torqueCtrl;
    wholeBodyInterface  *robot;
    bool                mustStop;
//    string              fileName;
    string              filename;
    string              codyco_root;
    ControlStatus       status;         //When on the planner is reading from file and streaming data.

    // Thread parameters
    
	VectorNd 	aj;				// Vector of nDOF integers representing the joints to control  (1: active, 0: inactive) 
    VectorNd	dq;				// Joint velocities 
    VectorNd 	kt;				// Vector of nDOF floats ( see Eq. (1) )"), 
    VectorNd 	kvp;			// Vector of nDOF floats ( see Eq. (2) )"), 
    VectorNd	kvn;			// Vector of nDOF floats ( see Eq. (2) )"), 
    VectorNd	kcp;			// Vector of nDOF floats ( see Eq. (2) )"), 
    VectorNd	kcn;			// Vector of nDOF floats ( see Eq. (2) )"), 
    VectorNd	ki;				// Vector of nDOF floats representing the position gains ( see Eq. (x) )"), 
    VectorNd	kp;				// Vector of nDOF floats representing the integral gains ( see Eq. (x) )"), 
    VectorNd	ks;				// Vector of nDOF floats representing the steepnes       ( see Eq. (x) )"), 
    VectorNd	etau;			// Errors between actual and desired torques 
    VectorNd	tau;			// Vector of nDOF floats representing the steepnes       ( see Eq. (x) )"), 
    VectorNd	tauD;			// Vector of nDOF floats representing the steepnes       ( see Eq. (x) )"), 
    VectorNd	tauM;			// Measured torques, 
    VectorNd	integralState;	// Vector of nDOF floats representing the steepnes       ( see Eq. (x) )"), 
    VectorNd	Vm;				// Vector of nDOF positive floats representing the tensions' bounds (|Vm| < Vmax"), 
    VectorNd	Vmax;			// Vector of nDOF positive floats representing the tensions' bounds (|Vm| < Vmax"), 
    double		DT;				// Time interval
           
    // Input streaming parameters

    // Output streaming parameters
    
    enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO) ;

    void sendMonitorData() ;
    
    void fromListToVector(Bottle * , VectorNd &); 

public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    jointTorqueControlThread(int period, string _name, string _robotName, ParamHelperServer *_ph, ParamHelperClient   *_lc, wholeBodyInterface *_wbi, string _filename) ;
	
    bool threadInit();	
    void run();
    void startSending();
    void stopSending();

    void threadRelease();

//     void stop(){ mustStop=true; Thread::stop(); }

    /** Callback function for parameter updates. */
    void parameterUpdated(const ParamDescription &pd);
    /** Callback function for rpc commands. */
    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);
    /** Callback function for reading text file parameters */
    string readParamsFile(ifstream &fp);

    string get_env_var( string const & key );
};

} // end namespace

#endif
