/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <Eigen/Core>                               // import most common Eigen types

#include <wbi/wbi.h>
#include <paramHelp/paramHelperServer.h>
#include <paramHelp/paramHelperClient.h>
#include <locomotionPlanner/locomotionPlannerConstants.h>


using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;

namespace locomotionPlanner
{

enum PlanningStatus {PLANNING_ON, PLANNING_OFF};
/** Locomotion planner thread: this thread sends the desired position trajectory of the COM,
    swinging foot and joint posture.**/

class LocomotionPlannerThread: public Thread, public ParamValueObserver, public CommandObserver
{
    string              name;
    string              robotName;
    ParamHelperServer   *paramHelper;
    ParamHelperClient   *locoCtrl;
    wholeBodyInterface  *robot;
    bool                mustStop;
//    string              fileName;
    string              filename;
    string              codyco_root;
    PlanningStatus      status;         //When on the planner is reading from file and streaming data.

    // Module parameters
    Vector2d            kp_com;
    Vector6d            kp_foot;
    VectorXd            kp_posture;
    double              tt_com, tt_foot, tt_posture;    // trajectory times of min jerk trajectory generators
    VectorXi            activeJoints;   // vector of bool indicating which joints are used (1 used, 0 blocked)
    int                 supportPhase;
    double              pinvDamp;

    // Output streaming parameters
    Vector2d            xd_com;
    Vector7d            xd_foot;
    VectorNd            qd;

    // Output streaming parameters
    
    enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO) ;

    void sendMonitorData() ;

public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LocomotionPlannerThread(string _name, string _robotName, ParamHelperServer *_ph, ParamHelperClient   *_lc, wholeBodyInterface *_wbi, string _filename) ;
	
    bool threadInit();	
    void run();
    void startSending();
    void stopSending();

    void threadRelease();

    void stop(){ mustStop=true; Thread::stop(); }

    /** Callback function for parameter updates. */
    void parameterUpdated(const ParamProxyInterface *pd);
    /** Callback function for rpc commands. */
    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);
    /** Callback function for reading text file parameters */
    string readParamsFile(ifstream &fp);

    string get_env_var( string const & key );
};

} // end namespace

#endif
