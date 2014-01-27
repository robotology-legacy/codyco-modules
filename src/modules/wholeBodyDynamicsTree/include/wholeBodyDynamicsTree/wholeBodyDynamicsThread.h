/* 
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef WHOLE_BODY_DYNAMICS_THREAD
#define WHOLE_BODY_DYNAMICS_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace Eigen;



/** 
 * 
  */
class wholeBodyDynamicsThread: public RateThread
{
    string              name;
    string              robotName;
    wbiIcub::icubWholeBodyStates    *robot;
    
    //output ports
    ///< \todo TODO add a proper structure for output ports, by dividing them for body parts or sensors
    BufferedPort<Bottle> *port_RATorques;
    BufferedPort<Bottle> *port_RLTorques;
    BufferedPort<Bottle> *port_RWTorques;
    BufferedPort<Bottle> *port_LATorques;
    BufferedPort<Bottle> *port_LLTorques;
    BufferedPort<Bottle> *port_LWTorques;
    BufferedPort<Bottle> *port_TOTorques;
    BufferedPort<Bottle> *port_HDTorques;
    
    BufferedPort<Vector> *port_external_wrench_RA;
    BufferedPort<Vector> *port_external_wrench_LA;
    BufferedPort<Vector> *port_external_wrench_RL;
    BufferedPort<Vector> *port_external_wrench_LL;
    BufferedPort<Vector> *port_external_wrench_RF;
    BufferedPort<Vector> *port_external_wrench_LF;
    
    BufferedPort<Vector> *port_external_cartesian_wrench_RA;
    BufferedPort<Vector> *port_external_cartesian_wrench_LA;
    BufferedPort<Vector> *port_external_cartesian_wrench_RL;
    BufferedPort<Vector> *port_external_cartesian_wrench_LL;
    BufferedPort<Vector> *port_external_cartesian_wrench_RF;
    BufferedPort<Vector> *port_external_cartesian_wrench_LF;
    
    BufferedPort<Vector> *port_sensor_wrench_RL;
    BufferedPort<Vector> *port_sensor_wrench_LL;
    BufferedPort<Vector> *port_model_wrench_RL;
    BufferedPort<Vector> *port_model_wrench_LL;
    
    BufferedPort<Vector> *port_external_wrench_TO;
    BufferedPort<Vector> *port_com_all;
    BufferedPort<Vector> *port_com_all_foot;
    BufferedPort<Vector> *port_monitor;
    
    BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts;
    BufferedPort<Vector> *port_dumpvel;
    BufferedPort<Vector> *port_COM_vel;
    BufferedPort<Matrix> *port_COM_Jacobian;
    
    BufferedPort<Vector> *port_all_velocities;
    BufferedPort<Vector> *port_all_positions;
    
    BufferedPort<Matrix> *port_root_position_mat;
    BufferedPort<Vector> *port_root_position_vec;

    // ports outputing the external dynamics seen at the F/T sensor
    BufferedPort<Vector> *port_external_ft_arm_left;
    BufferedPort<Vector> *port_external_ft_arm_right;
    BufferedPort<Vector> *port_external_ft_leg_left;
    BufferedPort<Vector> *port_external_ft_leg_right;
    yarp::os::Stamp timestamp;


public:
    
    wholeBodyDynamicsThread(string _name, string _robotName, int _period, wbiIcub::icubWholeBodyStatesLocal *_wbi);
    
    bool threadInit();
    void calibrate();
    void run();
    void threadRelease();

};


#endif
