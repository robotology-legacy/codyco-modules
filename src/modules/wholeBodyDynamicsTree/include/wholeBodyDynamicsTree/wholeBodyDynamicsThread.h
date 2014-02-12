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
#include <yarp/os/Stamp.h>
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
    wbiIcub::icubWholeBodyStatesLocal *estimator;
    
    int                 printCountdown;         // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    double              PRINT_PERIOD;           

    
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
    
    /*
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
    */
    
    BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts;
    
    /*
    BufferedPort<Vector> *port_all_accelerations;
    BufferedPort<Vector> *port_all_velocities;
    BufferedPort<Vector> *port_all_positions;
    */
    
    // ports outputing the external dynamics seen at the F/T sensor
    BufferedPort<Vector> *port_external_ft_arm_left;
    BufferedPort<Vector> *port_external_ft_arm_right;
    BufferedPort<Vector> *port_external_ft_leg_left;
    BufferedPort<Vector> *port_external_ft_leg_right;
    yarp::os::Stamp timestamp;

    template <class T> void broadcastData(T& _values, BufferedPort<T> *_port);
    void closePort(Contactable *_port);
    void writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port);
    void publishTorques();
    void publishContacts();

    //Buffer vectors
    yarp::sig::Vector all_torques;
    yarp::sig::Vector TOTorques;
    yarp::sig::Vector HDTorques;
    yarp::sig::Vector LATorques;
    yarp::sig::Vector RATorques;
    yarp::sig::Vector LLTorques;
    yarp::sig::Vector RLTorques;

    iCub::skinDynLib::skinContactList external_forces_list;


public:
    
    wholeBodyDynamicsThread(string _name, string _robotName, int _period, wbiIcub::icubWholeBodyStatesLocal *_wbi);
    
    bool threadInit();
    void calibrate();
    void run();
    void threadRelease();

};


#endif
