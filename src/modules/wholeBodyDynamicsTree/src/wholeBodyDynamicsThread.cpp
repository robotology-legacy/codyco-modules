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

#include "wholeBodyDynamicsTree/wholeBodyDynamicsThread.h"
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/math/SVD.h>

#include <string.h>


using namespace yarp::math;
using namespace yarp::sig;
using namespace wbiIcub;
using namespace std;

//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name, string _robotName, int _period, icubWholeBodyStatesLocal *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), robot(_wbi)
{
    bool autoconnect = false;
    
    //Copied from old wholeBodyDynamics 
    std::string robot_name = robotName;
    std::string local_name = name;
    port_RATorques = new BufferedPort<Bottle>;
    port_LATorques = new BufferedPort<Bottle>;
    port_RLTorques = new BufferedPort<Bottle>;
    port_LLTorques = new BufferedPort<Bottle>;
    port_RWTorques = new BufferedPort<Bottle>;
    port_LWTorques = new BufferedPort<Bottle>;
    port_TOTorques = new BufferedPort<Bottle>;
    port_HDTorques = new BufferedPort<Bottle>;
    port_external_wrench_RA = new BufferedPort<Vector>;
    port_external_wrench_LA = new BufferedPort<Vector>;
    port_external_wrench_RL = new BufferedPort<Vector>;
    port_external_wrench_LL = new BufferedPort<Vector>;
    port_external_wrench_RF = new BufferedPort<Vector>;
    port_external_wrench_LF = new BufferedPort<Vector>;
    
    port_external_wrench_TO = new BufferedPort<Vector>;
    port_external_cartesian_wrench_RA = new BufferedPort<Vector>;
    port_external_cartesian_wrench_LA = new BufferedPort<Vector>;
    port_external_cartesian_wrench_RL = new BufferedPort<Vector>;
    port_external_cartesian_wrench_LL = new BufferedPort<Vector>;
        port_external_cartesian_wrench_RF = new BufferedPort<Vector>;
        port_external_cartesian_wrench_LF = new BufferedPort<Vector>;
    port_com_all      = new BufferedPort<Vector>;
    port_com_all_foot = new BufferedPort<Vector>;
    port_monitor = new BufferedPort<Vector>;
    port_contacts = new BufferedPort<skinContactList>;
    port_dumpvel = new BufferedPort<Vector>;
    port_external_ft_arm_left = new BufferedPort<Vector>;
    port_external_ft_arm_right = new BufferedPort<Vector>;
    port_external_ft_leg_left = new BufferedPort<Vector>;
    port_external_ft_leg_right = new BufferedPort<Vector>;

    port_all_velocities = new BufferedPort<Vector>;
    port_all_positions = new BufferedPort<Vector>;


    //Opening ports
    port_LATorques->open(string("/"+local_name+"/left_arm/Torques:o").c_str());
    port_RLTorques->open(string("/"+local_name+"/right_leg/Torques:o").c_str());
    port_LLTorques->open(string("/"+local_name+"/left_leg/Torques:o").c_str());
    port_RWTorques->open(string("/"+local_name+"/right_wrist/Torques:o").c_str());
    port_LWTorques->open(string("/"+local_name+"/left_wrist/Torques:o").c_str());
    port_TOTorques->open(string("/"+local_name+"/torso/Torques:o").c_str());
    port_HDTorques->open(string("/"+local_name+"/head/Torques:o").c_str());
    
    port_external_wrench_RA->open(string("/"+local_name+"/right_arm/endEffectorWrench:o").c_str()); 
    port_external_wrench_LA->open(string("/"+local_name+"/left_arm/endEffectorWrench:o").c_str()); 
    port_external_wrench_RL->open(string("/"+local_name+"/right_leg/endEffectorWrench:o").c_str()); 
    port_external_wrench_LL->open(string("/"+local_name+"/left_leg/endEffectorWrench:o").c_str()); 
    port_external_wrench_RF->open(string("/"+local_name+"/right_foot/endEffectorWrench:o").c_str()); 
    port_external_wrench_LF->open(string("/"+local_name+"/left_foot/endEffectorWrench:o").c_str()); 

    port_external_cartesian_wrench_RA->open(string("/"+local_name+"/right_arm/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LA->open(string("/"+local_name+"/left_arm/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_RL->open(string("/"+local_name+"/right_leg/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LL->open(string("/"+local_name+"/left_leg/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_RF->open(string("/"+local_name+"/right_foot/cartesianEndEffectorWrench:o").c_str()); 
    port_external_cartesian_wrench_LF->open(string("/"+local_name+"/left_foot/cartesianEndEffectorWrench:o").c_str()); 
    port_external_wrench_TO->open(string("/"+local_name+"/torso/Wrench:o").c_str());
  
    port_monitor->open(string("/"+local_name+"/monitor:o").c_str());
    port_contacts->open(string("/"+local_name+"/contacts:o").c_str());
    port_dumpvel->open(string("/"+local_name+"/va:o").c_str());
    port_external_ft_arm_left->open(string("/"+local_name+"/left_arm/ext_ft_sens:o").c_str());
    port_external_ft_arm_right->open(string("/"+local_name+"/right_arm/ext_ft_sens:o").c_str());
    port_external_ft_leg_left->open(string("/"+local_name+"/left_leg/ext_ft_sens:o").c_str());
    port_external_ft_leg_right->open(string("/"+local_name+"/right_leg/ext_ft_sens:o").c_str());

    port_all_velocities->open(string("/"+local_name+"/all_velocities:o").c_str());
    port_all_positions->open(string("/"+local_name+"/all_positions:o").c_str());
    
    if (autoconnect)
    {
        //from iCub to wholeBodyDynamics
        Network::connect(string("/"+local_name+"/filtered/inertial:o").c_str(),string("/"+local_name+"/inertial:i").c_str(),"tcp",false);                       
        Network::connect(string("/"+robot_name+"/inertial").c_str(),           string("/"+local_name+"/unfiltered/inertial:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_arm/analog:o").c_str(),  string("/"+local_name+"/left_arm/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_arm/analog:o").c_str(), string("/"+local_name+"/right_arm/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_leg/analog:o").c_str(),  string("/"+local_name+"/left_leg/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_leg/analog:o").c_str(), string("/"+local_name+"/right_leg/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/left_foot/analog:o").c_str(),  string("/"+local_name+"/left_foot/FT:i").c_str(),"tcp",false);
        Network::connect(string("/"+robot_name+"/right_foot/analog:o").c_str(), string("/"+local_name+"/right_foot/FT:i").c_str(),"tcp",false);  
        //from wholeBodyDynamics to iCub (mandatory)
        Network::connect(string("/"+local_name+"/left_arm/Torques:o").c_str(), string("/"+robot_name+"/joint_vsens/left_arm:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/right_arm/Torques:o").c_str(),string("/"+robot_name+"/joint_vsens/right_arm:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/left_leg/Torques:o").c_str(), string("/"+robot_name+"/joint_vsens/left_leg:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/right_leg/Torques:o").c_str(),string("/"+robot_name+"/joint_vsens/right_leg:i").c_str(),"tcp",false);
        Network::connect(string("/"+local_name+"/torso/Torques:o").c_str(),    string("/"+robot_name+"/joint_vsens/torso:i").c_str(),"tcp",false);
        //from wholeBodyDynamics to iCub (optional)
        if (Network::exists(string("/"+robot_name+"/joint_vsens/left_wrist:i").c_str()))
        Network::connect(string("/"+local_name+"/left_wrist/Torques:o").c_str(), string("/"+robot_name+"/joint_vsens/left_wrist:i").c_str(),"tcp",false);
        if (Network::exists(string("/"+robot_name+"/joint_vsens/right_wrist:i").c_str()))
        Network::connect(string("/"+local_name+"/right_wrist/Torques:o").c_str(),string("/"+robot_name+"/joint_vsens/right_wrist:i").c_str(),"tcp",false);
    }
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::threadInit()
{
    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    //Get data for estimator
   
    //if normal mode, publish the 
    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*****************************************************************************
void wholeBodyDynamicsThread::threadRelease()
{

}

//*****************************************************************************
void wholeBodyDynamicsThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port->interrupt();
        _port->close();

        delete _port;
        _port = 0;
    }
}

//*****************************************************************************
template <class T> void wholeBodyDynamicsThread::broadcastData(T& _values, BufferedPort<T> *_port)
{
    if (_port && _port->getOutputCount()>0)
    {
        _port->setEnvelope(this->timestamp);
        _port->prepare()  = _values ;
        _port->write();
    }
}

//*****************************************************************************
void wholeBodyDynamicsThread::writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port)
{
    Bottle a;
    a.addInt(_address);
    for(size_t i=0;i<_values.length();i++)
        a.addDouble(_values(i));
    _port->prepare() = a;
    _port->write();
}