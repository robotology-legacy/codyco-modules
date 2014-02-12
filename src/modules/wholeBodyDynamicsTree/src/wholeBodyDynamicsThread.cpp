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

#include <wholeBodyDynamicsTree/wholeBodyDynamicsThread.h>
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
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name, string _robotName, int _period, icubWholeBodyStatesLocal *_wbs)
    :  RateThread(_period), name(_name), robotName(_robotName), estimator(_wbs), PRINT_PERIOD(1000)
    {
    bool autoconnect = false;
    
    //Resize buffer vectors
    all_torques.resize(_wbs->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE));
    /// \todo hardcoded checking 
    YARP_ASSERT(all_torques.size() == 32);
    
    HDTorques.resize(3);
    TOTorques.resize(3);
    LATorques.resize(7);
    RATorques.resize(7);
    LLTorques.resize(6);
    RLTorques.resize(6);
    
    
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
    
    /*
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
    
    port_external_ft_arm_left = new BufferedPort<Vector>;
    port_external_ft_arm_right = new BufferedPort<Vector>;
    port_external_ft_leg_left = new BufferedPort<Vector>;
    port_external_ft_leg_right = new BufferedPort<Vector>;

    
    port_all_velocities = new BufferedPort<Vector>;
    port_all_positions = new BufferedPort<Vector>;
    */
    
    port_contacts = new BufferedPort<skinContactList>;


    //Opening ports
    port_RATorques->open(string("/"+local_name+"/right_arm/Torques:o").c_str());
    port_LATorques->open(string("/"+local_name+"/left_arm/Torques:o").c_str());
    port_RLTorques->open(string("/"+local_name+"/right_leg/Torques:o").c_str());
    port_LLTorques->open(string("/"+local_name+"/left_leg/Torques:o").c_str());
    port_RWTorques->open(string("/"+local_name+"/right_wrist/Torques:o").c_str());
    port_LWTorques->open(string("/"+local_name+"/left_wrist/Torques:o").c_str());
    port_TOTorques->open(string("/"+local_name+"/torso/Torques:o").c_str());
    port_HDTorques->open(string("/"+local_name+"/head/Torques:o").c_str());
    
    port_contacts->open(string("/"+local_name+"/contacts:o").c_str());
    
    /*
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
    
    
    port_external_ft_arm_left->open(string("/"+local_name+"/left_arm/ext_ft_sens:o").c_str());
    port_external_ft_arm_right->open(string("/"+local_name+"/right_arm/ext_ft_sens:o").c_str());
    port_external_ft_leg_left->open(string("/"+local_name+"/left_leg/ext_ft_sens:o").c_str());
    port_external_ft_leg_right->open(string("/"+local_name+"/right_leg/ext_ft_sens:o").c_str());
    
    */
    
    
    //port_all_velocities->open(string("/"+local_name+"/all_velocities:o").c_str());
    
    if (autoconnect)
    {
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
void wholeBodyDynamicsThread::publishTorques()
{
    //Converting torques from the serialization used in wholeBodyStates interface to the port used by the robot
    /// \todo remove this hardcoded dependency on the serialization used in the interface (using getEstimateList)
    
    /// \note The torso has a weird dependency, basically the joint serialization in the robot and the one on the model are reversed
    TOTorques[0] = all_torques[2];
    TOTorques[1] = all_torques[1];
    TOTorques[2] = all_torques[0];
    
    for(int i=0; i < 3; i++ ) {
        HDTorques[i] = all_torques[3+i];
    }
    
    for(int i=0; i < 7; i++ ) {
        LATorques[i] = all_torques[3+3+i];
    }
   
    for(int i=0; i < 7; i++ ) {
        RATorques[i] = all_torques[3+3+7+i];
    }
    
    for(int i=0; i < 6; i++ ) {
        LLTorques[i] = all_torques[3+3+7+7+i];
    }
    
    for(int i=0; i < 6; i++ ) {
        RLTorques[i] = all_torques[3+3+7+7+6+i];
    }
    
    //Parameters copied from old wholeBodyDynamics
    writeTorque(TOTorques, 4, port_TOTorques);
    writeTorque(HDTorques, 0, port_HDTorques);
    writeTorque(LATorques, 1, port_LATorques);
    writeTorque(RATorques, 1, port_RATorques);
    writeTorque(LATorques, 3, port_LWTorques);
    writeTorque(RATorques, 3, port_RWTorques);
    writeTorque(LLTorques, 2, port_LLTorques);
    writeTorque(RLTorques, 2, port_RLTorques);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::publishContacts()
{
    broadcastData<skinContactList>(external_forces_list, port_contacts);
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    bool ret;
    //Get data from estimator and publish it
    
    //Get estimated torques
    assert(estimator->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE) == all_torques.size());
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_TORQUE,all_torques.data());
    YARP_ASSERT(ret);
    
    //Get estimated external contacts
    ret = estimator->getEstimatedExternalForces(external_forces_list);
    YARP_ASSERT(ret);
    
    //Send torques
    publishTorques();
    
    //Send external contacts
    publishContacts();
    
    //if normal mode, publish the 
    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*****************************************************************************
void wholeBodyDynamicsThread::threadRelease()
{
    std::cerr << "Closing RATorques port\n";
    closePort(port_RATorques);
    std::cerr << "Closing LATorques port\n";
    closePort(port_LATorques);
    std::cerr << "Closing RLTorques port\n";
    closePort(port_RLTorques);
    std::cerr << "Closing LLTorques port\n";
    closePort(port_LLTorques);
    std::cerr << "Closing RWTorques port\n";
    closePort(port_RWTorques);
    std::cerr << "Closing LWTorques port\n";
    closePort(port_LWTorques);
    std::cerr << "Closing TOTorques port\n";
    closePort(port_TOTorques);
    std::cerr << "Closing HDTorques port\n";
    closePort(port_HDTorques);
    std::cerr << "Closing contacts port\n";
    closePort(port_contacts);
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