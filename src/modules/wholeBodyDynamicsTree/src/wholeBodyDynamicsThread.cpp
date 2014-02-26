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

iCubTreeStatus::iCubTreeStatus(int nrOfDOFs, int nrOfFTsensors)
{
    setNrOfDOFs(nrOfDOFs);
    setNrOfFTSensors(nrOfFTsensors);
}

bool iCubTreeStatus::zero()
{
    domega_imu.zero();
    omega_imu.zero();
    proper_ddp_imu.zero();
    wbi_imu.zero();
    q.zero();
    dq.zero();
    ddq.zero();
    for(int i=0; i < estimated_ft_sensors.size(); i++ ) {
        estimated_ft_sensors[i].zero();
        measured_ft_sensors[i].zero();
    }
    return true;
}

bool iCubTreeStatus::setNrOfDOFs(int nrOfDOFs)
{
    domega_imu.resize(3);
    omega_imu.resize(3);
    proper_ddp_imu.resize(3);
    wbi_imu.resize(wbi::sensorTypeDescriptions[wbi::SENSOR_IMU].dataSize);
    q.resize(nrOfDOFs);
    dq.resize(nrOfDOFs);
    ddq.resize(nrOfDOFs);
    
    zero();
    return true;
}

bool iCubTreeStatus::setNrOfFTSensors(int nrOfFTsensors)
{
    estimated_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    measured_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
    zero();
    return true;
}

//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name, 
                                                 string _robotName,
                                                 int _period, 
                                                 icubWholeBodyStatesLocal *_wbs,
                                                 const iCub::iDynTree::iCubTree_version_tag _icub_version)
    :  RateThread(_period),
       name(_name),
       robotName(_robotName), 
       estimator(_wbs), 
       printCountdown(0), 
       PRINT_PERIOD(1000),
       icub_version(_icub_version),
       icub_model_calibration(icub_version),
       max_samples_used_for_calibration(10),
       samples_used_for_calibration(0)
    {
    
    std::cout << "Launching wholeBodyDynamicsThread with name : " << _name << " and robotName " << _robotName << " and period " << _period << std::endl;
        
        
    bool autoconnect = false;
    
    //Resize buffer vectors
    all_torques.resize(_wbs->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE));
    /// \todo hardcoded checking
    if( _wbs->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE) != 32 ) { std::cerr << "wholeBodyDynamicsThread() error: only " << _wbs->getEstimateNumber(wbi::ESTIMATE_JOINT_TORQUE) << " are available "  << std::endl; }
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
wbi::LocalId wholeBodyDynamicsThread::convertFTiDynTreeToFTwbi(int ft_sensor_id)
{
    LocalId lid;
    assert(ft_sensor_id >= 0 && ft_sensor_id <= icub_model_calibration.getNrOfFTSensors());
    if( icub_version.feet_ft ) {
        return ICUB_MAIN_FOOT_FTS.globalToLocalId(ft_sensor_id);
    } else {
        return ICUB_MAIN_FTS.globalToLocalId(ft_sensor_id);
    }
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::threadInit()
{
    //Calibration variables 
    int nrOfAvailableFTSensors = estimator->getEstimateNumber(wbi::ESTIMATE_FORCE_TORQUE);
    if( nrOfAvailableFTSensors != icub_model_calibration.getNrOfFTSensors() ) {
        std::cout << "wholeBodyDynamicsThread::threadInit() error: number of FT sensors different between model (" <<
        icub_model_calibration.getNrOfFTSensors() << " and interface" << nrOfAvailableFTSensors << std::endl;
        return false;
    }
    
    offset_buffer.resize(nrOfAvailableFTSensors,yarp::sig::Vector(6,0.0));
    calibrate_ft_sensor.resize(nrOfAvailableFTSensors,false);
    tree_status.setNrOfDOFs(icub_model_calibration.getNrOfDOFs());
    
    //the serialization is this one for foot v1 : 0 left arm  1 right arm 
    //                                            2 left leg  3 right leg 
    
    //and this one                  for foot v2 : 0 left arm  1 right arm 
    //                                            2 left leg  3 left foot
    //                                            4 right leg 5 right foot
    
    /// < \todo TODO ENFORCE match between interface and iCub sensors
    l_arm_ft_sensor_id = icub_model_calibration.getFTSensorIndex("l_arm_ft_sensor");
    r_arm_ft_sensor_id = icub_model_calibration.getFTSensorIndex("r_arm_ft_sensor");
    l_leg_ft_sensor_id = icub_model_calibration.getFTSensorIndex("l_leg_ft_sensor");
    r_leg_ft_sensor_id = icub_model_calibration.getFTSensorIndex("r_leg_ft_sensor");
    if( icub_version.feet_ft ) {
        l_foot_ft_sensor_id = icub_model_calibration.getFTSensorIndex("l_foot_ft_sensor");
        r_foot_ft_sensor_id = icub_model_calibration.getFTSensorIndex("r_foot_ft_sensor");
    } else { 
        l_foot_ft_sensor_id = -1;
        r_foot_ft_sensor_id = -1;
    }
    
    wbd_mode = NORMAL;
    printf("\n\n");
    return true;
}


//*************************************************************************************************************************
bool wholeBodyDynamicsThread::calibrateOffset(const std::string calib_code)
{
    if( max_samples_used_for_calibration <= 0 ) {
        std::cout << "wholeBodyDynamicsThread::calibrateOffset error: requested calibration with a negative (" << max_samples_used_for_calibration << ") number of samples." << std::endl;
        return false;
    }
    if( calib_code == "feet" && !(icub_version.feet_ft) ) { 
        std::cout << "wholeBodyDynamicsThread::calibrateOffset error: requested calibration of feet, but feet FT sensor not available." << std::endl;
        return false;
    }
    
    if( calib_code == "all" ) {
        calibrate_ft_sensor[l_arm_ft_sensor_id] = true;
        calibrate_ft_sensor[r_arm_ft_sensor_id] = true;
        calibrate_ft_sensor[l_leg_ft_sensor_id] = true;
        calibrate_ft_sensor[r_leg_ft_sensor_id] = true;
        if( icub_version.feet_ft ) {
            calibrate_ft_sensor[l_foot_ft_sensor_id] = true;
            calibrate_ft_sensor[r_foot_ft_sensor_id] = true;
        }
    } else if ( calib_code == "arms" ) {
        calibrate_ft_sensor[l_arm_ft_sensor_id] = true;
        calibrate_ft_sensor[r_arm_ft_sensor_id] = true;
    } else if ( calib_code == "legs" ) {
        calibrate_ft_sensor[l_leg_ft_sensor_id] = true;
        calibrate_ft_sensor[r_leg_ft_sensor_id] = true;
    } else if ( calib_code == "feet" ) {
        calibrate_ft_sensor[l_foot_ft_sensor_id] = true;
        calibrate_ft_sensor[r_foot_ft_sensor_id] = true;
    } else {
        std::cout << "wholeBodyDynamicsThread::calibrateOffset error: code " << calib_code << " not available" << std::endl;
        return false;
    }
    
    calibration_mutex.lock();
    wbd_mode = CALIBRATING;
    
    return true;
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::waitCalibrationDone()
{
    calibration_mutex.lock();
    assert(wbd_mode == NORMAL);
    calibration_mutex.unlock();
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
    if( wbd_mode == CALIBRATING ) {
        calibration_run();
    } else {
        assert(wbd_mode == NORMAL);
        normal_run();
    }
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::normal_run()
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

    if( printCountdown == 0 ) {
        double avgTime, stdDev, avgTimeUsed, stdDevUsed, period;
        period = getRate();
        getEstPeriod(avgTime, stdDev);
        getEstUsed(avgTimeUsed, stdDevUsed);
         printf("[PERFORMANCE INFORMATION]:\n");
        printf("Expected period %lf ms.\nReal period: %3.1lf+/-%3.1lf ms.\n", period, avgTime, stdDev);
        printf("Real duration of 'run' method: %3.1lf+/-%3.1lf ms.\n", avgTimeUsed, stdDevUsed);
        if(avgTimeUsed<0.5*period)
            printf("Next time you could set a lower period to improve the wholeBodyDynamics performance.\n");
        else if(avgTime>1.3*period)
            printf("The period you set was impossible to attain. Next time you could set a higher period.\n");
        std::cout << "Torques: " << std::endl;
        std::cout << all_torques.toString() << std::endl;
        std::cout << "Forces: " << std::endl;
        std::cout << external_forces_list.toString() << std::endl;

    }
    
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::calibration_run()
{   
    bool ret;
    ret = estimator->getEstimates(wbi::ESTIMATE_JOINT_POS,tree_status.q.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_VEL,tree_status.dq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_JOINT_ACC,tree_status.ddq.data());
    ret = ret && estimator->getEstimates(wbi::ESTIMATE_IMU,tree_status.wbi_imu.data());
    YARP_ASSERT(ret);    
    
    //Setting imu proper acceleration from measure (assuming omega e domega = 0)
    //acceleration are measures 4:6 (check wbi documentation)
    tree_status.proper_ddp_imu[0] = tree_status.wbi_imu[4];
    tree_status.proper_ddp_imu[1] = tree_status.wbi_imu[5];
    tree_status.proper_ddp_imu[2] = tree_status.wbi_imu[6];
    tree_status.omega_imu[0] = tree_status.wbi_imu[7];
    tree_status.omega_imu[1] = tree_status.wbi_imu[8];
    tree_status.omega_imu[2] = tree_status.wbi_imu[9];
    
    //Estimating sensors
    icub_model_calibration.setInertialMeasure(tree_status.omega_imu,tree_status.domega_imu,tree_status.proper_ddp_imu);
    icub_model_calibration.setAng(tree_status.q);
    icub_model_calibration.setDAng(tree_status.dq);
    icub_model_calibration.setD2Ang(tree_status.ddq);
    
    icub_model_calibration.kinematicRNEA();
    icub_model_calibration.dynamicRNEA();

    for(int ft_sensor_id=0; ft_sensor_id < offset_buffer.size(); ft_sensor_id++ ) {
        if( calibrate_ft_sensor[ft_sensor_id] ) {
            icub_model_calibration.getSensorMeasurement(ft_sensor_id,tree_status.estimated_ft_sensors[ft_sensor_id]);       
            offset_buffer[ft_sensor_id] += tree_status.measured_ft_sensors[ft_sensor_id]-tree_status.estimated_ft_sensors[ft_sensor_id];   
        } 
        samples_used_for_calibration++;
    }
    
    
    if( samples_used_for_calibration >= max_samples_used_for_calibration ) {
        //Calculating offset
        for(int ft_sensor_id=0; ft_sensor_id < offset_buffer.size(); ft_sensor_id++ ) {
            offset_buffer[ft_sensor_id] *= (1.0/(double)samples_used_for_calibration);
            assert(offset_buffer.size() == wbi::sensorTypeDescriptions[wbi::SENSOR_FORCE_TORQUE].dataSize);
            estimator->setEstimationOffset(wbi::ESTIMATE_FORCE_TORQUE,convertFTiDynTreeToFTwbi(ft_sensor_id),offset_buffer.data());
        }
        
        
        //Restoring default data for calibration data structures
        samples_used_for_calibration = 0;
        for(int ft_sensor_id=0; ft_sensor_id < offset_buffer.size(); ft_sensor_id++ ) {
            offset_buffer[ft_sensor_id].zero();
            calibrate_ft_sensor[ft_sensor_id] = false;
        } 
        
        wbd_mode = NORMAL;
        calibration_mutex.unlock();
    }
    
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
    /** \todo TODO avoid (as much as feasible) dynamic memory allocation */
    Bottle a;
    a.addInt(_address);
    for(size_t i=0;i<_values.length();i++)
        a.addDouble(_values(i));
    _port->prepare() = a;
    _port->write();
}