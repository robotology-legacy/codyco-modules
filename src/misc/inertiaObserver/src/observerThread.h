/* 
 * Copyright (C) 2012
 * Author: Silvio Traversaro
 * email:  pegua1@gmail.com
 * website: www.robotcub.org
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

#ifndef OBSERVER_THREAD
#define OBSERVER_THREAD

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iCub/learningMachine/IParameterLearner.h>


#include <iostream>
#include <iomanip>
#include <string.h>
#include <list>
#include <sstream>
#include <deque>
#include <fstream>

#include "iCubStateEstimator.h"

#include "onlineMean.h"

#define MAX_JN 12
#define MAX_FILTER_ORDER 6


enum thread_status_enum {STATUS_OK=0, STATUS_DISCONNECTED}; 

// filter
double lpf_ord1_3hz(double input, int j);
//Use instead iCub::ctrl::FirstOrderLowPassFilter::FirstOrderLowPassFilter(3,

using namespace yarp::os;
using namespace iCub::iDyn;



/**
 * 
 * \todo Add synchronization between call to suspend and call to run !!!
 * 
 */
class inertiaObserver_thread: public RateThread
{
public:


private:
    vector<iCubLimb> vectorLimbs;
    vector<iCubFT> vectorFT;
    map<iCubFT,iCubLimb> FTlimb;
    map<iCubLimb,string> limbNames;
    map<iCubFT,string> FTNames;    
    map<iCubFT,Vector> ftStdDev;

    int rateEstimation;
    
    int N_samples;


    string     robot_name;

    string     local_name;


    version_tag icub_type;

    string data_path;

    bool       autoconnect;

    
    bool right_leg_enabled;
    bool left_leg_enabled;
	bool right_arm_enabled;
    bool left_arm_enabled;
    
    map<iCubLimb,bool> is_enabled;
    
    bool debug_out_enabled;
    
    bool dump_static;
    
    bool verbose;
  
    onlineMean<double> run_period;
    
    string part;
    
    bool debug_out_parameters;
    
    string xml_yarpscope_file;
    
    map<iCubFT,Matrix> allStaticRegr;
    map<iCubFT,Vector> allStaticFT;
    map<iCubFT,int> static_dump_count;

        
    //input ports
    map<iCubFT,FTCollector *> port_ft;

    BufferedPort<skinContactList> * port_contact;
    
    BufferedPort<Vector> *port_inertial_thread;
    
    pwmCollector *port_pwm;
    
        
    map<iCubLimb,posCollector *> port_q;
    
    
    
    map<iCubFT,BufferedPort<Vector> *> param_output;
    map<iCubFT,BufferedPort<Matrix> *> identiable_param_output;
    

    //Map of estimator objects
    map<iCubFT, vector<iCub::learningmachine::IParameterLearner *> > paramEstimators;
            
    map<iCubFT, BufferedPort<Vector> * > measured_out_port;
    map<iCubFT, vector<BufferedPort<Vector> * > > estimated_out_port; 
    map<iCubFT, vector<BufferedPort<Vector> * > > estimated_out_port_inc; 

    
    map<iCubFT, vector<BufferedPort<Vector> * > > parameters_estimated_out_port; 
    
    map<iCubFT, vector<BufferedPort<Vector> * > > estimated_forward_inertial_torques; 
    map<iCubFT, vector<BufferedPort<Vector> * > > estimated_backward_inertial_torques; 
    map<iCubFT, vector<BufferedPort<Vector> * > > estimated_FT_sens_torques; 
    map<iCubFT, vector<BufferedPort<Vector> * > > measured_FT_sens_torques; 

    //Mixed static/dynamic estimation
    iCub::learningmachine::IParameterLearner * staticParamEstimator;
    iCub::learningmachine::IParameterLearner * dynamicParamEstimator;

    BufferedPort<Vector> * mixed_estimated_out_port;
    BufferedPort<Vector> * static_estimated_out_port;

    
    map<iCubFT, vector<BufferedPort<Vector> * > > estimated_projected_torques_inc; 


    
    map<iCubFT, vector<Vector> > params;


    bool first;
    thread_status_enum thread_status;

    
    int initial_time;
    
    int calibrate_call_count;
    
    bool learning_enabled;

    
    static const bool enable_log = true;

    int ctrlJnt;
    int allJnt;
    iCubWholeBody *icub;
    
    
    iCubStateEstimator current_state_estimator;    
    
    Vector Fend,Muend;
    
    //Measured Wrenches
    map<iCubFT,Vector> measuredW;

    map<iCubFT,Vector> offset;
    //Estimated Wrenches
    Vector W_ident_LArm, W_ident_RArm;
    Vector W_iDyn_LArm, W_iDyn_RArm, Offset_LArm, Offset_RArm;

    Vector W_ident_LLeg, W_ident_RLeg; 
    Vector W_iDyn_LLeg, W_iDyn_RLeg, Offset_LLeg, Offset_RLeg;
    
    Vector W_ident_RArm_y;
    
    //Used beta
    Vector beta_RArm, beta_CAD_RArm;
    Vector beta_LArm, beta_CAD_LArm;
    Vector beta_RLeg, beta_CAD_RLeg;
    Vector beta_LLeg, beta_CAD_LLeg;
    Vector beta_RArm_y;
    
    Matrix id_subspace_RArm;
    
    
    Matrix F_sens_up, F_sens_low, F_ext_up, F_ext_low;
    
    
    map<iCubFT,double> timestamp_lastFTsample_returned;
    
    map<iCubLimb,bool> wasStill;
    //map<iCubFT,onlineMean<Vector> > onlineMeanFT;
    //map<iCubFT,onlineMean<Matrix> > onlineMeanRegr;
    
    map<iCubFT,Matrix> identifiable_parameters;
    map<iCubFT,Matrix> static_identifiable_parameters;
    map<iCubFT,Matrix> dynamic_identifiable_parameters;
    //identifiable_parameters = static_identifiable_parameters + dynamic_identifiable_parameters
    //static_identifiable_parameters \cap  dynamic_identifiable_parameters = {0}
    

    Matrix ATA;
    Matrix ATA_forces;
    Matrix ATA_torques;
    vector<Matrix> TTT;
    Matrix TauTTau;

    // icub model
    int comp;
    Matrix FM_sens_up,FM_sens_low;
    
    Vector tot_cad_error, tot_ident_error, tot_ident_y_error;
    int error_steps;
    
    //Warning: use A and b can cause overflowing of int
    bool produceAb;
    bool produceAb_contact;
    bool produceAb_motors;
    Matrix A;
    Matrix local_A;
    Matrix local_Phi;
    Vector b;
    Vector local_b;
    int Ab_sample_count;
    std::ofstream A_file;
    std::ofstream b_file;
    std::ofstream beta_cad_file;
    std::ofstream contact_file;
    
    //produce Ab motors
    Matrix T_T, Y_s_reduced, Y_s_all, Y_I, Y_II, Y_tau, A_I, A_II, B_I, diagV_I, diagV_II;
    Matrix megazord; //The complete REGRESSOR!!!
    
    bool fail;
    
    //Warning

    void init_upper();
    void init_lower();

public:
    inertiaObserver_thread(int _rate, int _rateEstimation, string _robot_name, string _local_name, version_tag icub_type, string _data_path, bool _autoconnect, bool _right_leg_enabled, bool _left_leg_enabled, bool _right_arm_enabled, bool _left_arm_enabled, bool _debug_out_enabled, bool _dump_static, string _xml_yarpscope_file);
    bool threadInit();
    inline thread_status_enum getThreadStatus() 
    {
        return thread_status;
    }

    void run();
    void threadRelease();
    void openPort(Contactable *_port, string portName);
    Vector all_masses_regressor(int n);
    void closePort(Contactable *_port);
    bool calibrateOffset();
    bool readAndUpdate(bool waitMeasure=false, bool _init=false);
    bool readLastSuitableFT(std::string limbName, iCubWholeBody & icub, iCubStateEstimator & current_state_estimator, Vector & F_measured, double & F_timestamp );
    bool readAvailableFT(iCubFT ft, iCubWholeBody & icub, iCubStateEstimator & current_state_estimator, Vector & F_measured, double & F_timestamp );
    void setZeroJntAngVelAcc();  
    bool estimateSensorWrench(iCub::iDyn::iCubWholeBody &icub,const std::string limb,const yarp::sig::Vector beta,yarp::sig::Vector & wrench);
    
    /**
     * Assign a random act of motion to the limbs of the icub, uses yarp::os::Random,
     *  not checking autocollision.
     * If a limb is specified, only that limb is assigned a random configuration
     * (torso and head are still and in home position
     */
    void fillRandomMotion(iCubWholeBody * icub, string limb_name);

    /**
    * Assign a random configuration to the limbs of the icub (and velocity and acceleration set to zero)
    * if the limb is specified, only that limb is assigned a random configuration
    */
    void fillRandomPosition(iCubWholeBody * icub, string limb_name);
    
    /**
     * Get 
     */
     Matrix getiCubLimbIdentifiableSubspace(string limb_name, bool only_static = false, int num_samples = 1000, double tol = -1.0);
     
     void debug_generate_yarpscope_xml(iCubFT ft,bool debug_out_param_yarpscope = false);
     void debug_generate_yarpscope_xml_only_param(iCubFT ft);
    void enableLearning();
    void disableLearning();
    void finalAnalysis();
    Matrix diagonalMatrix(Vector S);
    Matrix getOnlyDynamicParam(Matrix all_param, double tol = -1.0);



};

#endif


