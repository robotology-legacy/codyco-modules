#ifndef REACH_COM_BALANCE
#define REACH_COM_BALANCE

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
//#include <iCub/iKin/iKinBody.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include "rangeCheck.h"

#include <iostream>
#include <iomanip>
#include <string.h>
#include <queue>
#include <list>

#define NOARM               0
#define LEFTARM             1
#define RIGHTARM            2

#define TOWARDS_LEFT    0
#define TOWARDS_RIGHT   1
#define TOWARDS_FRONT   2
#define TOWARDS_BACK    3

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace std;

enum phase { LEFT_SUPPORT=0, RIGHT_SUPPORT, BOTH_SUPPORT, SUPPORT_PLUS_RIGHT_HAND, SUPPORT_PLUS_LEFT_HAND, SUPPORT_PLUS_BOTH_HANDS };

const double PINV_DAMP = 0.04;          // damping factor for damped pseudoinverses
const double PINV_TOL  = 1e-6;          // singular value threshold for truncated pseudoinverses
//const double MIN_JERK_TRAJ_TIME = 0.5;  // trajectory time of "minimum jerk trajectory generators"
const double MIN_JERK_TRAJ_TIME = 2.5;  // trajectory time of "minimum jerk trajectory generators"

const std::string armsTypesName[3] = {"no_arms", "left_arm", "right_arm"};

enum ControlMode { VELOCITY=0, VELOCITY_TORQUE=1, TORQUE=2};



//=====================================================================
//=====================================================================
//
//                      CONTROLLER THREAD
//
//=====================================================================
//=====================================================================




class ISIR_Controller: public RateThread
{
public:
    
    bool on_ground;
    
    string robot;
    string name;    
    int rate;
    bool display_only; //only display, do not control the robot
    bool no_sensors; // avoid using sensors not available in icub_sim
    bool verbose;
    bool feet_sensors; // if FT sensors at the feet are used to compute COP
    bool springs_legs; // springs on the legs joints
    bool torso_compensation; // using torso to compensate for built up angular momentum

    // drivers icub parts
    PolyDriver *ddRA, *ddLA;
    PolyDriver *ddT, *ddH;
    PolyDriver *ddRL, *ddLL;
    // drivers cartesian
    PolyDriver *ddCartRA, *ddCartLA;
    PolyDriver *ddGaze;
    
    IPositionControl *Ipos_TO;
    IPositionControl *Ipos_RL;
    IPositionControl *Ipos_LL;
    IPositionControl *Ipos_HE;
    IPositionControl *Ipos_RA;
    IPositionControl *Ipos_LA;
    //
    IControlLimits   *Ilim_TO;
    IControlLimits   *Ilim_RL;
    IControlLimits   *Ilim_LL;
    IControlLimits   *Ilim_HE;
    IControlLimits   *Ilim_RA;
    IControlLimits   *Ilim_LA;
    //
    IControlMode     *Ictrl_TO;
    IControlMode     *Ictrl_LL;
    IControlMode     *Ictrl_RL;
    IControlMode     *Ictrl_HE;
    IControlMode     *Ictrl_LA;
    IControlMode     *Ictrl_RA;
    //
    IVelocityControl *Ivel_TO;
    IVelocityControl *Ivel_LL;
    IVelocityControl *Ivel_RL;
    IVelocityControl *Ivel_HE;
    IVelocityControl *Ivel_LA;
    IVelocityControl *Ivel_RA;
    //
    IEncoders        *Ienc_TO;
    IEncoders        *Ienc_LL;
    IEncoders        *Ienc_RL;
    IEncoders        *Ienc_HE;
    IEncoders        *Ienc_LA;
    IEncoders        *Ienc_RA;
    //
    IPidControl      *Ipid_TO;
    IPidControl      *Ipid_LL;
    IPidControl      *Ipid_RL;
    IPidControl      *Ipid_HE;
    IPidControl      *Ipid_LA;
    IPidControl      *Ipid_RA;
    //
    IImpedanceControl *Iimp_RA;
    IImpedanceControl *Iimp_LA;
    IImpedanceControl *Iimp_RL;
    IImpedanceControl *Iimp_LL;
    IImpedanceControl *Iimp_TO;
    //
    ICartesianControl *icrt_RA;
    ICartesianControl *icrt_LA;
    IGazeControl      *igaze;
    
    // params for controllers
    //-------------------------------------------
    // torso params
    bool torsoEnabled, trackingEnabled;
    // cartesian controller params
    double cartesian_tolerance;
    // control mode
    bool leftArmImpVelMode;
    bool rightArmImpVelMode;
    bool leftLegImpVelMode;
    bool rightLegImpVelMode;
    
    ControlMode controlMode;
    
    // params for right/left arm/leg
    //-------------------------------------------
    Vector stiffness_LA;
    Vector damping_LA;
    Vector stiffness_RA;
    Vector damping_RA;
    Vector stiffness_RL;
    Vector damping_RL;
    Vector stiffness_LL;
    Vector damping_LL;
    
    // a model of icub to use - just in case
    //-------------------------------------------
    iCub::iDyn::iCubWholeBody *icub;
    iCubLeg *Right_Leg;
    iCubLeg *Left_Leg;
    iCubInertialSensor *Inertia_Sens;

    
    //****************************************************************

    // number of joints for the limbs
    int njHE, njTO, njRL, njLL, njRA, njLA;

    // the command for the motors
    Vector cmd_dq_LL, cmd_dq_RL, cmd_dq_TO;
    Vector cmd_q_LL, cmd_q_RL, cmd_q_TO;
    Vector cmd_tau_LL, cmd_tau_RL, cmd_tau_TO;
    
    Vector cmd_dq_LA, cmd_dq_RA, cmd_dq_HE;
    Vector cmd_q_LA, cmd_q_RA, cmd_q_HE;
    Vector cmd_tau_LA, cmd_tau_RA, cmd_tau_HE;

    Vector dqLL, dqRL, dqTO;
    Vector qrLL, qrRL, qrTO;                   //reference/postural configuration
    Vector dqLA, dqRA, dqHE;
    Vector qrLA, qrRA, qrHE;                   //reference/postural configuration
    Vector q0LL_both;    Vector q0RL_both;     Vector q0TO_both;
    Vector q0LL_right;   Vector q0RL_right;    Vector q0TO_right;
    Vector q0LL_left;    Vector q0RL_left;     Vector q0TO_left;
    Vector zmp_xy;
    Vector zmp_xy_vel;
    Vector zmp_des;
    iCub::ctrl::AWLinEstimator      *zmp_xy_vel_estimator;

    //matrices involved in computing Jca
    Matrix Tba, Tbc, Tab, Tac, Tca;
    Matrix Rba, Rbc, Rac, Rca;
    Matrix pba, pbc, pac, pca;
    Matrix pab, pcb, rca_b, rac_b;
    Matrix Spac, Srca, Rrab;
    Matrix Spca, Srac, Rrcb;
    Matrix Jba, Jbc, Jac, Jca;
    
    //matrices involved in the computation of eac
    Matrix  pac_d,  Rac_d;        Matrix  pca_d,  Rca_d;
    Matrix dpac_d, dRac_d;        Matrix dpca_d, dRca_d;
    
    Matrix nd,         sd,     ad,     ne,     se, ae;
    Matrix nd_hat, sd_hat, ad_hat, ne_hat, se_hat, ae_hat;
    
    Matrix ep, eo;
    Matrix L;
    
    
    //Matrices involved in the computation of the center of mass projection
    Matrix Jb_p, p_b, Jb_com           ;   //position of the center of mass and its jacobian
    Matrix Jpi_b, Jpi_b_com            ;   //jacobian of the projection on 'b'
    Matrix pi_b                        ;   //projection of p_b and disred value expressed in r.f. 'a'
    Matrix PI                          ;   //projection matrix
    Matrix pi_c, pi_a                  ;   //projection of p_b and disred value expressed in r.f. 'c'
    Vector uY, uZ                      ;   //input to filters
    Vector udY, udZ                    ;   //input to velocity filters
    Matrix  pi_c_d,  pi_a_d            ;   //desired COM position (filter output)
    Matrix dpi_c_d, dpi_a_d            ;   //desired COM velocity (filter output)
    Matrix pi_c_t, pi_a_t              ;   //target COM position (filter input)
    Matrix Kcom, Kr2l                  ;
    Matrix zmp_a, zmpLL_a, zmpRL_a     ;   //ZMP in the right foot reference frame
    Matrix zmp_c, zmpLL_c, zmpRL_c     ;   //ZMP in the left  foot reference frame
    Vector *F_ext_RL, *F_ext_RL0       ;
    Vector *F_ext_LL, *F_ext_LL0       ;

    // vectors to print for debug
    Vector debugOutVec1, debugOutVec2, debugOutVec3, debugOutVec4;

    //Vectors for the controller masks
    Matrix Smask_r2l_swg, Smask_r2l_sup;
    Matrix Smask_com_torso;

    //support phase
    phase current_phase;
    
    //for checking joints within limits
    rangeCheck *rangeCheckTO, *rangeCheckRL, *rangeCheckLL, *rangeCheckLA, *rangeCheckRA, *rangeCheckHE;
    Vector      limitMaskTO,  limitMaskRL,   limitMaskLL,
    limitMaskLA, limitMaskRA, limitMaskHE;
    
    // home position for the arms (joints)
    Vector homePoss, homeVels;
    Vector left_init, right_init, left_reach, right_reach;
    
    
    // control params
    //-------------------------------------------
    bool onTable;
    double trajTime;
    double reachTol;
    double idleTimer, idleTmo, obsrTmo, obsrTime;
    double releaseTmo;
    double tableHeight;
    double latchTimer;
    
    bool using_gaze; // if we also control gaze
    bool using_cartesian_arm_left; //if we control cartesian arm left
    bool using_cartesian_arm_right; //if we control cartesian arm right
    // booleans for defining what we control
    bool useLeftArm, useRightArm, useHead, useTorso, useRightLeg, useLeftLeg;
    
private:
    string robot_name;
    string local_name;
    string wbsName;
//    bool springs;
//    bool torso;

    //PORTS
    //input ports
    BufferedPort<Vector> *EEWRightLeg;        //EE Wrench Right Leg
    BufferedPort<Vector> *EEWLeftLeg;         //EE Wrench Left Leg
    BufferedPort<Vector> *EEWRightAnkle;      //EE Wrench Right Ankle Sensor
    BufferedPort<Vector> *EEWLeftAnkle;       //EE Wrench Left Ankle Sensor

    BufferedPort<Vector> *objPort;
    BufferedPort<Vector> *objPort2;
//    BufferedPort<Matrix> *EEPRightLeg;        //EE Pose Right Leg
//    BufferedPort<Matrix> *EEPLeftLeg;         //EE Pose Left Leg
    BufferedPort<Vector> *desired_zmp;        //varying set point.
//    BufferedPort<Matrix> *COM_Jacob_port;
//    BufferedPort<Vector> *COM_Posit_port;
//    BufferedPort<Vector> *r2l_err_port;
//    BufferedPort<Vector> *COM_err_port;
    BufferedPort<Vector> *port_ankle_angle;        //Commanded ankle angle
    BufferedPort<Vector> *port_ft_foot_left;  //Left foot f/t sensor reading
    BufferedPort<Vector> *port_ft_foot_right; //Right foot f/t sensor reading

    Matrix rot_f;

    //controller gains
//    double vel_sat;
//    double Kp_zmp_h, Kp_zmp_x, Kp_zmp_y;
//    double Kd_zmp_h, Kd_zmp_x, Kd_zmp_y;
//    double Kp, Kd;
    
    //com ports
    string comPosPortString, comJacPortString;

    //err ports
    string r2lErrPortString, comErrPortString;
    
    //compute ZMP variables
    double xLL, yLL, xDS, yDS, yRL, xRL;
    Matrix m_a, f_a, Sf_a         ;           //moment and force on the right foot
    Matrix m_c, f_c, Sf_c         ;           //moment and force on the left  foot
    Matrix Ras, Rcs               ;           //rotations from sensors to feet

    //For plots only
    Vector *F_ext_rf;
    Vector *F_ext_lf;


    //Left and Right leg pose.
    Vector PoseLeftLeg;
    Vector PoseRightLeg;

    //Right and left leg and torso encoders
    Vector qRL, qRL_rad;
    Vector qLL, qLL_rad;
    Vector qTO, qTO_rad;
    
    Matrix Jac_FR;             //Jacobian matrix for right FOOT from ROOT.


    Matrix Hright;
    Matrix Hleft;

    //bool Opt_ankles_sens;
    
    Stamp zmp_time;

    Vector Offset_Lfoot, Offset_Rfoot;
   
    // for controlling
    //---------------------------------------------
    void setRefAcc(IEncoders* iencs, IVelocityControl* ivel);
    
    // for computing the icub model
    //---------------------------------------------
//    double timestamp;
    Vector all_q_up, all_dq_up, all_d2q_up;
    Vector all_q_low, all_dq_low, all_d2q_low;
    AWLinEstimator  *InertialEst;
    AWLinEstimator  *linEstUp;
    AWQuadEstimator *quadEstUp;
    AWLinEstimator  *linEstLow;
    AWQuadEstimator *quadEstLow;
    
    Vector evalVelUp(const Vector &x);
    Vector evalVelLow(const Vector &x);
    Vector eval_domega(const Vector &x);
    Vector evalAccUp(const Vector &x);
    Vector evalAccLow(const Vector &x);
    
    void init_upper();
    void init_lower();
    void setUpperLowerMeasure();
    
    Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;
    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;
    
    bool readAndUpdate(bool waitMeasure=false);
    bool getLowerEncodersSpeedAndAcceleration();
    bool getUpperEncodersSpeedAndAcceleration();
    
    Vector get_q_head();
    Vector get_dq_head();
    Vector get_d2q_head();
    Vector get_q_larm();
    Vector get_dq_larm();
    Vector get_d2q_larm();
    Vector get_q_rarm();
    Vector get_dq_rarm();
    Vector get_d2q_rarm();
    Vector get_q_torso();
    Vector get_dq_torso();
    Vector get_d2q_torso();
    Vector get_q_lleg();
    Vector get_dq_lleg();
    Vector get_d2q_lleg();
    Vector get_q_rleg();
    Vector get_dq_rleg();
    Vector get_d2q_rleg();
    

public:
    ISIR_Controller(int _rate,
                    PolyDriver *_ddRA, PolyDriver *_ddLA,
                    PolyDriver *_ddT, PolyDriver *_ddH,
                    PolyDriver *_ddRL, PolyDriver *_ddLL,
                    PolyDriver *_ddCartRA, PolyDriver *_ddCartLA,
                    PolyDriver *_ddGaze,
                    const string &_robot, const string &_name,
                    bool _display_only, bool _noSens,
                    bool _ankles_sens, bool _springs,
                    bool _torso_enable, bool _verbose,
                    Vector stiffness_RA, Vector stiffness_LA,
                    Vector stiffness_RL, Vector stiffness_LL,
                    Vector damping_RA, Vector damping_LA,
                    Vector damping_RL, Vector damping_LL,
                    ControlMode _controlMode);
    
    
    
    bool threadInit();
    void run();
    bool onStop();
    void threadRelease();
    void suspend();
    void defineUsedInterfaces(bool _using_gaze, bool _using_cartesian_arm_left, bool _using_cartesian_arm_right, bool _useLeftArm, bool _useRightArm, bool _useHead, bool _useTorso, bool _useRightLeg, bool _useLeftLeg);
    
    
    bool check_njTO(int _njTO);
    bool check_njRL(int _njRL);
    bool check_njLL(int _njLL);

    bool moveHand(const int sel_arm, const Vector &xd, const Vector &od);
    void computeControl_ISIR();
    void sendCommandsToRobot();
    
};





//=====================================================================
//=====================================================================
//
//                      CONTROLLER MODULE
//
//=====================================================================
//=====================================================================

class ISIR_Balancer: public RFModule
{
private:
    
    // basic params module
    //-------------------------------------------
    string name;
    string robot;
    Port rpcPort; // the port to handle messages
    int count;
    int rate;
    bool no_sensors; // avoid using sensors not available in icub_sim
    bool verbose;
    bool display_only; //only display, do not control the robot
    bool feet_sensors; // if FT sensors at the feet are used to compute COP
    bool springs_legs; // springs on the legs joints
    bool torso_compensation; // using torso to compensate for built up angular momentum
    bool using_gaze; // if we also control gaze
    bool using_cartesian_arm_left; //if we control cartesian arm left
    bool using_cartesian_arm_right; //if we control cartesian arm right
    
    // drivers
    //-------------------------------------------
    // properties for drivers
    Property    optionsRA, optionsLA,optionsT, optionsH, optionsLL, optionsRL, optionsCartRA, optionsCartLA, optionsGaze;
    
    // drivers icub parts
    PolyDriver *ddRA, *ddLA;
    PolyDriver *ddT, *ddH;
    PolyDriver *ddRL, *ddLL;
    // drivers cartesian
    PolyDriver *ddCartRA, *ddCartLA;
    PolyDriver *ddGaze;
    // motor interfaces
    IPositionControl *iposRA, *iposLA,*ipos;
    IPositionControl *iposRL, *iposLL;
    IPositionControl *iposT, *iposH;
    IEncoders *iencRA, *iencLA,*ienc;
    IEncoders *iencRL, *iencLL;
    IEncoders *iencT, *iencH;
    IImpedanceControl *iimpRA, *iimpLA, *iimp;
    IImpedanceControl *iimpRL, *iimpLL;
    IControlMode *icmdRA, *icmdLA, *icmd, *icmdRL, *icmdLL;
    // cartesian interfaces
    ICartesianControl *icrtRA, *icrtLA, *icrt;
    IGazeControl *igaze;
    
    // params for controllers
    //-------------------------------------------
    // contexts for cartesian controllers
    int startup_context_id_RA, startup_context_id_LA,startup_context_id_gaze;
    // booleans
    bool useLeftArm, useRightArm, useHead, useTorso, useRightLeg, useLeftLeg;
    // torso params
    bool torsoEnabled, trackingEnabled;
    Vector torsoSwitch;
    Matrix torsoLimits;
    Vector torso;
    // head
    Vector head;
    // home position for the head
    Vector home_front, home_table;
    // cartesian controller params
    double cartesian_tolerance;
    Vector cartesian_tolerances;
    // home position for the arms (joints)
    Vector homePoss, homeVels;
    Vector left_init, right_init, left_reach, right_reach, left_touchtable, right_touchtable;
    // control mode
    bool leftArmImpVelMode;
    bool rightArmImpVelMode;
    bool leftLegImpVelMode;
    bool rightLegImpVelMode;
    //
    ControlMode controlMode;
    
    // params for right/left arm/leg
    //-------------------------------------------
    Vector leftArmHandOrienTable;
    Vector rightArmHandOrienTable;
    Vector leftArmJointsStiffness;
    Vector leftArmJointsDamping;
    Vector rightArmJointsStiffness;
    Vector rightArmJointsDamping;
    Vector rightLegJointsStiffness;
    Vector rightLegJointsDamping;
    Vector leftLegJointsStiffness;
    Vector leftLegJointsDamping;
    
    // control params
    //-------------------------------------------
    bool onTable;
    double trajTime;
    double reachTol;
    double releaseTmo;
    double latchTimer;
    
    // params for experiments
    //--------------------------------------------
    int selectedArm;
    bool wentHome;
    
    // params used for safety checks
    double tableHeight;
    double minX_distanceFromTorso;
    double maxY_reachLeftArm;
    double maxY_reachRightArm;
    
    Vector openHandPoss, closeHandPoss;
    Vector handVels;
    Vector targetPos, pickedPos, observePos;
    Matrix R,Rx,Ry,Rz;
    int state;
    
    // my variables for ease of coding
    //------------------------------------------------
    Vector x_cmd, o_cmd, x_cur, o_cur, xd, od;
    bool was_on_ground;
    
    
    // the actual elements for the controller
    //------------------------------------------------
    
    ISIR_Controller *balThread;
    
    Vector pi_a_d_right, pi_a_d_both, pi_c_d_left;
    
    
    // UTILS
    //---------------------------------------------------------
    void getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim);
    void getArmOptions(Bottle &b, Vector &impStiff, Vector &impDamp,
                      Vector &orienTable);
    void getLegOptions(Bottle &b, Vector &impStiff, Vector &impDamp);
    void limitRangeHand(Vector &x, const int sel);
    void enableTorso(int pitch=1, int roll=1, int yaw=1);
    void disableTorso();
    void setCartesianTolerance();
    void changeTrackingMode();
    void setTrackingMode(bool mode);
    void limitTorsoPitch();
    
    void stopControl(const int sel);
    void initCartesianCtrl(Vector &sw, Matrix &lim, const int sel);
    
public:
    
    ISIR_Balancer();
    
    bool configure(ResourceFinder &rf);
    
    bool close();
    
    double getPeriod();
    
    bool updateModule();
    
    bool respond(const Bottle &command, Bottle &reply);
};




#endif
