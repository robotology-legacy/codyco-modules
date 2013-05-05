#ifndef COM_BALANCING
#define COM_BALANCING

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
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

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace std;

enum phase { LEFT_SUPPORT, RIGHT_SUPPORT, BOTH_SUPPORT };

const double PINV_DAMP = 0.04;          // damping factor for damped pseudoinverses
const double PINV_TOL  = 1e-6;          // singular value threshold for truncated pseudoinverses
const double MIN_JERK_TRAJ_TIME = 0.5;  // trajectory time of "minimum jerk trajectory generators"

class comStepperThread: public RateThread
{
public:
    bool on_ground;
    //****************** POLYDRIVERS AND INTERFACES ******************
    PolyDriver *dd_torso;
    PolyDriver *dd_rightLeg;
    PolyDriver *dd_leftLeg;
    
    IPositionControl *Ipos_TO;
    IPositionControl *Ipos_RL;
    IPositionControl *Ipos_LL;
    IControlLimits   *Ilim_TO;
    IControlLimits   *Ilim_RL;
    IControlLimits   *Ilim_LL;
    IControlMode     *Ictrl_TO;
    IControlMode     *Ictrl_LL;
    IControlMode     *Ictrl_RL;
    IVelocityControl *Ivel_TO;
    IVelocityControl *Ivel_LL;
    IVelocityControl *Ivel_RL;
    IEncoders        *Ienc_TO;
    IEncoders        *Ienc_LL;
    IEncoders        *Ienc_RL;
    IPidControl      *Ipid_TO;
    IPidControl      *Ipid_LL;
    IPidControl      *Ipid_RL;
    //****************************************************************

    int njHD;   int njTO;   int njRL;  int njLL; int njRA;  int njLA;

    Vector dqLL, dqRL, dqTO;
    Vector qrLL, qrRL, qrTO;                   //refernce/postural configuration
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
    minJerkTrajGen *minJerkY, *minJerkZ;   //minimum jerk trajectory generator
    minJerkTrajGen *minJerkH           ;   //minimum jerk trajectory generator
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
    Vector *mask_r2l_swg, *mask_r2l_sup; Matrix Smask_r2l_swg, Smask_r2l_sup;
    Vector *mask_com_torso;              Matrix Smask_com_torso;

    int *n_r2l_swg, *n_r2l_sup, *n_com_swg, *n_com_sup, *n_com_torso;

    //support phase
    phase current_phase;
    
    //for checking joints within limits
    rangeCheck *rangeCheckTO, *rangeCheckRL, *rangeCheckLL;
    Vector      limitMaskTO,  limitMaskRL,   limitMaskLL;
    
private:
    string robot_name;
    string local_name;
    string wbsName;
    bool springs;
    bool torso;
    bool verbose;

    //PORTS
    //input ports
    BufferedPort<Vector> *EEWRightLeg;        //EE Wrench Right Leg
    BufferedPort<Vector> *EEWLeftLeg;         //EE Wrench Left Leg
    BufferedPort<Vector> *EEWRightAnkle;      //EE Wrench Right Ankle Sensor
    BufferedPort<Vector> *EEWLeftAnkle;       //EE Wrench Left Ankle Sensor

    BufferedPort<Vector> *objPort;
    BufferedPort<Vector> *objPort2;
    BufferedPort<Matrix> *EEPRightLeg;        //EE Pose Right Leg
    BufferedPort<Matrix> *EEPLeftLeg;         //EE Pose Left Leg
    BufferedPort<Vector> *desired_zmp;        //varying set point.
    BufferedPort<Matrix> *COM_Jacob_port;
    BufferedPort<Vector> *COM_Posit_port;
    BufferedPort<Vector> *r2l_err_port;
    BufferedPort<Vector> *COM_err_port;
    BufferedPort<Vector> *ankle_angle;        //Commanded ankle angle
    BufferedPort<Vector> *COM_ref_port;       //COM_ref 
    BufferedPort<Vector> *port_ft_foot_left;  //Left foot f/t sensor reading
    BufferedPort<Vector> *port_ft_foot_right; //Right foot f/t sensor reading

    BufferedPort<Bottle> *port_lr_trf;

    double *angle;

    Matrix rot_f;

    //controller gains
    double vel_sat;
    double Kp_zmp_h, Kp_zmp_x, Kp_zmp_y;
    double Kd_zmp_h, Kd_zmp_x, Kd_zmp_y;
    double Kp, Kd;
    
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

    //Sensors not available in the simulator
    bool Opt_nosens;
    
    //display
    bool Opt_display;

    //Left and Right leg pose.
    Vector PoseLeftLeg;
    Vector PoseRightLeg;

    // To read from Inertial sensor. 
    Vector *inrtl_reading;
    // To read HEAD, RIGHT AND LEFT poses
    Vector *head_pose;

    //Right and left leg and torso encoders
    Vector qRL, qRL_rad;
    Vector qLL, qLL_rad;
    Vector qTO, qTO_rad;
    
    Matrix Jac_FR;             //Jacobian matrix for right FOOT from ROOT.
    iCubLeg *Right_Leg;
    iCubLeg *Left_Leg;
    iCubInertialSensor *Inertia_Sens;

    Matrix Hright;
    Matrix Hleft;

    bool Opt_ankles_sens;
    
    Stamp zmp_time;

    Vector Offset_Lfoot, Offset_Rfoot;
    

public:
    comStepperThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_lefLeg,\
                     Vector q0LL_both, Vector q0RL_both, Vector q0TO_both,\
                     Vector q0LL_right, Vector q0RL_right, Vector q0TO_right,\
                     Vector q0LL_left, Vector q0RL_left, Vector q0TO_left,\
                     string _robot_name, string _local_name, string _wbs_name, bool display, bool noSens, \
                     bool ankles_sens, bool springs, bool torso, bool verbose, Matrix pi_a_t0, double vel_sat, double Kp_zmp_h, double Kd_zmp_h, double Kp_zmp_x, double Kd_zmp_x,\
                     double Kp_zmp_y, double Kd_zmp_y, double Kp, double Kd, string comPosPortName, string comJacPortName, \
                     string r2lErrPortName, string comErrPortName);
    void setRefAcc(IEncoders* iencs, IVelocityControl* ivel);
    bool threadInit();
    void run();
    bool onStop();
    void threadRelease();
    void suspend();
    void computeZMPBoth ();
    void computeZMPLeft ();
    void computeZMPRight();
    bool read_lr_trf_port();
    void jacobianCOMrightSupport(Matrix &jacobianCOMrightSupport, Vector &eCOMrightSupport, Matrix pi_ad, Matrix dpi_ad);
    void jacobianR2LrightSupport(Matrix &jacobianR2LrightSupport, Vector &eR2LrightSupport, Matrix  pac_d, Matrix Rac_d, Matrix dpac_d, Matrix dRac_d);
    void jacobianCOMleftSupport(Matrix &jacobianCOMleftSupport, Vector &eCOMleftSupport, Matrix pi_cd, Matrix dpi_cd);
    void jacobianR2LleftSupport(Matrix &jacobianR2LleftSupport, Vector &eR2LleftSupport, Matrix  pca_d, Matrix Rca_d, Matrix dpca_d, Matrix dRca_d);
    void computeControl(Matrix j1, Matrix j2, Vector e1, Vector e2);
    void computeControlPrioritized(Matrix j1, Matrix j2, Vector e1, Vector e2);
    void computeControlTriangular(Matrix J1, Matrix J2, Vector e1, Vector e2);
    double separation(Vector qRL, Vector qLL);
    void closePort(Contactable *_port);
    void velTranslator(Matrix &out, Matrix p);
    void rotTranslator(Matrix &out, Matrix R);
    void hat(Matrix &p_hat, Matrix p);
    Vector computeDeltaError(Vector q0R, Vector q0L, Vector q1R, Vector q1L);
    Vector computeDeltaProjB(Vector q0R, Vector q0L, Vector q1R, Vector q1L);
    Vector computeDeltaProjA(Vector q0R, Vector q0L, Vector q1R, Vector q1L);
    void computeDeltaProjAReal(Vector dqR, Vector dqL, Vector dqT, Vector &delta_b, Vector &delta_pi_a, Vector &delta_pi_b);
    void computeDeltaProjCReal(Vector dqR, Vector dqL, Vector dqT, Vector &delta_b, Vector &delta_pi_c, Vector &delta_pi_b);
    void switchSupport(phase newPhase);
    void updateComFilters();
    bool check_njTO(int _njTO);
    bool check_njRL(int _njRL);
    bool check_njLL(int _njLL);
    void defineControlMasks(Vector mask_r2l_swg, Vector mask_r2l_sup, Vector mask_com_torso);
};

#endif
