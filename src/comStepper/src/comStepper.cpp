#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/iKin/iKinFwd.h>
#include <math.h>
#include <stdio.h>


#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include "comStepper.h"
#include "util.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

// #define COMPUTE_FINITE_DIFF
// #define DO_NOT_CONTROL_TORSO
// #define DO_NOT_CONTROL_LEGS

void saturateVector(Vector &v, double sat)
{
    double maxV = findMax(v);     double minV = findMin(v);
    
    if (maxV < -sat)
        v = -sat / minV * v;
    
    if (minV < -sat && maxV < sat)
        v = -sat / minV * v;
    
    if (minV < sat && sat < maxV)
        v = sat / maxV * v;
    
    if (sat < minV)
        v = sat / maxV * v;
    
}

comStepperThread::comStepperThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_leftLeg,
                                   Vector _q0LL_both,     Vector _q0RL_both,     Vector _q0TO_both,      Vector _q0LL_right,      Vector _q0RL_right,
                                   Vector _q0TO_right,     Vector _q0LL_left,     Vector _q0RL_left,     Vector _q0TO_left,      string _robot_name,
                                   string _local_name, string _wbs_name, bool display, bool noSens, bool ankles_sens, bool _springs, bool _torso, bool _verbose,
                                   Matrix pi_a_t0, double _vel_sat, double Kp_zmp_h, double Kd_zmp_h, double Kp_zmp_x, double Kd_zmp_x, double Kp_zmp_y,
                                   double Kd_zmp_y, double Kp, double Kd, string comPosPort, string comJacPort, string r2lErrPort, string comErrPort, string comCtrlPort, string zmpPort,
                                   string desiredComPosPort, string desiredR2lPosPort, string desiredComVelPort, string desiredR2lVelPort, string desiredComPhsPort,
                                   string  outputComPosPort, string  outputR2lPosPort, string  outputComVelPort, string  outputR2lVelPort, string  outputComPhsPort) :
RateThread(_rate),   dd_torso(_ddTor),dd_rightLeg(_dd_rightLeg), dd_leftLeg(_dd_leftLeg),
q0LL_both(_q0LL_both), q0RL_both(_q0RL_both), q0TO_both(_q0TO_both), q0LL_right(_q0LL_right),
q0RL_right(_q0RL_right), q0TO_right(_q0TO_right), q0LL_left(_q0LL_left), q0RL_left(_q0RL_left),
q0TO_left(_q0TO_left), robot_name(_robot_name), local_name(_local_name), wbsName(_wbs_name),
springs(_springs), torso(_torso), verbose(_verbose), pi_a_t(pi_a_t0), vel_sat(_vel_sat),
Kp_zmp_h(Kp_zmp_h), Kd_zmp_h(Kd_zmp_h), Kp_zmp_x(Kp_zmp_x), Kd_zmp_x(Kd_zmp_x), Kp_zmp_y(Kp_zmp_y),
Kd_zmp_y(Kd_zmp_y), Kp(Kp), Kd(Kd), comPosPortString(comPosPort), comJacPortString(comJacPort),
r2lErrPortString(r2lErrPort), comErrPortString(comErrPort), comCtrlPortString(comCtrlPort), zmpPortString(zmpPort),
desiredComPosPortString(desiredComPosPort), desiredR2lPosPortString(desiredR2lPosPort), desiredComVelPortString(desiredComVelPort), desiredR2lVelPortString(desiredR2lVelPort), desiredComPhsPortString(desiredComPhsPort),
outputComPosPortString( outputComPosPort),  outputR2lPosPortString( outputR2lPosPort),  outputComVelPortString( outputComVelPort),  outputR2lVelPortString( outputR2lVelPort),  outputComPhsPortString( outputComPhsPort)
{
    //------------------ INTERFACE INITIALIZATION ----------------------------------
    printf("rate: %d\n",_rate);
    
    Ilim_TO     = 0;
    Ilim_RL     = 0;
    Ilim_LL     = 0;
    Ipos_TO     = 0;
    Ipos_RL     = 0;
    Ipos_LL     = 0;
    Ictrl_TO    = 0;
    Ictrl_LL    = 0;
    Ictrl_RL    = 0;
    Ivel_TO     = 0;
    Ivel_LL     = 0;
    Ivel_RL     = 0;
    Ienc_TO     = 0;
    Ienc_LL     = 0;
    Ienc_RL     = 0;
    Ipid_TO     = 0;
    Ipid_LL     = 0;
    Ipid_RL     = 0;
    Kr2l        = zeros(6, 6);   Kr2l(0,0) = Kp;         Kr2l(1,1) = Kp;        Kr2l(2,2) = Kp;         Kr2l(3,3) = Kd;    Kr2l(4,4) = Kd;    Kr2l(5,5) = Kd;
    Kcom        = zeros(3, 3);   Kcom(0,0) = Kp_zmp_h;   Kcom(1,1) = Kp_zmp_x;   Kcom(2,2) = Kp_zmp_y;
    
    n_r2l_swg   = new int;    n_r2l_sup   = new int;    n_com_swg   = new int;
    n_com_sup   = new int;    n_com_torso = new int;
    
    Opt_display = display;
    Opt_nosens = noSens;
    Opt_ankles_sens = ankles_sens;
    
    zmp_xy_vel_estimator = new iCub::ctrl::AWLinEstimator(1,0.0);
    
    port_lr_trf = new BufferedPort<Bottle>;
    port_lr_trf->open(string("/"+local_name+"/lrTransform:i").c_str());
    
    //---------------------------- PORTS ---------------------------------------------
    if (!noSens)
    {
        EEWRightLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
        EEWRightLeg->open(string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/right_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str(),"tcp",false);
        
        EEWLeftLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
        EEWLeftLeg->open(string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/left_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str(),"tcp",false);
        
        EEWRightAnkle = new BufferedPort<Vector>;
        EEWRightAnkle->open(string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/right_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());
        
        EEWLeftAnkle = new BufferedPort<Vector>;
        EEWLeftAnkle->open(string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/left_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());
        
        objPort = new BufferedPort<Vector>;
        objPort->open(string("/"+local_name+"/DSPzmp:o").c_str());
        Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/myiCubGui/objects").c_str());
        
        objPort2 = new BufferedPort<Vector>;
        objPort2->open(string("/"+local_name+"/DSPzmp2iCubGui:o").c_str());
        
        //system output
        desired_zmp = new BufferedPort<Vector>;
        desired_zmp->open(string("/"+local_name+"/desired_zmp:o").c_str());
        
        //Connecting output of BalancerModule's zmp TO velocityObserver input port to get derivatives.
        Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/zmpVel/pos:i").c_str());
        
        //COM Jacobian
        //COM_Jacob_port = new BufferedPort<Matrix>;
        //COM_Jacob_port->open(string("/"+local_name+"/COM_Jacob_port:i").c_str());
        //Network::connect(string(wbsName+"/com_jacobian:o").c_str(),string("/"+local_name+"/COM_Jacob_port:i").c_str());
        
        port_ft_foot_left = new BufferedPort<Vector>;
        port_ft_foot_left->open(string("/"+local_name+"/left_foot/FT:i").c_str());
        Network::connect(string("/"+robot_name+"/left_foot/analog:o").c_str(), string("/"+local_name+"/left_foot/FT:i").c_str(), "tcp", false);
        
        port_ft_foot_right = new BufferedPort<Vector>;
        port_ft_foot_right->open(string("/"+local_name+"/right_foot/FT:i").c_str());
        Network::connect(string("/"+robot_name+"/right_foot/analog:o").c_str(), string("/"+local_name+"/right_foot/FT:i").c_str(),"tcp",false);
        
    }
    
    COM_ref_port = new BufferedPort<Vector>;
    COM_ref_port->open(string("/"+local_name+"/com_ref:o").c_str());
    
    ankle_angle = new BufferedPort<Vector>;
    ankle_angle->open(string("/"+local_name+"/commanded_ankle_ang:o").c_str());
    
    if (!comPosPort.empty())
    {
        fprintf(stderr, "Opening the port the COM position input!\n");
        COM_Posit_port = new BufferedPort<Vector>;
        COM_Posit_port->open(comPosPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port the COM position input!\n");
    
    if (!comJacPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM Jacobian input!\n");
        COM_Jacob_port = new BufferedPort<Matrix>;
        COM_Jacob_port->open(comJacPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port the COM Jacobian input!\n");
    
    
    if (!r2lErrPort.empty())
    {
        fprintf(stderr, "Opening the port the right/left leg trcking-error port!\n");
        r2l_err_port = new BufferedPort<Vector>;
        r2l_err_port->open(r2lErrPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port the right/left leg tracking-error port!\n");
    
    if (!comErrPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM error!\n");
        COM_err_port = new BufferedPort<Vector>;
        COM_err_port->open(comErrPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM error!\n");
    
    if (!comCtrlPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM error!\n");
        COM_ctrl_port = new BufferedPort<Vector>;
        COM_ctrl_port->open(comCtrlPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM controls!\n");
    
    if (!zmpPort.empty())
    {
        fprintf(stderr, "Opening a port for the ZMP!\n");
        zmp_port = new BufferedPort<Vector>;
        zmp_port->open(zmpPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM controls!\n");

    //desired ports
    if (!desiredComPosPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM pos desired port!\n");
        com_des_pos_port = new BufferedPort<Vector>;
        com_des_pos_port->open(desiredComPosPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM pos desired port!!\n");
    
    if (!desiredComVelPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM vel desired port!\n");
        com_des_vel_port = new BufferedPort<Vector>;
        com_des_vel_port->open(desiredComVelPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM vel desired port!!\n");

    if (!desiredComPhsPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM phs desired port!\n");
        com_des_phs_port = new BufferedPort<Bottle>;
        com_des_phs_port->open(desiredComPhsPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM pos desired port!!\n");
    
    if (!desiredR2lPosPort.empty())
    {
        fprintf(stderr, "Opening a port for the R2L pos desired port!\n");
        r2l_des_pos_port = new BufferedPort<Vector>;
        r2l_des_pos_port->open(desiredR2lPosPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the R2L pos desired port!!\n");
    
    if (!desiredR2lVelPort.empty())
    {
        fprintf(stderr, "Opening a port for the R2L vel desired port!\n");
        r2l_des_vel_port = new BufferedPort<Vector>;
        r2l_des_vel_port->open(desiredR2lVelPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the R2L vel desired port!!\n");

    //output ports
    if (!outputComPosPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM pos output port!\n");
        com_out_pos_port = new BufferedPort<Vector>;
        com_out_pos_port->open(outputComPosPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM pos output port!!\n");
    
    if (!outputComVelPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM vel output port!\n");
        com_out_vel_port = new BufferedPort<Vector>;
        com_out_vel_port->open(outputComVelPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM vel output port!!\n");
    
    if (!outputComPhsPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM phs output port!\n");
        com_out_phs_port = new BufferedPort<Bottle>;
        com_out_phs_port->open(outputComPhsPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the COM pos output port!!\n");
    
    if (!outputR2lPosPort.empty())
    {
        fprintf(stderr, "Opening a port for the R2L pos output port!\n");
        r2l_out_pos_port = new BufferedPort<Vector>;
        r2l_out_pos_port->open(outputR2lPosPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the R2L pos output port!!\n");
    
    if (!outputR2lVelPort.empty())
    {
        fprintf(stderr, "Opening a port for the R2L vel output port!\n");
        r2l_out_vel_port = new BufferedPort<Vector>;
        r2l_out_vel_port->open(outputR2lVelPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port for the R2L vel output port!!\n");
    
    
    Right_Leg    = new iCubLeg("right");
    Left_Leg     = new iCubLeg("left");
    Inertia_Sens = new  iCubInertialSensor("v2");
}

bool comStepperThread::threadInit()
{
    on_ground = true;
    
    fprintf(stderr, "STEP1: initializing the device drivers \n");
    F_ext_LL = new Vector(6);        F_ext_LL0 = new Vector(6);     *F_ext_LL0 = zeros(6);
    F_ext_RL = new Vector(6);        F_ext_RL0 = new Vector(6);     *F_ext_RL0 = zeros(6);
    
    //POLIDRIVERS AND INTERFACES
    
    // Torso Interface
    dd_torso->view(Ienc_TO);
    dd_torso->view(Ictrl_TO);
    dd_torso->view(Ivel_TO);
    dd_torso->view(Ipid_TO);
    dd_torso->view(Ipos_TO);
    dd_torso->view(Ilim_TO);
    if((!dd_torso) || (!Ienc_TO) || (!Ictrl_TO) || (!Ivel_TO) || (!Ipid_TO) || (!Ipos_TO) || (!Ilim_TO))
    {
        printf("ERROR acquiring torso interfaces\n");
        return false;
    }
    
    //Right leg interfaces
    dd_rightLeg->view(Ienc_RL);
    dd_rightLeg->view(Ictrl_RL);
    dd_rightLeg->view(Ivel_RL);
    dd_rightLeg->view(Ipid_RL);
    dd_rightLeg->view(Ipos_RL);
    dd_rightLeg->view(Ilim_RL);
    if((!dd_rightLeg) || (!Ienc_RL) || (!Ictrl_RL) || (!Ivel_RL) || (!Ipid_RL) || (!Ipos_RL) || (!Ilim_RL))
    {
        printf("ERROR acquiring right leg interfaces\n");
        return false;
    }
    
    
    //Left leg interfaces
    dd_leftLeg->view(Ienc_LL);
    dd_leftLeg->view(Ictrl_LL);
    dd_leftLeg->view(Ivel_LL);
    dd_leftLeg->view(Ipid_LL);
    dd_leftLeg->view(Ipid_LL);
    dd_leftLeg->view(Ipos_LL);
    dd_leftLeg->view(Ilim_LL);
    if((!dd_leftLeg) || (!Ienc_LL) || (!Ictrl_LL) || (!Ivel_LL) || (!Ipid_LL) || (!Ipos_LL) || (!Ilim_LL))
    {
        printf("ERROR acquiring left leg interfaces\n");
        return false;
    }
    
    fprintf(stderr, "STEP2: moving torso to a given configuration \n");
    //FOR Sending commands to the TORSO
    Ienc_TO->getAxes(&njTO);
    
    //FOR sending commands to the legs
    Ienc_LL->getAxes(&njLL); //Legs have 6 joints from 0 to 5.
    
    Ienc_RL->getAxes(&njRL);
    
    //Setting Reference Accelerations
#ifndef DO_NOT_CONTROL_TORSO
    setRefAcc(Ienc_TO, Ivel_TO);
#endif
    
#ifndef DO_NOT_CONTROL_LEGS
    setRefAcc(Ienc_RL, Ivel_RL);
    setRefAcc(Ienc_LL, Ivel_LL);
#endif
    
    //Initializing the ranges
    Vector qMinTO = zeros(njTO); Vector qMaxTO = zeros(njTO);
    for(int i = 0; i < njTO; i++)
        Ilim_TO->getLimits(i, (qMinTO.data()+ i), (qMaxTO.data()+ i));
    
    Vector qMinRL = zeros(njRL); Vector qMaxRL = zeros(njRL);
    for(int i = 0; i < njRL; i++)
        Ilim_RL->getLimits(i, (qMinRL.data()+ i), (qMaxRL.data()+ i));
    
    Vector qMinLL = zeros(njLL); Vector qMaxLL = zeros(njLL);
    for(int i = 0; i < njLL; i++)
        Ilim_LL->getLimits(i, (qMinLL.data()+ i), (qMaxLL.data()+ i));
    
    rangeCheckTO = new rangeCheck(qMinTO, qMaxTO, 0.95);  limitMaskTO = zeros(njTO);
    rangeCheckLL = new rangeCheck(qMinLL, qMaxLL, 0.95);  limitMaskLL = zeros(njLL);
    rangeCheckRL = new rangeCheck(qMinRL, qMaxRL, 0.95);  limitMaskRL = zeros(njRL);
    
    //INITIALIZING thread variables.
    rot_f = zeros(3,3);
    
    // rot_f(0,0) = rot_f(1,1) = -1;
    // rot_f(2,2) = 1;
    rot_f(1,1) = -1;
    rot_f(2,0) = rot_f(0,2) = 1;
    
    //matrix from sensor to foot
    Ras.resize(3, 3);   Rcs.resize(3, 3);
    Ras(0,0) = 0.0;     Ras(0,1) = 0.0;     Ras(0,2) =-1.0;
    Ras(1,0) = 0.0;     Ras(1,1) = 1.0;     Ras(1,2) = 0.0;
    Ras(2,0) = 1.0;     Ras(2,1) = 0.0;     Ras(2,2) = 0.0;
    
    Rcs(0,0) = 0.0;     Rcs(0,1) = 0.0;     Rcs(0,2) =-1.0;
    Rcs(1,0) = 0.0;     Rcs(1,1) = 1.0;     Rcs(1,2) = 0.0;
    Rcs(2,0) = 1.0;     Rcs(2,1) = 0.0;     Rcs(2,2) = 0.0;
    
    fprintf(stderr, "STEP3: inizializing the ZMP \n");
    //INITIALIZING zmp vector
    zmp_xy = zeros(2);
    
    //INITIALIZING zmpXd desired
    zmp_des.resize(2);
    zmp_des[0] = 0.08;
    zmp_des[1] = 0.0;
    
    zmp_xy_vel.resize(2);
    zmp_xy_vel[0] = 0.0;
    zmp_xy_vel[1] = 0.0;
    
    //FOR SAFETY
    if(Kd_zmp_x>0.025){
        Kd_zmp_x=0.025;
    }
    if(Kp_zmp_x>0.9){
        Kp_zmp_x=0.9;
    }
    if(Kp>100.0){
        Kp=100;
    }
    if(Kd>2.5){
        Kd=2.5;
    }
    
    //Initialization of the ZMP variables
    zmp_a = zeros(3,1);  zmpLL_a = zeros(3,1);    zmpRL_a = zeros(3,1);
    zmp_c = zeros(3,1);  zmpLL_c = zeros(3,1);    zmpRL_c = zeros(3,1);
    f_a   = zeros(3,1);  m_a     = zeros(3,1);    Sf_a    = zeros(3,3);
    f_c   = zeros(3,1);  m_c     = zeros(3,1);    Sf_c    = zeros(3,3);
    
    qRL.resize(njRL);      qLL.resize(njLL);      qTO.resize(njTO);         q.resize(njRL + njLL + njTO);
    qRL_rad.resize(njRL);  qLL_rad.resize(njLL);  qTO_rad.resize(njTO); q_rad.resize(njRL + njLL + njTO);
    
    Jac_FR.resize(6,32);
    Jac_FR.zero();
    
    Hright.resize(4,4);
    Hright.zero();
    Hright(0,0) = Hright(1,2) = Hright(3,3) = 1;
    Hright(2,1) = -1; Hright(1,3) = 0.0681; Hright(2,3) = -0.1199;
    
    Hleft.resize(4,4);
    Hleft.zero();
    Hleft(0,0) = Hleft(1,2) = Hleft(3,3) = 1;
    Hleft(2,1) = -1; Hleft(1,3) = -0.0681; Hleft(2,3) = -0.1199;
    
    fprintf(stderr, "STEP4: inizializing the desired movements \n");
#ifdef FOR_PLOTS_ONLY
    if (!Opt_nosens)
    {
        Vector* tmp;
        tmp = port_ft_foot_right->read(true);
        Offset_Rfoot = *tmp;
        tmp =  port_ft_foot_left->read(true);
        Offset_Lfoot = *tmp;
    }
    else
    {
        Offset_Lfoot.resize(6);
        Offset_Lfoot.zero();
        Offset_Rfoot.resize(6);
        Offset_Rfoot.zero();
    }
#endif
    //fprintf(stderr, "Offsets are: %s and %s \n", Offset_Rfoot.toString().c_str(), Offset_Lfoot.toString().c_str());
    
    // Setting initial configuration of entire robot.
    Vector tmp2;
    tmp2.resize(njRL);
    for(int i=0; i<njRL;i++){
        tmp2[i] = 10.0;
    }
    
    
#ifndef DO_NOT_CONTROL_LEGS
    Ipos_RL->setRefSpeeds(tmp2.data());
    Ipos_LL->setRefSpeeds(tmp2.data());
    
    if (!comJacPortString.empty())
    {
        
        bool ok;
        ok = Ipos_RL->positionMove(q0RL_both.data());
        if(ok)
            fprintf(stderr, "Right Leg set to initial position...%s\n", q0RL_both.toString().c_str());
        else
            return false;
        
        ok = Ipos_LL->positionMove(q0LL_both.data());
        if(ok)
            fprintf(stderr, "Left Leg set to initial position...%s\n", q0LL_both.toString().c_str());
        else
            return false;
    }
#endif
    
#ifndef DO_NOT_CONTROL_TORSO
    if (!comJacPortString.empty())
    {
        bool ok;
        ok = Ipos_TO->positionMove(q0TO_both.data());
        if(ok)
            printf("Torso set to initial position...%s\n", q0TO_both.toString().c_str());
        else
            return false;
    }
#endif
    
    //resizing matrices involved in computing Jca
    dqTO.resize(njTO);   dqLL.resize(njLL);   dqRL.resize(njRL);  dq.resize(njRL + njLL +njTO);
    Tba.resize(4,4)  ;   Tbc.resize(4,4)  ;   Tab.resize(4,4)  ;  Tac.resize(4,4);
    Rba.resize(3,3)  ;   Rbc.resize(3,3)  ;   Rac.resize(3,3)  ;
    pba.resize(3,1)  ;   pbc.resize(3,1)  ;   pac.resize(3,1)  ;
    pab.resize(3,1)  ;   pcb.resize(3,1)  ;   rca_b.resize(3,1);
    
    Jba.resize(6, njRL); Jbc.resize(6, njLL); Jac.resize(6, njLL+njRL);
    Spac.resize(6,6); Srca.resize(6,6); Rrab.resize(6,6);
    
    //resizing matrices involved in computing Jac
    Tca.resize(4,4);   Rca.resize(3,3);   pca.resize(3,1);  rac_b.resize(3,1);
    
    Jca.resize(6, njLL+njRL);
    Spca.resize(6,6); Srac.resize(6,6); Rrcb.resize(6,6);
    
    //resizing matrices involved in the computation of eac
    pac_d.resize(3, 1);   Rac_d.resize(3, 3);   pac_t.resize(3, 3);
    dpac_d.resize(3, 1);  dRac_d.resize(3, 1);
    
    Vector qRef = q0LL_both*CTRL_DEG2RAD;
    Matrix Tac0 = SE3inv(Right_Leg->getH(qRef)) * Left_Leg->getH(qRef);
    Rac_d = Tac0.submatrix(0, 2, 0, 2);
    pac_d = Tac0.submatrix(0, 2, 3, 3);
    pac_t = Tac0.submatrix(0, 2, 3, 3);
    
    nd.resize(3, 1);     sd.resize(3, 1);     ad.resize(3, 1);
    ne.resize(3, 1);     se.resize(3, 1);     ae.resize(3, 1);
    nd_hat.resize(3, 3); sd_hat.resize(3, 3); ad_hat.resize(3, 3);
    ne_hat.resize(3, 3); se_hat.resize(3, 3); ae_hat.resize(3, 3);
    
    ep.resize(3, 1); eo.resize(3, 1);
    L.resize(3, 3);
    
    //resizing matrices involved in the computation of eca
    pca_d.resize(3, 1);   Rca_d.resize(3, 3);    pca_t.resize(3, 1);
    dpca_d.resize(3, 1);  dRca_d.resize(3, 1);
    
    qRef = q0LL_both*CTRL_DEG2RAD;
    Matrix Tca0 = SE3inv(Left_Leg->getH(qRef)) * Right_Leg->getH(qRef);
    Rca_d = Tca0.submatrix(0, 2, 0, 2);
    pca_d = Tca0.submatrix(0, 2, 3, 3);
    pca_t = Tca0.submatrix(0, 2, 3, 3);
    
    //resizing matrices for the computation of the center of mass projection
    PI = eye(3)                ; PI(0,0) = 0.0              ; Jb_p.resize(3, Jba.cols());
    p_b.resize(3,1)            ; pi_a.resize(3,1)           ;
    
    Jpi_b.resize(3, Jba.cols());  Jpi_b.resize(3, Jbc.cols());
    
    //resizing matrices for the computation of the center of mass projection
    p_b.resize(3,1)            ; pi_c.resize(3,1)           ;
    pi_c_t.resize(3,1)         ; Jpi_b.resize(3, Jbc.cols());
    
    //initial support is double
    current_phase = BOTH_SUPPORT;
    qrLL = q0LL_both;    qrRL = q0RL_both;    qrTO = q0TO_both;
    
    //initializing filter for the COM
    pi_a_d = zeros(3, 1);        dpi_a_d = zeros(3, 1);
    pi_c_d = zeros(3, 1);        dpi_c_d = zeros(3, 1);
    
    //minimum jerk filters for the COM
    Vector iniX(1);     iniX(0) = pi_a_t(0,0);
    Vector iniY(1);     iniY(0) = pi_a_t(1,0);
    Vector iniZ(1);     iniZ(0) = pi_a_t(2,0);
    
    comMinJerkX = new minJerkTrajGen(iniX, getRate()/1000.0, MIN_JERK_COM_TIME);
    comMinJerkY = new minJerkTrajGen(iniY, getRate()/1000.0, MIN_JERK_COM_TIME);
    comMinJerkZ = new minJerkTrajGen(iniZ, getRate()/1000.0, MIN_JERK_COM_TIME);

    //low pass ZMP filter
    if (!strcmp(robot_name.c_str(), "icub") && on_ground && !Opt_nosens)
    {
        updateRotations();
        updateForceTorque();
        computeZMPBoth();
        inputFilter = new FirstOrderLowPassFilter(0.5, getRate()/1000.0, zmp_a.getCol(0));
    }
    
    //minimum jerk filters for the R2L
    iniX(1);     iniX(0) = pac_t(0,0);
    iniY(1);     iniY(0) = pac_t(1,0);
    iniZ(1);     iniZ(0) = pac_t(2,0);
    
    r2lMinJerkX = new minJerkTrajGen(iniX, getRate()/1000.0, MIN_JERK_R2L_TIME);
    r2lMinJerkY = new minJerkTrajGen(iniY, getRate()/1000.0, MIN_JERK_R2L_TIME);
    r2lMinJerkZ = new minJerkTrajGen(iniZ, getRate()/1000.0, MIN_JERK_R2L_TIME);
    
    
    //resizing masks vectors for the controllers
    mask_r2l_swg   = new Vector(njRL);      mask_r2l_sup = new Vector(njRL);
    mask_com_torso = new Vector(njTO);
    
    //resizing matrices given the dimensions of the com_jac read from port
    if (!comPosPortString.empty())
    {
        fprintf(stderr, "Waiting for incoming COM position from port %s...\n", comPosPortString.c_str());
        Vector comPosition = *COM_Posit_port->read();
    }
    
    if (!comJacPortString.empty())
    {
        fprintf(stderr, "Waiting for incoming COM jacobian from port %s...\n", comJacPortString.c_str());
        Jb_com = (*COM_Jacob_port->read());
        Jb_com = Jb_com.removeRows(3, 3);
        
        njHD = 3;
        njRA = (Jb_com.cols() - njTO - njRL -njLL -njHD)/2;
        njLA = (Jb_com.cols() - njTO - njRL -njLL -njHD)/2;
        Jpi_b_com.resize(3, Jb_com.cols());
        
    }
    
    return true;
}

void comStepperThread::updateForceTorque()
{
    //***************************** Reading F/T measurements and encoders ****************
    
    if (!Opt_nosens)
    {
        F_ext_RL = port_ft_foot_right->read(true);
        F_ext_LL = port_ft_foot_left->read(true);
        
        *F_ext_RL = *F_ext_RL - *F_ext_RL0;
        *F_ext_LL = *F_ext_LL - *F_ext_LL0;
        
        // fprintf(stderr, "Right forces in double support are %s\n", (* F_ext_RL).toString().c_str());
        // fprintf(stderr, "Left  forces in double support are %s\n", (* F_ext_LL).toString().c_str());
    }
    
    // check if the robot is not in contact with the ground
    if (!Opt_nosens)
    {
        if ((*F_ext_LL)[2] < -50.0 || (*F_ext_RL)[2] < -50.0  )
            on_ground = true;
        else
            on_ground = false;
    }
    
}

void comStepperThread::updateRotations()
{
    //********************** COMPUTE JACOBIANS ************************************
    // In this notation we define a,b,c as follows:
    // right_foot_reference_frame = 'a';
    // base_reference_frame       = 'b';
    // left_foot_reference_frame  = 'c';

    Ienc_RL->getEncoders(qRL.data());      qRL_rad = CTRL_DEG2RAD * qRL;       Right_Leg->setAng(qRL_rad);
    Ienc_LL->getEncoders(qLL.data());      qLL_rad = CTRL_DEG2RAD * qLL;       Left_Leg->setAng(qLL_rad);
    Ienc_TO->getEncoders(qTO.data());      qTO_rad = CTRL_DEG2RAD * qTO;
    
    q.setSubvector(   0     , qLL);    q_rad.setSubvector(   0     , qLL_rad);
    q.setSubvector(njLL     , qRL);    q_rad.setSubvector(njLL     , qRL_rad);
    q.setSubvector(njLL+njRL, qTO);    q_rad.setSubvector(njLL+njRL, qTO_rad);
    
    rangeCheckLL->isAtBoundaries(qLL, limitMaskLL);
    rangeCheckRL->isAtBoundaries(qRL, limitMaskRL);
    rangeCheckTO->isAtBoundaries(qTO, limitMaskTO);
    
    Tba = Right_Leg->getH();
    Tbc =  Left_Leg->getH();
    
    Tab = iCub::ctrl::SE3inv(Tba);
    Tac = Tab * Tbc;
    Tca = iCub::ctrl::SE3inv(Tac);
    
    Jba = Right_Leg->GeoJacobian();
    Jbc = Left_Leg->GeoJacobian();
    
    pba = Tba.submatrix(0, 2, 3, 3);
    pbc = Tbc.submatrix(0, 2, 3, 3);
    pac = Tac.submatrix(0, 2, 3, 3);
    pca = Tca.submatrix(0, 2, 3, 3);
    
    Rba = Tba.submatrix(0, 2, 0, 2);
    Rbc = Tbc.submatrix(0, 2, 0, 2);
    Rac = Tac.submatrix(0, 2, 0, 2);
    Rca = Tca.submatrix(0, 2, 0, 2);
    
    pab =   -1.0 * Rba.transposed() * pba;
    pcb =   -1.0 * Rbc.transposed() * pbc;
    rca_b = -1.0 * Rba * pac;
}

void comStepperThread::run()
{
    //updating Time Stamp
    static Stamp timeStamp;
    timeStamp.update();
    
    //upate the desired COM
    updateComFilters();
    
    //upate the desired COM and R2L pos, vel
    updateComDesired();
    
    //update rotation matrices
    updateRotations();
    
    if (comPosPortString.empty())
        p_b  = zeros(3,1);
    else
        p_b.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    
    if (comJacPortString.empty())
        Jb_p = zeros(3, Jba.cols());
    else
    {
        Jb_com   = (*COM_Jacob_port->read()).removeRows(3, 3);
        Jb_p     = Jb_com.submatrix(0, 2, njLL, njLL+njRL-1);
    }
    
    pi_a  =       PI * (Rba.transposed() * p_b + pab);
    pi_c  =       PI * (Rbc.transposed() * p_b + pcb);
    
    //***************************** Reading F/T measurements and encoders ****************
    updateForceTorque();
    
    //***************************** Computing ZMP ****************************************
    switch (current_phase)
    {
        case LEFT_SUPPORT:
            if (on_ground && !Opt_nosens)
                computeZMPBoth();
            break;
            
        case RIGHT_SUPPORT:
            if (on_ground && !Opt_nosens)
                computeZMPBoth();
            break;
            
        case BOTH_SUPPORT:
            if (on_ground && !Opt_nosens)
            {
                computeZMPBoth();
            }
            break;
    }
    if (!strcmp(robot_name.c_str(), "icub") && on_ground && !Opt_nosens)
    {
        zmp_a.setCol(0, inputFilter->filt(zmp_a.getCol(0)));
        zmp_c.setCol(0, inputFilter->filt(zmp_c.getCol(0)));
    }
    
    if(verbose){
        fprintf(stderr, "ZMP coordinates: %f %f       %d \n",zmp_xy[0],zmp_xy[1],(int)(torso));
        fprintf(stderr, "ZMP desired: %f %f\n",zmp_des[0], zmp_des[1]);
    }
    
    //*********************** SENDING ZMP COORDINATES TO GuiBalancer *********************
    if (!Opt_nosens)
        if (objPort->getOutputCount() > 0 && !Opt_nosens)
        {
            objPort->prepare() = zmp_xy;
            objPort->setEnvelope(timeStamp);
            objPort->write();
        }
    
    //********************** SENDING ZMP COORDS TO BE PLOT BY ICUBGUI *****************************
    if (!Opt_nosens)
        if (objPort2->getOutputCount() > 0)
        {
            Matrix Trans_lastRot(4,4); Trans_lastRot.zero();
            Trans_lastRot.setSubmatrix(rot_f,0,0);
            Trans_lastRot(3,3) = 1;
            Trans_lastRot(1,3) = -separation(qRL,qLL)/2;
            Vector zmp_xy_trans(4); zmp_xy_trans.zero();
            zmp_xy_trans.setSubvector(0,zmp_xy);
            zmp_xy_trans(3)=1;
            zmp_xy_trans = Hright*(Right_Leg->getH())*SE3inv(Trans_lastRot)*zmp_xy_trans;
            objPort2->prepare() = zmp_xy_trans.subVector(0,1);
            objPort2->write();
        }
    //*********************** Reading ZMP derivative **********************************************
    
    iCub::ctrl::AWPolyElement el;
    el.data=zmp_xy;
    el.time=Time::now();
    zmp_xy_vel = zmp_xy_vel_estimator->estimate(el);
    
    
    //********************** MAIN CONTROL LOOPS ************************************
    Matrix jCOM, jR2L;
    Vector eCOM, eR2L;
    
    if (!comJacPortString.empty())
    {
        switch (current_phase)
        {
            case LEFT_SUPPORT:
            {
                jacobianR2LleftSupport(jR2L, eR2L, pca_d, Rca_d, dpca_d, dRca_d);
                jacobianCOMleftSupport(jCOM, eCOM, pi_c_d, dpi_c_d);
            }
                break;
                
            case RIGHT_SUPPORT:
            {
                jacobianR2LrightSupport(jR2L, eR2L, pac_d, Rac_d, dpac_d, dRac_d);
                jacobianCOMrightSupport(jCOM, eCOM, pi_a_d, dpi_a_d);
            }
                break;
                
            case BOTH_SUPPORT:
            {
                jacobianR2LrightSupport(jR2L, eR2L, pac_d, Rac_d, dpac_d, dRac_d);
                jacobianCOMrightSupport(jCOM, eCOM, pi_a_d, dpi_a_d);
            }
                break;
        }
        //dq = computeControl(jCOM, jR2L, eCOM, eR2L);
        dq = computeControlPrioritized(jCOM, jR2L, eCOM, eR2L);
        executeControl(dq);
    }
    
    //********************** PORT OUTPUTS ************************************
    
    if(!r2lErrPortString.empty())
    {
        Vector ep_eo(ep.rows()+eo.rows());
        ep_eo.setSubvector(0, ep.getCol(0));
        ep_eo.setSubvector(ep.rows(), eo.getCol(0));
        r2l_err_port->prepare()=ep_eo;
        r2l_err_port->setEnvelope(timeStamp);
        r2l_err_port->write();
    }
    
    if(!comErrPortString.empty())
    {
        if (!strcmp(robot_name.c_str(), "icubSim"))
        {
            if (current_phase == BOTH_SUPPORT || current_phase == RIGHT_SUPPORT)
                COM_err_port->prepare()=pi_a_d.getCol(0) - pi_a.getCol(0);
            else
                COM_err_port->prepare()=pi_c_d.getCol(0) - pi_c.getCol(0);
        }
        else
        {
            if (current_phase == BOTH_SUPPORT || current_phase == RIGHT_SUPPORT)
                COM_err_port->prepare()=pi_a_d.getCol(0) - zmp_a.getCol(0);
            else
                COM_err_port->prepare()=pi_c_d.getCol(0) - zmp_c.getCol(0);
        }
        COM_err_port->setEnvelope(timeStamp);
        COM_err_port->write();
    }

    if(!comCtrlPortString.empty())
    {
        COM_ctrl_port->prepare()= dq;
        COM_ctrl_port->setEnvelope(timeStamp);
        COM_ctrl_port->write();
    }

    if(!zmpPortString.empty())
    {
        if (current_phase==RIGHT_SUPPORT || current_phase==BOTH_SUPPORT)
        {
            if (!strcmp(robot_name.c_str(), "icubSim"))
                zmp_port->prepare()= pi_a.getCol(0);
            else
                zmp_port->prepare()= zmp_a.getCol(0);
        }
        else
        {
            if (!strcmp(robot_name.c_str(), "icubSim"))
                zmp_port->prepare()= pi_c.getCol(0);
            else
                zmp_port->prepare()= zmp_c.getCol(0);
        }
        zmp_port->setEnvelope(timeStamp);
        zmp_port->write();
    }
    
    if(!outputComPosPortString.empty())
    {
        if (current_phase==RIGHT_SUPPORT || current_phase==BOTH_SUPPORT)
            com_out_pos_port->prepare() = pi_a_d.getCol(0);
        else
            com_out_pos_port->prepare() = pi_c_d.getCol(0);
        
        com_out_pos_port->setEnvelope(timeStamp);
        com_out_pos_port->write();
    }

    if(!outputComVelPortString.empty())
    {
        if (current_phase==RIGHT_SUPPORT || current_phase==BOTH_SUPPORT)
            com_out_vel_port->prepare() = dpi_a_d.getCol(0);
        else
            com_out_vel_port->prepare() = dpi_c_d.getCol(0);
        
        com_out_vel_port->setEnvelope(timeStamp);
        com_out_vel_port->write();
    }

    if(!outputComPhsPortString.empty())
    {
        Bottle& b = com_out_phs_port->prepare();
        b.clear();
        if (current_phase==RIGHT_SUPPORT)
            b.add("right");
        if (current_phase==LEFT_SUPPORT)
            b.add("left");
        if (current_phase==BOTH_SUPPORT)
            b.add("double");
        
        com_out_phs_port->setEnvelope(timeStamp);
        com_out_phs_port->write();
    }
    
    
    if(!outputR2lPosPortString.empty())
    {
        if (current_phase==RIGHT_SUPPORT || current_phase==BOTH_SUPPORT)
            r2l_out_pos_port->prepare() = pac_d.getCol(0);
        else
            r2l_out_pos_port->prepare() = pac_d.getCol(0);
        
        r2l_out_pos_port->setEnvelope(timeStamp);
        r2l_out_pos_port->write();
    }
    
    if(!outputR2lVelPortString.empty())
    {
        if (current_phase==RIGHT_SUPPORT || current_phase==BOTH_SUPPORT)
            r2l_out_vel_port->prepare() = dpac_d.getCol(0);
        else
            r2l_out_vel_port->prepare() = dpac_d.getCol(0);
        
        r2l_out_vel_port->setEnvelope(timeStamp);
        r2l_out_vel_port->write();
    }
    
}

void comStepperThread::setRefAcc(IEncoders* iencs, IVelocityControl* ivel)
{
    Vector tmp;  int nj;
    iencs->getAxes(&nj);
    
    tmp.resize(nj);
    int i;
    for (i=0;i<=nj;i++){
        tmp[i]=10000;
    }
    ivel->setRefAccelerations(tmp.data());
}

Vector comStepperThread::computeControl(Matrix J1, Matrix J2, Vector e1, Vector e2)
{
    int njTot = njLL + njRL + njTO;
    Vector dq(njTot, 0.0), eq(njTot, 0.0);  // dq: desired joint vel; eq: joint space position error
    
    Matrix J = pile(J1, J2); //(J1.rows() + J2.rows(), J1.cols());
    Vector e = cat(e1, e2); //(e1.size() + e2.size());
    //J.setSubmatrix(J1, 0, 0);    J.setSubmatrix(J2, J1.rows(), 0);
    //e.setSubvector(0, e1)   ;    e.setSubvector(e1.size(), e2)   ;
    
    Matrix Smask(njTot, njTot);
    if (current_phase == RIGHT_SUPPORT || current_phase == BOTH_SUPPORT)
    {
        Smask.setSubmatrix(Smask_r2l_swg, 0, 0);
        Smask.setSubmatrix(Smask_r2l_sup, njLL, njLL);
    }
    else
    {
        Smask.setSubmatrix(Smask_r2l_sup, 0, 0);
        Smask.setSubmatrix(Smask_r2l_swg, njLL, njLL);
    }
    Smask.setSubmatrix(Smask_com_torso, njLL+njRL, njLL+njRL);
    
    for (int i = 0; i < njLL; i++)
        if (Smask(i, i) == 0)
            eq(i) = qrLL(i) - qLL(i);
    
    for (int i = 0; i < njRL; i++)
        if (Smask(njLL+i, njLL+i) == 0)
            eq(njLL+i) = qrRL(i) - qRL(i);
    
    for (int i = 0; i < njTO; i++)
        if (Smask(njLL+njRL+i, njLL+njRL+i) == 0)
            eq(njLL+njRL+i) = qrTO(i) - qTO(i);
    
    Matrix Jpinv = pinvDamped(J, PINV_DAMP);
    dq = Jpinv * e + nullspaceProjection(J, PINV_TOL) * eq;
    // fprintf(stderr, "Jpinv * J  :\n %s\n", (eye(Jpinv.rows(), J.cols()) - Jpinv * J).submatrix(njLL, njLL+njRL-1, njLL, njLL+njRL-1).toString().c_str());
    // fprintf(stderr, "eqRL  : %s\n", eq.subVector(njLL, njLL+njRL-1).toString().c_str());
    return dq*CTRL_RAD2DEG;
}

// J1: Jcom, J2: J_R2L
Vector comStepperThread::computeControlPrioritized(Matrix J1, Matrix J2, Vector e1, Vector e2)
{
    int njTot = njLL + njRL + njTO;
    Vector dq(njTot, 0.0), eq(njTot, 0.0);  // dq: desired joint vel; eq: joint space position error
    Matrix J = pile(J1, J2), Smask(njTot, njTot);
    Vector e = cat(e1, e2);
    
    if (current_phase == RIGHT_SUPPORT || current_phase == BOTH_SUPPORT)
    {
        Smask.setSubmatrix(Smask_r2l_swg, 0, 0);        // left leg swings
        Smask.setSubmatrix(Smask_r2l_sup, njLL, njLL);  // right leg supports
    }
    else
    {
        Smask.setSubmatrix(Smask_r2l_sup, 0, 0);        // left leg supports
        Smask.setSubmatrix(Smask_r2l_swg, njLL, njLL);  // right leg swings
    }
    Smask.setSubmatrix(Smask_com_torso, njLL+njRL, njLL+njRL);
    
    eq.setSubvector(0, qrLL-qLL);   // angles in deg!
    eq.setSubvector(njLL, qrRL-qRL);
    eq.setSubvector(njLL+njRL, qrTO-qTO);
    //fprintf(stderr, "qrLL is: %s\n", qrRL.toString().c_str());
    //fprintf(stderr, "qLL  is: %s\n", qLL.toString().c_str());
    
    for (int i = 0; i < njTot; i++)
        if (Smask(i, i) == 1.0)
            eq(i) = 0.0;
    
    Matrix Jpinv(njTot, J.rows()), JpinvD(njTot, J.rows());
    pinvDampTrunc(J, Jpinv, JpinvD, PINV_TOL, PINV_DAMP);
    Matrix N = eye(njTot) - Jpinv*J;
    dq = Jpinv*e + N*eq*CTRL_DEG2RAD;
    
    // Vector err(pi_a.getCol(0).size()+6-1);
    // err.zero();
    // err.setSubvector(0, pi_a.submatrix(1, 2, 0, 0).getCol(0));
    // err.setSubvector(pi_a.getCol(0).size()-1, ep.getCol(0));
    // err.setSubvector(pi_a.getCol(0).size()+3-1, eo.getCol(0));
    // checkControl(q, dq, err, J, e);
    
    return dq*CTRL_RAD2DEG;
}

Vector comStepperThread::computeControlTriangular(Matrix J1, Matrix J2, Vector e1, Vector e2)
{
    Vector dqLegs  = zeros(njLL + njRL);
    Vector dqTorso = zeros(njTO);
    Vector dq      = zeros(njLL + njRL + njTO);
    
    Matrix Smask(njLL + njRL, njLL + njRL);
    if (current_phase == RIGHT_SUPPORT || current_phase == BOTH_SUPPORT)
    {
        Smask.setSubmatrix(Smask_r2l_swg, 0, 0);
        Smask.setSubmatrix(Smask_r2l_sup, njLL, njLL);
    }
    else
    {
        Smask.setSubmatrix(Smask_r2l_sup, 0, 0);
        Smask.setSubmatrix(Smask_r2l_swg, njLL, njLL);
    }
    
    dqLegs  = pinvDamped(J1.submatrix(0, e1.size()-1,         0, njLL+njRL     -1) * Smask, PINV_DAMP) * e1;
    dqTorso = pinvDamped(J2.submatrix(0, e2.size()-1, njRL+njLL, njRL+njLL+njTO-1), PINV_DAMP) * (e2 - J2.submatrix(0, e2.size()-1 , 0, njRL+njLL-1)*dqLegs);
    dq.setSubvector(0        ,  dqLegs) ;
    dq.setSubvector(njRL+njLL, dqTorso);
    
    return dq*CTRL_RAD2DEG;
        
}

void comStepperThread::checkControl(Vector q, Vector dq, Vector err, Matrix J, Vector e)
{
    static Vector ePrev = err;
    Vector eCurr = err;
    Vector De = (eCurr - ePrev);

    static Vector qPrev = q*CTRL_DEG2RAD;
    Vector qCurr = q*CTRL_DEG2RAD;
    Vector Dq = (qCurr - qPrev);
    
    fprintf(stderr, "JDq VS De: \n %s \n %s \n", (J*Dq).toString().c_str(), De.toString().c_str());
    //fprintf(stderr, "ep VS e: \n %s \n %s \n", ePrev.toString().c_str(), eCurr.toString().c_str());
    //fprintf(stderr, "JDq VS De: \n %s \n %s \n", (Jtmp1*dq).toString().c_str(), (J*dq).toString().c_str());
    //fprintf(stderr, "Dq VS dq LL:\n %s\n %s\n", Dq.subVector(0, njLL -1).toString().c_str(), dq.subVector(0, njLL -1).toString().c_str());
    //fprintf(stderr, "Dq VS dq RL:\n %s\n %s\n", Dq.subVector(njLL, njLL+ njRL -1).toString().c_str(), dq.subVector(njLL, njLL+ njRL -1).toString().c_str());
    //fprintf(stderr, "Dq VS dq TO:\n %s\n %s\n", Dq.subVector(njLL+ njRL, njLL+ njRL + njTO -1).toString().c_str(), dq.subVector(njLL+ njRL, njLL+ njRL + njTO -1).toString().c_str());
    //fprintf(stderr, "Vel ratio is:\n %f\n", ((1/dq(1)*Dq(1))));

    qPrev = qCurr;
    ePrev = eCurr;
}

void comStepperThread::executeControl(Vector dq)
{    
    saturateVector(dq, vel_sat);
    dqLL   = dq.subVector(0        , njLL          -1);
    dqRL   = dq.subVector(njLL     , njLL+njRL     -1);
    dqTO   = dq.subVector(njLL+njRL, njLL+njRL+njTO-1);
    
#ifndef DO_NOT_CONTROL_LEGS
    if (on_ground)
    {
        Ivel_RL->velocityMove(dqRL.data());
        Ivel_LL->velocityMove(dqLL.data());
    }
    else
    {
        Ivel_LL->velocityMove(zeros(njLL).data());
        Ivel_RL->velocityMove(zeros(njRL).data());
    }
#endif
    
#ifndef DO_NOT_CONTROL_TORSO
    if (on_ground)
    {
        Ivel_TO->velocityMove(dqTO.data());
    }
    else
        Ivel_TO->velocityMove(zeros(njTO).data());
#endif
}

void comStepperThread::jacobianR2LrightSupport(Matrix &jR2LrightSupport, Vector &eR2LrightSupport, Matrix  pacd, Matrix Racd, Matrix dpacd, Matrix dRacd)
{
    // We compute the velocity of 'c' with respect to 'a'. Linear velocity is dpac, angular is doac.
    // We assume that we can compute the velocity of 'a' and 'c' with respct to 'b'.
    //
    //   | I    S(pac)  |  | dpac |   |  Rab    S(pab)*Rab |  ( | I    S(pbc)  |  | dpbc |   | I    S(pba)  |  | dpba | )
    //   |              |  |      | = |                    |  ( |              |  |      | - |              |  |      | )
    //   | 0       I    |  | doac |   |   0          Rab   |  ( | 0       I    |  | dobc |   | 0       I    |  | doba | )
    //
    // which becomes:
    //
    //   | I    S(pac)  |  | dpac |   |  Rab          0    |  ( | I    S(rca_b)|  | dpbc |    | dpba | )
    //   |              |  |      | = |                    |  ( |              |  |      | -  |      | )
    //   | 0       I    |  | doac |   |   0          Rab   |  ( | 0       I    |  | dobc |    | doba | )
    //
    // with rca_b the vector from 'c' to 'a' in 'b', i.e. rca_b = -R_ab^T * pac = -Rba*pac. The above can be written:
    //
    //   | dpac |   | I    -S(pac) | |  Rab          0    |  ( | I   -S(rca_b) |  | dpbc |    | dpba | )
    //   |      | = |              | |                    |  ( |               |  |      | -  |      | )
    //   | doac |   | 0       I    | |   0          Rab   |  ( | 0       I     |  | dobc |    | doba | )
    //
    // In particular, as expected the velcoity of 'c' with repsect to 'a' is zeros iff:
    //
    //   | I  -S(rca_b) |  | dpbc |    | dpba |
    //   |              |  |      | -  |      | = 0
    //   | 0       I    |  | dobc |    | doba |
    //
    // where:
    //
    //    | dpba |               | dpbc |
    //    |      | = Jba * dqa,  |      | = Jbc * dqc,
    //    | doba |               | dobc |
    //
    // and:
    //                                                                |                                  |
    //   | dpac |            | I    -S(pac) | |  Rab          0    |  | ( I   -S(rca_b) )                |
    //   |      | = Jac dq = |              | |                    |  | (               ) *Jbc  | -Jba   |  |  dq
    //   | doac |            | 0       I    | |   0          Rab   |  | ( 0       I     )                |
    //                                                                |                                  |
    
    velTranslator(Spac, pac);
    velTranslator(Srca, rca_b);
    rotTranslator(Rrab, Rba.transposed());
    
    Jac.setSubmatrix(Srca * Jbc, 0,          0);
    Jac.setSubmatrix(-1.0 * Jba, 0, Jbc.cols());
    Jac = Spac * Rrab * Jac;
    
    // Let's now implement the control strategy for keeping the relative left-right foot constant. Using the same
    // notation used in Sciavicco-Siciliano (Section Orientation error, axis angle notation) we have:
    //
    // The position (ep) and orientation (eo) error are defined as:
    //
    //                             1
    // ep = pac_d - pac,     eo =  - (S(ne)nd + S(se)sd + S(ae)ad)
    //                             2
    //
    // where pac is the ac translation, pac_d its desired value and:
    //
    // Rac_d = [nd sd ad],    Rac = [ne se ae].
    //
    // The control loop is based on the following equation:
    //
    //   | I   0 |           |    dpac_d     |   | dep |   | Kp * ep |
    //  -|       |  Jac dq + |               | = |     | = |         |
    //   | 0   L |           |  L^T (dRac_d) |   | deo |   | Ko * eo |
    //
    // where:
    //         1 (                                               )
    //   L = - - ( S(nd) * S(ne) + S(sd) * S(se) + S(ad) * S(ae) )
    //         2 (                                               )
    //
    // Rearranging we have:
    //
    //            |    dpac_d    +      Kp * ep   |
    //   Jac dq = |                               |
    //            |  L^-1 (L^T dRac_d + Ko * eo)  |
    //
    // The current implementation computes dqc (left leg velocities) assuming dqr = 0:
    //
    //                       |    dpac_d    +      Kp * ep   |
    //    dqc= pinv(Jac_qc)  |                               |
    //                       |  L^-1 (L^T dRac_d + Ko * eo)  |
    //
    // where Jac = [ Jac_qa Jac_qc ].
    
    // Desired configuration and its derivative (currently zero)
    if (verbose)
    {
        fprintf(stderr, "pac_d: %s\n", pacd.toString().c_str());
        fprintf(stderr, "Rac_d: %s\n", iCub::ctrl::dcm2axis(Racd).toString().c_str());
    }
    
    nd = Racd.submatrix(0, 2, 0, 0);   ne = Rac.submatrix(0, 2, 0, 0);
    sd = Racd.submatrix(0, 2, 1, 1);   se = Rac.submatrix(0, 2, 1, 1);
    ad = Racd.submatrix(0, 2, 2, 2);   ae = Rac.submatrix(0, 2, 2, 2);
    
    hat(ne_hat, ne);    hat(se_hat, se);    hat(ae_hat, ae);
    hat(nd_hat, nd);    hat(sd_hat, sd);    hat(ad_hat, ad);
    
    ep = pacd - pac;
    eo = 0.5*(ne_hat * nd     + se_hat * sd     + ae_hat * ad);
    L  =-0.5*(nd_hat * ne_hat + sd_hat * se_hat + ad_hat * ae_hat);
    
    Matrix ded(6,1);
    Matrix Kp = Kr2l.submatrix(0, 2, 0, 2);
    Matrix Ko = Kr2l.submatrix(3, 5, 3, 5);
    ded.setSubmatrix(dpacd + Kp * (pacd-pac),                    0, 0);
    ded.setSubmatrix(luinv(L)*(L.transposed()*dRacd  + Ko * eo), 3, 0);
    
    eR2LrightSupport = ded.getCol(0);
    jR2LrightSupport = zeros(6, njLL + njRL + njTO);
    jR2LrightSupport.setSubmatrix(Jac, 0, 0);
    
    //swapping torso joint 0 with 2
    Vector swap = jR2LrightSupport.getCol(njLL+njRL);
    jR2LrightSupport.setCol(njLL+njRL, jR2LrightSupport.getCol(njLL+njRL+njTO-1)) ;
    jR2LrightSupport.setCol(njLL+njRL+njTO-1, swap) ;
    
    
#ifdef COMPUTE_FINITE_DIFF
    
    Matrix tmpM(6,6);
    tmpM.setSubmatrix(eye(3)     , 0, 0);   tmpM.setSubmatrix(zeros(3, 3), 0, 3);
    tmpM.setSubmatrix(zeros(3, 3), 3, 0);   tmpM.setSubmatrix(L,           3, 3);
    
    Vector dq(12);
    // dq.setSubvector(0, q1R-q0R);
    // dq.setSubvector(6, q1L-q0L);
    Vector de  = zeros(6) - tmpM*Jac*dq;
    
    if (verbose)
    {
        // fprintf(stderr, "delta_e: %s\n", delta_e.toString().c_str());
        fprintf(stderr, "de:      %s\n", de.toString().c_str());
        // fprintf(stderr, "Velocity right foot in b: %s\n", (Jba*(q1R-q0R)).toString().c_str());
        // fprintf(stderr, "Velocity left  foot in b: %s\n", (Spac*Rrab*Srca*Jbc*(q1L-q0L)).toString().c_str());
        // fprintf(stderr, "Res: %s\n", L.toString().c_str());
    }
#endif
}

void comStepperThread::jacobianCOMrightSupport(Matrix &jCOMrightSupport, Vector &eCOMrightSupport, Matrix pi_ad, Matrix dpi_ad)
{
    //Matrices involved in the computation of the center of mass projection
    static Matrix Spab(6,6),                 SRab_pb(6,6),       Srba_pi(6,6)      ;   //varius matrices used in the computations
    static Matrix Jab(6, Jba.cols())                                               ;   //Jabian from of 'b' with respect to 'a'
    static Matrix Jab_t(3, Jba.cols()),      Jba_t(3, Jba.cols())                  ;   //translated version of Jab and Jba
    static Matrix Jpi_a(3, Jba.cols()),      Jpi_a_com(3, Jb_com.cols())           ;   //jacobian of the projection on 'a'
    static Matrix pi_b_a(3,1);
    
    // Let's now compute the projection of a point p_b (eventually the COM)
    // passing trought the right foot (pba) and orthogonal to zg_b. All these
    // computations are in 'b'. The plane equation is:
    //
    // P_b : zg_b^T (x_b - pba) = 0
    //
    // The projection point belongs to a line parallel to zg_b and passing through p
    //
    // r_b : x_b = zg_b * v + p_b      for all v in R
    //
    // Intersecting plane and line and using zg_b^T * zg_b = 1 we have:
    //
    // zg_b^T (zg_b * v + p_b - pba) = 0
    //
    // v = zg_b^T (pba - p_b)
    //
    // which corresponds to the point:
    //
    // pi_b = zg_b zg_b^T (pba - p_b) + p_b
    //
    // or equivalently:
    //
    // pi_b = (I - zg_b zg_b^T) p_b + zg_b zg_b^T pba
    //
    // which gives the point x_b projection of p on the plane. We can also
    // consider referring all quantities to the reference frame 'a' (right foot)
    // with the advantage that zg will be constant during the right foot support.
    // In this case we have:
    //
    //             | 0 |                  | px_a |
    // pi_a = (I - | 0 | | 0 0 1 |) p_a = | py_a | = PI*p_a
    //             | 1 |                  |  0   |
    //
    // The point p_a is the center of mass, which is however expressed in
    // the reference frame 'b'. Therefore:
    //
    // pi_a = PI * (Rab * p_b + pab)    and pi_b = Rba * PI * (Rab * p_b + pab) + pba
    //
    // In homogenous coordinates:
    //
    // pi_b = Tba * PI2 * Tab * p_b
    //
    //          | 1  0  0  0 |
    //    PI2 = | 0  1  0  0 |
    //          | 0  0  0  0 |
    //          | 0  0  0  1 |
    //
    // Let's now compute the associated Jacobian. We have:
    //
    // dpi_a = PI * d(Rab * p_b + pab) = PI * (Rab dp_b(t) + [ I -S(Rab p_b) ] Jab * dq)
    //
    // where Jab is the 'a'-'b' Jacobian.
    //
    // dpi_b = d(Rba * pi_a + pba) = Rba * dpi_a + [ I -S(Rba pi_a) ] Jba*dq
    //
    // Finally, the velocity dp_b(t) is computed as:
    //
    // dp_b(t) = Jb_p*dq
    //
    // so that we have:
    //
    // dpi_b = Rba * PI * (Rab dp_b(t) + [ I -S(Rab p_b) ] Jab * dq) + [ I -S(Rba pi_a) ] Jba*dq
    //
    // dpi_b = Rba * PI * (Rab Jb_p*dq + [ I -S(Rab p_b) ] Jab * dq) + [ I -S(Rba pi_a) ] Jba*dq
    //
    // dpi_b = [Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba] dq
    //
    //               (                                  )
    // dpi_a = Rab * (dpi_b - [ I -S(Rba pi_a) ] Jba*dq )
    //               (                                  )
    //
    //               (                                               )
    // dpi_a = Rab * (Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab )
    //               (                                               )
    //
    //               (                                               )
    // dpi_a =       (    PI Rab Jb_p +     PI [ I -S(Rab p_b) ] Jab )
    //               (                                               )
    //
    // and therefore we have the following Jacobians:
    //
    // Jpi_b = Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba
    //
    // Jpi_a =     PI Rab Jb_p +     PI [ I -S(Rab p_b) ] Jab
    //
    // where as usual we can compute Jab as a function of Jba
    //
    //   | dpab |             | Rab  -S(pab)Rab |
    //   |      | = Jab dq = -|                 |  Jba dq
    //   | doab |             | 0       Rab     |
    //
    // Considering all the joints involve in the COM, we can consider the following
    // ordering that is equivalent to the one used in the definition of the
    // COM Jacobian Jb_com by the wholeBodyDynamics module.
    //
    //        q = [q_left_arm,  q_right_arm, q_torso, q_left_leg,  q_right_leg]
    //
    // Thus in the above formulas, Jb_p refers only to the last columns of Jb_com:
    //
    //   Jb_com = [Jb_com_left_arm,  Jb_com_right_arm, Jb_com_torso, Jb_com_left_leg,  Jb_com_right_leg]
    //
    // or in other words we used Jb_com_right_leg = Jb_p. Therefore the projection:
    //
    //   Jpi_b_com = [Rba PI Rab Jb_com_left_arm,  Rba PI Rab Jb_com_right_arm, Rba PI Rab Jb_com_torso, Rba PI Rab Jb_com_left_leg,  Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba]
    //
    //   Jpi_b_com = Rba PI Rab Jb_com + [0,  0, 0, 0,  Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba]
    
    
    velTranslator(Spab, pab);
    Jab   = (-1.0) * Spab * Rrab * Jba;
    
    velTranslator(SRab_pb, Rba.transposed()*p_b);
    Jab_t = SRab_pb.submatrix(0, 2, 0, 5) * Jab;
    
    velTranslator(Srba_pi, Rba*pi_a);
    Jba_t = Srba_pi.submatrix(0, 2, 0, 5) * Jba;
    
    pi_b_a  = Rba * PI * (Rba.transposed() * p_b + pab) + pba;
    
    Jpi_b     = Rba * PI * Rba.transposed() * Jb_p   + Rba * PI * Jab_t + Jba_t;
    Jpi_a     =       PI * Rba.transposed() * Jb_p   +       PI * Jab_t ;
    
#ifdef COMPUTE_FINITE_DIFF
    // Vector delta_pi_b = computeDeltaProjB(q0R, q0L, q1R, q1L);
    // fprintf(stderr, "delta_pi_b is:\n %s\n", delta_pi_b.toString().c_str());
    // fprintf(stderr, "dpi_b is:     \n %s\n", (Jpi_b*(q1R - q0R)).toString().c_str());
    // Vector delta_pi_a = computeDeltaProjA(q0R, q0L, q1R, q1L);
    // fprintf(stderr, "delta_pi_a is:\n %s\n", delta_pi_a.toString().c_str());
    // fprintf(stderr, "dpi_a is:     \n %s\n", (Jpi_a*(q1R - q0R)).toString().c_str());
#endif
    
    // Finally let's compute the control loop that brings the robot in a desired configuration
    // with respect to a reference position. The best is to force the dynamics of the COM
    // projection in the reference frame 'a', i.e. to impose:
    //
    //           dpi_a = Kpi_a * (pi_a_d - pi_a) + dpi_a_d
    //
    // To do so:
    //                          |                                   |
    //          dpi_a = Rab  *  | dpi_b - [ I -S(Rba pi_a) ] Jba*dq |
    //                          |                                   |
    //
    //                      |                                                |
    //      dpi_a = Rab  *  | Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab |  = Jpi_a * dq
    //                      |                                                |
    //
    //     dq = inv(Jpi_a_com) * [ Kpi_a * (pi_a_d - pi_a) + dpi_a_d]
    //
    // Assuming that we vant to use only the torso, we have:
    //
    //     Jpi_a_com_torso * dqtorso +  Jpi_a_com_left_leg * dqLL = Kpi_a * (pi_a_d - pi_a)
    //
    // and therefore:
    //
    //     dqtorso = inv(Jpi_a_com_torso) * (Kpi_a * (pi_a_d - pi_a) + dpi_a_d -  Jpi_a_com_left_leg * dqLL)
    
    
    Jpi_b_com = Rba * PI * Rba.transposed() * Jb_com;
    Jpi_a_com =       PI * Rba.transposed() * Jb_com;
    Jpi_b_com.setSubmatrix(Jpi_b_com.submatrix(0, 2, njLL, njLL+njRL-1) + Rba * PI * Jab_t + Jba_t, 0, njLL);
    Jpi_a_com.setSubmatrix(Jpi_a_com.submatrix(0, 2, njLL, njLL+njRL-1) +       PI * Jab_t        , 0, njLL);
    
    //Matrix Jpi_a_com_torso    = Jpi_a_com.submatrix(0, 2, njLL+njRL,  njLL+njRL+njTO-1);
    //Matrix Jpi_a_com_legs     = Jpi_a_com.submatrix(0, 2,         0,  njLL+njRL     -1);
    
    jCOMrightSupport = Jpi_a_com.submatrix(1, 2, 0, njLL + njRL + njTO - 1);
    
    //swapping torso joint 0 with 2
    Vector swap = jCOMrightSupport.getCol(njLL+njRL);
    jCOMrightSupport.setCol(njLL+njRL, jCOMrightSupport.getCol(njLL+njRL+njTO-1)) ;
    jCOMrightSupport.setCol(njLL+njRL+njTO-1, swap) ;
    
    if (!strcmp(robot_name.c_str(), "icubSim"))
        eCOMrightSupport = (Kcom* (pi_ad - pi_a) + dpi_ad).getCol(0).subVector(1, 2);
    else
    {
        Matrix pi_ad_zmp_a = zmp_a;
        eCOMrightSupport = (Kcom* (pi_ad - pi_ad_zmp_a) + dpi_ad).getCol(0).subVector(1, 2);
    }
    
    if (verbose)
    {
        // fprintf(stderr, "njHD: %d, njRA: %d, njLA: %d, njTO: %d, njRL: %d, njLL: %d, tot: %d\n", njHD, njRA, njLA, njTO, njRL, njLL, Jb_com.cols());
        // fprintf(stderr, "p_b is               :  \n%s\n", p_b.toString().c_str());
        // fprintf(stderr, "pi_a is              :  \n%s\n", (pi_a).toString().c_str());
        fprintf(stderr, "COM Position error is:  \n%s\n", (pi_a_t - pi_a).toString().c_str());
        // fprintf(stderr, "COM Jacobian        :\n%s\n", (Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)).toString().c_str());
        // fprintf(stderr, "Sent velocities     :\n%s\n",  dqTO.toString().c_str());
    }
#ifdef COMPUTE_FINITE_DIFF
    Vector dqTO_(3);
    dqTO_ = zeros(3);
    dqTO_(0) =  0.2;     dqTO_(1) = 0.5;     dqTO_(2) = .30;
    Vector dqTOrev(3);
    dqTOrev(0) = dqTO_(2);     dqTOrev(1) = dqTO_(1);     dqTOrev(2) = dqTO_(0);
    
    Vector dqRL_(6);
    dqRL_ = zeros(6);
    dqRL_(0) =  -.1;     dqRL_(1) =  0.2;     dqRL_(2) = -0.2;
    dqRL_(3) =  -.1;     dqRL_(4) =  0.5;     dqRL_(5) = -0.3;
    
    Vector dqLL_(6);
    dqLL_ = zeros(6);
    dqLL_(0) =  -.1;     dqLL_(1) =  0.1;     dqLL_(2) = -0.2;
    dqLL_(3) =  -.1;     dqLL_(4) =  0.2;     dqLL_(5) = -0.3;
    
    Vector delta_b_(3), delta_pi_a_(3), delta_pi_b_(3);
    
    computeDeltaProjAReal(dqRL_, dqLL_, dqTO_, delta_b_, delta_pi_a_, delta_pi_b_);
    
    dqTOrev  = dqTOrev * CTRL_DEG2RAD;
    dqRL_    = dqRL_   * CTRL_DEG2RAD;
    dqLL_    = dqLL_   * CTRL_DEG2RAD;
    
    fprintf(stderr, "Real delta_b is:     \n %s\n", delta_b_.toString().c_str());
    fprintf(stderr, "Real dp_b is:        \n %s\n",       (((Jb_com.submatrix(0, 2,         0, njLL          -1) * dqLL_) +
                                                            (Jb_com.submatrix(0, 2,      njLL, njLL+njRL     -1) * dqRL_) +
                                                            (Jb_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1) * dqTOrev))).toString().c_str());
    
    fprintf(stderr, "Real delta_pi_a is:  \n %s\n", delta_pi_a_.toString().c_str());
    fprintf(stderr, "Real dpi_a is:       \n %s\n", ((Jpi_a_com.submatrix(0, 2,         0, njLL          -1) *dqLL_) +
                                                     (Jpi_a_com.submatrix(0, 2,      njLL, njLL+njRL     -1) *dqRL_) +
                                                     (Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1) *dqTOrev)).toString().c_str());
    
    // fprintf(stderr, "Real delta_pi_b is:  \n %s\n", delta_pi_b.toString().c_str());
    // fprintf(stderr, "Real dpi_b is:       \n %s\n", ((Jpi_b_com.submatrix(0, 2, njLL, njLL+njRL-1)*dqRL_)+(Jpi_b_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)*dqTOrev)).toString().c_str());
    // fprintf(stderr, "Jpi_b_com_torso is: \n %s\n", Jpi_b_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1).toString().c_str());
    // fprintf(stderr, "Rba  is:            \n %s\n", Rba.toString().c_str());
    // fprintf(stderr, "Rba*Jpi_a_com  is:  \n %s\n", (Rba*Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)).toString().c_str());
#endif
    
}

void comStepperThread::jacobianR2LleftSupport(Matrix &jR2LleftSupport, Vector &eR2LleftSupport,Matrix  pcad, Matrix Rcad, Matrix dpcad, Matrix dRcad)
{
    
    pcb =   -1.0 * Rbc.transposed() * pbc;
    pab =   -1.0 * Rba.transposed() * pba;
    rac_b = -1.0 * Rbc * pca;
    
    velTranslator(Spca, pca);
    velTranslator(Srac, rac_b);
    rotTranslator(Rrcb, Rbc.transposed());
    
    Jca.setSubmatrix(-1.0 * Jbc, 0,          0);
    Jca.setSubmatrix(Srac * Jba, 0, Jba.cols());
    Jca = Spca * Rrcb * Jca;
    
    if (verbose)
    {
        fprintf(stderr, "pca_d: %s\n", pcad.toString().c_str());
        fprintf(stderr, "Rca_d: %s\n", iCub::ctrl::dcm2axis(Rcad).toString().c_str());
    }
    
    nd = Rcad.submatrix(0, 2, 0, 0);   ne = Rca.submatrix(0, 2, 0, 0);
    sd = Rcad.submatrix(0, 2, 1, 1);   se = Rca.submatrix(0, 2, 1, 1);
    ad = Rcad.submatrix(0, 2, 2, 2);   ae = Rca.submatrix(0, 2, 2, 2);
    
    hat(ne_hat, ne);    hat(se_hat, se);    hat(ae_hat, ae);
    hat(nd_hat, nd);    hat(sd_hat, sd);    hat(ad_hat, ad);
    
    ep = pcad - pca;
    eo = 0.5*(ne_hat * nd     + se_hat * sd     + ae_hat * ad);
    L  =-0.5*(nd_hat * ne_hat + sd_hat * se_hat + ad_hat * ae_hat);
    
    Matrix ded(6,1);
    Matrix Kp = Kr2l.submatrix(0, 2, 0, 2);
    Matrix Ko = Kr2l.submatrix(3, 5, 3, 5);
    ded.setSubmatrix(dpcad + Kp * (pcad-pca),                    0, 0);
    ded.setSubmatrix(luinv(L)*(L.transposed()*dRcad  + Ko * eo), 3, 0);
    
    eR2LleftSupport = ded.getCol(0);
    jR2LleftSupport = zeros(6, njLL + njRL + njTO);
    jR2LleftSupport.setSubmatrix(Jca, 0, 0);
    
    //swapping torso joint 0 with 2
    Vector swap = jR2LleftSupport.getCol(njLL+njRL);
    jR2LleftSupport.setCol(njLL+njRL, jR2LleftSupport.getCol(njLL+njRL+njTO-1)) ;
    jR2LleftSupport.setCol(njLL+njRL+njTO-1, swap) ;
    
    if (verbose)
    {
        fprintf(stderr, "Position    error is: %s\n", ep.transposed().toString().c_str());
        fprintf(stderr, "Orientation error is: %s\n", eo.transposed().toString().c_str());
    }
    
    
#ifdef COMPUTE_FINITE_DIFF
    
    Matrix tmpM(6,6);
    tmpM.setSubmatrix(eye(3)     , 0, 0);   tmpM.setSubmatrix(zeros(3, 3), 0, 3);
    tmpM.setSubmatrix(zeros(3, 3), 3, 0);   tmpM.setSubmatrix(L,           3, 3);
    
    Vector dq(12);
    // dq.setSubvector(0, q1R-q0R);
    // dq.setSubvector(6, q1L-q0L);
    Vector de  = zeros(6) - tmpM*Jac*dq;
    
    if (verbose)
    {
        // fprintf(stderr, "delta_e: %s\n", delta_e.toString().c_str());
        fprintf(stderr, "de:      %s\n", de.toString().c_str());
        // fprintf(stderr, "Velocity right foot in b: %s\n", (Jba*(q1R-q0R)).toString().c_str());
        // fprintf(stderr, "Velocity left  foot in b: %s\n", (Spac*Rrab*Srca*Jbc*(q1L-q0L)).toString().c_str());
        // fprintf(stderr, "Res: %s\n", L.toString().c_str());
    }
#endif
}

void comStepperThread::jacobianCOMleftSupport(Matrix &jCOMleftSupport, Vector &eCOMleftSupport, Matrix pi_cd, Matrix dpi_cd)
{
    
    //Matrices involved in the computation of the center of mass projection
    static Matrix Spcb(6,6),                 SRcb_pb(6,6),       Srbc_pi(6,6)      ;   //varius matrices used in the computations
    static Matrix Jcb(6, Jbc.cols())                                               ;   //Jabian from of 'b' with respect to 'a'
    static Matrix Jcb_t(3, Jbc.cols()),      Jbc_t(3, Jbc.cols())                  ;   //translated version of Jab and Jba
    static Matrix Jpi_c(3, Jbc.cols()),      Jpi_c_com(3, Jb_com.cols())           ;   //jacobian of the projection on 'a'
    static Matrix pi_b_c(3,1);
    
    velTranslator(Spcb, pcb);
    Jcb   = (-1.0) * Spcb * Rrcb * Jbc;
    
    velTranslator(SRcb_pb, Rbc.transposed()*p_b);
    Jcb_t = SRcb_pb.submatrix(0, 2, 0, 5) * Jcb;
    
    velTranslator(Srbc_pi, Rbc*pi_c);
    Jbc_t = Srbc_pi.submatrix(0, 2, 0, 5) * Jbc;
    
    pi_b_c  = Rbc * PI * (Rbc.transposed() * p_b + pcb) + pbc;
    
    Jpi_b     = Rbc * PI * Rbc.transposed() * Jb_p   + Rbc * PI * Jcb_t + Jbc_t;
    Jpi_c     =       PI * Rbc.transposed() * Jb_p   +       PI * Jcb_t ;
    
#ifdef COMPUTE_FINITE_DIFF
    // Vector delta_pi_b = computeDeltaProjB(q0R, q0L, q1R, q1L);
    // fprintf(stderr, "delta_pi_b is:\n %s\n", delta_pi_b.toString().c_str());
    // fprintf(stderr, "dpi_b is:     \n %s\n", (Jpi_b*(q1R - q0R)).toString().c_str());
    // Vector delta_pi_a = computeDeltaProjA(q0R, q0L, q1R, q1L);
    // fprintf(stderr, "delta_pi_a is:\n %s\n", delta_pi_a.toString().c_str());
    // fprintf(stderr, "dpi_a is:     \n %s\n", (Jpi_a*(q1R - q0R)).toString().c_str());
#endif
    
    Jpi_b_com = Rbc * PI * Rbc.transposed() * Jb_com;
    Jpi_c_com =       PI * Rbc.transposed() * Jb_com;
    Jpi_b_com.setSubmatrix(Jpi_b_com.submatrix(0, 2,     0, njLL-1) + Rbc * PI * Jcb_t + Jbc_t, 0, 0);
    Jpi_c_com.setSubmatrix(Jpi_c_com.submatrix(0, 2,     0, njLL-1) +       PI * Jcb_t        , 0, 0);
    
    jCOMleftSupport = Jpi_c_com.submatrix(1, 2, 0, njLL + njRL + njTO - 1);
    
    //swapping torso joint 0 with 2
    Vector swap = jCOMleftSupport.getCol(njLL+njRL);
    jCOMleftSupport.setCol(njLL+njRL, jCOMleftSupport.getCol(njLL+njRL+njTO-1)) ;
    jCOMleftSupport.setCol(njLL+njRL+njTO-1, swap) ;
    
    
    if (!strcmp(robot_name.c_str(), "icubSim"))
        eCOMleftSupport = (Kcom* (pi_cd - pi_c) + dpi_cd).getCol(0).subVector(1, 2);
    else
    {
        Matrix pi_cd_zmp_c = zmp_c;
        eCOMleftSupport = (Kcom* (pi_cd - pi_cd_zmp_c) + dpi_cd).getCol(0).subVector(1, 2);
    }
    
    
    //Vector dqRL = pinvDamped(Jpi_a_com.submatrix(0, 2, njLL, njLL+njRL-1), 0.01)*de_pi_a;
    //Ivel_RL->velocityMove(dqRL.data());
    
    if (verbose)
    {
        // fprintf(stderr, "njHD: %d, njRA: %d, njLA: %d, njTO: %d, njRL: %d, njLL: %d, tot: %d\n", njHD, njRA, njLA, njTO, njRL, njLL, Jb_com.cols());
        // fprintf(stderr, "p_b is               :  \n%s\n", p_b.toString().c_str());
        // fprintf(stderr, "pi_a is              :  \n%s\n", (pi_a).toString().c_str());
        fprintf(stderr, "COM Position error is:  \n%s\n", (pi_c_t - pi_c).toString().c_str());
        // fprintf(stderr, "COM Jacobian        :\n%s\n", (Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)).toString().c_str());
        // fprintf(stderr, "Sent velocities     :\n%s\n",  dqTO.toString().c_str());
    }
#ifdef COMPUTE_FINITE_DIFF
    Vector dqTO_(3);
    dqTO_ = zeros(3);
    dqTO_(0) =  0.4;     dqTO_(1) = 0.2;     dqTO_(2) = -0.3;
    Vector dqTOrev(3);
    dqTOrev(0) = dqTO_(2);     dqTOrev(1) = dqTO_(1);     dqTOrev(2) = dqTO_(0);
    
    Vector dqRL_(6);
    dqRL_ = zeros(6);
    dqRL_(0) =   .1;     dqRL_(1) =  0.2;     dqRL_(2) = -0.2;
    dqRL_(3) =  -.1;     dqRL_(4) =  0.5;     dqRL_(5) = -0.3;
    
    Vector dqLL_(6);
    dqLL_ = zeros(6);
    dqLL_(0) =  -.1;     dqLL_(1) =  0.1;     dqLL_(2) = -0.2;
    dqLL_(3) =  -.1;     dqLL_(4) =  0.2;     dqLL_(5) = -0.3;
    
    Vector delta_b_(3), delta_pi_c_(3), delta_pi_b_(3);
    computeDeltaProjCReal(dqRL_, dqLL_, dqTO_, delta_b_, delta_pi_c_, delta_pi_b_);
    
    dqTOrev  = dqTOrev * CTRL_DEG2RAD;
    dqRL_    = dqRL_   * CTRL_DEG2RAD;
    dqLL_    = dqLL_   * CTRL_DEG2RAD;
    
    fprintf(stderr, "Real delta_b is: \n %s\n", delta_b_.toString().c_str());
    fprintf(stderr, "Real dp_b is:    \n %s\n", (((Jb_com.submatrix(0, 2,         0, njLL          -1) * dqLL_) +
                                                  (Jb_com.submatrix(0, 2,      njLL, njLL+njRL     -1) * dqRL_) +
                                                  (Jb_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1) * dqTOrev))).toString().c_str());
    
    fprintf(stderr, "Real delta_pi_c is:  \n %s\n", delta_pi_c_.toString().c_str());
    fprintf(stderr, "Real dpi_c is:       \n %s\n", ((Jpi_c_com.submatrix(0, 2,         0, njLL          -1) * dqLL_) +
                                                     (Jpi_c_com.submatrix(0, 2,      njLL, njLL+njRL     -1) * dqRL_) +
                                                     (Jpi_c_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1) * dqTOrev)).toString().c_str());
    
    // fprintf(stderr, "Real delta_pi_b is:  \n %s\n", delta_pi_b.toString().c_str());
    // fprintf(stderr, "Real dpi_b is:       \n %s\n", ((Jpi_b_com.submatrix(0, 2, njLL, njLL+njRL-1)*dqRL_)+(Jpi_b_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)*dqTOrev)).toString().c_str());
    // fprintf(stderr, "Jpi_b_com_torso is: \n %s\n", Jpi_b_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1).toString().c_str());
    // fprintf(stderr, "Rba  is:            \n %s\n", Rba.toString().c_str());
    // fprintf(stderr, "Rba*Jpi_a_com  is:  \n %s\n", (Rba*Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)).toString().c_str());
#endif
}


bool comStepperThread::read_lr_trf_port(/*@@@ here to be completed*/)
{
    if (!port_lr_trf) return false;
    
    Bottle *bot = port_lr_trf->read();
    if (bot == NULL) return false;
    
    int count = bot->get(0).asInt();
    double time = bot->get(1).asInt();
    Matrix m; m.resize(4,4); m.zero();
    for (int ix=0;ix<4;ix++)
        for (int iy=0;iy<4;iy++)
            m(ix,iy) = bot->get(ix+iy*4).asDouble();
    ConstString str = bot->get(0+1+16).asString();
    //@@@ you can add stuff here...
    
    return true;
}

void comStepperThread::computeZMPBoth()
{
    //extract forces and moments and refer them to the foot reference frame
    f_a.setCol(0, Ras * F_ext_RL->subVector(0, 2));
    m_a.setCol(0, Ras * F_ext_RL->subVector(3, 5));
    f_c.setCol(0, Rcs * F_ext_LL->subVector(0, 2));
    m_c.setCol(0, Rcs * F_ext_LL->subVector(3, 5));
    
    Matrix Sac(3,3);  hat(Sac, pac);
    Matrix ftot_a = f_a + Rac * f_c;
    Matrix mtot_a = m_a + Sac * f_c + Rac * m_c;
    
    hat(Sf_a, ftot_a);
    Matrix zmp_a_tmp = -1.0 * luinv(Sf_a.submatrix(1, 2, 1, 2)) * mtot_a.submatrix(1, 2, 0, 0);
    zmp_a(0,0) = 0.0;  zmp_a(1,0) = zmp_a_tmp(0,0);  zmp_a(2,0) = zmp_a_tmp(1,0);
    
    zmp_c = Rca * zmp_a + pca;
    
    //fprintf(stderr, "ZMP in double support is %s\n", zmpRL_a.toString().c_str());
}

void comStepperThread::computeZMPRight()
{
    //extract forces and moments and refer them to the foot reference frame
    f_a.setCol(0, Ras*F_ext_RL->subVector(0, 2));
    m_a.setCol(0, Ras*F_ext_RL->subVector(3, 5));
    
    hat(Sf_a, f_a);
    Matrix zmp_a_tmp = -1.0 * luinv(Sf_a.submatrix(1, 2, 1, 2)) * m_a.submatrix(1, 2, 0, 0);
    zmp_a(0,0) = 0.0;  zmp_a(1,0) = zmp_a_tmp(0,0);  zmp_a(2,0) = zmp_a_tmp(1,0);
    
    //fprintf(stderr, "ZMP in right support is %s\n", zmpRL_a.toString().c_str());
}

void comStepperThread::computeZMPLeft()
{
    //extract forces and moments and refer them to the foot reference frame
    f_c.setCol(0, Rcs*F_ext_LL->subVector(0, 2));
    m_c.setCol(0, Rcs*F_ext_LL->subVector(3, 5));
    
    hat(Sf_c, f_c);
    Matrix zmp_c_tmp = -1.0 * luinv(Sf_c.submatrix(1, 2, 1, 2)) * m_c.submatrix(1, 2, 0, 0);
    zmp_c(0,0) = 0.0;  zmp_c(1,0) = zmp_c_tmp(0,0);  zmp_c(2,0) = zmp_c_tmp(1,0);
    
    //fprintf(stderr, "ZMP in left support is %s\n", zmpLL_c.toString().c_str());
}

double comStepperThread::separation(Vector qRL, Vector qLL){
    
    PoseRightLeg = Right_Leg->EndEffPose(CTRL_DEG2RAD*qRL,false);
    PoseRightLeg.resize(3);
    PoseRightLeg.push_back(1);
    PoseRightLeg = Hright*PoseRightLeg;
    
    PoseLeftLeg = Left_Leg->EndEffPose(CTRL_DEG2RAD*qLL,false);
    PoseLeftLeg.resize(3);
    PoseLeftLeg.push_back(1);
    PoseLeftLeg = Hleft*PoseLeftLeg;
    
    double sep;
    sep = fabs(PoseLeftLeg[1]) + fabs(PoseRightLeg[1]);
    return sep;
}

bool comStepperThread::onStop()
{
    fprintf(stderr, "Stopping the comStepperThread\n");
    return true;
}

void comStepperThread::suspend()
{    /* Should delete dynamically created data-structures*/
    
    Ivel_LL->stop();    Ipos_LL->setPositionMode();
    Ivel_RL->stop();    Ipos_RL->setPositionMode();
    Ivel_TO->stop();    Ipos_TO->setPositionMode();
}

void comStepperThread::threadRelease()
{    /* Should delete dynamically created data-structures*/
    
    Ivel_LL->stop();    Ipos_LL->setPositionMode();
    Ivel_RL->stop();    Ipos_RL->setPositionMode();
    Ivel_TO->stop();    Ipos_TO->setPositionMode();
    
    closePort(port_lr_trf);
    
    fprintf(stderr, "Releasing the comStepperThread\n");
    if(!Opt_nosens)
    {
        fprintf(stderr, "Closing EEWRightLeg port \n");
        closePort(EEWRightLeg);
        
        fprintf(stderr, "Closing EEWLeftLeg port \n");
        closePort(EEWLeftLeg);
        
        fprintf(stderr, "Closing EEWRightAnkle port\n");
        closePort(EEWRightAnkle);
        
        fprintf(stderr, "Closing EEWLeftAnkle port\n");
        closePort(EEWLeftAnkle);
        
        fprintf(stderr, "Closing Object Port \n");
        closePort(objPort);
        
        fprintf(stderr, "Closing Object2 Port \n");
        closePort(objPort2);
        
        fprintf(stderr, "Closing desired_zmp port\n");
        closePort(desired_zmp);
        
    }
    
    if(!comPosPortString.empty())
    {
        fprintf(stderr, "Closing COM_Posit_port\n");
        closePort(COM_Posit_port);
    }
    
    if(!comJacPortString.empty())
    {
        fprintf(stderr, "Closing COM_Jacob_port\n");
        closePort(COM_Jacob_port);
    }
    
    if(!r2lErrPortString.empty())
    {
        fprintf(stderr, "Closing r2l_err_port\n");
        closePort(r2l_err_port);
    }
    
    if(!comErrPortString.empty())
    {
        fprintf(stderr, "Closing COM_Jacob_port\n");
        closePort(COM_err_port);
    }
    
    if(!comCtrlPortString.empty())
    {
        fprintf(stderr, "Closing COM_Jacob_port\n");
        closePort(COM_ctrl_port);
    }

    if(!zmpPortString.empty())
    {
        fprintf(stderr, "Closing zmp_port\n");
        closePort(zmp_port);
    }
    
    if(!desiredComPosPortString.empty())
    {
        fprintf(stderr, "Closing com_des_pos_port\n");
        closePort(com_des_pos_port);
    }

    if(!desiredComVelPortString.empty())
    {
        fprintf(stderr, "Closing com_des_vel_port\n");
        closePort(com_des_vel_port);
    }

    if(!desiredComPhsPortString.empty())
    {
        fprintf(stderr, "Closing com_des_phs_port\n");
        closePort(com_des_phs_port);
    }
    
    if(!desiredR2lPosPortString.empty())
    {
        fprintf(stderr, "Closing r2l_des_pos_port\n");
        closePort(r2l_des_pos_port);
    }
    
    if(!desiredR2lVelPortString.empty())
    {
        fprintf(stderr, "Closing r2l_des_vel_port\n");
        closePort(r2l_des_vel_port);
    }

    
    fprintf(stderr, "Closing commanded_ankle_port\n");
    closePort(ankle_angle);
    fprintf(stderr, "Closing COM_ref_port\n");
    closePort(COM_ref_port);
    fprintf(stderr, "Deleting the right leg\n");
    delete Right_Leg;
    fprintf(stderr, "Deleting the left leg\n");
    delete Left_Leg;
    fprintf(stderr, "Deleting the velocity estimator\n");
    delete zmp_xy_vel_estimator;
    fprintf(stderr, "Deleting minJerk\n");
    delete comMinJerkY; delete comMinJerkZ; delete comMinJerkX;
    delete r2lMinJerkY; delete r2lMinJerkZ; delete r2lMinJerkX;
    if (!strcmp(robot_name.c_str(), "icub"))
        delete inputFilter;
    fprintf(stderr, "Deleting masks\n");
    delete mask_r2l_swg;     delete mask_r2l_sup;
    delete mask_com_torso;
    delete n_r2l_swg; delete n_r2l_sup;
    delete n_com_swg; delete n_com_sup;
    delete n_com_torso;
    
    delete rangeCheckTO; delete rangeCheckLL; delete rangeCheckRL;
    
    fprintf(stderr, "Exiting comStepperThread::threadRelease\n");
    
    
}

void comStepperThread::closePort(Contactable *_port)
{
    if(_port)
    {
        _port->interrupt();
        _port->close();
        
        delete _port;
        _port = 0;
    }
    
}

void comStepperThread::hat(Matrix &hat_p, Matrix p)
{
    
    hat_p(0,0)=       0;  hat_p(0,1)= -p(2,0);  hat_p(0,2)= +p(1,0);
    hat_p(1,0)= +p(2,0);  hat_p(1,1)=       0;  hat_p(1,2)= -p(0,0);
    hat_p(2,0)= -p(1,0);  hat_p(2,1)= +p(0,0);  hat_p(2,2)= 0;
    
}

void comStepperThread::velTranslator(Matrix &out, Matrix p)
{
    
    Matrix hat_p(3,3);
    hat(hat_p, p);
    out.setSubmatrix(eye(3,3)  , 0, 0);    out.setSubmatrix(zeros(3,3) - hat_p, 0, 3);
    out.setSubmatrix(zeros(3,3), 3, 0);    out.setSubmatrix(eye(3,3)          , 3, 3);
}

void comStepperThread::rotTranslator(Matrix &out, Matrix R)
{
    out.setSubmatrix(R         , 0, 0);    out.setSubmatrix(zeros(3,3), 0, 3);
    out.setSubmatrix(zeros(3,3), 3, 0);    out.setSubmatrix(R         , 3, 3);
}

Vector comStepperThread::computeDeltaError(Vector q0R, Vector q0L, Vector q1R, Vector q1L)
{
    Matrix Tba0 = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    Matrix ep0 = pac0 - pac0;
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Vector eo0_tmp = iCub::ctrl::dcm2axis(Rac0 * Rac0.transposed());
    Vector eo0 = eo0_tmp.subVector(0, 2) * sin(eo0_tmp(3));;
    
    Matrix Tba1 = Right_Leg->getH(q1R*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH(q1L*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    Matrix ep1 = pac0 - pac1;
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Vector eo1_tmp = iCub::ctrl::dcm2axis(Rac0 * Rac1.transposed());
    Vector eo1 = eo1_tmp.subVector(0, 2) * sin(eo1_tmp(3));;
    
    Matrix delta_ep = ep1 - ep0;
    Vector delta_eo = eo1 - eo0;
    
    Vector delta_e(6);
    delta_e(0) = delta_ep(0,0);  delta_e(1) = delta_ep(1,0);  delta_e(2) = delta_ep(2,0);
    delta_e.setSubvector(3, delta_eo);
    
    return delta_e;
    
}

Vector comStepperThread::computeDeltaProjB(Vector q0R, Vector q0L, Vector q1R, Vector q1L)
{
    
    Matrix Tba0 = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Matrix pab0 = -1.0*(Rba0.transposed()*pba0);
    Vector pi_b0 = Rba0 * PI * (Rba0.transposed() * zeros(3) + pab0.getCol(0)) + pba0.getCol(0);
    
    Matrix Tba1 = Right_Leg->getH(q1R*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH(q1L*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Matrix pab1 = -1.0*(Rba1.transposed()*pba1);
    Vector pi_b1 = Rba1 * PI * (Rba1.transposed() * zeros(3) + pab1.getCol(0)) + pba1.getCol(0);
    
    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());
    Vector delta_pi_b = pi_b1 - pi_b0;
    return delta_pi_b;
    
}

Vector comStepperThread::computeDeltaProjA(Vector q0R, Vector q0L, Vector q1R, Vector q1L)
{
    
    Matrix Tba0 = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Matrix pab0 = -1.0*(Rba0.transposed()*pba0);
    Vector pi_a0 = PI * (Rba0.transposed() * zeros(3) + pab0.getCol(0));
    
    Matrix Tba1 = Right_Leg->getH(q1R*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH(q1L*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Matrix pab1 = -1.0*(Rba1.transposed()*pba1);
    Vector pi_a1 = PI * (Rba1.transposed() * zeros(3) + pab1.getCol(0));
    
    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());
    //Vector delta_pi_b = pi_b1 - pi_b0;
    Vector delta_pi_a = pi_a1 - pi_a0;
    return delta_pi_a;
    
}

void comStepperThread::computeDeltaProjAReal(Vector dqR, Vector dqL, Vector dqT, Vector &delta_b, Vector &delta_pi_a, Vector &delta_pi_b)
{
    Matrix p_b0(3,1);
    Matrix p_b1(3,1);
    Vector j0RL(6);
    Ienc_RL->getEncoders(j0RL.data());
    Vector j0LL(6);
    Ienc_LL->getEncoders(j0LL.data());
    Vector j0TO(3);
    Ienc_TO->getEncoders(j0TO.data());
    if (comPosPortString.empty())
        p_b0  = zeros(3,1);
    else
        p_b0.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    
    Matrix Tba0 = Right_Leg->getH(j0RL*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(j0LL*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Matrix pab0 = -1.0*(Rba0.transposed()*pba0);
    Vector pi_a0 = PI * (Rba0.transposed() * p_b0.getCol(0) + pab0.getCol(0));
    Vector pi_b0 = Rba0 * PI * (Rba0.transposed() * p_b0.getCol(0) + pab0.getCol(0)) + pba0.getCol(0);
    
    fprintf(stderr, "q: \n %s\n", (j0RL-qRL).toString().c_str());
    fprintf(stderr, "R: \n%s\n", (Rba-Rba0).toString().c_str());
    fprintf(stderr, "p_b: \n%s\n", (p_b0-p_b).toString().c_str());
    fprintf(stderr, "pi_a: \n%s\n", (pab-pab0).toString().c_str());
#ifndef DO_NOT_CONTROL_LEGS
    Ipos_LL->setRefSpeed(0, 100);    Ipos_LL->setRefSpeed(3, 100);
    Ipos_LL->setRefSpeed(1, 100);    Ipos_LL->setRefSpeed(4, 100);
    Ipos_LL->setRefSpeed(2, 100);    Ipos_LL->setRefSpeed(5, 100);
    Ipos_LL->positionMove((j0LL+dqL).data());
    
    Ipos_RL->setRefSpeed(0, 100);    Ipos_RL->setRefSpeed(3, 100);
    Ipos_RL->setRefSpeed(1, 100);    Ipos_RL->setRefSpeed(4, 100);
    Ipos_RL->setRefSpeed(2, 100);    Ipos_RL->setRefSpeed(5, 100);
    Ipos_RL->positionMove((j0RL+dqR).data());
#endif
#ifndef DO_NOT_CONTROL_TORSO
    Ipos_TO->setRefSpeed(0, 100);
    Ipos_TO->setRefSpeed(1, 100);
    Ipos_TO->setRefSpeed(2, 100);
    if (verbose)
        fprintf(stderr, "COM is    %s \n", p_b0.toString().c_str());
    Ipos_TO->positionMove((j0TO+dqT).data());
#endif
    Time::delay(4);
    if (comPosPortString.empty())
        p_b1  = zeros(3,1);
    else
        p_b1.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    if (verbose)
        fprintf(stderr, "COM is now %s \n", p_b1.toString().c_str());
    
    Matrix Tba1 = Right_Leg->getH((j0RL+dqR)*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH((j0LL+dqL)*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Matrix pab1 = -1.0*(Rba1.transposed()*pba1);
    Vector pi_a1 = PI * (Rba1.transposed() * p_b1.getCol(0) + pab1.getCol(0));
    Vector pi_b1 = Rba1 * PI * (Rba1.transposed() * p_b1.getCol(0) + pab1.getCol(0)) + pba1.getCol(0);
    
    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());
    
    delta_b = p_b1.getCol(0) - p_b0.getCol(0);
    delta_pi_a = pi_a1 - pi_a0;
    delta_pi_b = pi_b1 - pi_b0;
}

void comStepperThread::computeDeltaProjCReal(Vector dqR, Vector dqL, Vector dqT, Vector &delta_b, Vector &delta_pi_c, Vector &delta_pi_b)
{
    Matrix p_b0(3,1);
    Matrix p_b1(3,1);
    Vector j0RL(6);
    Ienc_RL->getEncoders(j0RL.data());
    Vector j0LL(6);
    Ienc_LL->getEncoders(j0LL.data());
    Vector j0TO(3);
    Ienc_TO->getEncoders(j0TO.data());
    if (comPosPortString.empty())
        p_b0  = zeros(3,1);
    else
        p_b0.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    
    Matrix Tba0 = Right_Leg->getH(j0RL*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(j0LL*CTRL_DEG2RAD);
    Matrix Tcb0 = iCub::ctrl::SE3inv(Tbc0);
    Matrix Tca0 = Tcb0 * Tba0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pca0 = Tca0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rca0 = Tca0.submatrix(0, 2, 0, 2);
    Matrix pcb0 = -1.0*(Rbc0.transposed()*pbc0);
    Vector pi_c0 = PI * (Rbc0.transposed() * p_b0.getCol(0) + pcb0.getCol(0));
    Vector pi_b0 = Rbc0 * PI * (Rbc0.transposed() * p_b0.getCol(0) + pcb0.getCol(0)) + pbc0.getCol(0);
    
#ifndef DO_NOT_CONTROL_LEGS
    Ipos_LL->setRefSpeed(0, 100);    Ipos_LL->setRefSpeed(3, 100);
    Ipos_LL->setRefSpeed(1, 100);    Ipos_LL->setRefSpeed(4, 100);
    Ipos_LL->setRefSpeed(2, 100);    Ipos_LL->setRefSpeed(5, 100);
    Ipos_LL->positionMove((j0LL+dqL).data());
    
    Ipos_RL->setRefSpeed(0, 100);    Ipos_RL->setRefSpeed(3, 100);
    Ipos_RL->setRefSpeed(1, 100);    Ipos_RL->setRefSpeed(4, 100);
    Ipos_RL->setRefSpeed(2, 100);    Ipos_RL->setRefSpeed(5, 100);
    Ipos_RL->positionMove((j0RL+dqR).data());
#endif
#ifndef DO_NOT_CONTROL_TORSO
    Ipos_TO->setRefSpeed(0, 100);
    Ipos_TO->setRefSpeed(1, 100);
    Ipos_TO->setRefSpeed(2, 100);
    if (verbose)
        fprintf(stderr, "COM is    %s \n", p_b0.toString().c_str());
    Ipos_TO->positionMove((j0TO+dqT).data());
#endif
    
    Time::delay(4);
    if (comPosPortString.empty())
        p_b1  = zeros(3,1);
    else
        p_b1.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    if (verbose)
        fprintf(stderr, "COM is now %s \n", p_b1.toString().c_str());
    
    Matrix Tba1 = Right_Leg->getH((j0RL+dqR)*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH((j0LL+dqL)*CTRL_DEG2RAD);
    Matrix Tcb1 = iCub::ctrl::SE3inv(Tbc1);
    Matrix Tca1 = Tcb1 * Tba1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pca1 = Tca1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rca1 = Tca1.submatrix(0, 2, 0, 2);
    Matrix pcb1 = -1.0*(Rbc1.transposed()*pbc1);
    Vector pi_c1 = PI * (Rbc1.transposed() * p_b1.getCol(0) + pcb1.getCol(0));
    Vector pi_b1 = Rbc1 * PI * (Rbc1.transposed() * p_b1.getCol(0) + pcb1.getCol(0)) + pbc1.getCol(0);
    
    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());
    
    delta_b = p_b1.getCol(0) - p_b0.getCol(0);
    delta_pi_c = pi_c1 - pi_c0;
    delta_pi_b = pi_b1 - pi_b0;
}

void comStepperThread::switchSupport(phase newPhase)
{
    
    if (newPhase == RIGHT_SUPPORT || newPhase == BOTH_SUPPORT)
    {
        Vector H0(1);     H0(0) = pac(0,0);      r2lMinJerkX->init(H0);
        pac_d(0,0) = pac(0,0);
        Vector Y0(1);     Y0(0) = pac(1,0);      r2lMinJerkY->init(Y0);
        pac_d(1,0) = pac(1,0);
        Vector Z0(1);     Z0(0) = pac(2,0);      r2lMinJerkZ->init(Z0);
        pac_d(2,0) = pac(2,0);
        
        Rac_d = Rac;
        
        H0(1);     H0(0) = pi_a(0,0);      comMinJerkX->init(H0);
        pi_a_d(0,0) = pi_a(0,0);
        Y0(1);     Y0(0) = pi_a(1,0);      comMinJerkY->init(Y0);
        pi_a_d(1,0) = pi_a(1,0);
        Z0(1);     Z0(0) = pi_a(2,0);      comMinJerkZ->init(Z0);
        pi_a_d(2,0) = pi_a(2,0);
        
        current_phase = newPhase;
    }
    
    if (newPhase == LEFT_SUPPORT)
    {
        Vector H0(1);     H0(0) = pca(0,0);      r2lMinJerkX->init(H0);
        pca_d(0,0) = pca(0,0);
        Vector Y0(1);     Y0(0) = pca(1,0);      r2lMinJerkY->init(Y0);
        pca_d(1,0) = pca(1,0);
        Vector Z0(1);     Z0(0) = pca(2,0);      r2lMinJerkZ->init(Z0);

        Rca_d = Rca;
        
        H0(1);     H0(0) = pi_c(0,0);      comMinJerkX->init(H0);
        pi_c_d(0,0) = pi_c(0,0);
        Y0(1);     Y0(0) = pi_c(1,0);      comMinJerkY->init(Y0);
        pi_c_d(1,0) = pi_c(1,0);
        Z0(1);     Z0(0) = pi_c(2,0);      comMinJerkZ->init(Z0);
        pi_c_d(2,0) = pi_c(2,0);
        
        current_phase = newPhase;
    }
    
    if (newPhase == LEFT_SUPPORT)
    {
        qrLL = q0LL_left;    qrRL = q0RL_left;    qrTO = q0TO_left;
    }
    if (newPhase == BOTH_SUPPORT)
    {
        qrLL = q0LL_both;    qrRL = q0RL_both;    qrTO = q0TO_both;
    }
    if (newPhase == RIGHT_SUPPORT)
    {
        qrLL = q0LL_right;   qrRL = q0RL_right;   qrTO = q0TO_right;
    }
    
}

void comStepperThread::updateComDesired()
{
    Vector *tmp; Bottle *btmp;

    if (!desiredComPhsPortString.empty())
    {
        btmp = com_des_phs_port->read(false);
        if (!(btmp==NULL))
        {
            if (!strcmp(btmp->toString().c_str(), "right"))
                current_phase = RIGHT_SUPPORT;
            if (!strcmp(btmp->toString().c_str(), "left"))
                current_phase = LEFT_SUPPORT;
            if (!strcmp(btmp->toString().c_str(), "double"))
                current_phase = BOTH_SUPPORT;
        }
    }
    
    if (current_phase==RIGHT_SUPPORT || current_phase == BOTH_SUPPORT)
    {
        if (!desiredComPosPortString.empty())
        {
            tmp = com_des_pos_port->read(false);
            if (!(tmp==NULL))
            {
                pi_a_d.setCol(0, (*tmp).subVector(0, 2)) ;
                dpi_a_d.zero();
            }
        }
        
        if (!desiredComVelPortString.empty())
        {
            tmp = com_des_vel_port->read(false);
            if (!(tmp==NULL))
                dpi_a_d.setCol(0, (*tmp).subVector(0, 2)) ;
        }
        
        if (!desiredR2lPosPortString.empty())
        {
            tmp = r2l_des_pos_port->read(false);
            if (!(tmp==NULL))
            {
                pac_d.setCol(0, (*tmp).subVector(0, 2)) ;
                dpac_d.zero();
            }
        }
        
        if (!desiredR2lVelPortString.empty())
        {
            tmp = r2l_des_vel_port->read(false);
            if (!(tmp==NULL))
                dpac_d.setCol(0, (*tmp).subVector(0, 2)) ;
        }
    }
    else
    {
        if (!desiredComPosPortString.empty())
        {
            tmp = com_des_pos_port->read(false);
            if (!(tmp==NULL))
            {
                pi_c_d.setCol(0, (*tmp).subVector(0, 2)) ;
                dpi_c_d.zero();
            }
        }
        
        if (!desiredComVelPortString.empty())
        {
            tmp = com_des_vel_port->read(false);
            if (!(tmp==NULL))
                dpi_c_d.setCol(0, (*tmp).subVector(0, 2)) ;
        }
        
        if (!desiredR2lPosPortString.empty())
        {
            tmp = r2l_des_pos_port->read(false);
            if (!(tmp==NULL))
            {
                pca_d.setCol(0, (*tmp).subVector(0, 2)) ;
                dpca_d.zero();
            }
        }
        
        if (!desiredR2lVelPortString.empty())
        {
            tmp = r2l_des_vel_port->read(false);
            if (!(tmp==NULL))
                dpca_d.setCol(0, (*tmp).subVector(0, 2)) ;
        }
        
    }
    
}

void comStepperThread::updateComFilters()
{
    
    if (current_phase == RIGHT_SUPPORT || current_phase == BOTH_SUPPORT)
    {
        Vector  uH(1);      uH(0) = pac_t(0,0);
        Vector  uY(1);      uY(0) = pac_t(1,0);
        Vector  uZ(1);      uZ(0) = pac_t(2,0);
        
        r2lMinJerkX->computeNextValues(uH);   Vector yH = r2lMinJerkX->getPos();   Vector ydH = r2lMinJerkX->getVel();
        r2lMinJerkY->computeNextValues(uY);   Vector yY = r2lMinJerkY->getPos();   Vector ydY = r2lMinJerkY->getVel();
        r2lMinJerkZ->computeNextValues(uZ);   Vector yZ = r2lMinJerkZ->getPos();   Vector ydZ = r2lMinJerkZ->getVel();
        
        pac_d(0,0) =  yH(0);              dpac_d(0,0) = ydH(0);
        pac_d(1,0) =  yY(0);              dpac_d(1,0) = ydY(0);
        pac_d(2,0) =  yZ(0);              dpac_d(2,0) = ydZ(0);

        // fprintf(stderr, "dpac_d: %s VS %s, %s \n", dpac_d.transposed().toString().c_str(), ydY.toString().c_str(), ydZ.toString().c_str());
        // fprintf(stderr, " pac_d: %s -> %s\n", pac_d.transposed().toString().c_str(), pac_t.transposed().toString().c_str());
        
        uH(1);      uH(0) = pi_a_t(0,0);
        uY(1);      uY(0) = pi_a_t(1,0);
        uZ(1);      uZ(0) = pi_a_t(2,0);
        
        comMinJerkX->computeNextValues(uH);   yH = comMinJerkX->getPos();   ydH = comMinJerkX->getVel();
        comMinJerkY->computeNextValues(uY);   yY = comMinJerkY->getPos();   ydY = comMinJerkY->getVel();
        comMinJerkZ->computeNextValues(uZ);   yZ = comMinJerkZ->getPos();   ydZ = comMinJerkZ->getVel();
        
        // fprintf(stderr, "dpi_a_d: %s VS %s, %s \n", dpi_a_d.transposed().toString().c_str(), ydY.toString().c_str(), ydZ.toString().c_str());
        pi_a_d(0,0) =  yH(0);              dpi_a_d(0,0) = ydH(0);
        pi_a_d(1,0) =  yY(0);              dpi_a_d(1,0) = ydY(0);
        pi_a_d(2,0) =  yZ(0);              dpi_a_d(2,0) = ydZ(0);
        
        // fprintf(stderr, " pi_a_d: %s -> %s\n", pi_a_d.transposed().toString().c_str(), pi_a_t.transposed().toString().c_str());
    }
    if (current_phase == LEFT_SUPPORT)
    {
        Vector  uH(1);      uH(0) = pca_t(0,0);
        Vector  uY(1);      uY(0) = pca_t(1,0);
        Vector  uZ(1);      uZ(0) = pca_t(2,0);
        
        r2lMinJerkX->computeNextValues(uH);   Vector yH = r2lMinJerkX->getPos();   Vector ydH = r2lMinJerkX->getVel();
        r2lMinJerkY->computeNextValues(uY);   Vector yY = r2lMinJerkY->getPos();   Vector ydY = r2lMinJerkY->getVel();
        r2lMinJerkZ->computeNextValues(uZ);   Vector yZ = r2lMinJerkZ->getPos();   Vector ydZ = r2lMinJerkZ->getVel();
        
        pca_d(0,0) =  yH(0);              dpca_d(0,0) = ydH(0);
        pca_d(1,0) =  yY(0);              dpca_d(1,0) = ydY(0);
        pca_d(2,0) =  yZ(0);              dpca_d(2,0) = ydZ(0);

        // fprintf(stderr, "dpca_d: %s VS %s, %s \n", dpca_d.transposed().toString().c_str(), ydY.toString().c_str(), ydZ.toString().c_str());
        // fprintf(stderr, " pca_d: %s -> %s\n", pac_d.transposed().toString().c_str(), pac_t.transposed().toString().c_str());
        
        uH(1);      uH(0) = pi_c_t(0,0);
        uY(1);      uY(0) = pi_c_t(1,0);
        uZ(1);      uZ(0) = pi_c_t(2,0);
        
        // fprintf(stderr, "dpi_c_d: %s VS %s, %s \n", dpi_c_d.transposed().toString().c_str(), ydY.toString().c_str(), ydZ.toString().c_str());
        
        comMinJerkX->computeNextValues(uH);   yH = comMinJerkX->getPos();   ydH = comMinJerkX->getVel();
        comMinJerkY->computeNextValues(uY);   yY = comMinJerkY->getPos();   ydY = comMinJerkY->getVel();
        comMinJerkZ->computeNextValues(uZ);   yZ = comMinJerkZ->getPos();   ydZ = comMinJerkZ->getVel();
        
        pi_c_d(0,0) =  yH(0);              dpi_c_d(0,0) = ydH(0);
        pi_c_d(1,0) =  yY(0);              dpi_c_d(1,0) = ydY(0);
        pi_c_d(2,0) =  yZ(0);              dpi_c_d(2,0) = ydZ(0);
        
        // fprintf(stderr, " pi_c_d: %s -> %s\n", pi_c_d.transposed().toString().c_str(), pi_c_t.transposed().toString().c_str());
    }
}

bool comStepperThread::check_njLL(int _njLL)
{
    fprintf(stderr, "Joints  left leg %d VS %d \n", njLL, _njLL);
    if (njLL == _njLL)
        return true;
    else
        return false;
}

bool comStepperThread::check_njRL(int _njRL)
{
    // fprintf(stderr, "Joints right leg %d VS %d \n", njRL, _njRL);
    if (njRL == _njRL)
        return true;
    else
        return false;
}

bool comStepperThread::check_njTO(int _njTO)
{
    // fprintf(stderr, "Joints torso leg %d VS %d \n", njTO, _njTO);
    if (njTO == _njTO)
        return true;
    else
        return false;
}

void comStepperThread::defineControlMasks(Vector _mask_r2l_swg, Vector _mask_r2l_sup, Vector _mask_com_torso)
{
    *mask_r2l_swg = _mask_r2l_swg;       *mask_r2l_sup = _mask_r2l_sup;
    *mask_com_torso = _mask_com_torso;
    
    *n_r2l_swg   = (int) dot(*mask_r2l_swg,   *mask_r2l_swg);
    *n_r2l_sup   = (int) dot(*mask_r2l_sup,   *mask_r2l_sup);
    *n_com_torso = (int) dot(*mask_com_torso, *mask_com_torso);
    
    Smask_r2l_swg   = zeros(njRL, njRL);
    Smask_r2l_sup   = zeros(njRL, njRL);
    Smask_com_torso = zeros(njTO, njTO);
    
    for (int i = 0; i < njRL; i++)
        Smask_r2l_swg(i, i) = (*mask_r2l_swg)(i);
    for (int i = 0; i < njLL; i++)
        Smask_r2l_sup(i, i) = (*mask_r2l_sup)(i);
    for (int i = 0; i < njTO; i++)
        Smask_com_torso(i, i) = (*mask_com_torso)(i);
    
    // fprintf(stderr, "Mask r2l SWG is %s\n",   mask_r2l_swg->toString().c_str());
    // fprintf(stderr, "Mask r2l SUP is %s\n",   mask_r2l_sup->toString().c_str());
    // fprintf(stderr, "Mask com SWG is %s\n",   mask_com_swg->toString().c_str());
    // fprintf(stderr, "Mask com SWG is %s\n",   mask_com_sup->toString().c_str());
    // fprintf(stderr, "Mask com TO  is %s\n", mask_com_torso->toString().c_str());
    
    // fprintf(stderr, "Mask r2l SWG is \n%s\n",   Smask_r2l_swg.toString().c_str());
    // fprintf(stderr, "Mask r2l SUP is \n%s\n",   Smask_r2l_sup.toString().c_str());
    // fprintf(stderr, "Mask com SWG is \n%s\n",   Smask_com_swg.toString().c_str());
    // fprintf(stderr, "Mask com SWG is \n%s\n",   Smask_com_sup.toString().c_str());
    // fprintf(stderr, "Mask com TO  is \n%s\n", Smask_com_torso.toString().c_str());
    
}


