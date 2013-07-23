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
#include <modHelp/modHelp.h>
//
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include "reachComBalance.h"
#include "util.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace modHelp;
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



//=====================================================================
//=====================================================================
//
//                      CONTROLLER MODULE
//
//=====================================================================
//=====================================================================


//---------------------------------------------------------
ISIR_Balancer::ISIR_Balancer()
{
    // drivers
    ddRA=ddLA=ddRL=ddLL=0;
    ddT=ddH=0;
    ddCartRA=ddCartLA=0;
    ddGaze=0;
    iposRA=iposLA=iposRL=iposLL=ipos=0;
    iposT=iposH=0;
    iencRA=iencLA=iencRL=iencLL=ienc=0;
    iencT=iencH=0;
    iimpRA=iimpLA=iimpRL=iimpLL=iimp=0;
    icmdRA=icmdLA=icmdRL=icmdLL=icmd=0;
    icrtRA=icrtLA=icrt=0;
    igaze=0;
    
    torsoEnabled = false;
    trackingEnabled = false;
    cartesian_tolerance = 0.01;
    useLeftArm=useRightArm=useHead=false;
    
    //torso
    torsoSwitch.resize(3,0.0);
    torsoLimits.resize(3,4); torsoLimits.zero();
    
    // home configurations
    homePoss.resize(7,0.0); homeVels.resize(7,0.0);
    left_init.resize(7,0.0);
    right_init.resize(7,0.0);
    left_reach.resize(7,0.0);
    right_reach.resize(7,0.0);
    
    // others
    targetPos.resize(3,0.0);
    pickedPos.resize(3,0.0);
    observePos.resize(3,0.0);
    
    x_cmd.resize(3,0.0);
    o_cmd.resize(4,0.0);
    x_cur.resize(3,0.0);
    o_cur.resize(4,0.0);
    xd.resize(3,0.0);
    od.resize(4,0.0);
    //
    
    
}

//---------------------------------------------------------
bool ISIR_Balancer::configure(ResourceFinder &rf)
{
    Time::turboBoost();
    cout<<"Reading parameters from init file... "<<endl;
        
    readString(rf,"local_name",name,"ISIR");
    readString(rf,"robot",robot,"icubSim"); //by default simulator, so we dont break the real one
    //
    readBool(rf, "verbose", verbose, true);
    readInt(rf,"rate",rate,10);
    //
    readBool(rf, "display_only", display_only, true);
    readBool(rf,"no_sensors",no_sensors,true);
    readBool(rf, "feet_sensors", feet_sensors, false);
    readBool(rf, "springs_legs", springs_legs, false);
    readBool(rf, "torso_compensation", torso_compensation, false);
    readBool(rf, "using_gaze", using_gaze, false);
    //
    readBool(rf,"left_arm",useLeftArm,true);
    readBool(rf,"right_arm",useRightArm,true);
    readBool(rf,"head",useHead,true);
    readBool(rf,"enableTorso",torsoEnabled,false);
    readBool(rf,"enableTrackingMode",trackingEnabled,false);
    readBool(rf,"on_table",onTable,true);
    //
    readDouble(rf,"table_height",tableHeight,0.0);
    readDouble(rf,"minX_distanceFromTorso",minX_distanceFromTorso,-0.1);
    readDouble(rf,"maxY_reachLeftArm",maxY_reachLeftArm,0.05);
    readDouble(rf,"maxY_reachRightArm",maxY_reachRightArm,-0.05);
    readDouble(rf,"traj_time",trajTime,3.0);
    readDouble(rf,"reach_tol",reachTol,0.01);
    //
    readVector(rf,"cartesian_tolerances",cartesian_tolerances,7);
    
    // torso part
    //....................................................
    Bottle &bTorso=rf.findGroup("torso");
    getTorsoOptions(bTorso,"pitch",0,torsoSwitch,torsoLimits);
    getTorsoOptions(bTorso,"roll",1,torsoSwitch,torsoLimits);
    getTorsoOptions(bTorso,"yaw",2,torsoSwitch,torsoLimits);
    
    // limbs parts
    //....................................................
    Bottle &bLeftArm=rf.findGroup("left_arm");
    Bottle &bRightArm=rf.findGroup("right_arm");
    Bottle &bRightLeg=rf.findGroup("right_leg");
    Bottle &bLeftLeg=rf.findGroup("left_leg");
    
    getArmOptions(bLeftArm,leftArmJointsStiffness,leftArmJointsDamping,
                  leftArmHandOrienTable);
    getArmOptions(bRightArm, rightArmJointsStiffness,rightArmJointsDamping,
                  rightArmHandOrienTable);
    getLegOptions(bLeftLeg,leftLegJointsStiffness,leftLegJointsDamping);
    getLegOptions(bRightLeg,rightLegJointsStiffness,rightLegJointsDamping);
    
    
    //home configurations
    //....................................................
    readVector(rf,"home_arm_poss",homePoss,7);
    readVector(rf,"home_arm_vels",homeVels,7);
    readVector(rf,"left_init",left_init,7);
    readVector(rf,"right_init",right_init,7);
    readVector(rf,"left_reach",left_reach,7);
    readVector(rf,"right_reach",right_reach,7);
    readVector(rf,"left_touchtable",left_touchtable,7);
    readVector(rf,"right_touchtable",right_touchtable,7);
    
    
    if(verbose)
    {
        cout<<"*** Rate thread working at "<<rate<<"ms"<<endl;
        
        if(display_only)
            cout<<"*** Only displaying ZMP, not controllin the robot\n"<<endl;
        else
            cout<<"*** Controlling the robot for real"<<endl;
        
        if(no_sensors)
            cout<<"*** Only using the sensors available in the simulator."<<endl;
        else
            cout<<"*** Trying to use the sensors of the real robot."<<endl;
        
        if(feet_sensors)
            cout<<"*** Using feet FT sensors to compute COP"
            <<endl;
        else
            cout<<"*** Using legs FT sensors to compute COP"<<endl;
        
        if(torso_compensation)
            cout<<"*** Using torso to compensate for built up angular momentum"<<endl;
        else
            cout<<"*** Not using torso to compensate angular momentum"<<endl;
    }
    
    // rpc port to receive command
    //....................................................
    string rpcPortName = "/"+name+"/rpc:i";
    rpcPort.open(rpcPortName.c_str());
    attach(rpcPort);
    
    
    
    // FINISHED READING PARAMS FROM INI FILE
    
    //===========================================================
    // opening drivers
    
    // right arm
    optionsRA.put("device","remote_controlboard");
    optionsRA.put("local",string("/"+name+"/right_arm").c_str());
    optionsRA.put("remote",string("/"+robot+"/right_arm").c_str());
    // left arm
    optionsLA.put("device","remote_controlboard");
    optionsLA.put("local",string("/"+name+"/left_arm").c_str());
    optionsLA.put("remote",string("/"+robot+"/left_arm").c_str());
    // right leg
    optionsRL.put("device","remote_controlboard");
    optionsRL.put("local",string("/"+name+"/right_arm").c_str());
    optionsRL.put("remote",string("/"+robot+"/right_arm").c_str());
    // left leg
    optionsLL.put("device","remote_controlboard");
    optionsLL.put("local",string("/"+name+"/left_arm").c_str());
    optionsLL.put("remote",string("/"+robot+"/left_arm").c_str());
    // torso
    optionsT.put("device","remote_controlboard");
    optionsT.put("local",string("/"+name+"/torso").c_str());
    optionsT.put("remote",string("/"+robot+"/torso").c_str());
    // head
    optionsH.put("device","remote_controlboard");
    optionsH.put("local",string("/"+name+"/head").c_str());
    optionsH.put("remote",string("/"+robot+"/head").c_str());
    //
    optionsCartRA.put("device","cartesiancontrollerclient");
    optionsCartRA.put("remote",string("/"+robot+"/cartesianController/right_arm").c_str());
    optionsCartRA.put("local",string("/"+name+"/right_arm/cartesian").c_str());
    //
    optionsCartLA.put("device","cartesiancontrollerclient");
    optionsCartLA.put("remote",string("/"+robot+"/cartesianController/left_arm").c_str());
    optionsCartLA.put("local",string("/"+name+"/left_arm/cartesian").c_str());
    //
    optionsGaze.put("device","gazecontrollerclient");
    optionsGaze.put("remote","/iKinGazeCtrl");
    optionsGaze.put("local",string("/"+name+"/gaze").c_str());
    
    
    // torso
    //....................................................
    if(!createDriver(ddH, optionsT))
    {
        cout<<"Error: unable to create driver for torso"<<endl;
        close();
        return false;
    }
    if(!ddT->view(iencT) || !ddT->view(iposT) )
    {
        cout<<"Problems acquiring interfaces of torso"<<endl;
        close();
        return false;
    }
    // init torso
    int torsoAxes;
    iencT->getAxes(&torsoAxes);
    torso.resize(torsoAxes,0.0);
    
    // head
    //....................................................
    if(!createDriver(ddH,optionsH))
    {
        cout<<"Error: unable to create driver of head"<<endl;
        close();
        return false;
    }
    if(!ddH->view(iencH) || !ddH->view(iposH) )
    {
        cout<<"Error: problems acquiring interfaces of head"<<endl;
        close();
        return false;
    }
    // cartesian gaze control
    if(using_gaze)
    {
        if(!createDriver(ddGaze,optionsGaze))
        {
            cout<<"Errors: unable to create driver for gaze controller"<<endl;
            close();
            return false;
        }
        if(!ddGaze->isValid())
        {
            cout<<"Invalid gaze interface"<<endl;
            close();
            return false;
        }
        if(!ddGaze->view(igaze))
        {
            cout<<"Problems acquiring the gaze interface"<<endl;
            close();
            return false;
        }
    }
    
    // init head
    int headAxes;
    iencH->getAxes(&headAxes);
    head.resize(headAxes,0.0);
    
    // right arm
    //....................................................
    if(!createDriver(ddRA,optionsRA))
    {
        cout<<"Problems connecting to the remote driver of right_arm"<<endl;
        close();
        return false;
    }
    if(!ddRA->view(iencRA) || !ddRA->view(iposRA) )
    {
        cout<<"Problems acquiring interfaces of right_arm"<<endl;
        close();
        return false;
    }
    // cartesian interface
    if (!createDriver(ddCartRA,optionsCartRA))
    {
        cout<<"Problems connecting to the Cartesian interface of right_arm"<<endl;
        close();
        return false;
    }
    if(!ddCartRA->isValid())
    {
        cout<<"Invalid Cartesian interface for right_arm"<<endl;
        close();
        return false;
    }
    if(!ddCartRA->view(icrtRA))
    {
        cout<<"Problems acquiring the Cartesian interface of right_arm"<<endl;
        close();
        return false;
    }
    // impedance
    if (rightArmImpVelMode)
    {
        cout<<"Setting impedance in: right_arm"<<endl;
        ddRA->view(icmdRA);
        ddRA->view(iimpRA);
        
        int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
        rightArmJointsStiffness.length():rightArmJointsDamping.length();
        
        for (int j=0; j<len; j++)
        {
            icmdRA->setImpedanceVelocityMode(j);
            iimpRA->setImpedance(j,rightArmJointsStiffness[j],rightArmJointsDamping[j]);
        }
    }
    
    // left arm
    //....................................................
    
    if(!createDriver(ddLA,optionsLA))
    {
        cout<<"Problems connecting to the remote driver of left_arm"<<endl;
        close();
        return false;
    }
    if(!ddLA->view(iencLA) || !ddLA->view(iposLA) )
    {
        cout<<"Problems acquiring interfaces of left_arm"<<endl;
        close();
        return false;
    }
    // cartesian interface
    ddCartLA = new PolyDriver;
    if (!ddCartLA->open(optionsCartLA))
    {
        cout<<"Problems connecting to the Cartesian interface of left_arm"<<endl;
        close();
        return false;
    }
    if(!ddCartLA->isValid())
    {
        cout<<"Invalid Cartesian interface for left_arm"<<endl;
        close();
        return false;
    }
    if(!ddCartLA->view(icrtLA))
    {
        cout<<"Problems acquiring the Cartesian interface of left_arm"<<endl;
        close();
        return false;
    }
    // impedance
    if (leftArmImpVelMode)
    {
        cout<<"Setting impedance in: left_arm"<<endl;
        ddLA->view(icmdLA);
        ddLA->view(iimpLA);
        
        int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
        leftArmJointsStiffness.length():leftArmJointsDamping.length();
        
        for (int j=0; j<len; j++)
        {
            icmdLA->setImpedanceVelocityMode(j);
            iimpLA->setImpedance(j,leftArmJointsStiffness[j],leftArmJointsDamping[j]);
        }
    }
    
    // right leg
    //....................................................
    if(!createDriver(ddRL,optionsRL))
    {
        cout<<"Problems connecting to the remote driver of right_leg"<<endl;
        close();
        return false;
    }
    if(!ddRL->view(iencRL) || !ddRL->view(iposRL) )
    {
        cout<<"Problems acquiring interfaces of right_leg"<<endl;
        close();
        return false;
    }
    
    // left leg
    //....................................................
    if(!createDriver(ddLL,optionsLL))
    {
        cout<<"Problems connecting to the remote driver of left_leg"<<endl;
        close();
        return false;
    }
    if(!ddLL->view(iencLL) || !ddLL->view(iposLL) )
    {
        cout<<"Problems acquiring interfaces of left_leg"<<endl;
        close();
        return false;
    }
    
    cout<<"Latch controllers contexts"<<endl;
    
    // latch the controller context in order to preserve
    // it after closing the module
    // the context contains the dofs status, the tracking mode,
    // the resting positions, the limits and so on.
    //cout<<" .. NONE (DEBUG) "<<endl;
    cout<<" .. right arm"<<endl;
    icrtRA->storeContext(&startup_context_id_RA);
    cout<<" .. left arm"<<endl;
    icrtLA->storeContext(&startup_context_id_LA);
    
    if(using_gaze)
    {
        cout<<" .. gaze"<<endl;
        igaze->storeContext(&startup_context_id_gaze);
    }
    
    cout<<"Init Cartesian Controllers"<<endl;
    initCartesianCtrl(torsoSwitch,torsoLimits,LEFTARM);
    initCartesianCtrl(torsoSwitch,torsoLimits,RIGHTARM);
    
    //set tolerance
    setCartesianTolerance();
    
    // enable torso and limits torso pitch
    if(torsoEnabled==true)
        enableTorso();
    else
        disableTorso();
    
    if(using_gaze)
    {
        cout<<"Set gaze controllers params"<<endl;
        igaze->blockNeckRoll(0.0);
        // set trajectory time:
        igaze->setNeckTrajTime(2.0);
        igaze->setEyesTrajTime(1.0);
        
        // put the gaze in tracking mode, so that
        // when the torso moves, the gaze controller
        // will compensate for it
        igaze->setTrackingMode(true);
    }
    
    
    
    
    // CONTROLLER THREAD
    //-----------------------------------------------
    
    balThread = new ISIR_Controller(rate,
                                    ddRA, ddLA,
                                    ddT, ddH,
                                    ddRL, ddLL,
                                    ddCartRA, ddCartLA,
                                    ddGaze,
                                    robot, name,
                                    display_only, no_sensors,
                                    feet_sensors, springs_legs,
                                    torso_compensation, verbose,
                                    rightArmJointsStiffness, leftArmJointsStiffness,
                                    rightLegJointsStiffness, leftLegJointsStiffness,
                                    rightArmJointsDamping, leftArmJointsDamping,
                                    rightLegJointsDamping, leftLegJointsDamping,
                                    controlMode);
    fprintf(stderr, "Thread created!\n");
    
    // only if we want to type commands directly in the terminal.. may work for some
    // cases or debug
    attachTerminal();
    
    if (balThread->start())
    {
        cout<<"Starting thread!"<<endl;
        //        fcout<<"Initializing offsets on forces!"<<endl;
        //        (*balThread->F_ext_RL0) = w0RL;
        //        (*balThread->F_ext_LL0) = w0LL;
        //        fprintf(stderr, "Checking Device Drivers dimensions!\n");
        
        //        if (!balThread->check_njTO(njTO) || !balThread->check_njLL(njLL) || !balThread->check_njRL(njRL))
        //        {
        //            fprintf(stderr, "Wrong number of torso/legs joints!\n");
        //            return false;
        //        }
        
        // control masks
        //        balThread->defineControlMasks(mask_r2l_swg, mask_r2l_sup, mask_com_torso);
        
        return true;
    }
    else
    {
        cout<<"Something wrong with the thread initialization. Quitting."<<endl;
        return false;
    }
    
    return true;
}
//---------------------------------------------------------

//---------------------------------------------------------
bool ISIR_Balancer::close()
{
    
    cout<< "Stopping controller thread...\n";
    balThread->stop();
    cout<< "controller thread stopped\n";
    
    delete balThread;
    balThread=0;
    
    cout<<"Stopping cartesian controllers"<<endl;
    icrtRA->stopControl();
    icrtLA->stopControl();
    if(using_gaze) igaze->stopControl();
    Time::delay(0.5);
    
    cout<<"Restoring cartesian context"<<endl;
    icrtRA->restoreContext(startup_context_id_RA);
    icrtLA->restoreContext(startup_context_id_LA);
    if (using_gaze) igaze->restoreContext(startup_context_id_gaze);
    Time::delay(0.2);
    
    
    cout<<"Closing rpc port"<<endl;
    rpcPort.interrupt();
    rpcPort.close();
    Time::delay(0.2);
    
    cout<<"Closing drivers"<<endl;
    deleteDriver(ddCartRA);
    deleteDriver(ddCartLA);
    deleteDriver(ddGaze);
    deleteDriver(ddRA);
    deleteDriver(ddLA);
    deleteDriver(ddRL);
    deleteDriver(ddLL);
    deleteDriver(ddT);
    deleteDriver(ddH);
    
    cout<<"balancerModule was closed succesfully!\n";
    return true;
}
//---------------------------------------------------------
double ISIR_Balancer::getPeriod()
{
    return 1;
}
//---------------------------------------------------------
bool ISIR_Balancer::updateModule()
{
    if (balThread->on_ground)
        fprintf(stderr, "I'm running ON  ground!!\n");
    else
        fprintf(stderr, "I'm running OFF ground!!\n");
    
    double avgTime, stdDev, period;
    
    period = balThread->getRate();
    balThread->getEstPeriod(avgTime,stdDev);
    
    if(avgTime > 1.3*period)
    {
        printf("(real period: %3.3f +/- %3.3f. Expected %3.3f )\n",avgTime, stdDev, period);
    }
    
    
    const int p = 2; // number of decimals when printing
    if (balThread->current_phase == BOTH_SUPPORT || balThread->current_phase == RIGHT_SUPPORT)
    {
        fprintf(stderr,"Error in position and orientation: ep: %s;  eo: %s    \n",balThread->ep.getCol(0).toString(p).c_str(),balThread->eo.getCol(0).toString(p).c_str());
        fprintf(stderr,"Error COM projection:               e: %s             \n",(balThread->pi_a_d - balThread->pi_a).getCol(0).toString(p).c_str());
        fprintf(stderr,"Error ZMP projection:               e: %s             \n",(balThread->pi_a_d - balThread->zmp_a).getCol(0).toString(p).c_str());
        fprintf(stderr,"ZMP on right is                   ZMP: %s\n", balThread->zmp_a.toString(p,-1,"\t").c_str());
    }
    else
    {
        fprintf(stderr,"Error in position and orientation: ep: %s;  eo: %s    \n",balThread->ep.getCol(0).toString(p).c_str(),balThread->eo.getCol(0).toString(p).c_str());
        fprintf(stderr,"Error COM projection:               e: %s             \n",(balThread->pi_c_d - balThread->pi_c).getCol(0).toString(p).c_str());
        fprintf(stderr,"Error ZMP projection:               e: %s             \n",(balThread->pi_c_d - balThread->zmp_c).getCol(0).toString(p).c_str());
        fprintf(stderr,"ZMP on left is                    ZMP: %s\n", balThread->zmp_c.toString(p,-1,"\t").c_str());
    }
    
    Vector warningLimTO = balThread->limitMaskTO;
    Vector warningLimRL = balThread->limitMaskRL;
    Vector warningLimLL = balThread->limitMaskLL;
    
    if (norm(warningLimTO) != 0)
        fprintf(stderr, "WARNING reaching limit @ TORSO    : %s\n", warningLimTO.toString(p).c_str());
    if (norm(warningLimRL) != 0)
        fprintf(stderr, "WARNING reaching limit @ RIGHT LEG: %s\n", warningLimRL.toString(p).c_str());
    if (norm(warningLimLL) != 0)
        fprintf(stderr, "WARNING reaching limit @ LEFT  LEG: %s\n", warningLimLL.toString(p).c_str());
    
    return true;
}
//---------------------------------------------------------
bool ISIR_Balancer::respond(const Bottle &command, Bottle &reply)
{
    int b=0;
    ConstString cmd = command.get(b).asString(); b++;
    
    
    cout<<"first command = "<<cmd<<endl;
    
    if (cmd=="quit")
    {
        reply.fromString("Closing the module");
        return false;
    }
    if (cmd=="help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("This is the list of commands:");
        return true;
    }
    
    //the other commands are for commanding movements
    
    if(cmd=="enable_torso")
    {
        reply.clear();
        if(torsoEnabled==false)
        {
            enableTorso();
            reply.addString("Torso enabled");
        }
        else
            reply.addString("Torso already enabled");
        return true;
    }
    else if(cmd=="disable_torso")
    {
        reply.clear();
        if(torsoEnabled==true)
        {
            disableTorso();
            reply.addString("Torso disabled");
        }
        else
            reply.addString("Torso already disabled");
        return true;
    }
    else if(cmd=="recal")
    {
        reply.fromString("+++ Recalibrating! +++ ");
        *balThread->F_ext_RL0 = *balThread->F_ext_RL;
        *balThread->F_ext_LL0 = *balThread->F_ext_LL;
        return true;
    }
    else if(cmd=="goto")
    {
        
        if(command.size()!=9)
        {
            cout<<"Wrong number of parameters: "<<command.size()<<"instead of 9"<<endl;
            reply.clear();
            reply.addString("ERROR: not enough parameters");
            return true;
        }
        
        ConstString lrArm = command.get(b).asString(); b++;
        int selectedArm;
        
        if(lrArm=="left")
            selectedArm=LEFTARM;
        else if(lrArm=="right")
            selectedArm=RIGHTARM;
        else
        {
            cout<<"Left or right must be chosen"<<endl;
            reply.addString("ERROR Unknown arm type");
            return true;
        }
        
        reply.clear();
        
        xd.resize(3); od.resize(4); xd=0.0; od=0.0;
        
        xd[0]=command.get(b).asDouble(); b++;
        xd[1]=command.get(b).asDouble(); b++;
        xd[2]=command.get(b).asDouble(); b++;
        od[0]=command.get(b).asDouble(); b++;
        od[1]=command.get(b).asDouble(); b++;
        od[2]=command.get(b).asDouble(); b++;
        od[3]=command.get(b).asDouble(); b++;
        limitRangeHand(xd,selectedArm);
        cout<<"moving "<<armsTypesName[selectedArm]<<" : x= "<<xd.toString()<<" o= "<<od.toString()<<endl;
        
        if(balThread->moveHand(selectedArm,xd,od))
        {
            cout<<"Hand movement was successful!"<<endl;
            reply.addString("DONE");
            return true;
        }
        else
        {
            cout<<"Hand movement failed"<<endl;
            reply.addString("FAILED");
            return true;
        }
    }
    else if(cmd=="reach")
    {
        if(command.size()!=5)
        {
            cout<<"Wrong number of parameters: "<<command.size()<<"instead of 5"<<endl;
            reply.clear();
            reply.addString("ERROR: not enough parameters");
            return true;
        }
        
        ConstString lrArm = command.get(b).asString(); b++;
        int selectedArm;
        
        if(lrArm=="left")
        {
            selectedArm=LEFTARM;
            od = leftArmHandOrienTable;
        }
        else if(lrArm=="right")
        {
            selectedArm=RIGHTARM;
            od = rightArmHandOrienTable;
        }
        else
        {
            cout<<"Left or right must be chosen"<<endl;
            reply.addString("ERROR Unknown arm type");
            return true;
        }
        
        reply.clear();
        xd.resize(3); od.resize(4); xd=0.0; od=0.0;
        
        xd[0]=command.get(b).asDouble(); b++;
        xd[1]=command.get(b).asDouble(); b++;
        xd[2]=command.get(b).asDouble(); b++;
        limitRangeHand(xd,selectedArm);
        cout<<"moving "<<armsTypesName[selectedArm]<<" : x= "<<xd.toString()<<" o= "<<od.toString()<<endl;
        
        if(balThread->moveHand(selectedArm,xd,od))
        {
            cout<<"Hand movement was successful!"<<endl;
            reply.addString("DONE");
            return true;
        }
        else
        {
            cout<<"Hand movement failed"<<endl;
            reply.addString("FAILED");
            return true;
        }
        
    }
    
    return true;
}
//---------------------------------------------------------
//---------------------------------------------------------
//---------------------------------------------------------
void ISIR_Balancer::getLegOptions(Bottle &b,
                                  Vector &impStiff, Vector &impDamp)
{
    
    if (b.check("impedance_stiffness","Getting joints stiffness"))
    {
        Bottle &grp=b.findGroup("impedance_stiffness");
        int sz=grp.size()-1;
        int len=sz>impStiff.length()?impStiff.length():sz;
        
        for (int i=0; i<len; i++)
            impStiff[i]=grp.get(1+i).asDouble();
    }
    
    if (b.check("impedance_damping","Getting joints damping"))
    {
        Bottle &grp=b.findGroup("impedance_damping");
        int sz=grp.size()-1;
        int len=sz>impDamp.length()?impDamp.length():sz;
        
        for (int i=0; i<len; i++)
            impDamp[i]=grp.get(1+i).asDouble();
    }
    
}
//---------------------------------------------------------
void ISIR_Balancer::getArmOptions(Bottle &b,
                                  Vector &impStiff, Vector &impDamp,
                                  Vector &orienTable)
{
    
    if (b.check("hand_orientation_table","Getting hand orientation for table"))
    {
        Bottle &grp=b.findGroup("hand_orientation_table");
        int sz=grp.size()-1;
        int len=sz>4?4:sz;
        
        for (int i=0; i<len; i++)
            orienTable[i]=grp.get(1+i).asDouble();
    }
    
    if (b.check("impedance_stiffness","Getting joints stiffness"))
    {
        Bottle &grp=b.findGroup("impedance_stiffness");
        int sz=grp.size()-1;
        int len=sz>impStiff.length()?impStiff.length():sz;
        
        for (int i=0; i<len; i++)
            impStiff[i]=grp.get(1+i).asDouble();
    }
    
    if (b.check("impedance_damping","Getting joints damping"))
    {
        Bottle &grp=b.findGroup("impedance_damping");
        int sz=grp.size()-1;
        int len=sz>impDamp.length()?impDamp.length():sz;
        
        for (int i=0; i<len; i++)
            impDamp[i]=grp.get(1+i).asDouble();
    }
    
}

//---------------------------------------------------------
void ISIR_Balancer::limitRangeHand(Vector &x, const int sel)
{
    if(x[0]>minX_distanceFromTorso)
    {
        cout<<"DANGER: x could touch the body!  "<<x[0]<<" => "<<minX_distanceFromTorso<<endl;
        x[0]=-0.1;
    }
    if(onTable)
    {
        if(x[2]<tableHeight)
        {
            cout<<"DANGER: z could crash on table!  "<<x[2]<<" => "<<tableHeight<<endl;
            x[2]=tableHeight;
        }
    }
    if(sel==RIGHTARM)
    {
        if(x[1]<maxY_reachRightArm)
        {
            cout<<"DANGER: y could make icub turn around its torso very badly!  "<<x[1]<<" => "<<maxY_reachRightArm<<endl;
            x[1]=maxY_reachRightArm;
        }
    }
    if(sel==LEFTARM)
    {
        if(x[1]>maxY_reachLeftArm)
        {
            cout<<"DANGER: y could make icub turn around its torso very badly!  "<<x[1]<<" => "<<maxY_reachLeftArm<<endl;
            x[1]=maxY_reachLeftArm;
        }
    }
}
//---------------------------------------------------------
void ISIR_Balancer::getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim)
{
    if (b.check(type))
    {
        Bottle &grp=b.findGroup(type);
        sw[i]=grp.get(1).asString()=="on"?1.0:0.0;
        
        if (grp.check("min","Getting minimum value"))
        {
            lim(i,0)=1.0;
            lim(i,1)=grp.find("min").asDouble();
        }
        
        if (grp.check("max","Getting maximum value"))
        {
            lim(i,2)=1.0;
            lim(i,3)=grp.find("max").asDouble();
        }
    }
}
//---------------------------------------------------------
void ISIR_Balancer::initCartesianCtrl(Vector &sw, Matrix &lim, const int sel)
{
    Vector dof;
    string type;
    
    if (sel==LEFTARM)
    {
        if (useLeftArm)
        {
            ddCartLA->view(icrtLA);
            icrtLA->storeContext(&startup_context_id_LA);
            icrt = icrtLA;
        }
        else
            return;
        
        type="left_arm";
    }
    else if (sel==RIGHTARM)
    {
        if (useRightArm)
        {
            ddCartRA->view(icrtRA);
            icrtRA->storeContext(&startup_context_id_RA);
            icrt = icrtRA;
        }
        else
            return;
        
        type="right_arm";
    }
    else
        return;
    
    cout<<"Initializing cartesian controller for "<<type<<endl;
    
    // must do it outside
    //icrt->setTrackingMode(false);
    icrt->setTrackingMode(trackingEnabled);
    icrt->setTrajTime(trajTime);
    icrt->setInTargetTol(reachTol);
    icrt->getDOF(dof);
    
    for (int j=0; j<sw.length(); j++)
    {
        dof[j]=sw[j];
        
        if (sw[j] && (lim(j,0) || lim(j,2)))
        {
            double min, max;
            icrt->getLimits(j,&min,&max);
            
            if (lim(j,0))
                min=lim(j,1);
            
            if (lim(j,2))
                max=lim(j,3);
            
            icrt->setLimits(j,min,max);
            fprintf(stdout,"jnt #%d in [%g, %g] deg\n",j,min,max);
        }
    }
    
    icrt->setDOF(dof,dof);
    
    fprintf(stdout,"DOF's=( ");
    for (int i=0; i<dof.length(); i++)
        fprintf(stdout,"%s ",dof[i]>0.0?"on":"off");
    fprintf(stdout,")\n");
}
//---------------------------------------------------------
void ISIR_Balancer::enableTorso(int pitch, int roll, int yaw)
{
    Vector curDof;
    Vector newDof(3);
    
    double p = (pitch==0)?0:1;
    double r = (roll==0)?0:1;
    double y = (yaw==0)?0:1;
    
    cout<<"Enabling torso"<<endl;
    
    icrtRA->getDOF(curDof);
    newDof = curDof;
    newDof[0]=p;    // torso pitch: 1 => enable
    newDof[1]=r;    // torso roll:  1 => enable
    newDof[2]=y;    // torso yaw:   1 => enable
    cout<<"current DOF for Cartesian Right ["<<curDof.toString()<<"]"<<endl;
    icrtRA->setDOF(newDof,curDof);
    cout<<"new DOF for Cartesian Right ["<<curDof.toString()<<"]"<<endl;
    
    icrtLA->getDOF(curDof);
    newDof = curDof;
    newDof[0]=p;    // torso pitch: 1 => enable
    newDof[1]=r;    // torso roll:  1 => enable
    newDof[2]=y;    // torso yaw:   1 => enable
    cout<<"current DOF for Cartesian Left ["<<curDof.toString()<<"]"<<endl;
    icrtLA->setDOF(newDof,curDof);
    cout<<"new DOF for Cartesian Left ["<<curDof.toString()<<"]"<<endl;
    
    // impose some restriction on the torso pitch
    limitTorsoPitch();
    torsoEnabled=true;
}
//---------------------------------------------------------
void ISIR_Balancer::disableTorso()
{
    Vector curDof;
    Vector newDof(3);
    
    cout<<"Disabling torso"<<endl;
    
    icrtRA->getDOF(curDof);
    newDof = curDof;
    newDof[0]=0;    // torso pitch: 1 => enable
    newDof[1]=0;    // torso roll:  1 => enable
    newDof[2]=0;    // torso yaw:   1 => enable
    cout<<"current DOF for Cartesian Right ["<<curDof.toString()<<"]"<<endl;
    icrtRA->setDOF(newDof,curDof);
    cout<<"new DOF for Cartesian Right ["<<curDof.toString()<<"]"<<endl;
    
    icrtLA->getDOF(curDof);
    newDof = curDof;
    newDof[0]=0;    // torso pitch: 1 => enable
    newDof[1]=0;    // torso roll:  1 => enable
    newDof[2]=0;    // torso yaw:   1 => enable
    cout<<"current DOF for Cartesian Left ["<<curDof.toString()<<"]"<<endl;
    icrtLA->setDOF(newDof,curDof);
    cout<<"new DOF for Cartesian Left ["<<curDof.toString()<<"]"<<endl;
    
    torsoEnabled = false;
}
//---------------------------------------------------------
void ISIR_Balancer::setCartesianTolerance()
{
    cartesian_tolerance = norm(cartesian_tolerances);
    icrtRA->setInTargetTol(cartesian_tolerance);
    icrtLA->setInTargetTol(cartesian_tolerance);
    cout<<"Setting tolerance = "<<cartesian_tolerance<<" in the cartesian controllers"<<endl;
}
//---------------------------------------------------------
void ISIR_Balancer::changeTrackingMode()
{
    if(trackingEnabled==true)
    {
        // disable tracking mode
        setTrackingMode(false);
    }
    else
    {
        // enable tracking mode
        setTrackingMode(true);
    }
}
//---------------------------------------------------------
void ISIR_Balancer::setTrackingMode(bool mode)
{
    // false    disable
    // true     enable
    icrtRA->setTrackingMode(mode);
    icrtLA->setTrackingMode(mode);
    trackingEnabled=mode;
}
//---------------------------------------------------------
void ISIR_Balancer::limitTorsoPitch()
{
    int axis=0; // pitch joint
    double min, max;
    
    // we don't want the torso
    // to lean out more than 30 degrees forward
    
    cout<<"Limiting torso pitch"<<endl;
    icrtRA->getLimits(axis,&min,&max);
    icrtRA->setLimits(axis,min,30.0);
    icrtLA->getLimits(axis,&min,&max);
    icrtLA->setLimits(axis,min,30.0);
}
//---------------------------------------------------------
//---------------------------------------------------------

//---------------------------------------------------------




//=====================================================================
//=====================================================================
//
//                      CONTROLLER THREAD
//
//=====================================================================
//=====================================================================



//---------------------------------------------------------
ISIR_Controller::ISIR_Controller(int _rate,
                                 PolyDriver *_ddRA, PolyDriver *_ddLA,
                                 PolyDriver *_ddT, PolyDriver *_ddH,
                                 PolyDriver *_ddRL, PolyDriver *_ddLL,
                                 PolyDriver *_ddCartRA, PolyDriver *_ddCartLA,
                                 PolyDriver *_ddGaze,
                                 const string &_robot, const string &_name,
                                 bool _display_only, bool _no_sensors,
                                 bool _ankles_sens, bool _springs_legs,
                                 bool _torso_enable, bool _verbose,
                                 Vector _stiffness_RA, Vector _stiffness_LA,
                                 Vector _stiffness_RL, Vector _stiffness_LL,
                                 Vector _damping_RA, Vector _damping_LA,
                                 Vector _damping_RL, Vector _damping_LL,
                                 ControlMode _controlMode)
:RateThread(_rate), ddRA(_ddRA), ddLA(_ddLA),
ddT(_ddT), ddH(_ddH),
ddRL(_ddRL), ddLL(_ddLL),
ddCartRA(_ddCartRA), ddCartLA(_ddCartLA),
ddGaze(_ddGaze),robot(_robot),name(_name),rate(_rate),display_only(_display_only),no_sensors(_no_sensors),feet_sensors(_ankles_sens), springs_legs(_springs_legs),
torsoEnabled(_torso_enable), verbose(_verbose),
stiffness_RA(_stiffness_RA),  stiffness_LA(_stiffness_LA),
stiffness_RL(_stiffness_RL),  stiffness_LL(_stiffness_LL),
damping_RA(_damping_RA),  damping_LA(_damping_LA),
damping_RL(_damping_RL),  damping_LL(_damping_LL),
controlMode(_controlMode)
{
    // interfaces
    //----------------------------------------
    cout<<"rate: "<<_rate<<endl;
    
    Ilim_TO     = 0;
    Ilim_RL     = 0;
    Ilim_LL     = 0;
    Ilim_HE     = 0;
    Ilim_RA     = 0;
    Ilim_LA     = 0;
    //
    Ipos_TO     = 0;
    Ipos_RL     = 0;
    Ipos_LL     = 0;
    Ipos_HE     = 0;
    Ipos_RA     = 0;
    Ipos_LA     = 0;
    //
    Ictrl_TO    = 0;
    Ictrl_LL    = 0;
    Ictrl_RL    = 0;
    Ictrl_HE    = 0;
    Ictrl_LA    = 0;
    Ictrl_RA    = 0;
    //
    Ivel_TO     = 0;
    Ivel_LL     = 0;
    Ivel_RL     = 0;
    Ivel_HE     = 0;
    Ivel_LA     = 0;
    Ivel_RA     = 0;
    //
    Ienc_TO     = 0;
    Ienc_LL     = 0;
    Ienc_RL     = 0;
    Ienc_HE     = 0;
    Ienc_LA     = 0;
    Ienc_RA     = 0;
    //
    Ipid_TO     = 0;
    Ipid_LL     = 0;
    Ipid_RL     = 0;
    Ipid_HE     = 0;
    Ipid_LA     = 0;
    Ipid_RA     = 0;
    //
    Iimp_RA     = 0;
    Iimp_LA     = 0;
    Iimp_RL     = 0;
    Iimp_LL     = 0;
    Iimp_TO     = 0;
    //
    icrt_RA     = 0;
    icrt_LA     = 0;
    
//    
//    n_r2l_swg   = new int;    n_r2l_sup   = new int;    n_com_swg   = new int;
//    n_com_sup   = new int;    n_com_torso = new int;
    
    Opt_display = display_only;
    Opt_nosens = no_sensors;
    Opt_ankles_sens = feet_sensors;
    
    
    //---------------------------- PORTS ---------------------------------------------
    if (!no_sensors)
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
    
    
    Right_Leg    = new iCubLeg("right");
    Left_Leg     = new iCubLeg("left");
    Inertia_Sens = new iCubInertialSensor("v2");
    
    version_tag icub_type;
    
    icub_type.head_version = 2; //1;
    icub_type.legs_version = 2; //1;
    
    icub = new iCubWholeBody(icub_type, DYNAMIC, verbose);
    
    linEstUp =new AWLinEstimator(16,1.0);
    quadEstUp=new AWQuadEstimator(25,1.0);
    linEstLow =new AWLinEstimator(16,1.0);
    quadEstLow=new AWQuadEstimator(25,1.0);
    InertialEst = new AWLinEstimator(16,1.0);
}

//---------------------------------------------------------
bool ISIR_Controller::threadInit()
{
    on_ground = true;
    
    fprintf(stderr, " *** initializing the device drivers \n");
    F_ext_LL = new Vector(6);
    F_ext_LL0 = new Vector(6);
    *F_ext_LL0 = zeros(6);
    
    F_ext_RL = new Vector(6);
    F_ext_RL0 = new Vector(6);
    *F_ext_RL0 = zeros(6);
    
    //POLIDRIVERS AND INTERFACES
    
    // Torso Interface
    ddT->view(Ienc_TO);
    ddT->view(Ictrl_TO);
    ddT->view(Ivel_TO);
    ddT->view(Ipid_TO);
    ddT->view(Ipos_TO);
    ddT->view(Ilim_TO);
    if((!ddT) || (!Ienc_TO) || (!Ictrl_TO) || (!Ivel_TO) || (!Ipid_TO) || (!Ipos_TO) || (!Ilim_TO))
    {
        printf("ERROR acquiring torso interfaces\n");
        return false;
    }
    
    //Right leg interfaces
    ddRL->view(Ienc_RL);
    ddRL->view(Ictrl_RL);
    ddRL->view(Ivel_RL);
    ddRL->view(Ipid_RL);
    ddRL->view(Ipos_RL);
    ddRL->view(Ilim_RL);
    if((!ddRL) || (!Ienc_RL) || (!Ictrl_RL) || (!Ivel_RL) || (!Ipid_RL) || (!Ipos_RL) || (!Ilim_RL))
    {
        printf("ERROR acquiring right leg interfaces\n");
        return false;
    }
    
    
    //Left leg interfaces
    ddLL->view(Ienc_LL);
    ddLL->view(Ictrl_LL);
    ddLL->view(Ivel_LL);
    ddLL->view(Ipid_LL);
    ddLL->view(Ipid_LL);
    ddLL->view(Ipos_LL);
    ddLL->view(Ilim_LL);
    if((!ddLL) || (!Ienc_LL) || (!Ictrl_LL) || (!Ivel_LL) || (!Ipid_LL) || (!Ipos_LL) || (!Ilim_LL))
    {
        printf("ERROR acquiring left leg interfaces\n");
        return false;
    }
    
    //Left arm interfaces
    ddLA->view(Ienc_LA);
    ddLA->view(Ictrl_LA);
    ddLA->view(Ivel_LA);
    ddLA->view(Ipid_LA);
    ddLA->view(Ipid_LA);
    ddLA->view(Ipos_LA);
    ddLA->view(Ilim_LA);
    if((!ddLA) || (!Ienc_LA) || (!Ictrl_LA) || (!Ivel_LA) || (!Ipid_LA) || (!Ipos_LA) || (!Ilim_LA))
    {
        printf("ERROR acquiring left arm interfaces\n");
        return false;
    }
    
    //Right arm interfaces
    ddRA->view(Ienc_RA);
    ddRA->view(Ictrl_RA);
    ddRA->view(Ivel_RA);
    ddRA->view(Ipid_RA);
    ddRA->view(Ipid_RA);
    ddRA->view(Ipos_RA);
    ddRA->view(Ilim_RA);
    if((!ddRA) || (!Ienc_RA) || (!Ictrl_RA) || (!Ivel_RA) || (!Ipid_RA) || (!Ipos_RA) || (!Ilim_RA))
    {
        printf("ERROR acquiring right arm interfaces\n");
        return false;
    }
    
    fprintf(stderr, " *** moving torso to a given configuration \n");
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
    
    return true;
}


//---------------------------------------------------------
void ISIR_Controller::run()
{
    //updating Time Stamp
    static Stamp timeStamp;
    timeStamp.update();
    
    //read from the encoders and update the model of the robot
    readAndUpdate();
    
    
    //********************** COMPUTE JACOBIANS ************************************
    // In this notation we define a,b,c as follows:
    // right_foot_reference_frame = 'a';
    // base_reference_frame       = 'b';
    // left_foot_reference_frame  = 'c';
    
    Ienc_RL->getEncoders(qRL.data());
    qRL_rad = CTRL_DEG2RAD * qRL;
    Right_Leg->setAng(qRL_rad);
    
    Ienc_LL->getEncoders(qLL.data());
    qLL_rad = CTRL_DEG2RAD * qLL;
    Left_Leg->setAng(qLL_rad);
    
    Ienc_TO->getEncoders(qTO.data());
    qTO_rad = CTRL_DEG2RAD * qTO;
    
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
    
    if (!no_sensors)
    {
        F_ext_RL = port_ft_foot_right->read(true);
        F_ext_LL = port_ft_foot_left->read(true);
        
        *F_ext_RL = *F_ext_RL - *F_ext_RL0;
        *F_ext_LL = *F_ext_LL - *F_ext_LL0;
        
        // fprintf(stderr, "Right forces in double support are %s\n", (* F_ext_RL).toString().c_str());
        // fprintf(stderr, "Left  forces in double support are %s\n", (* F_ext_LL).toString().c_str());
    }
    
    // check if the robot is not in contact with the ground
    if (!no_sensors)
    {
        if ((*F_ext_LL)[2] < -50.0 || (*F_ext_RL)[2] < -50.0  )
            on_ground = true;
        else
            on_ground = false;
    }
    
    
    
    
    
    
    
    
    // the real controller from Joseph/MingXing
    //------------------------------------
    computeControl_ISIR();
    
    
    // output on ports
    //---------------------------------------
    
    
}

//---------------------------------------------------------
void ISIR_Controller::setRefAcc(IEncoders* iencs, IVelocityControl* ivel)
{
    Vector tmp;  int nj;
    iencs->getAxes(&nj);
    tmp.resize(nj);
    for (int i=0;i<=nj;i++)
    {
        tmp[i]=800;
    }
    ivel->setRefAccelerations(tmp.data());
}
//---------------------------------------------------------


//---------------------------------------------------------
bool ISIR_Controller::onStop()
{
    fprintf(stderr, "Stopping the comStepperThread\n");
    return true;
}
//---------------------------------------------------------
void ISIR_Controller::suspend()
{
    // Should delete dynamically created data-structures
    // Beware this is quite dangerous!! Could cause discontinuities in the control
    
    icrt_LA->stopControl();
    icrt_RA->stopControl();
    
    Ivel_LL->stop();    Ipos_LL->setPositionMode();
    Ivel_RL->stop();    Ipos_RL->setPositionMode();
    Ivel_TO->stop();    Ipos_TO->setPositionMode();
    Ivel_HE->stop();    Ipos_HE->setPositionMode();
    Ivel_LA->stop();    Ipos_LA->setPositionMode();
    Ivel_RA->stop();    Ipos_RA->setPositionMode();
}
//---------------------------------------------------------
void ISIR_Controller::threadRelease()
{
    // Should delete dynamically created data-structures
    
    icrt_LA->stopControl();
    icrt_RA->stopControl();
    
    Ivel_LL->stop();    Ipos_LL->setPositionMode();
    Ivel_RL->stop();    Ipos_RL->setPositionMode();
    Ivel_TO->stop();    Ipos_TO->setPositionMode();
    Ivel_HE->stop();    Ipos_HE->setPositionMode();
    Ivel_LA->stop();    Ipos_LA->setPositionMode();
    Ivel_RA->stop();    Ipos_RA->setPositionMode();
    
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
    delete minJerkY; delete minJerkZ; delete minJerkH;
    fprintf(stderr, "Deleting masks\n");
    delete mask_r2l_swg;     delete mask_r2l_sup;
    delete mask_com_torso;
    delete n_r2l_swg; delete n_r2l_sup;
    delete n_com_swg; delete n_com_sup;
    delete n_com_torso;
    
    delete rangeCheckTO; delete rangeCheckLL; delete rangeCheckRL;
    
    fprintf(stderr, "Exiting comStepperThread::threadRelease\n");
    
    
}
//---------------------------------------------------------


//---------------------------------------------------------
bool ISIR_Controller::check_njLL(int _njLL)
{
    fprintf(stderr, "Joints  left leg %d VS %d \n", njLL, _njLL);
    if (njLL == _njLL)
        return true;
    else
        return false;
}
//---------------------------------------------------------
bool ISIR_Controller::check_njRL(int _njRL)
{
    // fprintf(stderr, "Joints right leg %d VS %d \n", njRL, _njRL);
    if (njRL == _njRL)
        return true;
    else
        return false;
}
//---------------------------------------------------------
bool ISIR_Controller::check_njTO(int _njTO)
{
    // fprintf(stderr, "Joints torso leg %d VS %d \n", njTO, _njTO);
    if (njTO == _njTO)
        return true;
    else
        return false;
}
//---------------------------------------------------------


//---------------------------------------------------------
bool ISIR_Controller::readAndUpdate(bool waitMeasure)
{
    bool b = true;
    
    b &= getUpperEncodersSpeedAndAcceleration();
    b &= getLowerEncodersSpeedAndAcceleration();
    setUpperLowerMeasure();
    
    return b;
}
//---------------------------------------------------------
bool ISIR_Controller::getLowerEncodersSpeedAndAcceleration()
{
    bool b = true;
    if (Ienc_LL)
    {b &= Ienc_LL->getEncoders(encoders_leg_left.data());}
    else
    {encoders_leg_left.zero();}
    
    if (Ienc_RL)
    {b &= Ienc_RL->getEncoders(encoders_leg_right.data());}
    else
    {encoders_leg_right.zero();}
    
    if (Ienc_TO)
    {b &= Ienc_TO->getEncoders(encoders_torso.data());}
    else
    {encoders_torso.zero();}
    
    for (size_t i=0;i<3;i++)
    {
        all_q_low(i) = encoders_torso(2-i);
    }
    for (size_t i=0;i<6;i++)
    {
        all_q_low(3+i) = encoders_leg_left(i);
    }
    for (size_t i=0;i<6;i++)
    {
        all_q_low(3+6+i) = encoders_leg_right(i);
    }
    all_dq_low = evalVelLow(all_q_low);
    all_d2q_low = evalAccLow(all_q_low);
    
    return b;
}
//---------------------------------------------------------
bool ISIR_Controller::getUpperEncodersSpeedAndAcceleration()
{
    bool b = true;
    if (Ienc_LA) b &= Ienc_LA->getEncoders(encoders_arm_left.data());
    else encoders_arm_left.zero();
    if (Ienc_RA) b &= Ienc_RA->getEncoders(encoders_arm_right.data());
    else encoders_arm_right.zero();
    if (Ienc_HE) b &= Ienc_HE->getEncoders(encoders_head.data());
    else encoders_head.zero();
    
    for (size_t i=0;i<3;i++)
    {
        all_q_up(i) = encoders_head(i);
    }
    for (size_t i=0;i<7;i++)
    {
        all_q_up(3+i) = encoders_arm_left(i);
    }
    for (size_t i=0;i<7;i++)
    {
        all_q_up(3+7+i) = encoders_arm_right(i);
    }
    all_dq_up = evalVelUp(all_q_up);
    all_d2q_up = evalAccUp(all_q_up);
    
    return b;
}
//---------------------------------------------------------
Vector ISIR_Controller::get_q_head()    {return all_q_up.subVector(0,2);}
Vector ISIR_Controller::get_dq_head()   {return all_dq_up.subVector(0,2);}
Vector ISIR_Controller::get_d2q_head()  {return all_d2q_up.subVector(0,2);}
Vector ISIR_Controller::get_q_larm()    {return all_q_up.subVector(3,9);}
Vector ISIR_Controller::get_dq_larm()   {return all_dq_up.subVector(3,9);}
Vector ISIR_Controller::get_d2q_larm()  {return all_d2q_up.subVector(3,9);}
Vector ISIR_Controller::get_q_rarm()    {return all_q_up.subVector(10,16);}
Vector ISIR_Controller::get_dq_rarm()   {return all_dq_up.subVector(10,16);}
Vector ISIR_Controller::get_d2q_rarm()  {return all_d2q_up.subVector(10,16);}
Vector ISIR_Controller::get_q_torso()   {return all_q_low.subVector(0,2);}
Vector ISIR_Controller::get_dq_torso()  {return all_dq_low.subVector(0,2);}
Vector ISIR_Controller::get_d2q_torso() {return all_d2q_low.subVector(0,2);}
Vector ISIR_Controller::get_q_lleg()    {return all_q_low.subVector(3,8);}
Vector ISIR_Controller::get_dq_lleg()   {return all_dq_low.subVector(3,8);}
Vector ISIR_Controller::get_d2q_lleg()  {return all_d2q_low.subVector(3,8);}
Vector ISIR_Controller::get_q_rleg()    {return all_q_low.subVector(9,14);}
Vector ISIR_Controller::get_dq_rleg()   {return all_dq_low.subVector(9,14);}
Vector ISIR_Controller::get_d2q_rleg()  {return all_d2q_low.subVector(9,14);}
//---------------------------------------------------------
void ISIR_Controller::setUpperLowerMeasure()
{
    
    icub->lowerTorso->setAng("torso",CTRL_DEG2RAD * get_q_torso());
    icub->lowerTorso->setDAng("torso",CTRL_DEG2RAD * get_dq_torso());
    icub->lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * get_d2q_torso());
    
    icub->lowerTorso->setAng("left_leg",CTRL_DEG2RAD * get_q_lleg());
    icub->lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * get_dq_lleg());
    icub->lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * get_d2q_lleg());
    
    icub->lowerTorso->setAng("right_leg",CTRL_DEG2RAD * get_q_rleg());
    icub->lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * get_dq_rleg());
    icub->lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * get_d2q_rleg());
    
    icub->upperTorso->setAng("head",CTRL_DEG2RAD * get_q_head());
    icub->upperTorso->setDAng("head",CTRL_DEG2RAD * get_dq_head());
    icub->upperTorso->setD2Ang("head",CTRL_DEG2RAD * get_d2q_head());
    icub->upperTorso->setAng("left_arm",CTRL_DEG2RAD * get_q_larm());
    icub->upperTorso->setDAng("left_arm",CTRL_DEG2RAD * get_dq_larm());
    icub->upperTorso->setAng("right_arm",CTRL_DEG2RAD * get_q_rarm());
    icub->upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * get_d2q_larm());
    icub->upperTorso->setDAng("right_arm",CTRL_DEG2RAD * get_dq_rarm());
    icub->upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD * get_d2q_rarm());
    
}
//---------------------------------------------------------
Vector ISIR_Controller::evalVelUp(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return linEstUp->estimate(el);
}
//---------------------------------------------------------
Vector ISIR_Controller::evalVelLow(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return linEstLow->estimate(el);
}
//---------------------------------------------------------
Vector ISIR_Controller::eval_domega(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return InertialEst->estimate(el);
}
//---------------------------------------------------------
Vector ISIR_Controller::evalAccUp(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return quadEstUp->estimate(el);
}
//---------------------------------------------------------
Vector ISIR_Controller::evalAccLow(const Vector &x)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return quadEstLow->estimate(el);
}
//---------------------------------------------------------



//---------------------------------------------------------
void ISIR_Controller::computeControl_ISIR()
{
    //    iCub::iDyn::iCubLegDynV2 leftleg;
    //    Vector ddp0(6,0.0); ddp0[5]=9.81;
    //
    //    Matrix M_leftleg = leftleg.computeMassMatrix(q_leftleg);
    //    Vector G = leftleg.computeGravityTorques(ddp0, q_leftleg);
    //    Matrix J_leftleg = leftleg.GeoJacobian();
    //    Vector pos_leftleg = leftleg.EndEffPosition();
    //    Matrix DJ_leftleg = leftleg.DJacobian(dq_leftleg);
    //    //
    //    // velocity of the position of the end-effector (cartesian space)
    //    int nDof = leftleg.getDOF();
    //    Matrix Jac_link_i = leftleg.GeoJacobian(i);
    //    Vector q_left = leftleg.getAng();
    //    Vector dq_left = leftleg.getDAng();
    //    Vector ddq_left = leftleg.getD2Ang();
    //
    //
    //    iCub::iDyn::iCubTorsoDyn torso;
    //
    //    iCub::iDyn::iCubWholeBody wholeICub;
    //
    //    //given whole icub: wholeICub->lower->left
    //
    //    torso.getMass(i);
    
    
}
//---------------------------------------------------------
bool ISIR_Controller::moveHand(const int sel_arm, const Vector &xd, const Vector &od)
{
    if(sel_arm==LEFTARM)
    {
        return true;
    }
    else if(sel_arm==RIGHTARM)
    {
        return true;
    }
    else
    {
        return false;
    }
}
//---------------------------------------------------------


