#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "comStepper.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;   
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace std;

class Balancer: public RFModule
{
private:
    //Module parameters and class variables.
    Property OptionsTorso;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    PolyDriver *dd_torso;
    PolyDriver *dd_rightLeg;
    PolyDriver *dd_leftLeg;

    comStepperThread *balThread;
    Port rpcPort;

    Vector pi_a_d_right, pi_a_d_both, pi_c_d_left;
public:
    Balancer()
    {
        dd_torso = 0;
        dd_rightLeg = 0;
        dd_leftLeg = 0;
    }

    bool configure(ResourceFinder &rf)
    {
        //----------------------------------- LOCAL NAME --------------------------------------------------
        string local_name = "balanceModule";
        if (rf.check("local"))
        {
            local_name = rf.find("local").asString();
        }

        string wbs_name = "/wholeBodyDynamics";
        if (rf.check("local_wbs"))
        {
            wbs_name = rf.find("local_wbs").asString();
        }
        
        //Another way to get local module name parameter:
        // local_name = rf.check("local", Value("balanceModule"), "module name (string)").asString;
        // setName(local_name.c_str());             //setting the read local_name as module name

        //--------------------------------- SOME RPC PORT --------------------------------------------------
        string rpcPortName = "/"+local_name+"/rpc:i";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        //--------------------------------GETTING ROBOT NAME------------------------------------------------
        string robot_name;
        if (rf.check("robot"))
            robot_name = rf.find("robot").asString();
        else
            robot_name = "icub";

        //Another way to do this:
        //string robot_name = rf.check("robot", Value ("icub"), "Robot name (string)").asString();
        //string robotPortName = "/" +robot_name+ "/head";        //robotPortName can be used later on to create ports

        //------------------------Displaying ZMP / Controlling Real Robot----------------------------------
        bool display = false;
        if (rf.check("display_only"))
        {
            if(rf.find("display_only").asString() == "on"){
                display=true;
                fprintf(stderr, "Only displaying ZMP, not controllin the robot\n");
            }
            else
                display=false;
        }        
        //------------------------Avoid using the sensors not available in the iCub_SIM----------------------------------
        bool noSens = false;
        if (rf.check("no_sens"))
        {
            if(rf.find("no_sens").asString() == "on"){
                noSens=true;
                fprintf(stderr, "Only using the sensors available in the simulator. \n");
            }
            if(rf.find("no_sens").asString() == "off"){
                noSens=false;
                fprintf(stderr, "Trying to use the sensors of the real robot.\n");
            }
        }
        //----------------------------------- RATE --------------------------------------------------------
        int rate=10; //ms
        if (rf.check("rate"))
        {
            rate = rf.find("rate").asInt();
            fprintf(stderr,"rateThread working at %d ms\n",rate);            
        }
        else
        {
            fprintf(stderr,"Rate not specified. Using 10ms as default\n");            
            rate = 10;
        }

        //---------------------------------Ankles Sensors--------------------------------------------------
        bool ankles_sens = false;
        if (rf.check("ankles_sens"))
        {
            if(rf.find("ankles_sens").asString() == "on"){
                ankles_sens = true;
                fprintf(stderr, "Using ankles F/T sensors to compute COP\n");            
            }
            else
                ankles_sens = false;
        }
        else
        {
            fprintf(stderr, "Using Legs F/T sensors to compute COP\n");
        }
        //--------------------------------- SPRINGS -------------------------------------------------------
        bool springs = false;
        if (rf.check("springs"))
        {
            if(rf.find("springs").asString() == "on"){
                springs = true;
                fprintf(stderr, "Using springs on the leg joints\n");
            }
            else
                springs = false;
        }

        //--------------------------------- TORSO ---------------------------------------------------------
        bool torso = false;
        if (rf.check("torso"))
        {
            if(rf.find("torso").asString() == "on"){
                torso = true;
                fprintf(stderr, "Using torso to compensate for built up angular momentum \n");
            }
            else
                torso = false;
        }

        //-------------------------------- VERBOSE --------------------------------------------------------
        bool verbose = false;
        if (rf.check("verbose"))
        {
            if(rf.find("verbose").asString() == "on")
                verbose = true;
            else
                verbose = false;
        }

        //------------------------------- SATURATIONS -----------------------------------------------------------
        double vel_sat = 0.40;
        if(rf.findGroup("SATURATIONS").check("vel_sat"))
            vel_sat = rf.findGroup("SATURATIONS").check("vel_sat", Value(10.0)).asDouble();
        
        //------------------------------- DESIRED ZMP LOCATIONS -----------------------------------------------------------        
        
        pi_a_d_right.resize(3);
        if(rf.findGroup("COM_GAINS").check("pi_a_d_right"))
            for (int i = 0; i < 3; i++)
                pi_a_d_right(i) = rf.findGroup("COM_GAINS").check("pi_a_d_right", Value("none")).asList()->get(i).asDouble();

        pi_c_d_left.resize(3);
        if(rf.findGroup("COM_GAINS").check("pi_c_d_left"))
            for (int i = 0; i < 3; i++)
                pi_c_d_left(i) = rf.findGroup("COM_GAINS").check("pi_c_d_left", Value("none")).asList()->get(i).asDouble();

        pi_a_d_both.resize(3);
        if(rf.findGroup("COM_GAINS").check("pi_a_d_both"))
            for (int i = 0; i < 3; i++)
                pi_a_d_both(i) = rf.findGroup("COM_GAINS").check("pi_a_d_both", Value("none")).asList()->get(i).asDouble();

        Matrix pi_a_t0(3,1);   pi_a_t0(0,0) = pi_a_d_both(0);   pi_a_t0(1,0) = pi_a_d_both(1);   pi_a_t0(2,0) = pi_a_d_both(2);
        //------------------------------- GAINS [ANKLES] -----------------------------------------------------------
        double Kp_zmp_h = 0.40;
        if(rf.findGroup("COM_GAINS").check("Kp_h"))
            Kp_zmp_h = rf.findGroup("COM_GAINS").check("Kp_h", Value(0.40)).asDouble();
        
        double Kd_zmp_h = 0.005;
        if(rf.findGroup("COM_GAINS").check("Kd_h"))
            Kd_zmp_h = rf.findGroup("COM_GAINS").check("Kd_h", Value(0.005)).asDouble();

        double Kp_zmp_x = 0.40;
        if(rf.findGroup("COM_GAINS").check("Kp_x"))
            Kp_zmp_x = rf.findGroup("COM_GAINS").check("Kp_x", Value(0.40)).asDouble();
  
        double Kd_zmp_x = 0.005;
        if(rf.findGroup("COM_GAINS").check("Kd_x"))
            Kd_zmp_x = rf.findGroup("COM_GAINS").check("Kd_x", Value(0.005)).asDouble();

        double Kp_zmp_y = 0.0;
        if(rf.findGroup("COM_GAINS").check("Kp_y"))
            Kp_zmp_y = rf.findGroup("COM_GAINS").check("Kp_y", Value(0.0)).asDouble();

        double Kd_zmp_y = 0.0;
        if(rf.findGroup("COM_GAINS").check("Kd_y"))
            Kd_zmp_y = rf.findGroup("COM_GAINS").check("Kd_y", Value(0.0)).asDouble();

        //---------------------------- GAINS [TORSO] ---------------------------------------------------------------
        double Kp = 0.0;
        if(rf.findGroup("R2L_GAINS").check("Kp"))
            Kp = rf.findGroup("R2L_GAINS").check("Kp", Value(0.0)).asDouble();

        double Kd = 0.0;
        if(rf.findGroup("R2L_GAINS").check("Kd"))
            Kd = rf.findGroup("R2L_GAINS").check("Kd", Value(0.0)).asDouble();

        //---------------------------- COM PORTS ---------------------------------------------------------------
        string comPosInputPortName;
        if(rf.check("com_position"))
            comPosInputPortName = rf.check("com_position", Value("/comStepper/com_position:i")).asString().c_str();
        
        string comJacInputPortName;
        if(rf.check("com_jacobian"))
            comJacInputPortName = rf.check("com_jacobian", Value("/comStepper/com_jacobian:i")).asString().c_str();
        
        //---------------------------- ERR PORTS ---------------------------------------------------------------
        string r2lErrorPortName;
        if(rf.check("r2l_error"))
            r2lErrorPortName = rf.check("r2l_error", Value("/comStepper/r2l_error:o")).asString().c_str();
        
        string comErrorPortName;
        if(rf.check("com_error"))
            comErrorPortName = rf.check("com_error", Value("/comStepper/com_error:o")).asString().c_str();

        //---------------------------- WRENCH OFFSETS ---------------------------------------------------------------
        Vector w0RL(6);
        if(rf.check("w0RL"))
            for (int i = 0; i < 6; i++)
                w0RL(i) = rf.find("w0RL").asList()->get(i).asDouble();
        
        Vector w0LL(6);
        if(rf.check("w0LL"))
            for (int i = 0; i < 6; i++)
                w0LL(i) = rf.find("w0LL").asList()->get(i).asDouble();

        //---------------------------- CONTROL MASKS ---------------------------------------------------------------
        int njTO = 0;
        if(rf.check("njTO"))
            njTO = rf.check("njTO", Value(3)).asInt();

        int njRL = 0;
        if(rf.check("njRL"))
            njRL = rf.check("njRL", Value(6)).asInt();

        int njLL = 0;
        if(rf.check("njLL"))
            njLL = rf.check("njLL", Value(6)).asInt();

        Vector mask_r2l_swg = zeros(njRL);
        if(rf.check("r2l_jointsSwgLeg"))
            for (int i = 0; i < njRL; i++)
                mask_r2l_swg(i) = rf.find("r2l_jointsSwgLeg").asList()->get(i).asInt();

        Vector mask_r2l_sup = zeros(njRL);
        if(rf.check("r2l_jointsSupLeg"))
            for (int i = 0; i < njRL; i++)
                mask_r2l_sup(i) = rf.find("r2l_jointsSupLeg").asList()->get(i).asInt();
        
        Vector mask_com_torso = zeros(njTO);
        if(rf.check("com_jointsTorso"))
            for (int i = 0; i < njTO; i++)
                mask_com_torso(i) = rf.find("com_jointsTorso").asList()->get(i).asInt();
        
        //------------------------------- INITIAL POSITION ------------------------------------------------

        Vector q0LL_both = zeros(njLL);
        if(rf.check("q0LL_both"))
            for (int i = 0; i < njLL; i++)
                q0LL_both(i) = rf.find("q0LL_both").asList()->get(i).asDouble();
        
        Vector q0RL_both = zeros(njRL);
        if(rf.check("q0RL_both"))
            for (int i = 0; i < njRL; i++)
                q0RL_both(i) = rf.find("q0RL_both").asList()->get(i).asDouble();
        
        Vector q0TO_both = zeros(njTO);
        if(rf.check("q0TO_both"))
            for (int i = 0; i < njTO; i++)
                q0TO_both(i) = rf.find("q0TO_both").asList()->get(i).asDouble();
        
        if(!rf.check("q0LL_both") || !rf.check("q0TO_both") || !rf.check("q0RL_both") )
        {
            fprintf(stderr, "ERROR: could not find home position in config!\n");
            return false;
        }

        //------------------------------- RIGHT SUPPORT POSITION ------------------------------------------------
        
        Vector q0LL_right = zeros(njLL);
        if(rf.check("q0LL_right"))
            for (int i = 0; i < njLL; i++)
                q0LL_right(i) = rf.find("q0LL_right").asList()->get(i).asDouble();
        
        Vector q0RL_right = zeros(njRL);
        if(rf.check("q0RL_right"))
            for (int i = 0; i < njRL; i++)
                q0RL_right(i) = rf.find("q0RL_right").asList()->get(i).asDouble();
        
        Vector q0TO_right = zeros(njTO);
        if(rf.check("q0TO_right"))
            for (int i = 0; i < njTO; i++)
                q0TO_right(i) = rf.find("q0TO_right").asList()->get(i).asDouble();
        
        if(!rf.check("q0LL_right") || !rf.check("q0TO_right") || !rf.check("q0RL_right") )
        {
            fprintf(stderr, "ERROR: could not find home position in config!\n");
            return false;
        }
        
        //------------------------------- LEFT SUPPORT POSITION ------------------------------------------------
        
        Vector q0LL_left = zeros(njLL);
        if(rf.check("q0LL_left"))
            for (int i = 0; i < njLL; i++)
                q0LL_left(i) = rf.find("q0LL_left").asList()->get(i).asDouble();
        
        Vector q0RL_left = zeros(njRL);
        if(rf.check("q0RL_left"))
            for (int i = 0; i < njRL; i++)
                q0RL_left(i) = rf.find("q0RL_left").asList()->get(i).asDouble();
        
        Vector q0TO_left = zeros(njTO);
        if(rf.check("q0TO_left"))
            for (int i = 0; i < njTO; i++)
                q0TO_left(i) = rf.find("q0TO_left").asList()->get(i).asDouble();
        
        if(!rf.check("q0LL_left") || !rf.check("q0TO_left") || !rf.check("q0RL_left") )
        {
            fprintf(stderr, "ERROR: could not find home position in config!\n");
            return false;
        }

        
        //------------------------------- CREATING DEVICES ------------------------------------------------
        
        //Torso Device
        OptionsTorso.put("device","remote_controlboard");
        OptionsTorso.put("local",string("/"+local_name+"/torso/client").c_str());
        OptionsTorso.put("remote",string("/"+robot_name+"/torso").c_str());
        
        if(!CreateDevice(dd_torso, OptionsTorso))
        {
            fprintf(stderr,"ERROR: unable to create torso device\n ");
            return false;
        }
        else
            fprintf(stderr, "torso device driver created\n" );

        //Right leg device
        OptionsRightLeg.put("device","remote_controlboard");
        OptionsRightLeg.put("local",string("/"+local_name+"/right_leg/client").c_str());
        OptionsRightLeg.put("remote",string("/"+robot_name+"/right_leg").c_str());

        if(!CreateDevice(dd_rightLeg, OptionsRightLeg))
        {
            fprintf(stderr, "ERROR: unable to create right leg device\n");
            return false;
        }
        else
            fprintf(stderr, "right leg device driver created\n");

        //Left leg device
        OptionsLeftLeg.put("device","remote_controlboard");
        OptionsLeftLeg.put("local",string("/"+local_name+"/left_leg/client").c_str());
        OptionsLeftLeg.put("remote",string("/"+robot_name+"/left_leg").c_str());

        if(!CreateDevice(dd_leftLeg, OptionsLeftLeg))
        {
            fprintf(stderr, "ERROR: unable to create left leg device\n");
            return false;
        }
            fprintf(stderr, "left leg device driver created\n");

        //-------------------------- THREAD ---------------------------------------------------------------
        balThread  = new comStepperThread(rate,dd_torso,dd_rightLeg,dd_leftLeg,\
                                          q0LL_both ,q0RL_both ,q0TO_both ,\
                                          q0LL_right,q0RL_right,q0TO_right,\
                                          q0LL_left ,q0RL_left ,q0TO_left ,\
                                          robot_name,local_name,wbs_name,\
                                          display,noSens,ankles_sens,springs,torso,verbose,pi_a_t0,vel_sat,Kp_zmp_h,Kd_zmp_h,Kp_zmp_x,Kd_zmp_x,Kp_zmp_y,\
                                            Kd_zmp_y,Kp, Kd, comPosInputPortName,comJacInputPortName,
                                            r2lErrorPortName, comErrorPortName);
        fprintf(stderr, "Thread created!\n");
        
        attachTerminal();
        
        if (balThread->start())
        {
            fprintf(stderr, "Starting thread!\n");
            fprintf(stderr, "Initializing offsets on forces!\n");
            (*balThread->F_ext_RL0) = w0RL;     (*balThread->F_ext_LL0) = w0LL;
            fprintf(stderr, "Checking Device Drivers dimensions!\n");
            if (!balThread->check_njTO(njTO) || !balThread->check_njLL(njLL) || !balThread->check_njRL(njRL))
            {
                fprintf(stderr, "Wrong number of torso/legs joints!\n");
                return false;
            }
            
            // control masks
            balThread->defineControlMasks(mask_r2l_swg, mask_r2l_sup, mask_com_torso);
                        
            return true;
        }
        else
        {
            fprintf(stderr, "Something wrong with the thread initialization. Quitting. \n");
            return false;
        }
        
        return true;
    }

    virtual bool CreateDevice(PolyDriver *&_dd, Property options)
    {
        int trials=0;
        double start_time = yarp::os::Time::now();

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (_dd)
            {
                delete _dd;
                _dd=0;
            }

            //creates the new device driver
            _dd = new PolyDriver(options);
            bool connected =_dd->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (60s) is expired
            if (current_time-start_time > 60.0)
            {
                fprintf(stderr,"It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            fprintf(stderr,"\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        return true;
    }

    bool close()
    { //Close and shut down the module.

        //------ STOPPING THE THREAD ----------------------
        fprintf(stderr, "Stopping comStepperModule...\n");
        balThread->stop();
        fprintf(stderr, "comStepperThread stopped\n");
        
        delete balThread;
        balThread=0;

        //------- PURGING DRIVERS ---------------------------
        if (dd_torso)
        {
            fprintf(stderr, "Closing Torso driver\n");
            dd_torso->close();
            dd_torso = 0;
        }

        if (dd_rightLeg)
        {
            fprintf(stderr, "Closing Right leg driver\n");
            dd_rightLeg->close();
            dd_rightLeg = 0;
        }

        if (dd_leftLeg)
        {
            fprintf(stderr, "Closing left leg driver\n");
            dd_leftLeg->close();
            dd_leftLeg = 0;
        }

        fprintf(stderr, "balancerModule was closed succesfully!\n");
        return true;
    }
    double getPeriod()
    {   
        return 1;
    }

    bool updateModule()
    {        
        if (balThread->on_ground)
            fprintf(stderr, "I'm running ON  ground!!\n");
        else
            fprintf(stderr, "I'm running OFF ground!!\n");
        double avgTime, stdDev, period;
        period = balThread->getRate();
        balThread->getEstPeriod(avgTime,stdDev);
        if(avgTime > 1.3*period){
            printf("(real period: %3.3f +/- %3.3f. Expected %3.3f )\n",avgTime, stdDev, period);
        }

        //double avgTime2, stdDev2;
        //balThread->getEstUsed(avgTime2,stdDev2);
        //fprintf(stderr,"Prob this is real run() time:          %3.6f +/-     %3.6f \n",avgTime2,stdDev2);
        
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
        
        //fprintf(stderr, "Debug1: %s\n", balThread->debugOutVec1.toString(p).c_str());
        //fprintf(stderr, "Debug2: %s\n", balThread->debugOutVec2.toString(p).c_str());
        //fprintf(stderr, "Debug3: %s\n", balThread->debugOutVec3.toString(p).c_str());
        //fprintf(stderr, "Debug4: %s\n", balThread->debugOutVec4.toString(p).c_str());
        
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
    
    bool respond(const Bottle &command, Bottle &reply)
    {

        if (command.get(0).asString()=="recal")
        {
            balThread->suspend();
            reply.fromString("Recalibrating!");
            *balThread->F_ext_RL0 = *balThread->F_ext_RL;
            *balThread->F_ext_LL0 = *balThread->F_ext_LL;
            balThread->resume();
            return true;
        }

        
        if (command.get(0).asString()=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("This is the list of commands:");
            reply.addString("'right':  switch to right foot support");
            reply.addString("'left':   switch to right foot support");
            reply.addString("'double': switch to double foot support");
            reply.addString("'up':     lift the non-support foot of 2cm");
            reply.addString("'down':   lower the non-support foot of 2cm");
            reply.addString("'fwd':    move forward the non-support foot of 2cm");
            reply.addString("'bck':    move backward the non-support foot of 2cm");
            return true;
        }
        if (command.get(0).asString()=="fwd")
        {
            balThread->suspend();
            reply.fromString("Moving forward");
            if (balThread->current_phase == RIGHT_SUPPORT)
                balThread->pac_d(2,0) += 0.02;
            if (balThread->current_phase == LEFT_SUPPORT)
                balThread->pca_d(2,0) += 0.02;
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="bck")
        {
            balThread->suspend();
            reply.fromString("Moving backward");
            if (balThread->current_phase == RIGHT_SUPPORT)
                balThread->pac_d(2,0) -= 0.02;
            if (balThread->current_phase == LEFT_SUPPORT)
                balThread->pca_d(2,0) -= 0.02;
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="up")
        {
            balThread->suspend();
            if (balThread->current_phase != BOTH_SUPPORT)
                reply.fromString("Moving up");
            if (balThread->current_phase == RIGHT_SUPPORT)
                balThread->pac_d(0,0) += 0.02;
            if (balThread->current_phase == LEFT_SUPPORT)
                balThread->pca_d(0,0) += 0.02;
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="down")
        {
            balThread->suspend();
            reply.fromString("Moving down");
            if (balThread->current_phase == RIGHT_SUPPORT)
                balThread->pac_d(0,0) -= 0.02;
            if (balThread->current_phase == LEFT_SUPPORT)
                balThread->pca_d(0,0) -= 0.02;
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="left")
        {
            balThread->suspend();
            reply.fromString("Switching to left foot support");
            balThread->pi_c_t(0,0) = pi_c_d_left(0);    balThread->pi_c_t(1,0) =   pi_c_d_left(1);    balThread->pi_c_t(2,0) =  pi_c_d_left(2);
            balThread->switchSupport(LEFT_SUPPORT);
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="right")
        {
            balThread->suspend();
            reply.fromString("Switching to right foot support");
            balThread->pi_a_t(0,0) = pi_a_d_right(0);    balThread->pi_a_t(1,0) =   pi_a_d_right(1);    balThread->pi_a_t(2,0) = pi_a_d_right(2);
            balThread-> switchSupport(RIGHT_SUPPORT);
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="double")
        {
            balThread->suspend();
            reply.fromString("Switching to double");
            balThread->pi_a_t(0,0) = pi_a_d_both(0);    balThread->pi_a_t(1,0) =  pi_a_d_both(1);    balThread->pi_a_t(2,0) =  pi_a_d_both(2);
            balThread->switchSupport(BOTH_SUPPORT);
            balThread->resume();
            return true;
        }
        if (command.get(0).asString()=="quit")
        {
            reply.fromString("Closing the module");
            return false;
        }
        return true;
    }
};

int main (int argc, char * argv[])
{
    // we need this to initialize the drivers list for cartesian controller
    // YARP_REGISTER_DEVICES(icubmod)

    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("default.ini");         //default config file name.
    rf.setDefaultContext("comStepperModule/conf"); //when no parameters are given to the module this is the default context    
    rf.configure("ICUB_ROOT",argc,argv);
    // rf.setName("balancerModule");

    if (rf.check("help"))
    {
        cout<< "Possible parameters"                                                                                                                                          << endl << endl;
        cout<< "\t--context          :Where to find an user defined .ini file within $ICUB_ROOT/app e.g. /balancing/comBalancerModule/conf"                                   <<endl;
        cout<< "\t--from             :Name of the file.ini to be used for calibration."                                                                                       <<endl;
        cout<< "\t--rate             :Period used by the module. Default set to 10ms."                                                                                        <<endl;
        cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default."                                                                                  <<endl;
        cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. balancerModule."                                      <<endl;
        cout<< "\t--display_only     :[on/off] Flag whether to display computed zmp on iCubGui or on terminal if verbose is on."                                              <<endl;
        cout<< "\t--no_sens          :[on/off] Flag for precenting the module from reading additional sesnors (the ones not available in iCubSim)"                            <<endl;
        
        cout<< "\t--q0LL_both             :Initial position for the left  leg"                                                                                                      <<endl;
        cout<< "\t--q0RL_both             :Initial position for the right leg"                                                                                                      <<endl;
        cout<< "\t--q0TO_both             :Initial position for the torso    "                                                                                                      <<endl;
        
        cout<< "\t--verbose          :[on/off] Prints relevant data for debugging."                                                                                           <<endl;
        cout<< "\t--torso            :[on/off] Enables balancing using the torso to compensate for whole body angular momentum."                                              <<endl;
        cout<< "\t--springs          :[on/off] Uses right gains when springs have been set at the joints level."                                                              <<endl;
        cout<< "\t--ankle_sens       :[on/off] Uses F/T sensors at the ankles."                                                                                               <<endl;
        cout<< "\t--wbs_name         :Name of the wholeBodyDynamics module that is to be used by the module."                                                                 <<endl;
        
        cout<< "\t--sat_vel          :Maximum velocity (saturation level by scaling)."                                                                                        <<endl;
        
        cout<< "\t--pi_a_d_right     :Desired ZMP position while in right  foot support (expressed in * right * foot coordinates)."                                           <<endl;
        cout<< "\t--pi_a_d_both      :Desired ZMP position while in double foot support (expressed in * right * foot coordinates)."                                           <<endl;
        cout<< "\t--pi_c_d_left      :Desired ZMP position while in left   foot support (expressed in * left  * foot coordinates)."                                           <<endl;
        
        cout<< "\t--Kp_x             :Proportional gain for x direction of zmp controller."                                                                                   <<endl;
        cout<< "\t--Kd_x             :Derivative gain for x direction of zmp controller."                                                                                     <<endl;
        cout<< "\t--Kp_y             :Proportional gain for y direction of zmp controller."                                                                                   <<endl;
        cout<< "\t--Kd_y             :Derivative gain for y direction of zmp controller."                                                                                     <<endl;
        
        cout<< "\t--Kp               :Proportional gain for the right to left foot displacement controller."                                                                  <<endl;
        cout<< "\t--Kd               :Proportional gain for the right to left foor orientation controller."                                                                   <<endl;

        cout<< "\t--com_position     :specify a port to be opened in order to read the com_position in the root reference frame."                                             <<endl;
        cout<< "\t--com_jacobian     :specify a port to be opened in order to read the com_jacobian w.r.t the root reference frame."                                          <<endl;

        cout<< "\t--r2l_error        :specify a port to output the tracking errors on the right/left foot position and orientation."                                          <<endl;
        cout<< "\t--com_error        :specify a port to output the tracking errors for the COM."                                                                              <<endl;
        
        cout<< "\t--w0RL             :specify the wrench (force/torque) offset for the right leg F/T sensor."                                                                 <<endl;
        cout<< "\t--w0LL             :specify the wrench (force/torque) offset for the left  leg F/T sensor."                                                                 <<endl;

        cout<< "\t--njRL             :specify number of joints in the right leg."                                                                                             <<endl;
        cout<< "\t--njLL             :specify number of joints in the  left leg."                                                                                             <<endl;
        cout<< "\t--njTO             :specify number of joints in the     torso."                                                                                             <<endl;

        cout<< "\t--r2l_jointsSwgLeg :(0/1 vector) specify the joints of the swing   leg dedicated to the right to left foot displacement controller."                          <<endl;
        cout<< "\t--r2l_jointsSupLeg :(0/1 vector) specify the joints of the support leg dedicated to the right to left foot displacement controller."                          <<endl;
        cout<< "\t--com_jointsTorso  :(0/1 vector) specify the joints of the     torso dedicated to the COM controller."                                                      <<endl;
        
        return 0;
    }
    
    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    Balancer balancerModule;
    return balancerModule.runModule(rf);
}
