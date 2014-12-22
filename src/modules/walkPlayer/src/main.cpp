/* 
 * Copyright (C)2013  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

#define VCTP_TIME VOCAB4('t','i','m','e')
#define VCTP_OFFSET VOCAB3('o','f','f')
#define VCTP_CMD_NOW VOCAB4('c','t','p','n')
#define VCTP_CMD_QUEUE VOCAB4('c','t','p','q')
#define VCTP_CMD_FILE VOCAB4('c','t','p','f')
#define VCTP_POSITION VOCAB3('p','o','s')
#define VCTP_WAIT VOCAB4('w','a','i','t')

#define ACTION_IDLE    0
#define ACTION_START   1
#define ACTION_RUNNING 2

// ******************** ACTION CLASS
struct action_struct
{
    int         counter;
    double      time;
    double      q_left_leg  [6];
    double      q_right_leg [6];
    string      tag;

    public:
    action_struct()
    {
        for (int i=0; i<6; i++) q_left_leg[i]=q_right_leg[i]=0.0;
        tag = "UNKNOWN";
    }
};

class action_class
{
    public:
    size_t         current_action;
    int            current_status;
    std::vector<action_struct> action_vector;
   // bool parseCommandLine(char* command_line, int line);
   // bool parseCommandLine2(char* command_line1, char* command_line2, int line);

    action_class()
    {
        current_action = 0;
        current_status = ACTION_IDLE;
    }

    bool openFile2(string filename)
    {
        bool ret = true;
        FILE* data_file1 = 0;
        FILE* data_file2 = 0;
        string filename_left= filename + "_left" + ".txt";
        string filename_right= filename + "_right" + ".txt";
        data_file1 = fopen(filename_left.c_str(),"r");
        data_file2 = fopen(filename_right.c_str(),"r");
        if (data_file1!=NULL && data_file2!=NULL)
        {
            char* bb1 = 0;
            char* bb2 = 0;
            int   line =0;
            do
            {
                char command_line1[1024];
                char command_line2[1024];
                bb1 = fgets (command_line1, 1024, data_file1);
                bb2 = fgets (command_line2, 1024, data_file2);
                if (bb1 == 0 || bb2 == 0) break;
                if(!parseCommandLine2(command_line1, command_line2, line++))
                  {
                      printf ("error parsing file, line %d\n", line);
                      ret = false;
                      break;
                  };
            }
            while (1);

            fclose (data_file1);
            fclose (data_file2);
        }
        else
        {
            //file not opened
            ret = false;
        }
        return ret;
    }

    bool openFile(string filename)
    {
        bool ret = true;
        FILE* data_file = 0;
        data_file = fopen(filename.c_str(),"r");
        if (data_file!=NULL)
        {
            char* bb = 0;
            int   line =0;
            do
            {
                char command_line[1024];
                bb = fgets (command_line, 1024, data_file);
                if (bb == 0) break;
                if(!parseCommandLine(command_line, line++))
                  {
                      printf ("error parsing file, line %d\n", line);
                      ret = false;
                      break;
                  };
            }
            while (1);

            fclose (data_file);
        }
        return ret;
    }

    bool parseCommandLine2(char* command_line1, char* command_line2, int line)
    {
            action_struct tmp_action;
            int ret1 = sscanf(command_line1, "%d %lf    %lf %lf %lf %lf %lf %lf  ", 
            &tmp_action.counter,
            &tmp_action.time,
            
            &tmp_action.q_left_leg[0],
            &tmp_action.q_left_leg[1],
            &tmp_action.q_left_leg[2],
            &tmp_action.q_left_leg[3],
            &tmp_action.q_left_leg[4],
            &tmp_action.q_left_leg[5]
            );

            int ret2 = sscanf(command_line2, "%d %lf    %lf %lf %lf %lf %lf %lf  ", 
            &tmp_action.counter,
            &tmp_action.time,
            
            &tmp_action.q_right_leg[0],
            &tmp_action.q_right_leg[1],
            &tmp_action.q_right_leg[2],
            &tmp_action.q_right_leg[3],
            &tmp_action.q_right_leg[4],
            &tmp_action.q_right_leg[5]
            );

            if (ret1 == 8 && ret2 == 8) 
            {
                action_vector.push_back(tmp_action);
                return true;
            }
            
            return false;
    }

    bool parseCommandLine(char* command_line, int line)
    {
            action_struct tmp_action;
            int ret = sscanf(command_line, "%d %lf    %lf %lf %lf %lf %lf %lf    %lf %lf %lf %lf %lf %lf", 
            &tmp_action.counter,
            &tmp_action.time,
            
            &tmp_action.q_left_leg[0],
            &tmp_action.q_left_leg[1],
            &tmp_action.q_left_leg[2],
            &tmp_action.q_left_leg[3],
            &tmp_action.q_left_leg[4],
            &tmp_action.q_left_leg[5],

            &tmp_action.q_right_leg[0],
            &tmp_action.q_right_leg[1],
            &tmp_action.q_right_leg[2],
            &tmp_action.q_right_leg[3],
            &tmp_action.q_right_leg[4],
            &tmp_action.q_right_leg[5]);

            if (ret == 14) 
            {
                action_vector.push_back(tmp_action);
                return true;
            }
            
            return false;
    }
};

// ******************** ROBOT DRIVER CLASS
class robotDriver
{
public:
    bool verbose;
    bool connected;
    Property          drvOptions_ll;
    Property          drvOptions_rl;
    Property          drvOptions_to;
    PolyDriver       *drv_ll;
    PolyDriver       *drv_rl;
    PolyDriver       *drv_to;
    IPositionControl *ipos_ll;
    IPidControl      *ipid_ll;
    IEncoders        *ienc_ll;
    IPositionControl *ipos_rl;
    IPidControl      *ipid_rl;
    IEncoders        *ienc_rl;
    IPositionControl *ipos_to;
    IPidControl      *ipid_to;
    IEncoders        *ienc_to;
    IControlMode2    *icmd_ll;
    IControlMode2    *icmd_rl;
    IControlMode2    *icmd_to;
    IPositionDirect  *idir_rl;
    IPositionDirect  *idir_ll;
    iCub::iDyn::iCubWholeBody *icub_dyn;

public:
    robotDriver()
    {
        drvOptions_ll.clear();
        drvOptions_rl.clear();
        drvOptions_to.clear();
        drv_ll  = 0;
        drv_rl  = 0;
        drv_to  = 0;
        ipos_ll = 0;
        ipid_ll = 0;
        ienc_ll = 0;
        ipos_rl = 0;
        ipid_rl = 0;
        ienc_rl = 0;
        ipos_to = 0;
        ipid_to = 0;
        ienc_to = 0;
        icmd_ll = 0;
        icmd_rl = 0;
        icmd_to = 0;
        idir_ll = 0;
        idir_rl = 0;
        verbose=1;
        connected=false;
        iCub::iDyn::version_tag tag;
        icub_dyn = new iCub::iDyn::iCubWholeBody(tag);
    }

    Matrix compute_tranformations (action_struct act)
    {
        for (int i=0; i<6; i++)
        {
            icub_dyn->lowerTorso->left->setAng(i,act.q_left_leg[i]);
            icub_dyn->lowerTorso->right->setAng(i,act.q_right_leg[i]);
            icub_dyn->lowerTorso->left->setDAng(i,0.0);
            icub_dyn->lowerTorso->right->setDAng(i,0.0);
            icub_dyn->lowerTorso->left->setD2Ang(i,0.0);
            icub_dyn->lowerTorso->right->setD2Ang(i,0.0);
        }
        Matrix Hl= icub_dyn->lowerTorso->HLeft  * icub_dyn->lowerTorso->left->getH();
        Matrix Hr= icub_dyn->lowerTorso->HRight * icub_dyn->lowerTorso->right->getH();

        //cout << endl<< "HL to string:" << endl << Hl.toString() << endl;

        Matrix Hlr = SE3inv(icub_dyn->lowerTorso->left->getH()) * SE3inv(icub_dyn->lowerTorso->HLeft) * icub_dyn->lowerTorso->HRight * icub_dyn->lowerTorso->right->getH();

        //cout << endl<< "HLR to string:" << endl << Hlr.toString() << endl;

        return Hlr;
    }

    bool configure(const Property &copt)
    {
        bool ret=true;
        Property &options=const_cast<Property &> (copt);

        drvOptions_ll.put("device","remote_controlboard");
        drvOptions_rl.put("device","remote_controlboard");
        drvOptions_to.put("device","remote_controlboard");

        string remote;
        string local;
        remote = string("/") + string(options.find("robot").asString()) + string("/left_leg");
        local  = string("/") + string("walkPlayer") + string("/left_leg");
        drvOptions_ll.put("remote",remote.c_str());
        drvOptions_ll.put("local",local.c_str());
        remote = string("/") + string(options.find("robot").asString()) + string("/right_leg");
        local  = string("/") + string("walkPlayer") + string("/right_leg");
        drvOptions_rl.put("remote",remote.c_str());
        drvOptions_rl.put("local",local.c_str());
        remote = string("/") + string(options.find("robot").asString()) + string("/torso");
        local  = string("/") + string("walkPlayer") + string("/torso");
        drvOptions_to.put("remote",remote.c_str());
        drvOptions_to.put("local",local.c_str());

        if (verbose)
        {
            cout << "right leg driver options:\n" << drvOptions_rl.toString().c_str();
            cout << "left  leg driver options:\n" << drvOptions_ll.toString().c_str();
            cout << "torso     driver options:\n" << drvOptions_to.toString().c_str();
        }

        return ret;
     }

    bool init()
    {
        //return true; //@@@@@

        drv_ll=new PolyDriver(drvOptions_ll);
        drv_rl=new PolyDriver(drvOptions_rl);
        drv_to=new PolyDriver(drvOptions_to);

        if (drv_ll->isValid() && drv_rl->isValid() && drv_to->isValid())
            connected = drv_ll->view(ipos_ll) && drv_ll->view(ienc_ll) && drv_ll->view(ipid_ll) && drv_ll->view(icmd_ll) && drv_ll->view(idir_ll) &&
                        drv_rl->view(ipos_rl) && drv_rl->view(ienc_rl) && drv_rl->view(ipid_rl) && drv_rl->view(icmd_rl) && drv_rl->view(idir_rl) &&
                        drv_to->view(ipos_to) && drv_to->view(ienc_to) && drv_to->view(ipid_to) && drv_to->view(icmd_to);
        else
            connected=false;

        if (!connected)
        {    
            if (drv_ll)
            {
                delete drv_ll;
                drv_ll=0;
            }
            if (drv_rl)
            {
                delete drv_rl;
                drv_rl=0;
            }
            if (drv_to)
            {
                delete drv_to;
                drv_to=0;
            }
        }

        //set the intial reference speeds
        double speeds_arm[6];
        double speeds_to[3];
        for (int i=0; i<6; i++) speeds_arm[i] = 20.0;
        for (int i=0; i<3; i++) speeds_to[i] = 20.0;
        ipos_ll->setRefSpeeds(speeds_arm);
        ipos_rl->setRefSpeeds(speeds_arm);
        ipos_to->setRefSpeeds(speeds_to);

        return connected;
    }

    ~robotDriver()
    {
        if (drv_ll)
        {
            delete drv_ll;
            drv_ll=0;
        }
        if (drv_rl)
        {
            delete drv_rl;
            drv_rl=0;
        }
        if (drv_to)
        {
            delete drv_to;
            drv_to=0;
        }
    }
};

// ******************** THE THREAD
class WorkingThread: public RateThread
{
private:

public:
    robotDriver           *driver;
    action_class          actions;
    Semaphore             mutex;
    BufferedPort<Bottle>  port_command_out;
    BufferedPort<Bottle>  port_command_joints_ll;
    BufferedPort<Bottle>  port_command_joints_rl;
    bool                  enable_execute_joint_command;
    double                speed_factor;          

    WorkingThread(int period=5): RateThread(period)
    {
        enable_execute_joint_command = true;
        //*** open the output port
        port_command_out.open("/walkPlayer/port_command_out:o");
        port_command_joints_ll.open("/walkPlayer/port_joints_ll:o");
        port_command_joints_rl.open("/walkPlayer/port_joints_rl:o");
        speed_factor = 1.0;
    }

    ~WorkingThread()
    {
        port_command_out.interrupt();
        port_command_out.close();
        port_command_joints_ll.interrupt();
        port_command_joints_ll.close();
        port_command_joints_rl.interrupt();
        port_command_joints_rl.close();
    }

    void attachRobotDriver(robotDriver *p)
    {
        if (p)  driver=p;
    }

    bool threadInit()
    {
        if (!driver)
            return false;
        return true;
    }
    
    bool execute_joint_command(int j)
    {
        if (!driver) return false;
        if (!enable_execute_joint_command) return true;

        double *ll = actions.action_vector[j].q_left_leg;
        double *rl = actions.action_vector[j].q_right_leg;
        double encs_ll[6]; double spd_ll[6]; double spd_to[3];
        double encs_rl[6]; double spd_rl[6]; double encs_to[3];
        int    modes[6];
        double to[3];
        to[0] = 0.0;
        to[1] = 0.0;
        to[2] = -6.0;

        if (j==0)
        {
            //the first step
            driver->ienc_ll->getEncoders(encs_ll);
            driver->ienc_rl->getEncoders(encs_rl);
            driver->ienc_to->getEncoders(encs_to);

            for (int i=0; i<6; i++)
            {
                spd_ll[i] = fabs(encs_ll[i]-ll[i])/4.0;
                spd_rl[i] = fabs(encs_rl[i]-rl[i])/4.0;
            }

            spd_to[0] = fabs(encs_to[0]-to[0])/4.0;
            spd_to[1] = fabs(encs_to[1]-to[1])/4.0;
            spd_to[2] = fabs(encs_to[2]-to[2])/4.0;

            driver->ipos_ll->setRefSpeeds(spd_ll);
            driver->ipos_rl->setRefSpeeds(spd_rl);
            driver->ipos_to->setRefSpeeds(spd_to);
            
            driver->ipos_to->positionMove(to);

            for (int i=0; i< 6; i++) modes[i] = VOCAB_CM_POSITION;
            driver->icmd_ll->setControlModes(modes);
            driver->icmd_rl->setControlModes(modes);

            driver->ipos_ll->positionMove(ll);
            driver->ipos_rl->positionMove(rl);

            cout << "going to to home position" << endl;

            yarp::os::Time::delay(6.0);
            cout << "done" << endl;

        }
        else
        {
            int mode_l =0; 
            int mode_r =0;
            driver->icmd_rl->getControlMode(0,&mode_r);
            driver->icmd_ll->getControlMode(0,&mode_l);
            if (mode_l != VOCAB_CM_POSITION_DIRECT &&
                mode_r != VOCAB_CM_POSITION_DIRECT)
            {
                //change control mode
                for (int i=0; i< 6; i++) modes[i] = VOCAB_CM_POSITION_DIRECT;
                driver->icmd_ll->setControlModes(modes);
                driver->icmd_rl->setControlModes(modes);
            }

            driver->idir_ll->setPositions(ll);
            driver->idir_rl->setPositions(rl);
        }
        return true;
    }

    void compute_and_send_command(int j)
    {
        //compute the transformations
        Matrix m = driver->compute_tranformations(actions.action_vector[j]);
        //prepare the output command
        Bottle& bot = port_command_out.prepare();
        bot.clear();
        bot.addInt(actions.action_vector[j].counter);
        bot.addDouble(actions.action_vector[j].time);
        for (int ix=0;ix<4;ix++)
                for (int iy=0;iy<4;iy++)
                    bot.addDouble(m(ix,iy));
        bot.addString(actions.action_vector[j].tag.c_str());
        //@@@ you can add stuff here...
        //send the output command
        port_command_out.write();
        execute_joint_command(j);

        //send the joints angles on debug port
        double *ll = actions.action_vector[j].q_left_leg;
        double *rl = actions.action_vector[j].q_right_leg;
        Bottle& bot2 = this->port_command_joints_ll.prepare();
        bot2.clear();
        Bottle& bot3 = this->port_command_joints_rl.prepare();
        bot3.clear();
        bot2.addInt(actions.action_vector[j].counter);
        bot2.addDouble(actions.action_vector[j].time);
        bot3.addInt(actions.action_vector[j].counter);
        bot3.addDouble(actions.action_vector[j].time);
        for (int ix=0;ix<6;ix++)
        {
            bot2.addDouble(ll[ix]);
            bot3.addDouble(rl[ix]);
        }
        this->port_command_joints_ll.write();
        this->port_command_joints_rl.write();
    }

    void run()
    {
        mutex.wait();
        double current_time = yarp::os::Time::now();
        static double last_time = yarp::os::Time::now();
        if (actions.current_status==ACTION_IDLE)
        {
            last_time = current_time;
        }
        else if (actions.current_status==ACTION_RUNNING)
        {
            //if it's not the last action
            size_t last_action = actions.action_vector.size();
            if (last_action == 0)
            {
                printf("sequence empty!\n");
                actions.current_status=ACTION_IDLE;
                return;
            }

            if (actions.current_action < last_action-1)
            {
                //if enough time is passed from the previous action
                double duration = actions.action_vector[actions.current_action+1].time -
                                  actions.action_vector[actions.current_action].time;
                if (current_time-last_time > duration*speed_factor)
                {
                    last_time = current_time;
                    actions.current_action++;
                    compute_and_send_command(actions.current_action);
                    //printf("EXECUTING %d, elapsed_time:%.5f requested_time:%.5f\n", actions.current_action, current_time-last_time, duration);
                }
                else
                {
                    //printf("WAITING %d, elapsed_time:%.5f requested_time:%.5f\n", actions.current_action, current_time-last_time, duration);
                }
            }
            else
            {
                printf("sequence complete\n");
                actions.current_status=ACTION_IDLE;
            }
        }
        else if (actions.current_status==ACTION_START)
        {
            compute_and_send_command(0);
            actions.current_status=ACTION_RUNNING;
        }
        else
        {
            printf("ERROR: unknown current_status\n");
        }
        mutex.post();
    }
};

// ******************** THE MODULE
class scriptModule: public RFModule
{
protected:
    Port                rpcPort;
    string              name;
    string              contextPath;
    bool                verbose;
    robotDriver         robot;
    WorkingThread       thread;

public:
    scriptModule() 
    {
        verbose=true;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        if (rf.check("name"))
            name=string("/")+rf.find("name").asString().c_str();
        else
            name="/walkPlayer";

        contextPath=rf.getContextPath().c_str();

        rpcPort.open((name+"/rpc").c_str());
        attach(rpcPort);

        Property portProp;
        portProp.put("robot", rf.find("robot"));

        //*** start the robot driver
        if (!robot.configure(portProp))
        {
            cerr<<"Error configuring position controller, check parameters"<<endl;
            return false;
        }
        
        if (!robot.init())
        {
            cerr<<"Error cannot connect to remote ports"<<endl;
            return false;
        }

        //*** attach the robot driver to the thread and start it
        thread.attachRobotDriver(&robot);
        if (!thread.start())
        {
            cerr<<"ERROR: Thread did not start, queue will not work"<<endl;
        }
        else
        {
            cout<<"Thread started"<<endl;
        }

        if (rf.check("execute")==true)
        {
            cout << "Enablig iPid->setReference() controller"<< endl;
            thread.enable_execute_joint_command = true;
        }
        else
        {
            cout << "Not using iPid->setReference() controller"<< endl;
            thread.enable_execute_joint_command = false;
        }

        if (rf.check("period")==true)
        {
            int period = rf.find("period").asInt();
            cout << "Thread period set to "<<period<< "ms" <<endl;
            thread.setRate(period);
        }
        cout << "Using parameters:" << endl << rf.toString() << endl;

        if (rf.check("speed")==true)
        {
            double fact = rf.find("speed").asDouble();
            cout << "speed factor set to "<<fact<< endl;
            thread.speed_factor= fact;
        }
        cout << "Using parameters:" << endl << rf.toString() << endl;

        //*** open the position file
        cout << "opening file..." << endl;
        if (rf.check("filename")==true)
        {
            string filename = rf.find("filename").asString().c_str();
            if (!thread.actions.openFile(filename))
            {
                cout << "ERROR: Unable to parse file" << endl;
                return false;
            };
        }
        else
        if (rf.check("filename2")==true)
        {
            string filename = rf.find("filename2").asString().c_str();
            if (!thread.actions.openFile2(filename))
            {
                cout << "ERROR: Unable to parse file" << endl;
                return false;
            };
        }
        else
        {
            cout << "ERROR: Unable to find file with both --filename --filename2 paramters" << endl;
            return false;
        }



        cout << "module succesffully configured. ready." << endl;
        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        bool ret=true;
        this->thread.mutex.wait();

        if (command.size()!=0)
        {
            string cmdstring = command.get(0).asString().c_str();
            {
                if  (cmdstring == "help")
                    {                    
                        cout << "Available commands:"          << endl;
                        cout << "start" << endl;
                        cout << "stop"  << endl;
                        cout << "reset" << endl;
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "start")
                    {
                        if (this->thread.actions.current_action == 0)
                            this->thread.actions.current_status = ACTION_START;
                        else
                            this->thread.actions.current_status = ACTION_RUNNING;
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "stop")
                    {
                        this->thread.actions.current_status = ACTION_IDLE;
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "reset")
                    {
                        this->thread.actions.current_status = ACTION_IDLE;
                        this->thread.actions.current_action = 0;
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else
                    {
                        reply.addVocab(Vocab::encode("nack"));
                        ret = false;
                    }
            }
        }
        else
        {
            reply.addVocab(Vocab::encode("nack"));
            ret = false;
        }

        this->thread.mutex.post();
        return ret;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("ctpService/conf");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name         <moduleName>: set new module name" << endl;
        cout << "\t--robot        <robotname>:  robot name"          << endl;
        cout << "\t--file         <filename>:   the positions file (with both legs)"  << endl;
        cout << "\t--filename2    <filename>:   to specifiy to use two files (left and leg separate). _left.txt and _right.txt automatically appended"  << endl;
        cout << "\t--execute      activate the iPid->setReference() control"  << endl;
        cout << "\t--period       <period>: the period in ms of the internal thread (default 5)"  << endl;
        cout << "\t--speed        <factor>: speed factor (default 1.0 normal, 0.5 double speed, 2.0 half speed etc)"  << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        cout << "ERROR: yarp.checkNetwork() failed."  << endl;
        return -1;
    }

    scriptModule mod;
 
    return mod.runModule(rf);
}



