/* 
 * Copyright (C) 2012
 * Author: Silvio Traversaro
 * email: 
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
@ingroup icub_module

\defgroup inertiaObserver
 
Estimates the inertial parameters of the link of the robot using 
measurments of the joint position and force/torque sensors
 
\author Silvio Traversaro
 
\section intro_sec Description

This module estimates the inertial parameters (mass, center of mass,
inertia matrix) for each link of a limb of the iCub. 
 
\section lib_sec Libraries 
- YARP libraries. 
- ctrlLib library. 
- iKin library.
- iDyn library.  

\section parameters_sec Parameters

--robot \e name 
- The parameter \e name identifies the robot name. If not specified
  \e icub is assumed. 

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
  specified \e 10ms is assumed. 

--no_left_leg
- this option disables the parameter estimation computation for the left leg

--no_right_leg
- this option disables the parameter estimation computation for the right leg

--no_right_arm 
- this option disables the parameter estimation computation for the right arm

--no_left_arm 
- this option disables the parameter estimation computation for the left arm


--enable_debug_output
- this option enables the output ports, used for debug

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e <name>/<part>/FT:i (e.g. /inertiaObserver/right_arm/FT:i) 
  receives the input data vector.

- \e <name>/<part>/
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None
 
\section tested_os_sec Tested OS
Linux.

\section example_sec Example
By launching the following command: 
 
\code 
inertiaObserver --rate 10  
\endcode 

 
\author Silvio Traversaro

This file can be edited at .
*/ 

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "observerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

// The main module
class inertiaObserver_module: public RFModule
{
private:
    bool     left_leg_enabled;
    bool     right_leg_enabled;
    bool     w0_dw0_enabled;
    bool     left_arm_enabled;
    bool     right_arm_enabled;
    bool     debug_out_enabled;
    bool     debug_beta_enabled;
    
    int rate;
    int rateEstimation;
    
    bool log_still_measures;
    
    string data_path;
    string xml_yarpscope_file;
    
    //dataFilter *port_inertial_input;
    BufferedPort<Vector> port_filtered_output;    
    Port rpcPort;

    inertiaObserver_thread *ine_obs_thr;
    
    bool learning_enabled;
    
public:
    inertiaObserver_module()
    {
        ine_obs_thr=0;
        left_leg_enabled = true;
        right_leg_enabled = true;
        left_arm_enabled = true;
        right_arm_enabled = true;
        debug_out_enabled = false;
        rate = 10;
        rateEstimation = 1000;
    }
    
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        fprintf(stderr,"receiving command from port rpc\n");
        int index = 0;
        int cmdSize = command.size();
    
        while(cmdSize>0)
            {
                switch(command.get(index).asVocab())  {
                case  VOCAB4('s','u','s','p'):
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        ine_obs_thr->suspend();
                        cmdSize--;
                        index++;
                        break;
                    }
                case VOCAB3('r','u','n'):
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        ine_obs_thr->resume();
                        cmdSize--;
                        index++;
                        break;
                    }
                case VOCAB4('t','e','s','t'):
                    {
                        fprintf(stderr,"Stopping learning parameters, fixing them to current value");
                        if( ine_obs_thr ) 
                        {
							ine_obs_thr->suspend();
							ine_obs_thr->disableLearning(); 
							ine_obs_thr->resume();
                        }
                        cmdSize--;
                        index++;
                        break;
                    }
                case VOCAB4('n','t','e','s'):
                    {
                        fprintf(stderr,"Starting learning parameters, movimeng them from current value");
                        if( ine_obs_thr ) 
                        {
							ine_obs_thr->suspend();
							ine_obs_thr->enableLearning(); 
							ine_obs_thr->resume();
                        }
                        cmdSize--;
                        index++;
                        break;
                    }
                
                /*
                case VOCAB3('c','a','l'):    
					{
						fprintf(stderr,"Asking recalibration...\n");
						if (inertiaObserver_thread)
						{
							inertiaObserver_thread->suspend();
							//inertiaObserver_thread->calibrateOffset(); 
							inertiaObserver_thread->resume();
						}
						fprintf(stderr,"Recalibration complete.\n");
						reply.addVocab(Vocab::encode("rec");
						cmdSize--;
                        index++;
                        break;
					}
				*/
                default:
                    {
                        cmdSize--;
                        index++;
                        //Commented because caused an infinite recursion 
                        //return respond(command, reply); // call default
                        break;
                    }
                }
                return true;
            }

        return false;
    }

    bool configure(ResourceFinder &rf)
    {
        //---------------------LOCAL NAME-----------------------//
        string local_name = "inertiaObserver";
        if (rf.check("local"))
        {
            local_name=rf.find("local").asString();
        }
        
        //---------------OPEN RPC PORT--------------------//
        string rpcPortName = "/"+local_name+"/rpc:i";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);                  

        //---------------OPEN INERTIAL PORTS--------------------//
        port_filtered_output.open(string("/"+local_name+"/filtered/inertial:o").c_str());
       // port_inertial_input = new dataFilter(port_filtered_output/*, rf*/);
        //port_inertial_input->useCallback();
        //port_inertial_input->open(string("/"+local_name+"/unfiltered/inertial:i").c_str());
        
        //-----------------GET THE ROBOT NAME-------------------//
        string robot_name;
        if (rf.check("robot"))
             robot_name = rf.find("robot").asString();
        else robot_name = "icub";

        //------------SPECIAL PARAM TP DEFINE THE HEAD TYPE-----//
        version_tag icub_type;

        icub_type.head_version = 1;
        icub_type.legs_version = 1;

        if (rf.check("headV2"))
        {
            fprintf(stderr,"'headV2' option found. Using icubV2 head kinematics.\n");
            icub_type.head_version = 2;
        }

        //-----------------CHECK IF AUTOCONNECT IS ON-----------//
        bool autoconnect;
        if (rf.check("autoconnect"))
        {
             fprintf(stderr,"'autoconnect' option enabled.\n");
             autoconnect = true;
        }
        else
        { 
              autoconnect = false;
        }

        //------------------CHECK IF LEGS ARE ENABLED-----------//
        if (rf.check("no_left_leg"))
        {
            left_leg_enabled= false;
            fprintf(stderr,"'no_left_leg' option found. Left leg will be disabled.\n");
        }
        
        if (rf.check("no_right_leg"))
        {
            right_leg_enabled= false;
            fprintf(stderr,"'no_right_leg' option found. Right leg will be disabled.\n");
        }

        //------------------CHECK IF ARMS ARE ENABLED-----------//
        if (rf.check("no_left_arm"))
        {
            left_arm_enabled= false;
            fprintf(stderr,"'no_left_arm' option found. Left arm will be disabled.\n");
        }
        if (rf.check("no_right_arm"))
        {
            right_arm_enabled= false;
            fprintf(stderr,"'no_right_arm' option found. Right arm will be disabled.\n");
        }
        
        if (rf.check("enable_debug_output"))
        {
             debug_out_enabled= true;
            fprintf(stderr,"'debug_output_enable' option found. Debug output port will be enabled.\n");

        }

        //---------------------RATE-----------------------------//
        if (rf.check("rate"))
        {
            rate = rf.find("rate").asInt();
            fprintf(stderr,"rateThread working at %d ms\n", rate);
        }
        else
        {
            fprintf(stderr,"Could not find rate in the config file\nusing 10ms as default\n");
            rate = 10;
        }
        
        //---------------------RATE-----------------------------//
        if (rf.check("rateEstimation"))
        {
            rateEstimation = rf.find("rateEstimation").asInt();
            fprintf(stderr,"parameter estimated with a period of %d ms\n", rateEstimation);
        }
        else
        {
            fprintf(stderr,"Could not find rateEstimation in the config file\nusing 1000ms as default\n");
            rateEstimation = 1000;
        }
        //--------------------DATA PATH--------------------//
        data_path = rf.findPath("dataPath");//.toString();
        fprintf(stderr,"Data path found: %s \n",data_path.c_str());
        
        //--------------------XML YARPSCOPE----------------//
        xml_yarpscope_file = rf.find("yarpscope_xml").asString();
        fprintf(stderr,"Genearing xml yarpscope file at: %s \n",xml_yarpscope_file.c_str());

        
        
        //---------------------DUMP------------------------//
        bool dump_static = false;
        if (rf.check("dump_static") ) 
        {
            dump_static = true; 
            fprintf(stderr,"'dump_static' option found.\n");
        }
        
        //--------------------CHECK FT SENSOR------------------------
        if (( (Network::exists(string("/"+robot_name+"/left_arm/analog:o").c_str())  == false) && left_arm_enabled ) || 
                ( (Network::exists(string("/"+robot_name+"/right_arm/analog:o").c_str()) == false) && right_arm_enabled ) ||
                ( (Network::exists(string("/"+robot_name+"/left_leg/analog:o").c_str())  == false) && left_leg_enabled ) ||
                ( (Network::exists(string("/"+robot_name+"/right_leg/analog:o").c_str()) == false) && right_leg_enabled ) )
                {     
                    fprintf(stderr,"Unable to detect the presence of F/T sensors in your iCub...quitting\n");
                    return false;
                }
        //--------------------------THREAD--------------------------
        ine_obs_thr = new inertiaObserver_thread(rate, rateEstimation, robot_name, local_name, icub_type, data_path, autoconnect, right_leg_enabled, left_leg_enabled, right_arm_enabled, left_arm_enabled, debug_out_enabled, dump_static, xml_yarpscope_file);

        fprintf(stderr,"ft thread istantiated...\n");
        Time::delay(5.0);

        ine_obs_thr->start();
        fprintf(stderr,"thread started\n");
        return true;
    }

    bool close()
    {
    //The order of execution of the following closures is important, do not change it.
        fprintf(stderr,"Closing inertiaObserver module... \n");     

        if (ine_obs_thr)
          {
            fprintf(stderr,"Stopping the ine_obs_thr module...");     
            ine_obs_thr->stop();
            fprintf(stderr,"ine_obs_thr module stopped\n");     
            delete ine_obs_thr;
            ine_obs_thr=0;
          }
        
        //if(port_inertial_input)
        /*
        {
            fprintf(stderr,"Closing the inertial input port \n");     
            port_inertial_input->interrupt();
            port_inertial_input->close();
            delete port_inertial_input;
            port_inertial_input=0;
        }
        */

        fprintf(stderr,"Closing the filtered inertial output port \n");     
        port_filtered_output.interrupt();
        port_filtered_output.close();

        fprintf(stderr,"Closing the rpc port \n");     
        rpcPort.close();

        fprintf(stderr,"inertiaObserver module was closed successfully! \n");     
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }
    bool updateModule() 
    {
        double avgTime, stdDev, period;
        period = ine_obs_thr->getRate();
        ine_obs_thr->getEstPeriod(avgTime, stdDev);
        if(avgTime > 1.3 * period){
            printf("(real period: %3.3f +/- %3.3f; expected period %3.3f)\n", avgTime, stdDev, period);
        }

        static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            printf ("inertiaObserver is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

        if (ine_obs_thr==0) 
            return false;
        thread_status_enum thread_status = ine_obs_thr->getThreadStatus();
        if (thread_status==STATUS_OK)
            return true;
        else if (thread_status==STATUS_DISCONNECTED)
        {
            printf ("inertiaObserver module lost connection, now closing...\n");
            return false;
        }
        else
        {
            fprintf(stderr,"inertiaObserver module was closed successfully! \n");    
            return true;
        }
            
    }
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("inertiaObserver");
    rf.setDefaultConfigFile("conf/inertiaObserver.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default inertiaObserver/conf)" << endl;
        cout << "\t--from           from: the name of the file.ini to be used for calibration"                                            << endl;
        cout << "\t--rate           rate: the period used by the module. default: 10ms"                                                   << endl;
        cout << "\t--rateEstimation         rateEstimation: the period used for the update of the estimated parameters. default 1000ms"           << endl;
        cout << "\t--robot          robot:  the robot name. default: icub"                                                                 << endl;
        cout << "\t--local                  name: the prefix of the ports opened by the module. default: inertiaObserver"                       << endl;
        cout << "\t--autoconnect            automatically connects the module ports to iCubInterface"                                        << endl;        
        cout << "\t--no_left_leg            disables the left leg"                               << endl;  
        cout << "\t--no_right_leg 		    disable the right leg"         << endl;
        cout << "\t--no_left_arm            disables the left arm"                                                                           << endl;
        cout << "\t--no_right_arm           disables the right arm"     << endl;
        cout << "\t--enable_debug_output    enable the debug output"  << endl;
        cout << "\t--dump_static    for the considered limbs dump the static FT measurments" << endl; 
        cout << "\t--yarpscope_xml file_path print a yarpscope xml file for debug of the installed learners " << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    inertiaObserver_module obs;
    return obs.runModule(rf);
}

