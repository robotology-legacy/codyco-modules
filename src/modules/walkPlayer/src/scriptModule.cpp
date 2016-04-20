#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <string.h>

#include "scriptModule.h"

using namespace std;
using namespace yarp::os;

scriptModule::scriptModule()
{
    verbose=true;
}

bool scriptModule::configure(ResourceFinder &rf) {
    Time::turboBoost();
    
    this->rfCopy = rf;

    if (rf.check("name"))
        name=string("/")+rf.find("name").asString().c_str();
    else
        name="/walkPlayer";

    contextPath=rf.getContextPath().c_str();
    fprintf(stderr,"||| contextPath: %s\n", contextPath.c_str());

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
    else {
        cout << "Configuration done." << endl;
    }

    if (!robot.init())
    {
        cerr<<"Error cannot connect to remote ports"<<endl;
        return false;
    }
    else {
        cout << "Initialization done." << endl;
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

    if (rf.check("speed")==true)
    {
        double fact = rf.find("speed").asDouble();
        cout << "speed factor set to "<<fact<< endl;
        thread.speed_factor= fact;
    }

    //*** open the position file
    cout << "opening file..." << endl;

    if (rf.check("filename")==true)
    {
        string filename = rf.find("filename").asString().c_str();
        if (!thread.actions.openFile(filename, rf))
        {
            cout << "ERROR: Unable to parse file" << endl;
            return false;
        };
    }
    
    if (rf.check("minJerkLimit")==true )
    {
        int tmpLimit = rf.find("minJerkLimit").asInt();
        thread.minJerkLimit = tmpLimit;
    }
    
    if (rf.check("refSpeedMinJerk")==true )
    {
        thread.refSpeedMinJerk = rf.find("refSpeedMinJerk").asDouble();
    } else {
        thread.refSpeedMinJerk = 0.0;
    }
    
    if (rf.check("torqueBalancingSequence")==true)
    {
        string filenamePrefix = rf.find("torqueBalancingSequence").asString().c_str();
        // Overwrite the execute flag value. This option has higher priority and
        // should simply stream trajectories use by the torqueBalancing module.
        thread.enable_execute_joint_command = true;
        if (!thread.actions.openTorqueBalancingSequence(filenamePrefix,rf))
        {
            cout << "ERROR: Unable to parse torque balancing sequence" << endl;
            return false;
        }
    }

    cout << "Using parameters:" << endl << rf.toString() << endl;
    cout << "module successfully configured. ready." << endl;
    return true;
}

bool scriptModule::respond(const Bottle &command, Bottle &reply)
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
                    // Re-read sequence for torqueBalancing
                    string filenamePrefix;
                    if (rfCopy.check("torqueBalancingSequence")==true)
                    {
                        filenamePrefix = rfCopy.find("torqueBalancingSequence").asString().c_str();
                        // Overwrite the execute flag value. This option has higher priority and
                        // should simply stream trajectories use by the torqueBalancing module.
                        //!!!!: Temporarily put this flag to true
                        thread.enable_execute_joint_command = true;
                        if (!thread.actions.openTorqueBalancingSequence(filenamePrefix,rfCopy))
                        {
                            cout << "ERROR: Unable to parse torque balancing sequence" << endl;
                            return false;
                        }
                    }

                    this->thread.actions.openFile(filenamePrefix, rfCopy);
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

bool scriptModule::close()
{
    rpcPort.interrupt();
    rpcPort.close();

    return true;
}

double getPeriod()    { return 1.0;  }
bool   updateModule() { return true; }
