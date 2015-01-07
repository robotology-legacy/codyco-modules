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
        if (!thread.actions.openFile2(filename, rf))
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
