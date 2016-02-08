#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <string.h>
#include <yarp/os/Vocab.h>

#include "WholeBodySensorTesterModule.h"

#include <wbi/wbiConstants.h>
#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>

using namespace std;
using namespace yarp::os;

WholeBodySensorTesterModule::WholeBodySensorTesterModule() : wholeBodySensorTesterThread(1000.0)
{

}

bool WholeBodySensorTesterModule::configure(ResourceFinder& rf)
{
    std::cout<<"Configuring the WholeBodySensorTesterModule\n";
    
    if (rf.check("name"))
        name=string("/")+rf.find("name").asString().c_str();
    else
        name="/wholeBodySensorTester";

    contextPath=rf.getContextPath().c_str();
    fprintf(stderr,"||| contextPath: %s\n", contextPath.c_str());

    rpcPort.open((name+"/rpc").c_str());
    attach(rpcPort);

//     Property portProp;
//     portProp.put("robot", rf.find("robot"));

    yarp::os::Property wbiProperties;
    if (!rf.check("wbi_config_file", "Checking wbi configuration file")) {
        yError("No WBI configuration file found.");
        return false;
        
    }
//     if (!rf.check("wbi_joint_list", "Checking wbi joint list name")) {
//                 yError("No joint list found. Please specify a joint list in \"wbi_joint_list\"");
//                 return false;
//     }

     if (!wbiProperties.fromConfigFile(rf.findFile("wbi_config_file"))) {
         yError("Not possible to load WBI properties from file.");
                return false;
         
    }
    wbiProperties.fromString(rf.toString(), false);
    m_moduleName = rf.check("name", Value("wholeBodySensorTester"), "Looking for module name").asString();
    m_robotName = rf.check("robot", Value("icub"), "Looking for robot name").asString();
    
    //create reference to wbs
    wbS = new yarpWbi::yarpWholeBodySensors(m_moduleName.c_str(), wbiProperties);
    if (!wbS) {
        yError("Could not create wbi object.");
                return false;
    }
    
    wbS->addAllSensors(wbi::SENSOR_ACCELEROMETER);
    
    if(!wbS->init()) {
        yError("Could not initialise wbi object.");
    }
    

    //*** start the robot driver
//     if (!robot.configure(portProp))
//     {
//         cerr<<"Error configuring position controller, check parameters"<<endl;
//         return false;
//     }
//     else {
//         cout << "Configuration done." << endl;
//     }
    wholeBodySensorTesterThread.attachWholeBodySensor(wbS);
   
    if (!wholeBodySensorTesterThread.start())
    {
        cerr<<"ERROR: Thread did not start, queue will not work"<<endl;
    }
    else
    {
        cout<<"Thread started"<<endl;
    }

    return(true);
//     return yarp::os::RFModule::configure(resourceFinder);
}

bool WholeBodySensorTesterModule::respond(const Bottle& command, Bottle& reply)
{
        bool ret=true;
    this->wholeBodySensorTesterThread.mutex.wait();

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
                }
            else if  (cmdstring == "start")
                {
//                     if (this->wholeBodySensorTesterThread.actions.current_action == 0)
//                         this->wholeBodySensorTesterThread.actions.current_status = ACTION_START;
//                     else
//                         this->wholeBodySensorTesterThread.actions.current_status = ACTION_RUNNING;
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "stop")
                {
//                     this->wholeBodySensorTesterThread.actions.current_status = ACTION_IDLE;
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "reset")
                {
//                     this->wholeBodySensorTesterThread.actions.current_status = ACTION_IDLE;
//                     this->wholeBodySensorTesterThread.actions.current_action = 0;
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

    this->wholeBodySensorTesterThread.mutex.post();
    return ret;
}


WholeBodySensorTesterModule::~WholeBodySensorTesterModule()
{
    std::cout<<"WholeBodySensorTesterModule destructor\n";
}

void WholeBodySensorTesterModule::cleanup()
{

}

bool WholeBodySensorTesterModule::updateModule()
{

}


/*
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
bool   updateModule() { return true; }*/
