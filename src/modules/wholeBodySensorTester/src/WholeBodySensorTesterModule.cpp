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

bool WholeBodySensorTesterModule::setupWholeBodyState()
{

    yarp::os::Property wbiProperties;

    if (!rf.check("wbi_config_file", "Checking wbi configuration file")) {
        yError("No WBI configuration file found.");
        return false;

    }

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
    yarp::os::Bottle acclists = wbiProperties.findGroup("WBI_YARP_ACCELEROMETERS");
    yarp::os::Bottle gyrolists = wbiProperties.findGroup("WBI_YARP_GYROSCOPES");

    unsigned int numAcc = acclists.size()-1;
    unsigned int numGyro = gyrolists.size()-1;

    wholeBodySensorTesterThread.setMaxSensors(numAcc,numGyro);

    wholeBodySensorTesterThread.attachWholeBodySensor(wbS);

    return(true);
}


bool WholeBodySensorTesterModule::configure(ResourceFinder& resourceFinder)
{
    rf = resourceFinder;
    if (rf.check("name"))
        name=string("/")+rf.find("name").asString().c_str();
    else
        name="/wholeBodySensorTester";

    contextPath=rf.getContextPath().c_str();
    fprintf(stderr,"||| contextPath: %s\n", contextPath.c_str());

    rpcPort.open((name+"/rpc").c_str());
    attach(rpcPort);


    std::cout<<"Configuring the WholeBodySensorTesterModule\n";
    this->setupWholeBodyState();

    if (!wholeBodySensorTesterThread.start())
    {
        cerr<<"ERROR: Thread did not start, queue will not work"<<endl;
    }
    else
    {
        cout<<"Thread started"<<endl;
    }

    std::cout<<"WholeBodySensorTester Module running..\n";

    return(true);
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
                    std::cout<<"Starting the module\n\n";
                    wholeBodySensorTesterThread.setThreadState(THREAD_RUN);
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "stop")
                {
                    std::cout<<"Stopping the module but not killing\n\n";
                    wholeBodySensorTesterThread.setThreadState(THREAD_STOP);
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "reset")
                {
                    wholeBodySensorTesterThread.setThreadState(THREAD_PAUSE);
                    delete(wbS);
                    std::cout<<"Resetting wholeBodyState configuration\n\n";
                    setupWholeBodyState();
                    wholeBodySensorTesterThread.attachWholeBodySensor(wbS);
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
