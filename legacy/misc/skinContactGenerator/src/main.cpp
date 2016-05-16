#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>

#include <iCub/skinDynLib/skinContactList.h>

#include "skinContactGenerator_IDLServer.h"

using namespace yarp::os;
using namespace iCub::skinDynLib;
using namespace std;

class skinContactGeneratorModule: public RFModule, public skinContactGenerator_IDLServer
{
    /* module parameters */
    string  moduleName;
    string  robotName;
    double     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

    Port                rpcPort;        // a port to handle rpc messages

    BufferedPort<skinContactList> skinContactsPort;

    skinContactList skinContacts;

    yarp::os::Mutex buffer_mutex;

public:
    skinContactGeneratorModule();

    bool attach(yarp::os::Port &source);          // Attach the module to a RPC port
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    double getPeriod(){ return period;  }
    bool updateModule();

    /** RPC methods (Thrift) */
/**
 * Configure the skinContactGenerator to produce a contact
 * at the origin of a given link.
 */
  virtual bool setContact(const int32_t bodyPart, const int32_t link);

  /**
 * Configure the skinContactGenerator to produce a contact
 * at a prescribed position of a given link.
 */
  virtual bool setContactForce(const int32_t bodyPart,
                               const int32_t link,
                               const double f_x,
                               const double f_y,
                               const double f_z,
                               const double p_x,
                               const double p_y,
                               const double p_z,
                               const int32_t skinPart = 0);


  /**
 * Configure the skinContactGenerator to produce a contact
 * at the origin of a given link.
 */
  virtual bool setContactName(const std::string& bodyPart, const std::string& link);
/**
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();

};

skinContactGeneratorModule::skinContactGeneratorModule():
    skinContacts(0)
{
}


bool skinContactGeneratorModule::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}

bool skinContactGeneratorModule::configure(ResourceFinder& rf)
{
    std::string moduleName = rf.check("name",
            yarp::os::Value("skinContactGenerator"),
            "module name (string)").asString().c_str();
    setName(moduleName.c_str());

    period = rf.check("period",
            yarp::os::Value(0.5),
            "period of the module").asDouble();

    std::cout << "["<<getName()<<"] : period : " << getPeriod() << std::endl;

    std::string skin_port_suffix =
         rf.check("port",yarp::os::Value("skin_events:o"),"").asString().c_str();
    std::string skin_port_name = "/" + getName() + "/" + skin_port_suffix;
    if( !skinContactsPort.open(skin_port_name.c_str()) )
    {
        std::cerr << "[" << getName() << "]" << ": Unable to open port " << skin_port_name << std::endl;
        return false;
    }

    //Rpc port
    attach(rpcPort);
    std::string rpcPortName= "/";
    rpcPortName+= getName();
    rpcPortName += "/rpc:i";
    if (!rpcPort.open(rpcPortName.c_str()))
    {
        std::cerr << "[" << getName() << "]" << ": Unable to open port " << rpcPortName << std::endl;
        return false;
    }

    return yarp::os::RFModule::configure(rf);
}

bool skinContactGeneratorModule::interruptModule()
{
    return yarp::os::RFModule::interruptModule();
}

bool skinContactGeneratorModule::updateModule()
{
    buffer_mutex.lock();
    skinContactsPort.prepare() = skinContacts;
    skinContactsPort.write();
    buffer_mutex.unlock();
    return true;
}

bool skinContactGeneratorModule::quit()
{
    return this->close();
}

bool skinContactGeneratorModule::setContact(const int32_t bodyPart, const int32_t link)
{
    buffer_mutex.lock();
    dynContact dyn_contact((BodyPart)bodyPart,link,yarp::sig::Vector(3,0.0));
    skinContact skin_contact(dyn_contact);
    skinContactList new_skinContacts = skinContactList(1,skin_contact);
    skinContacts = new_skinContacts;
    buffer_mutex.unlock();
    return true;
}

bool skinContactGeneratorModule::setContactForce(const int32_t bodyPart,
                     const int32_t link,
                     const double f_x,
                     const double f_y,
                     const double f_z,
                     const double p_x,
                     const double p_y,
                     const double p_z,
                     const int32_t skinPart)
{
    buffer_mutex.lock();
    yarp::sig::Vector cop(3,0.0), f(3,0.0);
    cop[0] = p_x;
    cop[1] = p_y;
    cop[2] = p_z;
    f[0] = f_x;
    f[1] = f_y;
    f[2] = f_z;
    dynContact dyn_contact((BodyPart)bodyPart,link,cop);
    dyn_contact.setForce(f);
    skinContact skin_contact(dyn_contact);
    skin_contact.setSkinPart((SkinPart)skinPart);
    skinContactList new_skinContacts = skinContactList(1,skin_contact);
    skinContacts = new_skinContacts;
    buffer_mutex.unlock();
    return true;
}


bool skinContactGeneratorModule::setContactName(const std::string& bodyPart, const std::string& link)
{
    int32_t bodyPart_id = -1;
    int32_t link_id = -1;
    if(bodyPart=="right_arm")
    {
        bodyPart_id=RIGHT_ARM;
        if(link=="r_hand")
        {
            link_id=6;
        }
        else if(link=="r_forearm")
        {
            link_id=4;
        }
    }
    else if(bodyPart=="left_arm")
    {
        bodyPart_id=LEFT_ARM;
        if(link=="l_hand")
        {
            link_id=6;
        }
        else if(link=="l_forearm")
        {
            link_id=4;
        }
    }
    else if(bodyPart=="torso")
    {
        bodyPart_id=LEFT_ARM;
        if(link=="root_link")
        {
            link_id=0;
        }
        else if(link=="chest")
        {
            link_id=4;
        }
    }
    if( bodyPart_id != -1 &&
        link_id != -1 )
    {
        return setContact(bodyPart_id,link_id);
    }
    else
    {
        return false;
    }
}

bool skinContactGeneratorModule::close()
{
    skinContactsPort.close();
    rpcPort.close();
    return true;
}



int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("default.ini");         //default config file name.
    rf.setDefaultContext("skinContactGenerator"); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<< "Possible parameters" << endl;
        cout<< "TORSO ID : " << TORSO << endl;
        cout<< "TORSO SKIN ID : " << SKIN_FRONT_TORSO << endl;
        cout<< "TORSO SKIN ID BODY PART : " << getBodyPart(SKIN_FRONT_TORSO) << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    skinContactGeneratorModule module;
    return module.runModule(rf);
}


