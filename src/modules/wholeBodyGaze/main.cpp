/**
 * @ingroup icub_tutorials
 *
 * \defgroup icub_fwInvKinematics Forward/Inverse Kinematics
 *           Example
 *
 * A tutorial on how to use iKin library for forward/inverse
 * kinematics.
 *
 * \author Ugo Pattacini
 *
 * CopyPolicy: Released under the terms of GPL 2.0 or later
 */

#include <iostream>
#include <iomanip>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>

#include <iCub/iKin/iKinFwd.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

class wholeBodyGazeThread:public RFModule
{
    //port
    BufferedPort<Vector> port;
    
    // create a device
    PolyDriver *robotDevice;
    
    IPositionControl *pos;
    IEncoders *encs;
    
    //number of joints
    int nj;

    //leg chain
    iKinChain *chain;
    iCubLeg *leg;
    
    Vector q;
    Matrix Hroot_effector;
    
    Vector fixation_effector;
    Vector fixation_root;
public:
    
    bool configure (yarp::os::ResourceFinder &rf)
    {        
        cout << "Configurig module" << endl;
        // instantiate left leg
        leg = new iCubLeg("left");
        chain=leg->asChain();

        std::string robotName;
        std::string partName;
        std::string moduleName;
        cout << "Checking configuraions" << endl; 
        if (!rf.check("robot"))
        {
            fprintf(stderr, "Please specify the name of the robot\n");
            fprintf(stderr, "--robot name (e.g. icub)\n");
            return false;
        }
        else
        {
            robotName=rf.find("robot").asString().c_str();
            cout << "robot was found!" << endl;
        }
        
        if (!rf.check("part"))
        {
            fprintf(stderr, "Please specify the name of the robot\n");
            fprintf(stderr, "--part name (e.g. icub)\n");
            return false;
        }
        else
        {
            partName=rf.find("part").asString().c_str();
            cout << "part was found! " << partName.c_str() << endl;
        }

        if (!rf.check("name"))
        {
            fprintf(stderr, "Module name was not found setting default\n");
            moduleName = "/wholeBodyGaze";
        }
        else
        {
            moduleName=rf.find("name").asString().c_str();
            cout << "part was found! " << moduleName.c_str() << endl;
        }
        
        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", moduleName.c_str());   //local port names
        options.put("remote", ("/"+robotName+"/"+partName).c_str());         //where we connect to
        
        robotDevice = new PolyDriver(options);
        // create a device
        if (!robotDevice->isValid()) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 0;
        }
        else
            cout << "Device was found valid" << endl;
                
        bool ok;
        ok = robotDevice->view(pos);
        ok = ok && robotDevice->view(encs);
        
        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return 0;
        }
        
        cout << "Getting axes" << endl;
        pos->getAxes(&nj);
        cout << "Axes are: " << nj << endl;
        q.resize(nj);
        
        cout << "Trying to get encoders" << endl;
        while(!encs->getEncoders(q.data()))
        {
            cout << "Problem getting encoders!" << endl;
            Time::delay(1);
        }
        
        cout << "Encoders are " << q.toString().c_str() << endl;
        
        port.open(moduleName+"/xd:o");
        return true;
    }
    
    bool close()
    {
        printf("ControlThread:stopping the robot\n");
        
        if (robotDevice!=NULL)
        {
            robotDevice->close();
            delete robotDevice;
        }
        printf("Done, goodbye from ControlThread\n");
        
        return true;
    }
    
    bool updateModule()
    {
       
        while(!encs->getEncoders(q.data()))
        {
            cout << "Problem getting encoders!" << endl;
        }

        // cout << "Encoders are " << q.toString().c_str() << endl;
        
        q=chain->setAng(q*CTRL_DEG2RAD);
        // cout << "Actual joints set to " << (CTRL_RAD2DEG*q).toString().c_str() << endl;
        
        
        // retrieve the end-effector pose.
        // Translational part is in meters.
        // Rotational part is in axis-angle representation
        Hroot_effector=chain->getH();
        // cout << "Current leg end-effector pose: " << endl << Hroot_effector.toString().c_str() << endl;
        
        // Matrix Heffector_root = SE3inv(Hroot_effector);
        // cout << "Current root pose: " << endl << Heffector_root.toString().c_str() << endl;
        
        fixation_effector(4);
        fixation_effector(0) = 1.0;  fixation_effector(1) = 0.0;
        fixation_effector(2) = 3.0;  fixation_effector(3) = 1.0;
        
        fixation_root = Hroot_effector * fixation_effector;
        // cout << "Current gazed point: " << endl << fixation_root.toString().c_str() << endl;
        
        Vector& b = port.prepare();
        b.resize(3);
        b = fixation_root.subVector(0, 2);
        port.write();
        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile("config.ini");
    rf.setDefaultContext("randomMotion");
    rf.configure(argc, argv);
    
    wholeBodyGazeThread ct;
    if(ct.configure(rf))
        ct.runModule();
    cout << "Exiting" << endl;
    
    return 0;
}


