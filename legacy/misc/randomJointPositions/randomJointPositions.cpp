// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/math/Rand.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>

#include <vector>

#include <string>
#include <boost/iterator/iterator_concepts.hpp>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;

struct boringJoint
{
public:
    int part;
    int joint_axis;
    double min_pos;
    double max_pos;
    double delta;
};

class randomJointPositionModule: public RFModule
{
private:
    std::vector< std::string > parts;
    std::vector< IPositionControl * > poss;
    std::vector< IEncoders * > encss;
    std::vector< IControlLimits * > limss;
    std::vector< PolyDriver * > drivers;
    std::vector< Vector > q_maxs;
    std::vector< Vector > q_mins;
    std::vector< Vector > commanded_pos;
    std::vector< boringJoint > boring_joints;
    std::vector< bool > boring_joints_initialized;

    double period;
    bool send_new_position;
    
public:
    virtual bool configure(ResourceFinder &rf)
    {
         std::string robotName=rf.find("robot").asString().c_str();


        if( rf.check("part" ) )
        {
            std::string partName = rf.find("part").asString().c_str();
            parts.push_back(partName);
        }

        if( rf.check("parts") )
        {
            yarp::os::Bottle * b_parts = rf.find("parts").asList();
            for( int i = 0; i < b_parts->size(); i++ )
            {
                parts.push_back(b_parts->get(i).asString().c_str());
            }
        }
        
        for( int i = 0; i < parts.size(); i++ )
        {
            std::string remotePort="/";
            remotePort+=robotName;
            remotePort+="/";
            remotePort+=parts[i];

            std::string localPort="/randomJointPositions"+remotePort;

            Property options;
            options.put("device", "remote_controlboard");
            options.put("local", localPort.c_str());   //local port names
            options.put("remote", remotePort.c_str());         //where we connect to

            // create a device
            drivers.push_back(new PolyDriver(options));
            if (!drivers[i]->isValid())
            {
                printf("Device not available.  Here are the known devices:\n");
                printf("%s", Drivers::factory().toString().c_str());
                return 0;
            }

            IPositionControl *pos;
            IEncoders *encs;
            IControlLimits *lims;

            bool ok;
            ok = drivers[i]->view(pos);
            ok = ok && drivers[i]->view(encs);
            ok = ok && drivers[i]->view(lims);

            poss.push_back(pos);
            encss.push_back(encs);
            limss.push_back(lims);

            if (!ok)
            {
                printf("Problems acquiring interfaces\n");
                return 0;
            }


            int nj=0;
            pos->getAxes(&nj);
            Vector encoders;
            Vector tmp;
            Vector q_max,q_min;
            Vector commanded;
            encoders.resize(nj);
            tmp.resize(nj);
            commanded.resize(nj);
            q_max.resize(nj);
            q_min.resize(nj);

            for(int i=0; i < nj; i++ )
            {
                lims->getLimits(i,q_min.data()+i,q_max.data()+i);
            }

            q_mins.push_back(q_min);
            q_maxs.push_back(q_max);
            commanded_pos.push_back(commanded);

            /*
            *     int i;
                for (i = 0; i < nj; i++) {
                tmp[i] = 50.0;
            }
            pos->setRefAccelerations(tmp.data());

            for (i = 0; i < nj; i++) {
                tmp[i] = 10.0;
            pos->setRefSpeed(i, tmp[i]);
            }*/

            printf("waiting for encoders");
            while(!encs->getEncoders(encoders.data()))
            {
                Time::delay(0.1);
                printf(".");
            }
            printf("\n;");

            command=encoders;
        }
        
        if( rf.check("boringMode") )
        {
            std::cout << "Entering boring mode" << std::endl;
            Bottle list = rf.findGroup("boringMode");
            for(int i=1; i < list.size(); i++)
            {
                boringJoint boring_joint;

                std::string part_name = list.get(i).asList()->get(0).asString().c_str();
                for( int part = 0; part < parts.size(); part++ )
                {
                    if( parts[part] == part_name )
                    {
                        boring_joint.part = part;
                    }
                }
                
                boring_joint.joint_axis = list.get(i).asList()->get(1).asInt();
                boring_joint.min_pos = list.get(i).asList()->get(2).asDouble();
                boring_joint.max_pos = list.get(i).asList()->get(3).asDouble();
                boring_joint.delta = list.get(i).asList()->get(4).asDouble();
                
                boring_joints.push_back(boring_joint);
            }
 
            boring_joints_initialized.resize(boring_joints.size(),false);
        }

        
        period = rf.check("period",Value(0.1),"Period of the module");
        waiting_time = rf.check("waitingTime",Value(1.0),"Period to wait in each desired configuration");
        send_new_position = true;
    }
    
    virtual bool close()
    {
        for( int part = 0; part < parts.size(); part++ )
        {
            drivers[part]->close();
        }
        return true;
    }

    virtual double getPeriod() 
    { 
        return period; 
    }
    
    virtual bool updateModule()
    { 
        if( !send_new_position )
        {
            //Wait that all position motion are completed
            bool dones=true;
            for( int part = 0; part < parts.size(); part++ )
            {
               bool done;
               poss[part]->checkMotionDone(&done);
               dones = dones && done;
            }
            if( dones ) 
            {
                //compute new position                
                send_new_position = true;
            }
        }
        
        if( send_new_position  )
        {
            //Compute new position
            if( boring_joints.size() == 0 )
            {
                //Not in boring mode
                for( int part = 0; part < parts.size(); part++ )
                {
                   commanded_pos[part] = yarp::math::Rand::vector(q_mins[part],q_maxs[part]);
                }
            } 
            else
            {
                //boring mode
                bool iteration_overflow = true;
                for( int boring_jnt=0; boring_jnt < boring_joints.size(); boring_jnt++ ) 
                {
                     boringJoint boring_joint = boring_joints[boring_jnt];
                     int part = boring_joint.part;
                     int joint = boring_joint.joint_axis;
                     if( !boring_joints_initialized[boring_jnt] ) 
                     {
                         commanded_pos[part](joint) = boring_jnt.min_pos;
                         boring_joints_initialized[boring_jnt] = true;
                     } 
                     else
                     {
                        if( iteration_overflow ) 
                        {
                            commanded_pos[part](joint) = commanded_pos[part](joint)+boring_joint.delta;
                            iteration_overflow = false;
                            if( commanded_pos[part](joint) > boring_joint.pos_max )
                            {   
                                commanded_pos[part](joint) = boring_joint.pos_min;
                                iteration_overflow = true;
                            } 
                        } 
                     }
                     
                }
            }
            
            //Send new position
            for( int part = 0; part < parts.size(); part++ )
            {
                std::string msg = "Commading new random position for part "+parts[part]+"\n";
                std::cout << msg;
    
                std::cout << command.toString() << std::endl;

                poss[part]->positionMove(commanded_pos[part].data());
            }
            send_new_position = false;
        }
        else
        
        
    }
}


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);
    
    if (rf.check("help") || !rf.check("robot") || !( rf.check("part") || rf.check("parts") ) )
    {
        cerr << "Please specify the name and joints of the robot\n";
        cerr << "--robot name (e.g. icub)\n";
        cerr << "--joints ((part_name axis_number lower_limit upper_limit) ... )\n";
        return -1;       
    }
    
    Network yarp;
    if (!yarp.checkNetwork())
    {

        cout<<"YARP server not available!"<<endl;
        return -1;
    }
    
    randomJointPositionModule randJntModule;
    return randJntModule.runModule(rf);


    while(true)
    {
      


        printf("Random position reached, waiting\n");
        //Wait for 5 seconds after reaching the position
        Time::delay(5.0);
    }

   
    return 0;
}
