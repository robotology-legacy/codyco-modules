// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/math/Rand.h>
#include <yarp/sig/Vector.h>

#include <vector>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot") && ( !params.check("part") || !params.check("parts") ) )
    {
        fprintf(stderr, "Please specify the name and part of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--part name (e.g. right_arm)\n");
        fprintf(stderr, "--parts (name1 name2 ... nameN) (e.g. (right_arm left_arm))");
        return -1;
    }

    std::string robotName=params.find("robot").asString().c_str();

    std::vector< std::string > parts;
    std::vector< IPositionControl * > poss;
    std::vector< IEncoders * > encss;
    std::vector< IControlLimits * > limss;
    std::vector< PolyDriver * > drivers;
    std::vector< Vector > q_maxs;
    std::vector< Vector > q_mins;


    if( params.check("part" ) )
    {
        std::string partName = params.find("part").asString().c_str();
        parts.push_back(partName);
    }

    if( params.check("parts") )
    {
        yarp::os::Bottle * b_parts = params.find("parts").asList();
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
        Vector command;
        Vector tmp;
        Vector q_max,q_min;
        encoders.resize(nj);
        tmp.resize(nj);
        command.resize(nj);
        q_max.resize(nj);
        q_min.resize(nj);

        for(int i=0; i < nj; i++ )
        {
            lims->getLimits(i,q_min.data()+i,q_max.data()+i);
        }

        q_mins.push_back(q_min);
        q_maxs.push_back(q_max);

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


    while(true)
    {
        for( int part = 0; part < parts.size(); part++ )
        {
            std::string msg = "Commading new random position for part "+parts[part]+"\n";
            std::cout << msg;

            yarp::sig::Vector command = yarp::math::Rand::vector(q_mins[part],q_maxs[part]);
            std::cout << command.toString() << std::endl;

            poss[part]->positionMove(command.data());
        }

       bool dones=false;
       while(!dones)
       {
           dones = true;
           for( int part = 0; part < parts.size(); part++ )
           {
               bool done;
               poss[part]->checkMotionDone(&done);
               dones = dones && done;
           }
           if( !dones) { Time::delay(0.1); }
       }

        printf("Random position reached, waiting\n");
        //Wait for 5 seconds after reaching the position
        Time::delay(5.0);
    }

    for( int part = 0; part < parts.size(); part++ )
    {
        drivers[part]->close();
    }
    return 0;
}
