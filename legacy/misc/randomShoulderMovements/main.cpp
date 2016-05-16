// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/api.h>
#include <yarp/os/Random.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include <string>
#include <vector>
#include <cstdlib>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;

int generateRandomShoulderPosition(Vector & desPosInDeg,
                                   const Vector & minInDeg,
                                   const Vector & maxInDeg,
                                   const Vector & lowerBoundConstriant,
                                   const Vector & upperBoundConstriant,
                                   const Matrix & constraintMatrixInDeg)
{
    desPosInDeg.resize(3);

    bool ok = false;
    while(!ok)
    {
        for(int i=0; i < 3; i++ )
        {
            double randUniform = yarp::os::Random::uniform();
            desPosInDeg(i) = minInDeg(i) + randUniform*(maxInDeg(i)-minInDeg(i));
        }

        Vector b = constraintMatrixInDeg*desPosInDeg;

        ok = true;
        for(int i=0; i < b.size(); i++ )
        {
            if( ( b(i) < lowerBoundConstriant(i) ) ||
                ( b(i) > upperBoundConstriant(i) ) )
            {
                ok = false;
            }
        }
    }
}

void getShoulderConstraint(Vector & _lB,
                           Vector & _uB,
                           Matrix & _C)
{
    int constrMatrixCols = 3;
    double upperBoundInf = 1000000.0;
    double lowerBoundInf = -100000.0;

    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    joint1_0= 28.0;
    joint1_1= 23.0;
    joint2_0=-37.0;
    joint2_1= 80.0;
    double shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    double shou_n=joint1_0-shou_m*joint2_0;

    double joint3_0, joint3_1;
    double joint4_0, joint4_1;
    joint3_0= 85.0;
    joint3_1=105.0;
    joint4_0= 90.0;
    joint4_1= 40.0;
    double elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
    double elb_n=joint4_0-elb_m*joint3_0;

    // resize
    _lB.resize(0);
    _uB.resize(0);
    _C.resize(0,3);

    Vector row;
    row.resize(constrMatrixCols);

    int offs = 0;
    // constraints on the cables length
    row.zero();
    row[offs]=1.71; row[offs+1]=-1.71;
    _C=pile(_C,row);
    _lB=cat(_lB,-347.00);
    _uB=cat(_uB,upperBoundInf);

    row.zero();
    row[offs]=1.71; row[offs+1]=-1.71; row[offs+2]=-1.71;
    _C=pile(_C,row);
    _lB=cat(_lB,-366.57);
    _uB=cat(_uB,112.42);

    row.zero();
    row[offs+1]=1.0; row[offs+2]=1.0;
    _C=pile(_C,row);
    _lB=cat(_lB,-66.60);
    _uB=cat(_uB,213.30);

    // constraints to prevent arm from touching torso
    row.zero();
    row[offs+1]=1.0; row[offs+2]=-shou_m;
    _C=pile(_C,row);
    _lB=cat(_lB,shou_n);
    _uB=cat(_uB,upperBoundInf);

    // constraints to limit shoulder abduction
    double IKINIPOPT_SHOULDER_MAXABDUCTION = 100.0;
    row.zero();
    row[offs+1]=1.0;
    _C=pile(_C,row);
    _lB=cat(_lB,lowerBoundInf);
    _uB=cat(_uB,IKINIPOPT_SHOULDER_MAXABDUCTION);
}

int main(int argc, char *argv[])
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--part name (right_arm or left_arm)\n");
        fprintf(stderr, "--durationInS duration (in seconds)\n");
        fprintf(stderr, "--sampleInMS sample time (in milliseconds)\n");
        fprintf(stderr, "--refTimeInMS Reference time of minimum jerk trajectory generator (in milliseconds)\n");
        return 1;
    }

    if( !params.check("sampleInMS") ||
        !params.check("durationInS") ||
        !params.check("refTimeInMS") )
    {
        fprintf(stderr, "Necessary params not passed\n");
        return EXIT_FAILURE;
    }

    std::string robotName=params.find("robot").asString().c_str();
    std::string partName=params.find("part").asString().c_str();
    double durationInS = params.find("durationInS").asDouble();
    double samplesInMS = params.find("sampleInMS").asDouble();
    double refTimeInMS = params.find("refTimeInMS").asDouble();

    std::cout << "robotName : " << robotName << std::endl;
    std::cout << "partName : " << partName << std::endl;
    std::cout << "durationInS : " << durationInS << std::endl;
    std::cout << "samplesInMS : " << samplesInMS << std::endl;
    std::cout << "refTimeInMS : " << refTimeInMS << std::endl;

    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/"+partName;

    std::string localPorts="/randomShoulderMovements/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionDirect *pos;
    IEncoders *encs;
    IControlLimits * lims;
    IControlMode2 * ictrl;


    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);
    ok = ok && robotDevice.view(lims);
    ok = ok && robotDevice.view(ictrl);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encodersInDeg(nj);
    Vector desPosInDeg(3);
    Vector commandInDeg(3);
    Vector minShoulderInDeg(3);
    Vector maxShoulderInDeg(3);
    Vector lowerBoundConstraint;
    Vector upperBoundConstraint;
    Matrix constraintMatrix;

    for(int i = 0; i < 3; i++ )
    {
        lims->getLimits(i,&(minShoulderInDeg[i]),&(maxShoulderInDeg[i]));
    }

    bool encodersRead = encs->getEncoders(encodersInDeg.data());

    while( !encodersRead )
    {
        encodersRead = encs->getEncoders(encodersInDeg.data());
    }


    getShoulderConstraint(lowerBoundConstraint,upperBoundConstraint,constraintMatrix);

    // Create the minimum jerk filter
    iCub::ctrl::minJerkTrajGen filter(3,samplesInMS,refTimeInMS);

    filter.init(encodersInDeg.subVector(0,2));

    std::cout << "Filter initial value " << encodersInDeg.subVector(0,2).toString() << std::endl;

    std::vector<int> indexToControl;
    indexToControl.push_back(0);
    indexToControl.push_back(1);
    indexToControl.push_back(2);

    // Set control modes
    ictrl->setControlMode(0,VOCAB_CM_POSITION_DIRECT);
    ictrl->setControlMode(1,VOCAB_CM_POSITION_DIRECT);
    ictrl->setControlMode(2,VOCAB_CM_POSITION_DIRECT);


    double samplesInS = samplesInMS/1000.0;
    int samples = (int)(durationInS/samplesInS);
    int samplesForNewPosition = (int)(refTimeInMS/samplesInMS);
    for(int i=0; i < samples; i++ )
    {
        if( i % samplesForNewPosition == 0 )
        {
            // Generate a new desired shoulder position
            generateRandomShoulderPosition(desPosInDeg,minShoulderInDeg,maxShoulderInDeg,
                                           lowerBoundConstraint,upperBoundConstraint,constraintMatrix);

            std::cout << "New position generated" << std::endl;
            std::cout << "Position generated " << desPosInDeg.toString() << std::endl;
            std::cout << "Remaining time " << (samples-i)*(samplesInS) << std::endl;
        }

        filter.computeNextValues(desPosInDeg);

        commandInDeg = filter.getPos();

        std::cout << "Position set " << commandInDeg.toString() << std::endl;


        pos->setPositions(3,indexToControl.data(),commandInDeg.data());

        yarp::os::Time::delay(samplesInS);
    }

    robotDevice.close();

    return 0;
}