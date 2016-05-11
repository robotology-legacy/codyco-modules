#include <yarp/os/RateThread.h>
#include <cmath>
#include <unistd.h>

#include "WholeBodySensorTesterThread.h"

WholeBodySensorTesterThread::WholeBodySensorTesterThread(int period): RateThread(period)
{
    maxAccelerometers = 3;
    maxGyroscopes = 0;
}

WholeBodySensorTesterThread::~WholeBodySensorTesterThread()
{

}

bool WholeBodySensorTesterThread::setThreadState(const ThreadState& ts)
{
    threadState = ts;
    return(true);
}


bool WholeBodySensorTesterThread::threadInit()
{
    std::cout<<"Thread inited \n";
    return true;
}


void WholeBodySensorTesterThread::run()
{
    static int ctr = 0;
    mutex.wait();
    double current_time = yarp::os::Time::now();

    std::cout<<"Tester Thread Run Counter :"<<ctr++<<"\n";

    if(threadState==THREAD_RUN){
        scanAndDisplaySensorData();
    }
    else
    {
        yarp::os::Time::delay(1);
        std::cout<<"Thread paused..waiting.\n";
    }
    mutex.post();
}

void WholeBodySensorTesterThread::attachWholeBodySensor(wbi::iWholeBodySensors* s)
{
    wbs = s;
}

bool WholeBodySensorTesterThread::scanAndDisplaySensorData()
{
    double sensorReading[3], multiSensorReading[9];

    for(int accIdx = 0; accIdx<maxAccelerometers; accIdx++)
    {
        wbs->readSensor(wbi::SENSOR_ACCELEROMETER,accIdx,sensorReading);
        std::cout<<"---Accelerometer read sensor"<<accIdx<<" : ("<<sensorReading[0]<<","<<sensorReading[1]<<","<<sensorReading[2]<<")\n";
    }
    std::cout<<"---------\n\n";

    wbs->readSensors(wbi::SENSOR_ACCELEROMETER,multiSensorReading);
    for(int accIdx = 0;accIdx<maxAccelerometers; accIdx++)
    {
        std::cout<<"---Accelerometer multiread sensor"<<accIdx<<" : ("<<multiSensorReading[accIdx*3]<<","<<multiSensorReading[accIdx*3+1]<<","<<multiSensorReading[accIdx*3+2]<<")\n";
    }
    std::cout<<"----------------------------\n";
}

bool WholeBodySensorTesterThread::setMaxSensors(unsigned int maxAcc, unsigned int maxGyro)
{
    std::cout<<"------MaxSensors set in tester thread as "<<maxAcc<<", "<<maxGyro<<"\n\n";
    maxAccelerometers = maxAcc;
    maxGyroscopes = maxGyro;
    return(true);
}
