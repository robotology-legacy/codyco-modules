#ifndef WORKINGTHREAD_H
#define WORKINGTHREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <math.h>

#include "robotDriver.h"
#include "actionClass.h"




// ******************** THE THREAD
class WorkingThread: public yarp::os::RateThread {
private:

    yarp::os::Stamp timestamp;
public:
    robotDriver                               *driver;
    actionClass                               actions;
    yarp::os::Semaphore                       mutex;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_out;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_joints_ll;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_joints_rl;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_joints_to;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_com;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_postural;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_constraints;
    bool                                      enable_execute_joint_command;
    double                                    speed_factor;
    int                                       minJerkLimit;
    double                                    refSpeedMinJerk;

    WorkingThread(int period=5);
    ~WorkingThread();
    void attachRobotDriver(robotDriver *p);
    bool threadInit();
    bool execute_joint_command(int j);
    void compute_and_send_command(int j);
    void run();
};

#endif