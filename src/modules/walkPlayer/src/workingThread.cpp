#include <yarp/os/RateThread.h>
#include "workingThread.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

WorkingThread::WorkingThread(int period): RateThread(period)
{
    enable_execute_joint_command = true;
    //*** open the output port
    port_command_out.open("/walkPlayer/port_command_out:o");
    port_command_joints_ll.open("/walkPlayer/port_joints_ll:o");
    port_command_joints_rl.open("/walkPlayer/port_joints_rl:o");
    port_command_joints_to.open("/walkPlaer/port_joints_to:o");
    speed_factor = 1.0;
}

WorkingThread::~WorkingThread()
{
    port_command_out.interrupt();
    port_command_out.close();
    port_command_joints_ll.interrupt();
    port_command_joints_ll.close();
    port_command_joints_rl.interrupt();
    port_command_joints_rl.close();
    port_command_joints_to.interrupt();
    port_command_joints_to.close();
}

void WorkingThread::attachRobotDriver(robotDriver *p)
{
    if (p)  driver=p;
}

bool WorkingThread::threadInit()
{
    if (!driver)
        return false;
    return true;
}

bool WorkingThread::execute_joint_command(int j)
{
    if (!driver) return false;
    if (!enable_execute_joint_command) return true;

    double *ll = actions.action_vector[j].q_left_leg;
    double *rl = actions.action_vector[j].q_right_leg;
    double encs_ll[6]; double spd_ll[6]; double spd_to[3];
    double encs_rl[6]; double spd_rl[6]; double encs_to[3];
    int    modes[6];
    double to[3];
    to[0] = 0.0;
    to[1] = 0.0;
    to[2] = -6.0;

    if (j==0)
    {
        //the first step
        driver->ienc_ll->getEncoders(encs_ll);
        driver->ienc_rl->getEncoders(encs_rl);
        driver->ienc_to->getEncoders(encs_to);

        for (int i=0; i<6; i++)
        {
            spd_ll[i] = fabs(encs_ll[i]-ll[i])/4.0;
            spd_rl[i] = fabs(encs_rl[i]-rl[i])/4.0;
        }

        spd_to[0] = fabs(encs_to[0]-to[0])/4.0;
        spd_to[1] = fabs(encs_to[1]-to[1])/4.0;
        spd_to[2] = fabs(encs_to[2]-to[2])/4.0;

        driver->ipos_ll->setRefSpeeds(spd_ll);
        driver->ipos_rl->setRefSpeeds(spd_rl);
        driver->ipos_to->setRefSpeeds(spd_to);

        driver->ipos_to->positionMove(to);

        for (int i=0; i< 6; i++) modes[i] = VOCAB_CM_POSITION;
        driver->icmd_ll->setControlModes(modes);
        driver->icmd_rl->setControlModes(modes);

        driver->ipos_ll->positionMove(ll);
        driver->ipos_rl->positionMove(rl);

        cout << "going to to home position" << endl;

        yarp::os::Time::delay(6.0);
        cout << "done" << endl;

    }
    else
    {
        int mode_l =0;
        int mode_r =0;
        driver->icmd_rl->getControlMode(0,&mode_r);
        driver->icmd_ll->getControlMode(0,&mode_l);
        if (mode_l != VOCAB_CM_POSITION_DIRECT &&
            mode_r != VOCAB_CM_POSITION_DIRECT)
        {
            //change control mode
            for (int i=0; i< 6; i++) modes[i] = VOCAB_CM_POSITION_DIRECT;
            driver->icmd_ll->setControlModes(modes);
            driver->icmd_rl->setControlModes(modes);
        }

        driver->idir_ll->setPositions(ll);
        driver->idir_rl->setPositions(rl);
    }
    return true;
}

void WorkingThread::compute_and_send_command(int j)
{
    //compute the transformations
    Matrix m = driver->compute_tranformations(actions.action_vector[j]);
    //prepare the output command
    Bottle& bot = port_command_out.prepare();
    bot.clear();
    bot.addInt(actions.action_vector[j].counter);
    bot.addDouble(actions.action_vector[j].time);
    for (int ix=0;ix<4;ix++)
            for (int iy=0;iy<4;iy++)
                bot.addDouble(m(ix,iy));
    bot.addString(actions.action_vector[j].tag.c_str());
    //@@@ you can add stuff here...
    //send the output command
    port_command_out.write();
    execute_joint_command(j);

    //send the joints angles on debug port
    double *ll = actions.action_vector[j].q_left_leg;
    double *rl = actions.action_vector[j].q_right_leg;
    Bottle& bot2 = this->port_command_joints_ll.prepare();
    bot2.clear();
    Bottle& bot3 = this->port_command_joints_rl.prepare();
    bot3.clear();
    bot2.addInt(actions.action_vector[j].counter);
    bot2.addDouble(actions.action_vector[j].time);
    bot3.addInt(actions.action_vector[j].counter);
    bot3.addDouble(actions.action_vector[j].time);
    for (int ix=0;ix<6;ix++)
    {
        bot2.addDouble(ll[ix]);
        bot3.addDouble(rl[ix]);
    }
    this->port_command_joints_ll.write();
    this->port_command_joints_rl.write();
}

void WorkingThread::run()
{
    mutex.wait();
    double current_time = yarp::os::Time::now();
    static double last_time = yarp::os::Time::now();
    if (actions.current_status == ACTION_IDLE)
    {
        last_time = current_time;
    }
    else if (actions.current_status == ACTION_RUNNING)
    {
        //if it's not the last action
        size_t last_action = actions.action_vector.size();
        if (last_action == 0)
        {
            printf("sequence empty!\n");
            actions.current_status = ACTION_IDLE;
            return;
        }

        if (actions.current_action < last_action - 1)
        {
            //if enough time has passed from the previous action
            double duration = actions.action_vector[actions.current_action + 1].time -
                                actions.action_vector[actions.current_action].time;
            if (current_time - last_time > duration*speed_factor)
            {
                last_time = current_time;
                actions.current_action++;
                compute_and_send_command(actions.current_action);
                //printf("EXECUTING %d, elapsed_time:%.5f requested_time:%.5f\n", actions.current_action, current_time-last_time, duration);
            }
            else
            {
                //printf("WAITING %d, elapsed_time:%.5f requested_time:%.5f\n", actions.current_action, current_time-last_time, duration);
            }
        }
        else
        {
            printf("sequence complete\n");
            actions.current_status = ACTION_IDLE;
        }
    }
    else if (actions.current_status == ACTION_START)
    {
        compute_and_send_command(0);
        actions.current_status = ACTION_RUNNING;
    }
    else
    {
        printf("ERROR: unknown current_status\n");
    }
    mutex.post();
}