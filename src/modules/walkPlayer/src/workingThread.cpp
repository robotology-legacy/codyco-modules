#include <yarp/os/RateThread.h>
#include <iCub/ctrl/math.h>
#include "workingThread.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

WorkingThread::WorkingThread(int period): RateThread(period)
{
    enable_execute_joint_command = true;
    //*** open the output ports
    port_command_out.open("/walkPlayer/port_command_out:o");
    port_command_joints_ll.open("/walkPlayer/port_joints_ll:o");
    port_command_joints_rl.open("/walkPlayer/port_joints_rl:o");
    port_command_joints_to.open("/walkPlayer/port_joints_to:o");
    port_command_com.open("/walkPlayer/com:o");
    port_command_postural.open("/walkPlayer/postural:o");
    port_command_constraints.open("/walkPlayer/constraints:o");
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
    port_command_postural.interrupt();
    port_command_postural.close();
    port_command_com.interrupt();
    port_command_com.close();
    port_command_constraints.interrupt();
    port_command_constraints.close();
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

    double tmp_q_left_leg[6];
    double tmp_q_right_leg[6];
    double tmp_q_torso[3];

    for (int i = 0; i < 6; i++) {
       tmp_q_left_leg[i]  = CTRL_RAD2DEG * actions.action_vector[j].q_left_leg[i];
       tmp_q_right_leg[i] = CTRL_RAD2DEG * actions.action_vector[j].q_right_leg[i];
    }

    for (int i = 0; i < 3; i++) {
        tmp_q_torso[i] = CTRL_RAD2DEG * actions.action_vector[j].q_torso[i];
    }

    double *ll = tmp_q_left_leg;
    double *rl = tmp_q_right_leg;
    double *to = tmp_q_torso;

    // Swapping torso joints
    double tmp;
    tmp = to[0];
    to[0] = to[2];
    to[2] = tmp;

    double encs_ll[6]; double spd_ll[6];
    double encs_rl[6]; double spd_rl[6];
    double encs_to[3]; double spd_to[3];
    int    modes[6];
    int    modes_to[3];
    double REF_SPEED_FACTOR = this->refSpeedMinJerk; //0.5 -> position direct like
    int    LIMIT_MIN_JERK = this->minJerkLimit;

    if (REF_SPEED_FACTOR != 0 && j>=0)
    {
        bool checkMotionDone = false;
        //the whole body joint trajectory for the first step is done with min jerk controllers rather than position direct
        driver->ienc_ll->getEncoders(encs_ll);
        driver->ienc_rl->getEncoders(encs_rl);
        driver->ienc_to->getEncoders(encs_to);

        for (int i=0; i<6; i++)
        {
            spd_ll[i] = fabs(encs_ll[i]-ll[i])/REF_SPEED_FACTOR;
            spd_rl[i] = fabs(encs_rl[i]-rl[i])/REF_SPEED_FACTOR;
        }

        for (int i=0; i<3; i++)
        {
            spd_to[i] = fabs(encs_to[i] - to[i])/REF_SPEED_FACTOR;
        }

        driver->ipos_ll->setRefSpeeds(spd_ll);
        driver->ipos_rl->setRefSpeeds(spd_rl);
        driver->ipos_to->setRefSpeeds(spd_to);

        driver->ipos_to->positionMove(to);

        for (int i=0; i< 6; i++) modes[i] = VOCAB_CM_POSITION;
        for (int i=0; i< 3; i++) modes_to[i] = VOCAB_CM_POSITION;
        driver->icmd_ll->setControlModes(modes);
        driver->icmd_rl->setControlModes(modes);
        driver->icmd_to->setControlModes(modes_to);

        driver->ipos_ll->positionMove(ll);
        driver->ipos_rl->positionMove(rl);
        driver->ipos_to->positionMove(to);

        cout << "going to home position" << endl;

        while(!checkMotionDone) {
            cout << "Checking motion done" << endl;
        driver->ipos_ll->checkMotionDone(&checkMotionDone);
        driver->ipos_rl->checkMotionDone(&checkMotionDone);
        driver->ipos_to->checkMotionDone(&checkMotionDone);
        }
        if ( j==0 )
            yarp::os::Time::delay(3.0);
        cout << "done" << endl;

    }
    else
    {
        int mode_l =0;
        int mode_r =0;
        int mode_t =0;
        driver->icmd_rl->getControlMode(0,&mode_r);
        driver->icmd_ll->getControlMode(0,&mode_l);
        driver->icmd_to->getControlMode(0,&mode_t);
        if (mode_l != VOCAB_CM_POSITION_DIRECT &&
            mode_r != VOCAB_CM_POSITION_DIRECT &&
            mode_t != VOCAB_CM_POSITION_DIRECT)
        {
            //change control mode
            // VOCAB_CM_POSITION_DIRECT
            for (int i=0; i < 6; i++) modes[i] = VOCAB_CM_POSITION_DIRECT;
            for (int i=0; i < 3; i++) modes_to[i] = VOCAB_CM_POSITION_DIRECT;
            driver->icmd_ll->setControlModes(modes);
            driver->icmd_rl->setControlModes(modes);
            driver->icmd_to->setControlModes(modes_to);
        }

        driver->idir_ll->setPositions(ll);
        driver->idir_rl->setPositions(rl);
        driver->idir_to->setPositions(to);
    }
    return true;
}

void WorkingThread::compute_and_send_command(int j)
{
    this->timestamp.update();
    // TODO Remove compute_transformations as it seems to be unused.
    //compute the transformations
    Matrix m = driver->compute_transformations(actions.action_vector[j]);
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
    port_command_out.setEnvelope(this->timestamp);
    port_command_out.write();
    execute_joint_command(j);

    //send the joints angles on debug port
    double *ll = actions.action_vector[j].q_left_leg;
    double *rl = actions.action_vector[j].q_right_leg;
    double *to = actions.action_vector[j].q_torso;
    Bottle& bot2 = this->port_command_joints_ll.prepare();
    bot2.clear();
    Bottle& bot3 = this->port_command_joints_rl.prepare();
    bot3.clear();
    Bottle& bot4 = this->port_command_joints_to.prepare();
    bot4.clear();
    bot2.addInt(actions.action_vector[j].counter);
    bot2.addDouble(actions.action_vector[j].time);
    bot3.addInt(actions.action_vector[j].counter);
    bot3.addDouble(actions.action_vector[j].time);
    bot4.addInt(actions.action_vector[j].counter);
    bot4.addDouble(actions.action_vector[j].time);
    for (int ix=0;ix<6;ix++)
    {
        bot2.addDouble(ll[ix]);
        bot3.addDouble(rl[ix]);
    }
    for (int ix=0; ix < 3; ix++)
    {
        bot4.addDouble(to[ix]);
    }
    this->port_command_joints_ll.setEnvelope(this->timestamp);
    this->port_command_joints_ll.write();
    this->port_command_joints_rl.setEnvelope(this->timestamp);
    this->port_command_joints_rl.write();
    this->port_command_joints_to.setEnvelope(this->timestamp);
    this->port_command_joints_to.write();
    
    //Send data for torqueBalancing
    Bottle& bot_com = this->port_command_com.prepare();
    Bottle& bot_postural = this->port_command_postural.prepare();
    Bottle& bot_constraints = this->port_command_constraints.prepare();
    bot_com.clear();
    bot_postural.clear();
    bot_constraints.clear();
    
    if ( !actions.action_vector_torqueBalancing.com_traj.empty() )
    {
        std::deque<double> tmpCom_i = actions.action_vector_torqueBalancing.com_traj.front();
        while ( !tmpCom_i.empty() )
        {
            bot_com.addDouble( tmpCom_i.front() );
            tmpCom_i.pop_front();
        }
        actions.action_vector_torqueBalancing.com_traj.pop_front();
        tmpCom_i.clear();
    }
    
    if ( !actions.action_vector_torqueBalancing.postural_traj.empty() )
    {
        std::deque<double> tmpPostural_i = actions.action_vector_torqueBalancing.postural_traj.front();
        while ( !tmpPostural_i.empty() )
        {
            bot_postural.addDouble( tmpPostural_i.front() );
            tmpPostural_i.pop_front();
        }
        actions.action_vector_torqueBalancing.postural_traj.pop_front();
        tmpPostural_i.clear();
    }
    
    if ( !actions.action_vector_torqueBalancing.constraints.empty() )
    {
        std::deque<int> tmpConstraints_i = actions.action_vector_torqueBalancing.constraints.front();
        while ( !tmpConstraints_i.empty() )
        {
            bot_constraints.addInt( tmpConstraints_i.front() );
            tmpConstraints_i.pop_front();
        }
        actions.action_vector_torqueBalancing.constraints.pop_front();
        tmpConstraints_i.clear();
    }
    
    this->port_command_com.setEnvelope(this->timestamp);
    this->port_command_com.write();
    this->port_command_postural.setEnvelope(this->timestamp);
    this->port_command_postural.write();
    this->port_command_constraints.setEnvelope(this->timestamp);
    this->port_command_constraints.write();

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
                //cout << "Current action: " << actions.current_action << endl;
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
