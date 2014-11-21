#include "JointTorqueControl.h"
#include <yarp/os/Property.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <algorithm>
#include <math.h>
#include <cstring>

#include <yarp/os/Time.h>

namespace yarp {
namespace dev {

template <class T>
bool contains(std::vector<T> const &v, T const &x) {
    return ! (std::find(v.begin(), v.end(), x) == v.end());
}


void JointTorqueControl::startHijackingTorqueControl(int j)
{
    this->hijackingTorqueControl[j] = true;
}

void JointTorqueControl::stopHijackingTorqueControl(int j)
{
    this->hijackingTorqueControl[j] = false;
}


JointTorqueControl::JointTorqueControl():
                    PassThroughControlBoard(), RateThread(10)
{
}

JointTorqueControl::~JointTorqueControl()
{
}

bool checkVectorExistInConfiguration(yarp::os::Bottle & bot,
                                     const std::string & name,
                                     const int expected_vec_size)
{
    //std::cerr << " checkVectorExistInConfiguration(" << name
    //         << " , " << expected_vec_size << " )" << std::endl;
    //std::cerr << "bot.check(name) : " << bot.check(name) << std::endl;
    //std::cerr << "bot.find(name).isList(): " << bot.find(name).isList() << std::endl;
    //std::cerr << "bot.find(name).asList()->size() : " << bot.find(name).asList()->size()<< std::endl;

    return (bot.check(name) &&
            bot.find(name).isList() &&
            bot.find(name).asList()->size() == expected_vec_size );
}

bool JointTorqueControl::loadGains(yarp::os::Searchable& config)
{
    if( !config.check("TRQ_PIDS") )
    {
        yError("No TRQ_PIDS group find, initialization failed");
        return false;
    }

    yarp::os::Bottle & bot = config.findGroup("TRQ_PIDS");

    bool gains_ok = true;

    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"kff",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"kp",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"ki",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"maxPwm",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"maxInt",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"stictionUp",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"stictionDown",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"bemf",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"coulombVelThr",this->axes);


    if( !gains_ok )
    {
        yError("TRQ_PIDS group is missing some information, initialization failed");
        return false;
    }

    for(int j=0; j < this->axes; j++)
    {
        jointTorqueLoopGains[j].reset();
        motorParameters[j].reset();

        jointTorqueLoopGains[j].kp = bot.find("kp").asList()->get(j).asDouble();
        jointTorqueLoopGains[j].ki = bot.find("ki").asList()->get(j).asDouble();
        jointTorqueLoopGains[j].max_pwm = bot.find("maxPwm").asList()->get(j).asDouble();
        jointTorqueLoopGains[j].max_int = bot.find("maxInt").asList()->get(j).asDouble();
        motorParameters[j].kff            = bot.find("kff").asList()->get(j).asDouble();
        motorParameters[j].kcp            = bot.find("stictionUp").asList()->get(j).asDouble();
        motorParameters[j].kcn            = bot.find("stictionDown").asList()->get(j).asDouble();
        motorParameters[j].kv             = bot.find("bemf").asList()->get(j).asDouble();
        motorParameters[j].coulombVelThr  = bot.find("coulombVelThr").asList()->get(j).asDouble();
    }

    return true;

}

bool JointTorqueControl::open(yarp::os::Searchable& config)
{
    //Workaround: writeStrict option is not documented but
    //            is necessary for getting correctly working
    //            single joint streaming, hardcoding it to on
    yarp::os::Property pass_through_controlboard_config;
    pass_through_controlboard_config.fromString(config.toString());
    pass_through_controlboard_config.put("writeStrict","on");
    PassThroughControlBoard::open(pass_through_controlboard_config);
    this->getAxes(&axes);
    hijackingTorqueControl.assign(axes,false);
    controlModesBuffer.resize(axes);
    motorParameters.resize(axes);
    jointTorqueLoopGains.resize(axes);
    measuredJointPositions.resize(axes,0.0);
    measuredJointVelocities.resize(axes,0.0);
    desiredJointTorques.resize(axes,0.0);
    measuredJointTorques.resize(axes,0.0);
    measuredJointPositionsTimestamps.resize(axes,0.0);
    jointControlOutput.resize(axes,0.0);
    jointTorquesError.resize(axes,0.0);
    oldJointTorquesError.resize(axes,0.0);
    derivativeJointTorquesError.resize(axes,0.0);
    integralJointTorquesError.resize(axes,0.0);
    integralState.resize(axes,0.0);
    jointControlOutput.resize(axes,0.0);

    //Start control thread
    this->setRate(config.check("controlPeriod",0.01,"update period of the torque control thread").asDouble());

    //Load Gains configurations
    bool ret = this->loadGains(config);

    if( ret )
    {
        ret = ret && this->start();
    }

    return ret;
}

bool JointTorqueControl::close()
{
    this->RateThread::stop();
    PassThroughControlBoard::close();
}

//CONTROL MODE
bool JointTorqueControl::setPositionMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_POSITION);
}

bool JointTorqueControl::setVelocityMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_VELOCITY);
}


bool JointTorqueControl::getControlMode(int j, int *mode)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    if( hijackingTorqueControl[j] )
    {
        //The hijacked jointshould be in openloop mode,
        //if not this means someone changed the hijacked joint
        //control mode of the proxy (for example a robotMotorGui
        //acting on the hijacked control board) and so we should
        //stop hijacking
        int proxy_mode;
        bool ret = proxyIControlMode2->getControlMode(j,&proxy_mode);
        if( !ret )
        {
            return false;
        }
        if( proxy_mode != VOCAB_CM_OPENLOOP )
        {
            this->stopHijackingTorqueControl(j);
            *mode = proxy_mode;
        }
        else
        {
            *mode = VOCAB_CM_TORQUE;
        }
        return true;
    }
    else
    {
        return proxyIControlMode2->getControlMode(j,mode);
    }
}

bool JointTorqueControl::setTorqueMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_TORQUE);
}

bool JointTorqueControl::getControlModes(int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    bool ret = proxyIControlMode2->getControlModes(modes);
    for(int j=0; j < this->axes; j++ )
    {
        if( hijackingTorqueControl[j] )
        {
            if( modes[j] == VOCAB_CM_OPENLOOP )
            {
                modes[j] = VOCAB_CM_TORQUE;
            }
            else
            {
                //joint j is not anymore in openloop contorl mode, stop hijacking
                this->stopHijackingTorqueControl(j);
            }
        }
    }
    return ret;
}

bool JointTorqueControl::setOpenLoopMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_OPENLOOP);
}

bool JointTorqueControl::setImpedancePositionMode(int j)
{
    return false;
}

bool JointTorqueControl::setImpedanceVelocityMode(int j)
{
    return false;
}

// CONTROL MODE 2
bool JointTorqueControl::getControlModes(const int n_joint, const int *joints, int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    bool ret = proxyIControlMode2->getControlModes(n_joint,joints,modes);

    for(int i=0; i < n_joint; i++ )
    {
        int j = joints[i];
        if( this->hijackingTorqueControl[j] )
        {
            if( modes[i] == VOCAB_CM_OPENLOOP )
            {
                modes[i] = VOCAB_CM_TORQUE;
            }
            else
            {
                //joint j is not anymore in openloop contorl mode, stop hijacking
                this->stopHijackingTorqueControl(j);
            }
        }
    }

    return ret;
}

bool JointTorqueControl::setControlMode(const int j, const int mode)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    int new_mode = mode;
    if( new_mode == VOCAB_CM_TORQUE )
    {
        this->startHijackingTorqueControl(j);
        new_mode = VOCAB_CM_OPENLOOP;
    }
    else
    {
        // The if is not really necessary
        // but it is for clearly state the semantics of
        // this operation (if a joint was in (hijacking) torque mode
        // and you switch to another mode stop hijacking
        // \todo TODO FIXME consider also the case if someone
        // else changes the control mode of a hijacked joint in the proxy
        // controlBoard: we should add a check in the update of the thread
        // that stop the hijacking if a joint control modes is not anymore
        // in openloop
        if( this->hijackingTorqueControl[j] )
        {
            this->stopHijackingTorqueControl(j);
        }
    }

    return proxyIControlMode2->setControlMode(j, new_mode);
}

bool JointTorqueControl::setControlModes(const int n_joint, const int *joints, int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    for(int i=0; i < n_joint; i++ )
    {
        int j = joints[i];
        if( modes[i] == VOCAB_CM_TORQUE )
        {
            this->startHijackingTorqueControl(j);
            modes[i] = VOCAB_CM_OPENLOOP;
        }
        else
        {
            if( this->hijackingTorqueControl[j] )
            {
                this->stopHijackingTorqueControl(j);
            }
        }
    }

    return proxyIControlMode2->setControlModes(n_joint,joints,modes);
}

bool JointTorqueControl::setControlModes(int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    for(int j=0; j < this->axes; j++ )
    {
        if( modes[j] == VOCAB_CM_TORQUE )
        {
            this->startHijackingTorqueControl(j);
            modes[j] = VOCAB_CM_OPENLOOP;
        }
        else
        {
            if( this->hijackingTorqueControl[j] )
            {
                this->stopHijackingTorqueControl(j);
            }
        }
    }

    return proxyIControlMode2->setControlModes(modes);
}



// Various methods of different interfaces related to control mode switching
bool JointTorqueControl::setTorqueMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_TORQUE);
    return this->setControlModes(controlModesBuffer.data());
}


bool JointTorqueControl::setPositionMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_POSITION);
    return this->setControlModes(controlModesBuffer.data());
}

bool JointTorqueControl::setVelocityMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_VELOCITY);
    return this->setControlModes(controlModesBuffer.data());
}

//to add if necessary: setVelocityMode, setPositionDirectMode

//TORQUE CONTROL
bool JointTorqueControl::setRefTorque(int j, double t)
{
    yarp::os::LockGuard(this->controlMutex);
    desiredJointTorques[j] = t;
    return true;
}

bool JointTorqueControl::setRefTorques(const double *t)
{
    yarp::os::LockGuard(this->controlMutex);
    ::memcpy(desiredJointTorques.data(),t,this->axes*sizeof(double));
    return true;
}


bool JointTorqueControl::getRefTorque(int j, double *t)
{
    yarp::os::LockGuard(this->controlMutex);
    *t = desiredJointTorques[j];
    return true;
}

bool JointTorqueControl::getRefTorques(double *t)
{
    yarp::os::LockGuard(this->controlMutex);
    memcpy(t,desiredJointTorques.data(),this->axes*sizeof(double));
    return true;
}

bool JointTorqueControl::getTorque(int j, double *t)
{
    yarp::os::LockGuard(this->controlMutex);
    *t = measuredJointTorques[j];
    return true;
}

bool JointTorqueControl::getTorques(double *t)
{
    yarp::os::LockGuard(this->controlMutex);
    memcpy(t,measuredJointTorques.data(),this->axes*sizeof(double));
    return true;
}

bool JointTorqueControl::getBemfParam(int j, double *bemf)
{
    yarp::os::LockGuard(this->controlMutex);
    *bemf = motorParameters[j].kv;
    return true;
}

bool JointTorqueControl::setBemfParam(int j, double bemf)
{
    yarp::os::LockGuard(this->controlMutex);
    motorParameters[j].kv = bemf;
    return true;
}

bool JointTorqueControl::setTorquePid(int j, const Pid &pid)
{
    //WARNING: the PID structure mixes up motor and joint information
    //WARNING: THIS COULD MAPPING COULD CHANGE AT ANY TIME
    yarp::os::LockGuard(this->controlMutex);
    // Joint level torque loop gains
    jointTorqueLoopGains[j].kp      = pid.kp;
    jointTorqueLoopGains[j].kd      = pid.kd;
    jointTorqueLoopGains[j].ki      = pid.ki;
    jointTorqueLoopGains[j].max_int = pid.max_int;

    // Motor level friction compensation parameters
    motorParameters[j].kcp = pid.stiction_up_val;
    motorParameters[j].kcn = pid.stiction_down_val;
    motorParameters[j].kff = pid.kff;

    return true;
}

bool JointTorqueControl::getTorqueRange(int j, double *min, double *max)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorqueRanges(double *min, double *max)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::setTorquePids(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j < this->axes; j++)
    {
        ret = ret && this->setTorquePid(j,pids[j]);
    }
    return ret;
}

bool JointTorqueControl::setTorqueErrorLimit(int j, double limit)
{
    //yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::setTorqueErrorLimits(const double *limits)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorqueError(int j, double *err)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorqueErrors(double *errs)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorquePidOutput(int j, double *out)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorquePidOutputs(double *outs)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorquePid(int j, Pid *pid)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorquePids(Pid *pids)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorqueErrorLimit(int j, double *limit)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::getTorqueErrorLimits(double *limits)
{
    yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::resetTorquePid(int j)
{
    //yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::disableTorquePid(int j)
{
    yWarning("JointTorqueControl::enableTorquePid not implemented");
    //yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::enableTorquePid(int j)
{
    //yarp::os::LockGuard(this->controlMutex);
    yWarning("JointTorqueControl::enableTorquePid not implemented");
    return false;
}

bool JointTorqueControl::setTorqueOffset(int j, double v)
{
    //yarp::os::LockGuard(this->controlMutex);
    yWarning("JointTorqueControl::setTorqueOffset not implemented");
    return false;
}

// HIJACKED CONTROL THREAD
bool JointTorqueControl::threadInit()
{
}

void JointTorqueControl::readStatus()
{
    bool enc_time;
    this->PassThroughControlBoard::getEncodersTimed(measuredJointPositions.data(),measuredJointPositionsTimestamps.data());
    this->PassThroughControlBoard::getEncoderSpeeds(measuredJointVelocities.data());
    this->PassThroughControlBoard::getTorques(measuredJointTorques.data());
}

/** Saturate the specified value between the specified bounds. */
inline double saturation(const double x, const double xMax, const double xMin)
{
    return x>xMax ? xMax : (x<xMin?xMin:x);
}

double JointTorqueControl::sign(double x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

void JointTorqueControl::threadRelease()
{
    yarp::os::LockGuard guard(controlMutex);
}


void JointTorqueControl::run()
{
    //Read status (position, velocity, torque) from the controlboard
    this->readStatus();

    //Compute joint level torque PID
    double dt = this->getRate();

    controlMutex.lock();
    for(int j=0; j < this->axes; j++ )
    {
        JointTorqueLoopGains &gains = jointTorqueLoopGains[j];
        jointTorquesError[j]  = measuredJointTorques[j] - desiredJointTorques[j];
        integralState[j]      = saturation(integralState[j] + gains.ki*dt*jointTorquesError(j),gains.max_int,-gains.max_int);
        jointControlOutput[j] = desiredJointTorques[j] - gains.kp*jointTorquesError[j] - integralState[j];
        /** note that we are explicitly removing the D part of the controller.
         *  To enable it uncomment the following two lines. */
        // derivativeJointTorquesError[j] = (jointTorquesError[j]-oldJointTorquesError[j])/(dt);
        // jointControlOutput[j] -= gains.kd*(derivativeJointTorquesError[j]);
    }


    // Evaluation of coulomb friction with smoothing close to zero velocity
    double coulombFriction;
    double coulombVelThre;
    for(int j=0; j < this->axes; j++ )
    {
        MotorParameters motorParam = motorParameters[j];
        coulombVelThre =  motorParam.coulombVelThr;
        //viscous friction compensation
        if (fabs(measuredJointVelocities[j])>coulombVelThre)
        {
            coulombFriction = sign(measuredJointVelocities[j]);
        }
        else
        {
            coulombFriction = pow(measuredJointVelocities[j]/coulombVelThre,3);
        }
        if (measuredJointVelocities[j] > 0 )
        {
            coulombFriction = motorParam.kcp*coulombFriction;
        }
        else
        {
            coulombFriction = motorParam.kcn*coulombFriction;
        }

        jointControlOutput[j] = motorParam.kff*jointControlOutput[j] + motorParam.kv*measuredJointVelocities[j] + coulombFriction ;
    }

    //Send resulting output
    if( !contains(hijackingTorqueControl,false) )
    {
        this->setRefOutputs(jointControlOutput.data());
    }
    else
    {
        for(int j=0; j < this->axes; j++)
        {
            if( hijackingTorqueControl[j] )
            {
                this->setRefOutput(j,jointControlOutput[j]);
            }
        }
    }

    controlMutex.unlock();
}


}
}