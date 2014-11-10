#include "JointTorqueControl.h"
#include <yarp/os/Property.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/Log.h>

#include <algorithm>

#include <cstdlib>
#include <boost/concept_check.hpp>

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

bool JointTorqueControl::open(yarp::os::Searchable& config)
{
    PassThroughControlBoard::open(config);
    this->getAxes(&axes);
    hijackingTorqueControl.assign(axes,false);
    controlModesBuffer.resize(axes);
    motorFrictionCompensationParameters.resize(axes);
    jointTorqueLoopGains.resize(axes);
    measuredJointPositions.resize(axes,0.0);
    measuredJointVelocities.resize(axes,0.0);
    desiredJointTorques.resize(axes,0.0);
    measuredJointTorques.resize(axes,0.0);
    measuredJointPositionsTimestamps.resize(axes,0.0);
    encoderPolyElement.data.resize(axes,0.0);
    outputJointTorques.resize(axes,0.0);
    jointTorquesError.resize(axes,0.0);
    oldJointTorquesError.resize(axes,0.0);
    derivativeJointTorquesError.resize(axes,0.0);
    integralJointTorquesError.resize(axes,0.0);
    integralState.resize(axes,0.0);
    outputJointTorques.resize(axes,0.0);

    //Start control thread
    this->setRate(config.check("controlPeriod",0.01,"update period of the torque control thread").asDouble());
    this->start();
}

bool JointTorqueControl::close()
{
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
        *mode = VOCAB_CM_TORQUE;
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
            modes[j] = VOCAB_CM_TORQUE;
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
            modes[i] = VOCAB_CM_TORQUE;
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
    }

    return proxyIControlMode2->setControlModes(modes);
}



// Various methods of different interfaces related to control mode switching
bool JointTorqueControl::setTorqueMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_TORQUE);
    return this->setControlModes(controlModesBuffer.data());
}

/*
bool JointTorqueControl::setPositionMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_POSITION);
    return this->setControlModes(controlModesBuffer.data());
}
*/
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
    memcpy(desiredJointTorques.data(),t,this->axes*sizeof(double));
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
    return false;
}

bool JointTorqueControl::getTorques(double *t)
{
    yarp::os::LockGuard(this->controlMutex);
    memcpy(t,measuredJointTorques.data(),this->axes*sizeof(double));
    return false;
}

bool JointTorqueControl::getBemfParam(int j, double *bemf)
{
    yarp::os::LockGuard(this->controlMutex);
    *bemf = motorFrictionCompensationParameters[j].kv;
    return false;
}

bool JointTorqueControl::setBemfParam(int j, double bemf)
{
    yarp::os::LockGuard(this->controlMutex);
    motorFrictionCompensationParameters[j].kv = bemf;
    return false;
}

bool JointTorqueControl::setTorquePid(int j, const Pid &pid)
{
    //WARNING: the PID structure mixes up motor and joint information
    //WARNING: THIS COULD MAPPING COULD CHANGE AT ANY TIME
    yarp::os::LockGuard(this->controlMutex);
    // Joint level torque loop gains
    jointTorqueLoopGains[j].kp = pid.kp;
    jointTorqueLoopGains[j].kd = pid.kd;
    jointTorqueLoopGains[j].ki = pid.ki;
    jointTorqueLoopGains[j].max_int = pid.max_int;

    // Motor level friction compensation parameters
    motorFrictionCompensationParameters[j].kff = pid.kff;
    motorFrictionCompensationParameters[j].kcp = pid.stiction_up_val;
    motorFrictionCompensationParameters[j].kcn = pid.stiction_down_val;

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
    velocityFilter = new iCub::ctrl::AWLinEstimator(5,1.0);

}

void JointTorqueControl::readStatus()
{
    bool enc_time;
    this->getEncodersTimed(measuredJointPositions.data(),measuredJointPositionsTimestamps.data());
    this->getTorques(measuredJointTorques.data());

    // \todo TODO FIXME how to deal with multiple timestamps? Do the mean ?
    encoderPolyElement.time = measuredJointPositionsTimestamps[0];
    memcpy(encoderPolyElement.data.data(),measuredJointPositions.data(),(this->axes)*sizeof(double));

    measuredJointVelocities = velocityFilter->estimate(encoderPolyElement);
}

/** Saturate the specified value between the specified bounds. */
inline double saturation(const double x, const double xMax, const double xMin)
{
    return x>xMax ? xMax : (x<xMin?xMin:x);
}

void JointTorqueControl::threadRelease()
{
    yarp::os::LockGuard guard(controlMutex);
    delete velocityFilter;
    velocityFilter = 0;
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
        JointTorqueLoopGains gains = jointTorqueLoopGains[j];
        jointTorquesError[j] = measuredJointTorques[j] - desiredJointTorques[j];
        derivativeJointTorquesError[j] = (jointTorquesError[j]-oldJointTorquesError[j])/(dt);
        integralState[j] = saturation(integralState[j] + gains.ki*dt*jointTorquesError(j),gains.max_int,-gains.max_int);
        //tau(i)              = tauD(i) - kp(i)*etau(i) - integralState(i) -kd(i)*Detau(i);
        outputJointTorques[j] = desiredJointTorques[j] - gains.kp*(jointTorquesError[j]) -integralState[j] -gains.kd*derivativeJointTorquesError[j];
    }

    //Compute friction compensation (missing for now)

    //Send resulting output
    if( !contains(hijackingTorqueControl,false) )
    {
        this->setRefOutputs(outputJointTorques.data());
    }
    else
    {
        for(int j=0; j < this->axes; j++)
        {
            if( hijackingTorqueControl[j] )
            {
                this->setRefOutput(j,outputJointTorques[j]);
            }
        }
    }

    controlMutex.unlock();
}


}
}