#include "PassThroughControlBoard.h"
#include <yarp/os/Property.h>

namespace yarp {
namespace dev {

PassThroughControlBoard::PassThroughControlBoard():
                         proxyIEncodersTimed(0),
                         proxyIPositionControl2(0),
                         proxyIControlMode2(0),
                         proxyITorqueControl(0),
                         proxyIOpenLoopControl(0)
{
}

PassThroughControlBoard::~PassThroughControlBoard()
{
    this->close();
}

bool PassThroughControlBoard::open(yarp::os::Searchable& config)
{
    bool ok = true;
    ok = ok && config.check("proxy_remote");
    ok = ok && config.check("proxy_local");
    if(!ok)
    {
        return false;
    }

    ok = ok && config.find("proxy_remote").isString();
    ok = ok && config.find("proxy_local").isString();

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", config.find("proxy_local").asString()); //local port names
    options.put("remote", config.find("proxy_remote").asString()); //where we connect to


    proxyDevice.open(options);

    proxyDevice.view(proxyIEncodersTimed);
    proxyDevice.view(proxyIPositionControl2);
    proxyDevice.view(proxyIControlMode2);
    proxyDevice.view(proxyITorqueControl);
    proxyDevice.view(proxyIOpenLoopControl);

}

bool PassThroughControlBoard::close()
{
    proxyDevice.close();
}

bool PassThroughControlBoard::getEncoder(int j, double* v)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoder(j,v);
}

bool PassThroughControlBoard::getEncoders(double* encs)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoders(encs);
}

bool PassThroughControlBoard::resetEncoder(int j)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->resetEncoder(j);
}

bool PassThroughControlBoard::resetEncoders()
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->resetEncoders();
}

bool PassThroughControlBoard::setEncoder(int j, double val)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->setEncoder(j,val);
}

bool PassThroughControlBoard::setEncoders(const double* vals)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->setEncoders(vals);
}

bool PassThroughControlBoard::getEncoderSpeed(int j, double* sp)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderSpeed(j,sp);
}

bool PassThroughControlBoard::getEncoderSpeeds(double* spds)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderSpeeds(spds);
}

bool PassThroughControlBoard::getEncoderAcceleration(int j, double* spds)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderAcceleration(j,spds);
}

bool PassThroughControlBoard::getEncoderAccelerations(double* accs)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderAccelerations(accs);
}

    // ENCODERS TIMED
bool PassThroughControlBoard::getEncodersTimed(double* encs, double* time)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncodersTimed(encs,time);
}

bool PassThroughControlBoard::getEncoderTimed(int j, double* encs, double* time)
{
    if( !proxyIEncodersTimed )
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderTimed(j,encs,time);
}

        //POSITION CONTROL
bool PassThroughControlBoard::stop(int j)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->stop(j);
}

bool PassThroughControlBoard::stop()
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->stop();
}

bool PassThroughControlBoard::positionMove(int j, double ref)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->positionMove(j,ref);
}

bool PassThroughControlBoard::getAxes(int* ax)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getAxes(ax);
}

bool PassThroughControlBoard::positionMove(const double* refs)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->positionMove(refs);
}

bool PassThroughControlBoard::setRefSpeed(int j, double sp)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setRefSpeed(j,sp);
}

bool PassThroughControlBoard::getRefSpeed(int j, double* ref)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getRefSpeed(j,ref);
}

bool PassThroughControlBoard::getRefSpeeds(double* spds)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getRefSpeeds(spds);
}


bool PassThroughControlBoard::relativeMove(int j, double delta)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->relativeMove(j,delta);
}

bool PassThroughControlBoard::relativeMove(const double* deltas)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->relativeMove(deltas);
}

bool PassThroughControlBoard::checkMotionDone(int j, bool* flag)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->checkMotionDone(j,flag);
}

bool PassThroughControlBoard::checkMotionDone(bool* flag)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->checkMotionDone(flag);
}

bool PassThroughControlBoard::setPositionMode()
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setPositionMode();
}

//POSITION CONTROL 2
bool PassThroughControlBoard::positionMove(const int n_joint, const int* joints, const double* refs)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->positionMove(n_joint,joints,refs);
}

bool PassThroughControlBoard::relativeMove(const int n_joint, const int* joints, const double* deltas)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->relativeMove(n_joint,joints,deltas);
}

bool PassThroughControlBoard::checkMotionDone(const int n_joint, const int* joints, bool* flags)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->checkMotionDone(n_joint,joints,flags);
}

bool PassThroughControlBoard::setRefSpeeds(const int n_joint, const int* joints, const double* spds)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setRefSpeeds(n_joint,joints,spds);
}

bool PassThroughControlBoard::setRefAccelerations(const int n_joint, const int* joints, const double* accs)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setRefAccelerations(n_joint,joints,accs);
}

bool PassThroughControlBoard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getRefSpeeds(n_joint,joints,spds);
}

bool PassThroughControlBoard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getRefAccelerations(n_joint,joints,accs);
}

bool PassThroughControlBoard::stop(const int n_joint, const int *joints)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->stop(n_joint,joints);
}

bool PassThroughControlBoard::setRefSpeeds(const double *spds)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setRefSpeeds(spds);
}

bool PassThroughControlBoard::setRefAcceleration(int j, double acc)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setRefAcceleration(j,acc);
}

bool PassThroughControlBoard::setRefAccelerations(const double *accs)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->setRefAccelerations(accs);
}

bool PassThroughControlBoard::getRefAcceleration(int j, double *acc)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getRefAcceleration(j,acc);
}

bool PassThroughControlBoard::getRefAccelerations(double *accs)
{
    if( !proxyIPositionControl2 )
    {
        return false;
    }
    return proxyIPositionControl2->getRefAccelerations(accs);
}

//CONTROL MODE
bool PassThroughControlBoard::setPositionMode(int j)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setPositionMode(j);
}

bool PassThroughControlBoard::setVelocityMode(int j)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setVelocityMode(j);
}

bool PassThroughControlBoard::getControlMode(int j, int *mode)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->getControlMode(j,mode);
}

bool PassThroughControlBoard::setTorqueMode(int j)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setTorqueMode(j);
}

bool PassThroughControlBoard::getControlModes(int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->getControlModes(modes);
}

bool PassThroughControlBoard::setImpedancePositionMode(int j)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setImpedancePositionMode(j);
}

bool PassThroughControlBoard::setImpedanceVelocityMode(int j)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setImpedanceVelocityMode(j);
}

bool PassThroughControlBoard::setOpenLoopMode(int j)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setOpenLoopMode(j);
}

// CONTROL MODE 2
bool PassThroughControlBoard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->getControlModes(n_joint,joints,modes);
}

bool PassThroughControlBoard::setControlMode(const int j, const int mode)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setControlMode(j,mode);
}

bool PassThroughControlBoard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setControlModes(n_joint,joints,modes);
}

bool PassThroughControlBoard::setControlModes(int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }
    return proxyIControlMode2->setControlModes(modes);
}

//TORQUE CONTROL
bool PassThroughControlBoard::setRefTorque(int j, double t)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setRefTorque(j,t);
}

bool PassThroughControlBoard::setRefTorques(const double *t)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setRefTorques(t);
}

bool PassThroughControlBoard::setTorqueMode()
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setTorqueMode();
}

bool PassThroughControlBoard::getRefTorque(int j, double *t)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getRefTorque(j,t);
}

bool PassThroughControlBoard::getRefTorques(double *t)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getRefTorques(t);
}

bool PassThroughControlBoard::getTorque(int j, double *t)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorque(j,t);
}

bool PassThroughControlBoard::getTorques(double *t)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorques(t);
}

bool PassThroughControlBoard::getBemfParam(int j, double *bemf)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getBemfParam(j,bemf);
}

bool PassThroughControlBoard::setBemfParam(int j, double bemf)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setBemfParam(j,bemf);
}

bool PassThroughControlBoard::setTorquePid(int j, const Pid &pid)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setTorquePid(j,pid);
}

bool PassThroughControlBoard::getTorqueRange(int j, double *min, double *max)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorqueRange(j,min,max);
}

bool PassThroughControlBoard::getTorqueRanges(double *min, double *max)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorqueRanges(min,max);
}

bool PassThroughControlBoard::setTorquePids(const Pid *pids)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setTorquePids(pids);
}

bool PassThroughControlBoard::setTorqueErrorLimit(int j, double limit)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setTorqueErrorLimit(j,limit);
}

bool PassThroughControlBoard::setTorqueErrorLimits(const double *limits)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setTorqueErrorLimits(limits);
}

bool PassThroughControlBoard::getTorqueError(int j, double *err)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorqueError(j,err);
}

bool PassThroughControlBoard::getTorqueErrors(double *errs)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorqueErrors(errs);
}

bool PassThroughControlBoard::getTorquePidOutput(int j, double *out)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorquePidOutput(j,out);
}

bool PassThroughControlBoard::getTorquePidOutputs(double *outs)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorquePidOutputs(outs);
}

bool PassThroughControlBoard::getTorquePid(int j, Pid *pid)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorquePid(j,pid);
}

bool PassThroughControlBoard::getTorquePids(Pid *pids)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorquePids(pids);
}

bool PassThroughControlBoard::getTorqueErrorLimit(int j, double *limit)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorqueErrorLimit(j,limit);
}

bool PassThroughControlBoard::getTorqueErrorLimits(double *limits)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->getTorqueErrorLimits(limits);
}

bool PassThroughControlBoard::resetTorquePid(int j)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->resetTorquePid(j);
}

bool PassThroughControlBoard::disableTorquePid(int j)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->disableTorquePid(j);
}

bool PassThroughControlBoard::enableTorquePid(int j)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->enableTorquePid(j);
}

bool PassThroughControlBoard::setTorqueOffset(int j, double v)
{
    if( !proxyITorqueControl )
    {
        return false;
    }
    return proxyITorqueControl->setTorqueOffset(j,v);
}

bool PassThroughControlBoard::setOpenLoopMode()
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->setOpenLoopMode();
}

bool PassThroughControlBoard::setRefOutput(int j, double v)
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->setRefOutput(j,v);
}

bool PassThroughControlBoard::setRefOutputs(const double *v)
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->setRefOutputs(v);
}

bool PassThroughControlBoard::getRefOutput(int j, double *v)
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->getRefOutput(j,v);
}

bool PassThroughControlBoard::getRefOutputs(double *v)
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->getRefOutputs(v);
}

bool PassThroughControlBoard::getOutput(int j, double *v)
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->getOutput(j,v);
}

bool PassThroughControlBoard::getOutputs(double *v)
{
    if( !proxyIOpenLoopControl )
    {
        return false;
    }
    return proxyIOpenLoopControl->getOutputs(v);
}


}
}