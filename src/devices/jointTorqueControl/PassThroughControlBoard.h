#ifndef CODYCO_PASS_THROUGHT_CONTROL_BOARD_H
#define CODYCO_PASS_THROUGHT_CONTROL_BOARD_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IOpenLoopControl.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>


namespace yarp {
    namespace dev {
        class PassThroughControlBoard;
    }
}

class yarp::dev::PassThroughControlBoard :  public DeviceDriver,
                                            public IEncodersTimed,
                                            public IPositionControl2,
                                            public IVelocityControl,
                                            public IControlMode2,
                                            public ITorqueControl,
                                            public IControlLimits2,
                                            public IInteractionMode
{
protected:
    yarp::dev::PolyDriver proxyDevice;
    yarp::dev::IEncodersTimed * proxyIEncodersTimed;
    yarp::dev::IPositionControl2 * proxyIPositionControl2;
    yarp::dev::IVelocityControl2 * proxyIVelocityControl2;
    yarp::dev::IControlMode2 * proxyIControlMode2;
    yarp::dev::ITorqueControl * proxyITorqueControl;
    yarp::dev::IOpenLoopControl * proxyIOpenLoopControl;
    yarp::dev::IControlLimits2  * proxyIControlLimits2;
    yarp::dev::IInteractionMode * proxyIInteractionMode;
    yarp::dev::IAxisInfo *        proxyIAxisInfo;

public:
    //CONSTRUCTOR
    PassThroughControlBoard();
    ~PassThroughControlBoard();

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ENCODERS
    virtual bool getEncoder(int j, double* v);
    virtual bool getEncoders(double* encs);
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double* vals);

    virtual bool getEncoderSpeed(int j, double* sp);
    virtual bool getEncoderSpeeds(double* spds);

    virtual bool getEncoderAcceleration(int j, double* spds);
    virtual bool getEncoderAccelerations(double* accs);

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double* encs, double* time);
    virtual bool getEncoderTimed(int j, double* encs, double* time);

    //POSITION CONTROL
    virtual bool stop(int j);
    virtual bool stop();
    virtual bool positionMove(int j, double ref);
    virtual bool getAxes(int* ax); // WORKS
    virtual bool positionMove(const double* refs);
    virtual bool setRefSpeed(int j, double sp);
    virtual bool getRefSpeed(int j, double* ref);
    virtual bool getRefSpeeds(double* spds);

    virtual bool relativeMove(int j, double delta);
    virtual bool relativeMove(const double* deltas);
    virtual bool checkMotionDone(int j, bool* flag);
    virtual bool checkMotionDone(bool* flag);
    virtual bool setPositionMode();

    //POSITION CONTROL 2
    virtual bool positionMove(const int n_joint, const int* joints, const double* refs);
    virtual bool relativeMove(const int n_joint, const int* joints, const double* deltas);
    virtual bool checkMotionDone(const int n_joint, const int* joints, bool* flags);
    virtual bool setRefSpeeds(const int n_joint, const int* joints, const double* spds);
    virtual bool setRefAccelerations(const int n_joint, const int* joints, const double* accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);

    virtual bool setRefSpeeds(const double *spds);

    virtual bool setRefAcceleration(int j, double acc);
    virtual bool setRefAccelerations(const double *accs);
    virtual bool getRefAcceleration(int j, double *acc);
    virtual bool getRefAccelerations(double *accs);

    //VELOCITY CONTROL 2
    virtual bool setVelocityMode();
    virtual bool velocityMove(int j, double sp);
    virtual bool velocityMove(const double *sp);
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);

    virtual bool setVelPid(int j, const yarp::dev::Pid &pid);
    virtual bool setVelPids(const yarp::dev::Pid *pids);
    virtual bool getVelPid(int j, yarp::dev::Pid *pid);
    virtual bool getVelPids(yarp::dev::Pid *pids);

    //CONTROL LIMITS 2
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double *min, double *max);
    virtual bool setVelLimits(int axis, double min, double max);

    //CONTROL MODE
    virtual bool setPositionMode(int j);
    virtual bool setVelocityMode(int j);
    virtual bool getControlMode(int j, int *mode);

    virtual bool setTorqueMode(int j);
    virtual bool getControlModes(int *modes);

    virtual bool setImpedancePositionMode(int j);
    virtual bool setImpedanceVelocityMode(int j);
    virtual bool setOpenLoopMode(int j);

    // CONTROL MODE 2
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModes(int *modes);

    //TORQUE CONTROL
    virtual bool setRefTorque(int j, double t);
    virtual bool setRefTorques(const double *t);
    virtual bool setTorqueMode();
    virtual bool getRefTorque(int j, double *t);
    virtual bool getRefTorques(double *t);
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);

    virtual bool getBemfParam(int j, double *bemf);
    virtual bool setBemfParam(int j, double bemf);
    virtual bool setTorquePid(int j, const Pid &pid);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);
    virtual bool setTorquePids(const Pid *pids);
    virtual bool setTorqueErrorLimit(int j, double limit);
    virtual bool setTorqueErrorLimits(const double *limits);
    virtual bool getTorqueError(int j, double *err);
    virtual bool getTorqueErrors(double *errs);
    virtual bool getTorquePidOutput(int j, double *out);
    virtual bool getTorquePidOutputs(double *outs);
    virtual bool getTorquePid(int j, Pid *pid);
    virtual bool getTorquePids(Pid *pids);
    virtual bool getTorqueErrorLimit(int j, double *limit);
    virtual bool getTorqueErrorLimits(double *limits);
    virtual bool resetTorquePid(int j);
    virtual bool disableTorquePid(int j);
    virtual bool enableTorquePid(int j);
    virtual bool setTorqueOffset(int j, double v);

    //OPEN LOOP CONTROL
    virtual bool setRefOutput(int j, double v);
    virtual bool setRefOutputs(const double *v);;
    virtual bool getRefOutput(int j, double *v);
    virtual bool getRefOutputs(double *v);
    virtual bool getOutput(int j, double *v);
    virtual bool getOutputs(double *v);
    virtual bool setOpenLoopMode();

    //INTERACTION MODE
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

    //AXIS INFO
    virtual bool getAxisName(int axis, yarp::os::ConstString& name);

};

#endif /* CODYCO_PASS_THROUGHT_CONTROL_BOARD_H */