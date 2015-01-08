#ifndef CODYCO_JOINT_TORQUE_CONTROL_H
#define CODYCO_JOINT_TORQUE_CONTROL_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/Mutex.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include <yarp/sig/Vector.h>

#include "PassThroughControlBoard.h"
#include <Eigen/Core>
#include <vector>

namespace yarp {
    namespace dev {
        class JointTorqueControl;
    }
}

//class JointTorqueControlLoop
/**
 * \note DO NOT USE. USING THIS MODULE WILL SERIOUSLY DAMAGE YOUR ROBOT.
 *
 * This device hijack the reference torque signal send to the control board to implement a higher level torque loop.
 * Is not intended for use in production, but for debug of the low level torque control.
 *
 * The robot's motors are controlled by using
 * PWM signals applied to the motors. Assuming that each motors affects the position of only one link
 * via rigid transmission mechanisms, the relationship between the link's torque \f$ \tau \f$ and the motor's duty cycle \f$ PWM \f$ is assumed to be:
\f[
    PWM  = {PWM}_{\text{control}} + k_v \dot{q} + k_c \mbox{sign}(\dot{q}),
\f]
with \f$k_{ff}\f$, \f$k_v\f$, \f$k_c\f$ three constants, and \f$\dot{q}\f$ the joint's velocity.
Since discontinuities may be challenging in practice, it is best to smooth the sign function.
For the sake of simplicity during the implementation process, we choose a pseudo sign function defined as follows:
\f[
        PWM  = {PWM}_{\text{control}} + k_v \dot{q} + k_c \tanh(k_s \dot{q}).
\f]

Then one has:
\f[
        PWM  = {PWM}_{\text{control}} + k_v \dot{q} + k_c \tanh(k_s \dot{q}).
\f]
This model can be improved by considering possible parameters' asymmetries with respect to the joint velocity \f$\dot{q}\f$.
In particular, the coulomb friction parameter  \f$k_c\f$ may depend on the sign of \f$\dot{q}\f$, and have different
values depending on this sign. Then, an improved model is:
\f[
    PWM  = {PWM}_{\text{control}} + k_{v} \dot{q} + [k_{cp} s(\dot{q}) + k_{cn} s(-\dot{q})] \tanh(k_s \dot{q}),
\f]
where the function \f$s(x)\f$ is the step function, i.e.
\f[
    s(x) =  1 \quad \mbox{if} \quad x >= 0; s(x)=0 \quad \mbox{if} \quad x < 0.
\f]
As stated, the above equation constitutes the relation between the tension applied to the motor and the link torque.
Then, to generate a desired torque \f$\tau_d\f$ coming from an higher control loop, it suffices to evaluate the above equation
with \f$\tau = \tau_d\f$. In practice, however, it is a best practice to add a lower loop to generate \f$\tau\f$ so that \f$\tau\f$
will converge to \f$\tau_d\f$, i.e:
\f[
    {PWM}_control = k_{ff} \tau_d - k_p e_{\tau} - k_i \int e_{\tau} \mbox{dt},
\f]
where \f$ e_{\tau} := \tau - \tau_d \f$.

\section intro_sec To do and warning list

a) Syncronization between aJ and taoD;
b) Anti wind-up and associated parameters;
c) Observer and a.p.;
d) Filtering parameters for velocity estimation and torque measurement;
*/

/**
 * Coupling matrices
 *
 */
struct CouplingMatrices
{
    Eigen::MatrixXd    fromJointTorquesToMotorTorques;
    Eigen::MatrixXd    fromMotorTorquesToJointTorques;
    Eigen::MatrixXd    fromJointVelocitiesToMotorVelocities;

    void reset(int NDOF)
    {
        fromJointTorquesToMotorTorques       = Eigen::MatrixXd::Identity(NDOF, NDOF);
        fromMotorTorquesToJointTorques       = Eigen::MatrixXd::Identity(NDOF, NDOF);
        fromJointVelocitiesToMotorVelocities = Eigen::MatrixXd::Identity(NDOF, NDOF);
        
    }
};


/**
 * Parameters for the motor level friction compensation
 *
 */
struct MotorParameters
{
    double kv;
    double kcp;
    double kcn;
    double coulombVelThr; ///<  joint vel (deg/s) at which Coulomb friction is completely compensate
    double kff;

    void reset()
    {
        kff = kv = kcp = kcn = 0.0;
        coulombVelThr = 0.0;
    }
};

/**
 * Gains for the joint level torque loop
 *
 */
struct JointTorqueLoopGains
{
    double kp;            ///<  proportional gain
    double ki;
    double kd;
    double max_int;
    double max_pwm;

    void reset()
    {
        kp = ki = kd = max_int = 0.0;
    }
};

class yarp::dev::JointTorqueControl :  public yarp::dev::PassThroughControlBoard,
                                       public yarp::os::RateThread
{
private:
    /**
     *  vector of getAxes() size.
     *  For each axis contains true if we are hijacking the torque control
     *  for this joint, or false otherwise.
     */
    std::vector<bool> hijackingTorqueControl;
    int axes;

    std::vector<int>  controlModesBuffer;

    void startHijackingTorqueControl(int j);
    void stopHijackingTorqueControl(int j);

    double sign(double j);

    CouplingMatrices couplingMatrices;
    CouplingMatrices couplingMatricesFirmware;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPort;
    
    //joint torque loop methods & attributes
    yarp::os::Mutex controlMutex; ///< mutex protecting control variables
    yarp::os::Mutex interfacesMutex; ///< mutex  protecting interfaces

    std::vector<JointTorqueLoopGains>                jointTorqueLoopGains;
    std::vector<MotorParameters> 		             motorParameters;
    yarp::sig::Vector                                desiredJointTorques;
    yarp::sig::Vector                                measuredJointTorques;
    yarp::sig::Vector                                measuredJointPositionsTimestamps;
    yarp::sig::Vector                                measuredJointPositions;
    yarp::sig::Vector                                measuredJointVelocities;
    yarp::sig::Vector                                measuredMotorVelocities;
    yarp::sig::Vector                                jointTorquesError;
    yarp::sig::Vector                                oldJointTorquesError;
    yarp::sig::Vector                                derivativeJointTorquesError;
    yarp::sig::Vector                                integralJointTorquesError;
    yarp::sig::Vector                                integralState;
    yarp::sig::Vector                                jointControlOutput;
    yarp::sig::Vector                                jointControlOutputBuffer;
    

    void readStatus();

    bool loadGains(yarp::os::Searchable& config);

    /**
     * Load the coupling matrices from the group whose name 
     *      is specified in group_name 
     *                             
     * 
     */
    bool loadCouplingMatrix(yarp::os::Searchable& config,
                            CouplingMatrices & coupling_matrices,
                            std::string group_name);

public:
    //CONSTRUCTOR
    JointTorqueControl();
    ~JointTorqueControl();

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ENCODERS
    /*
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
    */

    //CONTROL MODE related methods
    virtual bool setPositionMode();
    virtual bool setVelocityMode();
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

    //CONTROL THREAD
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

};

#endif /* CODYCO_JOINT_TORQUE_CONTROL_H */
