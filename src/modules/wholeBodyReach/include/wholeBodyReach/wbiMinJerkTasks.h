/*
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef WBI_MIN_JERK_TASK
#define WBI_MIN_JERK_TASK

#include <Eigen/Core>               // import most common Eigen types
#include <yarp/sig/Vector.h>
#include <wholeBodyReach/wbiAbstractTasks.h>
#include <wholeBodyReach/minJerkTrajGen.h>
#include <wbi/wbiUtil.h>

namespace wholeBodyReach
{
    
#define DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS true
    
    /** An abstract task with a minimum-jerk trajectory generator.
     */
    class MinJerkTask : public paramHelp::ParamValueObserver
    {
    protected:
        MinJerkTrajGen              _trajGen;           /// trajectory generator
        double                      _trajDuration;      // trajectory duration
        int                         _paramId_trajDur;   // id of the parameter trajectory duration
        
    public:
        MinJerkTask(int size, double sampleTime=1e-3, double trajectoryDuration=1.0)
        :_trajGen(size,sampleTime,trajectoryDuration),
         _trajDuration(trajectoryDuration)
        {}
        
        virtual ~MinJerkTask(){}
        
        /** Link the proportional gain of this object to a parameter managed by the specified
         * instance of ParamHelperServer.
         */
        virtual void linkParameterTrajectoryDuration(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        /** Method called every time a parameter (for which a callback is registered)
         * is changed.
         */
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp);
        
        virtual bool setTrajectoryDuration(double trajectoryDuration)
        { return _trajGen.setTrajectoryDuration(trajectoryDuration); }
        
        virtual double getTrajectoryDuration()
        { return _trajGen.getTrajectoryDuration(); }
    };
    
    
    /** Task to control the pose (i.e. position and orientation) of
      * a link of the robot.
     */
    class MinJerkPDLinkPoseTask:    public WbiEqualityTask,
                                    public WbiPDTask,
                                    public MinJerkTask,
                                    public paramHelp::ParamValueObserver
    {
    protected:
        Eigen::VectorXd             _dJdq;      /// product of the Jacobian time derivative and the joint velocities
        Eigen::MatrixRXd            _J;         /// Jacobian
        wbi::Frame                  _H;         /// homogenous matrix from world frame to link frame
        Eigen::Vector7d             _x;         /// pose of the link in world frame in axis/angle notation
        Eigen::Vector6d             _v;         /// linear-angular velocity of the link frame
        
        int                         _linkId;    /// id of the link
        std::string                 _linkName;  /// name of the link
        bool                        _initSuccessfull;   /// true if initialization was successfull
        
        Eigen::VectorXd             _positionDesired;
        wbi::Frame                  _Hdesired;
        Eigen::Vector6d             _dvStar;    /// acceleration to use in optimization
        Eigen::Vector3d             _orientationError;  /// orientation error expressed as a rotation vector

    public:
        MinJerkPDLinkPoseTask(std::string taskName, std::string linkName, wbi::wholeBodyInterface* robot);
        virtual ~MinJerkPDLinkPoseTask(){}
        
        /** Link the desired pose of this task to a parameter managed by the specified
         * instance of ParamHelperServer.
         */
        virtual void linkParameterDesiredPose(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        /** Method called every time a parameter (for which a callback is registered) is changed. */
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp);

        virtual bool update(RobotState& state);
        
        virtual void    setDesiredPose(const wbi::Frame& poseDesired)
        { _Hdesired = poseDesired; }
    };
    
    
    /** Task to control the pose (i.e. position and orientation) of
     * a link of the robot.
     */
    class MinJerkPDMomentumTask:    public WbiEqualityTask,
                                    public WbiPDTask,
                                    public MinJerkTask
    {
    protected:
        Eigen::VectorXd             _dJdq;      /// product of the Jacobian time derivative and the joint velocities
        Eigen::MatrixRXd            _J;         /// Jacobian
        wbi::Frame                  _H;         /// homogenous matrix from world frame to link frame
        Eigen::Vector7d             _x;         /// pose of the link in world frame in axis/angle notation
        Eigen::Vector6d             _v;         /// linear-angular velocity of the link frame
        
        int                         _linkId;    /// id of the link
        std::string                 _linkName;  /// name of the link
        bool                        _initSuccessfull;   /// true if initialization was successfull
        
        Eigen::VectorXd             _positionDesired;
        Eigen::Vector3d             _comDesired;
        Eigen::Vector6d             _dvStar;    /// acceleration to use in optimization
        Eigen::Vector3d             _orientationError;  /// orientation error expressed as a rotation vector
        
    public:
        MinJerkPDMomentumTask(std::string taskName, wbi::wholeBodyInterface* robot);
        virtual ~MinJerkPDMomentumTask(){}
        
        virtual bool update(RobotState& state);
        
        virtual void    setDesiredCoM(Eigen::VectorConst comDesired)
        { _comDesired = comDesired; }
    };
    
    
    /** Task to control the posture (i.e. joint configuration)
      * of the robot.
     */
    class MinJerkPDPostureTask: public WbiEqualityTask,
                                public WbiPDTask,
                                public MinJerkTask
    {
    public:
        MinJerkPDPostureTask(std::string taskName, wbi::wholeBodyInterface* robot);
        virtual ~MinJerkPDPostureTask(){}
        
        virtual bool update(RobotState& state);
    };
    
    
    /** A constraint representing a rigid contact between one link of
      * the robot and the environment. The contact may constrain all
      * 6 directions of motion (linear + angular) or only the linear part.
      */
    class ContactConstraint:    public WbiEqualityTask,
                                public WbiInequalityTask
    {
    protected:
        std::string         _linkName;
        
    public:
        ContactConstraint(std::string name, std::string linkName, wbi::wholeBodyInterface* robot);
        virtual ~ContactConstraint() {}
        
        virtual bool update(RobotState& state);
    };


    /** Task to model the joint limits of the robot.
      * This task computes the bounds on the joint accelerations
      * so that the robot won't hit its joint limits.
      * It can also take into account joint velocity/acceleration 
      * bounds.
      */
    class JointLimitTask:   public WbiInequalityTask,
                            public WbiPDTask
    {
    protected:
        Eigen::VectorXd     _qMin;       /// Lower bound of joint positions
        Eigen::VectorXd     _qMax;       /// Upper bound of joint positions
        Eigen::VectorXd     _dqMin;      /// Lower bound of joint velocities
        Eigen::VectorXd     _dqMax;      /// Upper bound of joint velocities
        Eigen::VectorXd     _ddqMin;     /// Lower bound of joint accelerations
        Eigen::VectorXd     _ddqMax;     /// Upper bound of joint accelerations
        
        bool checkVectorSize(Eigen::VectorConst v)
        { return v.size()==this->_m; }
        
    public:
        JointLimitTask(std::string taskName, wbi::wholeBodyInterface* robot);
        virtual ~JointLimitTask(){}
        
        virtual bool update(RobotState& state);
        
        /** Set the joint position limits.
         * @return True if the operation succeeded, false otherwise.
         */
        virtual bool setPositionLimits(Eigen::VectorConst qMin, Eigen::VectorConst qMax);
        
        /** Set the joint velocity limits. 
          * @return True if the operation succeeded, false otherwise. 
         */
        virtual bool setVelocityLimits(Eigen::VectorConst dqMin, Eigen::VectorConst dqMax);
        
        /** Set the joint acceleration limits.
         * @return True if the operation succeeded, false otherwise.
         */
        virtual bool setAccelerationLimits(Eigen::VectorConst ddqMin, Eigen::VectorConst ddqMax);
        
    };
    
    
    /** Compute the 6d error vector given the measured frame and the desired frame. */
    void compute6DError(const wbi::Frame& H, const wbi::Frame& H_des, Eigen::VectorRef res);
    
    /** Compute the 3d orientation error given the measured orientation and the desired orientation. */
    void computeOrientationError(const wbi::Rotation& R, const wbi::Rotation& R_des, Eigen::VectorRef res);
    
    
} // end namespace wholeBodyReach

#endif
