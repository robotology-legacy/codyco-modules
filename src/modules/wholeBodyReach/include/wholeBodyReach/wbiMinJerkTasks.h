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
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <wholeBodyReach/wbiAbstractTasks.h>
#include <wholeBodyReach/minJerkTrajGen.h>
#include <wbi/wbiUtil.h>

namespace wholeBodyReach
{
    
#define DEFAULT_AUTOMATIC_CRITICALLY_DAMPED_GAINS true
    
    /** An abstract task with a minimum-jerk trajectory generator.
     */
    class MinJerkTask : public virtual paramHelp::ParamValueObserver
    {
        /* Use "virtual" inheritance to solve the diamond problem.
         * See http://www.deitel.com/articles/cplusplus_tutorials/20060225/virtualBaseClass/
         * for more information.
         */
        
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
      * a link of the robot. It uses a minimum-jerk trajectory generator
      * and a proportional-derivative control law to generate the desired
      * task-space accelerations.
     */
    class MinJerkPDLinkPoseTask:    public WbiAbstractTask,
                                    public WbiEqualityTask,
                                    public WbiPDTask,
                                    public MinJerkTask
    {
    protected:
        int                         _linkId;    /// id of the link
        std::string                 _linkName;  /// name of the link
        bool                        _initSuccessfull;   /// true if initialization was successfull
        
        Eigen::VectorXd             _dJdq;      /// product of the Jacobian time derivative and the joint velocities
        Eigen::MatrixRXd            _J;         /// Jacobian
        wbi::Frame                  _H;         /// homogenous matrix from world frame to link frame
        Eigen::Vector6d             _v;         /// linear-angular velocity of the link frame
        
        // RPC PARAMETERS
        Eigen::Vector7d             _pose;              /// measured position+orientation (axis/angle)
        Eigen::Vector7d             _poseDes;           /// desired position + orientation (axis/angle)
        int                         _paramId_poseDes;   /// id of the parameter associated to _poseDesired
        int                         _paramId_pose;      /// id of the parameter associated to _pose
        
        wbi::Frame                  _Hdes;              /// same as _poseDes but as homogeneous matrix
        Eigen::Vector6d             _dvStar;            /// acceleration to use in optimization
        Eigen::Vector3d             _orientationError;  /// orientation error expressed as a rotation vector

    public:
        MinJerkPDLinkPoseTask(std::string taskName, std::string linkName,
                              double sampleTime, wbi::wholeBodyInterface* robot);
        virtual ~MinJerkPDLinkPoseTask(){}
        
        /** Link the desired pose of this task to a parameter managed by the specified
         * instance of ParamHelperServer.
         */
        virtual void linkParameterPoseDes(paramHelp::ParamHelperServer* paramHelper, int paramId);
        virtual void linkParameterPose(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        /** Method called every time a parameter (for which a callback is registered) is changed. */
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp);

        virtual bool update(RobotState& state);
        
        virtual void init(RobotState& state);
    };
    
    
    /** Task to control the 6d momentum of the robot.
      * It computes the desired momentum given the desired position of the center of mass
      * and the desired angular momentum (assumed to be 0). It uses a minimum-jerk trajectory
      * generator and a PD control law.
      */
    class MinJerkPDMomentumTask:    public WbiAbstractTask,
                                    public WbiEqualityTask,
                                    public WbiPDTask,
                                    public MinJerkTask
    {
    protected:
        wbi::Frame                  _H;         /// homogenous matrix from world frame to CoM frame
        Eigen::Vector6d             _momentum;  /// 6d centroidal momentum
        double                      _robotMass; /// total mass of the robot
        
        std::string                 _linkName;  /// name of the link
        bool                        _initSuccessfull;   /// true if initialization was successfull
        
        // RPC PARAMETERS
        Eigen::Vector3d             _com;
        Eigen::Vector3d             _v;         /// CoM velocity
        Eigen::Vector3d             _comDes;
        Eigen::Vector3d             _comRef;
        
        // com vel estimation
        iCub::ctrl::AWPolyEstimator*        _comFilt;           // derivative filters for com velocities
        yarp::sig::Vector                  _com_yarp;          // com position
        yarp::sig::Vector                  _v_yarp;            // com velocity

        
    public:
        MinJerkPDMomentumTask(std::string taskName, double sampleTime, wbi::wholeBodyInterface* robot);
        virtual ~MinJerkPDMomentumTask(){}
        
        virtual bool update(RobotState& state);
        
        virtual void init(RobotState& state);
        
        /** Link the desired pose of this task to a parameter managed by the specified
         * instance of ParamHelperServer.
         */
        virtual void linkParameterComDes(paramHelp::ParamHelperServer* paramHelper, int paramId);
        virtual void linkParameterCom(paramHelp::ParamHelperServer* paramHelper, int paramId);
        virtual void linkParameterComRef(paramHelp::ParamHelperServer* paramHelper, int paramId);
        virtual void linkParameterComVel(paramHelp::ParamHelperServer* paramHelper, int paramId);
        virtual void linkParameterMomentum(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        /** Method called every time a parameter (for which a callback is registered) is changed. */
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp)
        {
            WbiPDTask::parameterUpdated(pp);
            MinJerkTask::parameterUpdated(pp);
        }
    };
    
    
    /** Task to control the posture (i.e. joint configuration)
      * of the robot.
     */
    class MinJerkPDPostureTask: public WbiAbstractTask,
                                public WbiEqualityTask,
                                public WbiPDTask,
                                public MinJerkTask
    {
    protected:
        Eigen::VectorXd         _qDes;          /// desired joint positions
        int                     _paramId_qDes;  /// id of the parameter associated to _qDes
        
    public:
        MinJerkPDPostureTask(std::string taskName, double sampleTime, wbi::wholeBodyInterface* robot);
        virtual ~MinJerkPDPostureTask(){}
        
        virtual bool update(RobotState& state);
        
        virtual void init(RobotState& state);
        
        /** Link the desired posture of this task to a parameter managed by the specified
         * instance of ParamHelperServer.
         */
        virtual void linkParameterPostureDes(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        /** Method called every time a parameter (for which a callback is registered) is changed. */
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp)
        {
            WbiPDTask::parameterUpdated(pp);
            MinJerkTask::parameterUpdated(pp);
        }
    };
    

    /** Task to model the joint limits of the robot.
      * This task computes the bounds on the joint accelerations
      * so that the robot won't hit its joint limits.
      * It can also take into account joint velocity/acceleration 
      * bounds.
      */
    class JointLimitTask:   public WbiAbstractTask,
                            public WbiInequalityTask
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
