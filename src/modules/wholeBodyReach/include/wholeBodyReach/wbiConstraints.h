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

#ifndef WBI_CONSTRAINTS
#define WBI_CONSTRAINTS

#include <Eigen/Core>               // import most common Eigen types
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <wholeBodyReach/wbiAbstractTasks.h>
#include <wholeBodyReach/minJerkTrajGen.h>
#include <wbi/wbiUtil.h>

namespace wholeBodyReach
{
    /** A constraint representing a rigid contact between one link of
      * the robot and the environment. The contact may constrain all
      * 6 directions of motion (linear + angular) or only the linear part.
      */
    class ContactConstraint:    public WbiAbstractTask,
                                public WbiEqualityTask,
                                public WbiInequalityTask,
                                public paramHelp::ParamValueObserver
    {
    protected:
        std::string         _linkName;
        int                 _linkId;
        
        Eigen::MatrixRXd    _X;     /// momentum mapping matrix
        wbi::Frame          _Hcom;  /// homogeneous transformation from world to CoM
        wbi::Frame          _H;     /// homogeneous transformation from world to this link
        Eigen::Vector3d     _p_com; /// vector from CoM to contact link in world frame
        
        Eigen::VectorXd     _fDes;          /// desired constraint force computed by the solver
        Eigen::VectorXd     _fIneq;         /// 1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment
        double              _muF;           /// force friction coefficient
        int                 _paramId_muF;
        double              _fNormalMin;    /// max value for normal force
        double              _fNormalMax;    /// min value for normal force
        
        Eigen::Vector3d     _normalDir;     /// normal direction
        Eigen::Vector3d     _tangentDir1;   /// tangent direction 1
        Eigen::Vector3d     _tangentDir2;   /// tangent direction 2
        
        virtual void updateForceFrictionConeInequalities();
        
    public:
        ContactConstraint(std::string name, std::string linkName, int numberOfForces,
                          int numberOfInequalityConstraints, wbi::wholeBodyInterface* robot);
        
        virtual ~ContactConstraint() {}
        
        virtual bool update(RobotState& state) = 0;
        
        virtual void init(RobotState& state){}
        
        virtual void linkParameterForceFrictionCoefficient(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        virtual void linkParameterForceInequalities(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp);
        
        bool setNormalDirection(Eigen::Vector3d normalDir);
        
        virtual bool setForceFrictionCoefficient(double muF)
        {
            if(muF<=0.0) return false;
            _muF = muF;
            updateForceFrictionConeInequalities();
            return true;
        }
        
        virtual bool setMinNormalForce(double mnf)
        {
            if(mnf<0.0) return false;
            _fNormalMin = mnf;
            updateForceFrictionConeInequalities();
            return true;
        }
        
        virtual bool setMaxNormalForce(double mnf)
        {
            if(mnf<0.0) return false;
            _fNormalMax = mnf;
            updateForceFrictionConeInequalities();
            return true;
        }
        
        /** Get the matrix that maps this constraint forces into rate of change
          * of the momentum of the robot. 
          */
        virtual void getMomentumMapping(Eigen::MatrixRef X) const
        { X = _X; }
        
        /** Method used by the solver to let the Constraint know what the associated
         * constraint force is. */
        virtual bool setDesiredConstraintForce(Eigen::VectorConst fDes)=0;
    };
    
    
    
    /** A constraint representing a rigid contact between one link of
     * the robot and the environment. The contact may constrain all
     * 6 directions of motion (linear + angular) or only the linear part.
     */
    class PlaneContactConstraint:    public ContactConstraint
    {
    protected:
        ContactPlaneSize    _planeSize;     // sizes of the contact plane used to compute zmp limits
        
        double              _muM;           /// moment friction coefficient
        int                 _paramId_muM;
        
        virtual void updateMomentFrictionConeInequalities();
        virtual void updateZmpInequalities();
        
    public:
        PlaneContactConstraint(std::string name, std::string linkName,
                               const ContactPlaneSize &planeSize, wbi::wholeBodyInterface* robot);

        virtual ~PlaneContactConstraint() {}
        
        virtual bool update(RobotState& state);
        
        virtual void linkParameterMomentFrictionCoefficient(paramHelp::ParamHelperServer* paramHelper, int paramId);
        
        virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp);
        
        virtual bool setMomentFrictionCoefficient(double muM)
        {
            if(muM<=0.0) return false;
            _muM = muM;
            updateMomentFrictionConeInequalities();
            return true;
        }
        
        /** Method used by the solver to let the Constraint know what the associated
         * constraint force is. */
        virtual bool setDesiredConstraintForce(Eigen::VectorConst fDes);
    };
    
    
    
    /** A constraint representing a rigid contact between one link of
     * the robot and the environment. The contact may constrain only the
     * 3 linear directions of motion.
     */
    class PointContactConstraint:    public ContactConstraint
    {
    protected:
        Eigen::Vector3d        _p;     // contact point in link reference frame
        
        Eigen::MatrixRXd        _Jc;
        Eigen::Vector6d         _dJcdq;
        
    public:
        PointContactConstraint(std::string name, std::string linkName, wbi::wholeBodyInterface* robot);
        virtual ~PointContactConstraint() {}
        
        virtual bool update(RobotState& state);
        
        virtual bool setContactPoint(Eigen::VectorConst p)
        {
            if(p.size()!=3) return false;
            _p = p;
        }
        
        /** Method used by the solver to let the Constraint know what the associated
         * constraint force is. */
        virtual bool setDesiredConstraintForce(Eigen::VectorConst fDes);
    };
    
    
    Eigen::MatrixR3d crossProductMatrix(Eigen::VectorConst v);
    
    
} // end namespace wholeBodyReach

#endif
