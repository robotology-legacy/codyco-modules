/**
 * Copyright (C) 2014 CoDyCo
 * @author: Francesco Romano
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

#ifndef TORQUEBALANCINGCONTROLLER_H
#define TORQUEBALANCINGCONTROLLER_H

#include "config.h"
#include <yarp/os/RateThread.h>
#include <yarp/os/Mutex.h>
#include <wbi/wbiUtil.h>
#include <Eigen/Core>

namespace wbi {
    class wholeBodyInterface;
    class Frame;
}

namespace codyco {
    namespace torquebalancing {
        
        class ControllerReferences;
        
        /** @brief Represents the actual controller
         *
         */
        class TorqueBalancingController: public yarp::os::RateThread
        {
        public:
            TorqueBalancingController(int period, ControllerReferences& references, wbi::wholeBodyInterface& robot);
            virtual ~TorqueBalancingController();
            
#pragma mark - RateThread methods
            virtual bool threadInit();
            virtual void threadRelease();
            virtual void run();
            
#pragma mark - Getter and setter
            
            /** Returns the currently used gain in the centroidal momentum computation
             * @return the centroidal momentum gain
             */
            double centroidalMomentumGain();
            
            /** Sets the gain to be used in the centroidal momentum computation
             * @param centroidalMomentumGain the new centroidal momentum gain
             */
            void setCentroidalMomentumGain(double centroidalMomentumGain);
            
            /** Returns the gains used in the impedance control
             * @return the gains for the impedance control
             */
            const Eigen::VectorXd& impedanceGains();
            
            /** Sets the new gains to be used in the impedance control
             * @param impedanceGains the new gains for the impedance control
             */
            void setImpedanceGains(Eigen::VectorXd& impedanceGains);
            
            /** Sets the state of the controller.
             *
             * If the controller is already in the state requested as input the command is ignored.
             * @param isActive the new state of the controller
             */
            void setActiveState(bool isActive);
            
            /** Returns the current state of the controller
             * @return true if the controller is active. False otherwise.
             */
            bool isActiveState();
            
            /** Returns the currently used desired joints configuration
             *  for the impedance/postural task.
             * @return the currently used desired joints configuration
             */
            const Eigen::VectorXd& desiredJointsConfiguration();

            /** Sets the new desired joints configuration
             *  for the impedance/postural task.
             * @param desiredJointsConfiguration the new desired joints configuration
             */
            void setDesiredJointsConfiguration(Eigen::VectorXd& desiredJointsConfiguration);
            
#pragma mark - Monitorable variables
            
        private:
            void readReferences();
            bool updateRobotState();
            void computeFeetForces(const Eigen::Ref<Eigen::MatrixXd>& desiredCOMAcceleration, Eigen::Ref<Eigen::MatrixXd> desiredFeetForces);
            void computeTorques(const Eigen::Ref<Eigen::MatrixXd>& desiredFeetForces, Eigen::Ref<Eigen::MatrixXd> torques);
            void writeTorques();
            
            //return value should be optimized by compiler RVO
            void skewSymmentricMatrix(const Eigen::Ref<const Eigen::Vector3d>& vector, Eigen::Ref<Eigen::Matrix3d> skewSymmetricMatrix);

            wbi::wholeBodyInterface& m_robot;
            wbi::Frame m_world2BaseFrame;
            wbi::Frame m_leftFootToBaseRotationFrame;
            
            yarp::os::Mutex m_mutex;
            
            bool m_active;
            
            //configuration-time constants
            int m_leftFootLinkID;
            int m_rightFootLinkID;
            int m_centerOfMassLinkID;
            
            //References
            ControllerReferences& m_references;
            Eigen::VectorXd m_desiredJointsConfiguration; /*!< actuatedDOFs */
            Eigen::VectorXd m_internal_desiredJointsConfiguration; /*!< actuatedDOFs */
            
            //Gains
            double m_centroidalMomentumGain;
            double m_internal_centroidalMomentumGain;
            Eigen::VectorXd m_impedanceGains; /*!< actuatedDOFs */
            Eigen::VectorXd m_internal_impedanceGains; /*!< actuatedDOFs */

            //references
            Eigen::Vector3d m_desiredCOMAcceleration;
            Eigen::VectorXd m_desiredFeetForces; /*!< 12 */
            Eigen::VectorXd m_desiredCentroidalMomentum;  /*!< 6 */
            
            //state of the robot
            Eigen::VectorXd m_jointPositions;  /*!< totalDOFs */
            Eigen::VectorXd m_jointVelocities;  /*!< totalDOFs */
            Eigen::VectorXd m_torques; /*!< actuatedDOFs */
            Eigen::VectorXd m_baseVelocity; /*!< 6 */
            Eigen::Vector3d m_centerOfMassPosition;
            Eigen::VectorXd m_rightFootPosition; /*!< 7 */
            Eigen::VectorXd m_leftFootPosition; /*!< 7 */
            
            //Jacobians
            Eigen::MatrixXd m_feetJacobian; /*!< 12 x totalDOFs */
            Eigen::VectorXd m_feetDJacobianDq; /*!< 12 */
            
            //Kinematic and dynamic variables
            Eigen::MatrixXd m_massMatrix; /*!< totalDOFs x totalDOFs */
            Eigen::VectorXd m_generalizedBiasForces; /*!< totalDOFs */
            Eigen::VectorXd m_gravityBiasTorques; /*!< totalDOFs */
            Eigen::VectorXd m_centroidalMomentum; /*!< 6 */
            
            //variables used in computation.
            Eigen::MatrixXd m_pseudoInverseOfJcMInvSt; /*!< actuatedDOFs x 12 */
            Eigen::MatrixXd m_centroidalForceMatrix; /*!< 6 x 12 */
            Eigen::VectorXd m_gravityForce; /*!< 6 */
            Eigen::MatrixXd m_torquesSelector; /*!< totalDOFs x actuatedDOFs */
            
            //constant auxiliary variables
            double m_gravityUnitVector[3];
            Eigen::Matrix<double, 7, 1> m_rotoTranslationVector; /*!< 7 */
            Eigen::VectorXd m_jointsZeroVector; /*!< actuatedDOFs */
            Eigen::Matrix<double, 6, 1> m_esaZeroVector; /*!< 6 */
            
        };
    }
}

#endif /* end of include guard: TORQUEBALANCINGCONTROLLER_H */
