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

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything -Wdocumentation"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#endif
#include <Eigen/Core>
#include <Eigen/SVD>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

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
            TorqueBalancingController(int period, ControllerReferences& references, wbi::wholeBodyInterface& robot, int actuatedDOFs);
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
            
            /** Sets the torque saturation limits for the actuators
             * @param newSaturation new value for the saturation limit
             */
            void setTorqueSaturationLimit(Eigen::VectorXd& newSaturation);
            
            /** Returns the current torque saturation limits
             * @return the saturation limits
             */
            const Eigen::VectorXd& torqueSaturationLimit();
            
#pragma mark - Monitorable variables
            
            const Eigen::VectorXd& desiredFeetForces();
            
            const Eigen::VectorXd& outputTorques();
            
        private:
            void readReferences();
            bool updateRobotState();
            void computeContactForces(const Eigen::Ref<Eigen::MatrixXd>& desiredCOMAcceleration, Eigen::Ref<Eigen::MatrixXd> desiredContactForces);
            void computeTorques(const Eigen::Ref<Eigen::VectorXd>& desiredContactForces, Eigen::Ref<Eigen::MatrixXd> torques);
            void writeTorques();
            
            wbi::wholeBodyInterface& m_robot;
            int m_actuatedDOFs;
            wbi::Frame m_world2BaseFrame;
            wbi::Frame m_leftFootToBaseRotationFrame;
            
            yarp::os::Mutex m_mutex;
            
            bool m_active;
            
            //configuration-time constants
            int m_leftFootLinkID;
            int m_rightFootLinkID;
            int m_centerOfMassLinkID;
            int m_leftHandLinkID;
            int m_rightHandLinkID;
            
            //References
            ControllerReferences& m_references;
            Eigen::VectorXd m_desiredJointsConfiguration; /*!< actuatedDOFs */
            
            //Gains
            double m_centroidalMomentumGain;
            Eigen::VectorXd m_impedanceGains; /*!< actuatedDOFs */

            //references
            Eigen::Vector3d m_desiredCOMAcceleration;
            Eigen::VectorXd m_desiredFeetForces; /*!< 12 */
            Eigen::VectorXd m_desiredCentroidalMomentum;  /*!< 6 */
            Eigen::VectorXd m_desiredHandsForces; /*!< 12 */
            Eigen::VectorXd m_desiredContactForces; /*!< 18 (forces + torques for feet, only forces for hands) */
            
            bool m_leftHandForcesActive;
            bool m_rightHandForcesActive;
            
            //state of the robot
            Eigen::VectorXd m_jointPositions;  /*!< totalDOFs */
            Eigen::VectorXd m_jointVelocities;  /*!< totalDOFs */
            Eigen::VectorXd m_torques; /*!< actuatedDOFs */
            Eigen::VectorXd m_baseVelocity; /*!< 6 */
            Eigen::Vector3d m_centerOfMassPosition;
            Eigen::VectorXd m_rightFootPosition; /*!< 7 */
            Eigen::VectorXd m_leftFootPosition; /*!< 7 */
            
            Eigen::VectorXd m_torqueSaturationLimit; /* actuatedDOFs */
            
            //Jacobians
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_contactsJacobian; /*!< (6x2 + 3x2) x totalDOFs (forces and torques for feet, only forces for hands)*/
            Eigen::VectorXd m_contactsDJacobianDq; /*!< (6x2 + 3x2) */
            
            //Kinematic and dynamic variables
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_massMatrix; /*!< totalDOFs x totalDOFs */
            Eigen::VectorXd m_generalizedBiasForces; /*!< totalDOFs */
            Eigen::VectorXd m_gravityBiasTorques; /*!< totalDOFs */
            Eigen::VectorXd m_centroidalMomentum; /*!< 6 */
            
            //variables used in computation.
            Eigen::MatrixXd m_centroidalForceMatrix; /*!< 6 x 12 */
            Eigen::VectorXd m_gravityForce; /*!< 6 */
            Eigen::MatrixXd m_torquesSelector; /*!< totalDOFs x actuatedDOFs */
            //pseuo inverses
            Eigen::MatrixXd m_pseudoInverseOfJcMInvSt; /*!< actuatedDOFs x (6x2 + 3x2) */
            Eigen::MatrixXd m_pseudoInverseOfJcBase; /*!< 6 x 12 */
            Eigen::MatrixXd m_pseudoInverseOfCentroidalForceMatrix; /*!< 12 x 6 */
            Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject> m_svdDecompositionOfJcMInvSt; /*!< (6x2 + 3x2) x actuatedDOFs */
            Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject> m_svdDecompositionOfJcBase; /*!< 12 x 6 */
            Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject> m_svdDecompositionOfCentroidalForceMatrix; /*!< 6 x 12 */
            
            //constant auxiliary variables
            double m_gravityUnitVector[3];
            Eigen::Matrix<double, 7, 1> m_rotoTranslationVector; /*!< 7 */
            Eigen::VectorXd m_jointsZeroVector; /*!< actuatedDOFs */
            Eigen::Matrix<double, 6, 1> m_esaZeroVector; /*!< 6 */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_jacobianTemporary; /* 6 x totalDOFs */
            Eigen::VectorXd m_dJacobiaDqTemporary; /* 6 */
            
        };
    }
}

#endif /* end of include guard: TORQUEBALANCINGCONTROLLER_H */
