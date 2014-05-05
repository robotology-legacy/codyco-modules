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

#include "ReferenceGeneratorInputReaderImpl.h"
#include "config.h"
#include <wbi/wholeBodyInterface.h>

namespace codyco {
    namespace torquebalancing {
        
#pragma mark - COMReader implementation
        COMReader::COMReader(wbi::wholeBodyInterface& robot)
        : m_robot(robot)
        , m_jointsPosition(totalDOFs)
        , m_jointsVelocity(totalDOFs)
        , m_comPosition(7)
        , m_comVelocity(7)
        , m_outputSignal(3)
        , m_outputSignalDerivative(3)
        , m_jacobian(7, totalDOFs)
        {
            m_robot.getLinkId("l_sole", m_leftFootLinkID);
            m_leftFootToBaseRotationFrame.R = wbi::Rotation(0, 0, 1,
                                                            0, -1, 0,
                                                            1, 0, 0);
        }
        
        COMReader::~COMReader() {}
        
        void COMReader::updateStatus()
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
            
            //update world to base frame
            m_robot.computeH(m_jointsPosition.data(), wbi::Frame(), m_leftFootLinkID, m_world2BaseFrame);
            m_world2BaseFrame = m_world2BaseFrame * m_leftFootToBaseRotationFrame;
            m_world2BaseFrame.setToInverse();

            
            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, wbi::iWholeBodyModel::COM_LINK_ID, m_comPosition.data());
            m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, wbi::iWholeBodyModel::COM_LINK_ID, m_jacobian.data());
            
            m_outputSignal = m_comPosition.head(3);
            m_outputSignalDerivative = (m_jacobian * m_comVelocity).head(3);
        }
        
        const Eigen::VectorXd& COMReader::getSignal()
        {
            //TODO: Optimize call to status. For example asking for a "context" variable in input If it is different from the last one update the status.
            updateStatus();
            return m_outputSignal;
        }
        
        const Eigen::VectorXd& COMReader::getSignalDerivative()
        {
            updateStatus();
            return m_outputSignalDerivative;
        }
        
        int COMReader::signalSize() const { return 3; }
        
#pragma mark - HandsPositionReader implementation
        HandsPositionReader::HandsPositionReader(wbi::wholeBodyInterface& robot)
        : m_robot(robot)
        , m_jointsPosition(totalDOFs)
        , m_jointsVelocity(totalDOFs)
        , m_outputSignal(14)
        , m_outputSignalDerivative(14)
        , m_jacobian(14, totalDOFs)
        {
            //TODO: this class can become in the future more generic: it can compute the position of a generic link
            m_robot.getLinkId("l_gripper", m_leftHandLinkID);
            m_robot.getLinkId("r_gripper", m_rightHandLinkID);
            m_robot.getLinkId("l_sole", m_leftFootLinkID);
            m_leftFootToBaseRotationFrame.R = wbi::Rotation(0, 0, 1,
                                                            0, -1, 0,
                                                            1, 0, 0);
        }
        
        HandsPositionReader::~HandsPositionReader() {}
        
        void HandsPositionReader::updateStatus()
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
            
            //update world to base frame
            m_robot.computeH(m_jointsPosition.data(), wbi::Frame(), m_leftFootLinkID, m_world2BaseFrame);
            m_world2BaseFrame = m_world2BaseFrame * m_leftFootToBaseRotationFrame;
            m_world2BaseFrame.setToInverse();

            
            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, m_leftHandLinkID, m_outputSignal.head(7).data());
            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, m_rightHandLinkID, m_outputSignal.tail(7).data());
            m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, m_leftHandLinkID, m_jacobian.topRows(7).data());
            m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, m_leftHandLinkID, m_jacobian.bottomRows(7).data());
        }
        
        const Eigen::VectorXd& HandsPositionReader::getSignal()
        {
            updateStatus();
            return m_outputSignal;
        }
        
        const Eigen::VectorXd& HandsPositionReader::getSignalDerivative()
        {
            updateStatus();
            return m_outputSignalDerivative;
        }
        
        int HandsPositionReader::signalSize() const { return 14; }
        
#pragma mark - HandsForceReader implementation
        HandsForceReader::HandsForceReader(wbi::wholeBodyInterface& robot)
        : m_robot(robot)
        , m_jointsPosition(totalDOFs)
        , m_jointsVelocity(totalDOFs)
        , m_outputSignal(12)
        , m_outputSignalDerivative(12) {}
        
        HandsForceReader::~HandsForceReader() {}
        
        void HandsForceReader::updateStatus()
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
            //TODO: read forces at end-effector
        }
        
        const Eigen::VectorXd& HandsForceReader::getSignal()
        {
            updateStatus();
            return m_outputSignal;
        }
        
        const Eigen::VectorXd& HandsForceReader::getSignalDerivative()
        {
            updateStatus();
            return m_outputSignalDerivative;
        }
        
        int HandsForceReader::signalSize() const { return 12; }
    }
}
