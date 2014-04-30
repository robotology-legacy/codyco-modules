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
        COMReader::COMReader(wbi::wholeBodyInterface& robot, const wbi::Frame& world2BaseFrame)
        : m_robot(robot)
        , m_world2BaseFrame(world2BaseFrame)
        , m_jointsPosition(TOTAL_DOFS)
        , m_jointsVelocity(TOTAL_DOFS)
        , m_comPosition(7)
        , m_comVelocity(7)
        , m_outputSignal(3)
        , m_outputSignalDerivative(3)
        , m_jacobian(7, TOTAL_DOFS) {}
        
        COMReader::~COMReader() {}
        
        void COMReader::updateStatus()
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, wbi::iWholeBodyModel::COM_LINK_ID, m_comPosition.data());
            m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, wbi::iWholeBodyModel::COM_LINK_ID, m_jacobian.data());
            
            m_outputSignal = m_comPosition.head(3);
            m_outputSignalDerivative = (m_jacobian * m_comVelocity).head(3);
        }
        
        Eigen::VectorXd& COMReader::getSignal()
        {
            //TODO: Optimize call to status. For example asking for a "context" variable in input If it is different from the last one update the status.
            updateStatus();
            return m_outputSignal;
        }
        
        Eigen::VectorXd& COMReader::getSignalDerivative()
        {
            updateStatus();
            return m_outputSignalDerivative;
        }
        
        int COMReader::signalSize() const { return 3; }
        
#pragma mark - HandsPositionReader implementation
        HandsPositionReader::HandsPositionReader(wbi::wholeBodyInterface& robot, const wbi::Frame& world2BaseFrame)
        : m_robot(robot)
        , m_world2BaseFrame(world2BaseFrame)
        , m_jointsPosition(TOTAL_DOFS)
        , m_jointsVelocity(TOTAL_DOFS)
        , m_outputSignal(14)
        , m_outputSignalDerivative(14)
        , m_jacobian(14, TOTAL_DOFS) {}
        
        HandsPositionReader::~HandsPositionReader() {}
        
        void HandsPositionReader::updateStatus()
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, m_leftHandLinkID, m_outputSignal.head(7).data());
            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, m_rightHandLinkID, m_outputSignal.tail(7).data());
            m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, m_leftHandLinkID, m_jacobian.topRows(7).data());
            m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, m_leftHandLinkID, m_jacobian.bottomRows(7).data());
        }
        
        Eigen::VectorXd& HandsPositionReader::getSignal()
        {
            updateStatus();
            return m_outputSignal;
        }
        
        Eigen::VectorXd& HandsPositionReader::getSignalDerivative()
        {
            updateStatus();
            return m_outputSignalDerivative;
        }
        
        int HandsPositionReader::signalSize() const { return 14; }
        
#pragma mark - HandsForceReader implementation
        HandsForceReader::HandsForceReader(wbi::wholeBodyInterface& robot, const wbi::Frame& world2BaseFrame)
        : m_robot(robot)
        , m_world2BaseFrame(world2BaseFrame)
        , m_jointsPosition(TOTAL_DOFS)
        , m_jointsVelocity(TOTAL_DOFS)
        , m_outputSignal(12)
        , m_outputSignalDerivative(12) {}
        
        HandsForceReader::~HandsForceReader() {}
        
        void HandsForceReader::updateStatus()
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
//            m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, wbi::iWholeBodyModel::COM_LINK_ID, m_comPosition.data());
//            
//            m_outputSignal = m_comPosition.head(3);
        }
        
        Eigen::VectorXd& HandsForceReader::getSignal()
        {
            updateStatus();
            return m_outputSignal;
        }
        
        Eigen::VectorXd& HandsForceReader::getSignalDerivative()
        {
            updateStatus();
            return m_outputSignalDerivative;
        }
        
        int HandsForceReader::signalSize() const { return 12; }
    }
}
