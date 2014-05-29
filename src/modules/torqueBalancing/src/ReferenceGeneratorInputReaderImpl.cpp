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
#include <codyco/Utils.h>

//this is temporary until a fix of @traversaro
//TODO: move methods to generic interface
#include <wbiIcub/wholeBodyInterfaceIcub.h>

namespace codyco {
    namespace torquebalancing {
        
#pragma mark - HandsPositionReader implementation
        EndEffectorPositionReader::EndEffectorPositionReader(wbi::wholeBodyInterface& robot, std::string endEffectorLinkName)
        : m_robot(robot)
        , m_jointsPosition(totalDOFs)
        , m_jointsVelocity(totalDOFs)
        , m_outputSignal(7)
        , m_outputSignalDerivative(7)
        , m_jacobian(7, totalDOFs)
        , m_previousContext(0)
        {
            m_robot.getLinkId(endEffectorLinkName.c_str(), m_endEffectorLinkID);
            initializer();
        }
        
        EndEffectorPositionReader::EndEffectorPositionReader(wbi::wholeBodyInterface& robot, int linkID)
        : m_robot(robot)
        , m_endEffectorLinkID(linkID)
        , m_jointsPosition(actuatedDOFs)
        , m_jointsVelocity(totalDOFs)
        , m_outputSignal(7)
        , m_outputSignalDerivative(7)
        , m_jacobian(7, totalDOFs)
        , m_previousContext(0)
        {
            initializer();
        }
        
        EndEffectorPositionReader::~EndEffectorPositionReader() {}
        
        void EndEffectorPositionReader::initializer()
        {
            m_robot.getLinkId("l_sole", m_leftFootLinkID);
            m_leftFootToBaseRotationFrame.R = wbi::Rotation(0, 0, 1,
                                                            0, -1, 0,
                                                            1, 0, 0);
        }
        
        void EndEffectorPositionReader::updateStatus(long context)
        {
            if (context != 0 && context == m_previousContext) return;
//             if (dynamic_cast<COMReader*>(this))
//                 std::cerr << "EndEffectorPositionReader::updateStatus\n";
            m_jointsVelocity.head(6).setZero();
            bool status;
            status = m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            if (!status) {
                std::cerr << FUNCTION_NAME << ": Error while reading positions\n";
            }
            status = status && m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.tail(actuatedDOFs).data());
            if (!status) {
                std::cerr << FUNCTION_NAME << ": Error while reading velocities\n";
            }
//             if (dynamic_cast<COMReader*>(this))
//                 std::cerr << "pos" << m_jointsPosition.transpose() << "\n";
            
            //update world to base frame
            status = status && m_robot.computeH(m_jointsPosition.data(), wbi::Frame(), m_leftFootLinkID, m_world2BaseFrame);
            if (!status) {
                std::cerr << FUNCTION_NAME << ": Error while computing homogenous transformation\n";
            }
            m_world2BaseFrame = m_world2BaseFrame * m_leftFootToBaseRotationFrame;
            m_world2BaseFrame.setToInverse();
            if (dynamic_cast<COMReader*>(this)) {
                
                std::cerr << Eigen::Map<Eigen::Vector3d>(m_world2BaseFrame.p).transpose() << "    " << Eigen::Map<Eigen::Matrix3d>(m_world2BaseFrame.R.data) << "\n";
            }
            

            m_jacobian.setZero();
            status = status && m_robot.forwardKinematics(m_jointsPosition.data(), m_world2BaseFrame, m_endEffectorLinkID, m_outputSignal.data());
            if (!status) {
                std::cerr << FUNCTION_NAME << ": Error while computing forward kinematic\n";
                m_outputSignal.setZero();
            }
            status = m_robot.computeJacobian(m_jointsPosition.data(), m_world2BaseFrame, m_endEffectorLinkID, m_jacobian.data());
            if (!status) {
                std::cerr << FUNCTION_NAME << ": Error while computing Jacobian\n";
                
            } else {
                m_outputSignalDerivative = m_jacobian * m_jointsVelocity;
            }
            m_previousContext = context;
        }
        
        const Eigen::VectorXd& EndEffectorPositionReader::getSignal(long context)
        {
            updateStatus(context);
            return m_outputSignal;
        }
        
        const Eigen::VectorXd& EndEffectorPositionReader::getSignalDerivative(long context)
        {
            updateStatus(context);
            return m_outputSignalDerivative;
        }
        
        int EndEffectorPositionReader::signalSize() const { return 7; }
        
#pragma mark - COMReader implementation
        COMReader::COMReader(wbi::wholeBodyInterface& robot)
        : EndEffectorPositionReader(robot, wbi::wholeBodyInterface::COM_LINK_ID)
        , m_outputCOM(3)
        , m_outputCOMVelocity(3) {}

        COMReader::~COMReader() {}
        
        const Eigen::VectorXd& COMReader::getSignal(long context)
        {
            m_outputCOM = EndEffectorPositionReader::getSignal(context).head(3);
            return m_outputCOM;
        }
        
        const Eigen::VectorXd& COMReader::getSignalDerivative(long context)
        {
            m_outputCOMVelocity = EndEffectorPositionReader::getSignalDerivative(context).head(3);
            return m_outputCOMVelocity;
        }
        
        int COMReader::signalSize() const { return 3; }
        
#pragma mark - HandsForceReader implementation
        EndEffectorForceReader::EndEffectorForceReader(wbi::wholeBodyInterface& robot,
                                                       std::string endEffectorLinkName)
        : m_robot(robot)
        , m_jointsPosition(totalDOFs)
        , m_jointsVelocity(totalDOFs)
        , m_outputSignal(6)
        , m_previousContext(0)
        {
            m_robot.getLinkId("l_sole", m_leftFootLinkID);
            m_leftFootToBaseRotationFrame.R = wbi::Rotation(0, 0, 1,
                                                            0, -1, 0,
                                                            1, 0, 0);
            int endEffectorGlobalID = -1;
            m_robot.getLinkId(endEffectorLinkName.c_str(), endEffectorGlobalID);
            m_endEffectorLocalID = m_robot.getJointList().globalToLocalId(endEffectorGlobalID);
            
        }
        
        EndEffectorForceReader::~EndEffectorForceReader() {}
        
        void EndEffectorForceReader::updateStatus(long context)
        {
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointsPosition.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointsVelocity.data());
            
            //update world to base frame
            m_robot.computeH(m_jointsPosition.data(), wbi::Frame(), m_leftFootLinkID, m_world2BaseFrame);
            m_world2BaseFrame = m_world2BaseFrame * m_leftFootToBaseRotationFrame;
            m_world2BaseFrame.setToInverse();
            
            //TODO: to be fixed
            ((wbiIcub::icubWholeBodyInterface&)m_robot).setWorldBasePosition(m_world2BaseFrame);

            m_robot.getEstimate(wbi::ESTIMATE_EXTERNAL_FORCE_TORQUE, m_endEffectorLocalID, m_outputSignal.data());
        }
        
        const Eigen::VectorXd& EndEffectorForceReader::getSignal(long context)
        {
            updateStatus(context);
            return m_outputSignal;
        }
        
        const Eigen::VectorXd& EndEffectorForceReader::getSignalDerivative(long context)
        {
            return Eigen::VectorXd::Zero(signalSize());
        }
        
        int EndEffectorForceReader::signalSize() const { return 6; }
        
#pragma mark - VoidReader implementation
        
        VoidReader::VoidReader(int size)
        : m_size(size)
        , m_voidVector(size) { m_voidVector.setZero(); }
        
        VoidReader::~VoidReader() {}
        
        const Eigen::VectorXd& VoidReader::getSignal(long/*context*/)
        {
            return m_voidVector;
        }
        const Eigen::VectorXd& VoidReader::getSignalDerivative(long/*context*/)
        {
            return m_voidVector;
        }
        
        int VoidReader::signalSize() const { return m_size; }
    }
}
