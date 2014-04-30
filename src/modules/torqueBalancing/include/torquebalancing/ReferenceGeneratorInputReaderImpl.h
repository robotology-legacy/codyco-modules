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

#ifndef REFERENCEGENERATORINPUTREADERIMPL_H
#define REFERENCEGENERATORINPUTREADERIMPL_H

#include "ReferenceGenerator.h"
#include <Eigen/Core>

namespace wbi {
    class wholeBodyInterface;
    class Frame;
}

namespace codyco {
    namespace torquebalancing {
        
        class COMReader : public ReferenceGeneratorInputReader {
        private:
            wbi::wholeBodyInterface& m_robot;
            const wbi::Frame& m_world2BaseFrame;
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_comPosition;
            Eigen::VectorXd m_comVelocity;
            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;

            Eigen::MatrixXd m_jacobian;
            
            void updateStatus();
        public:
            COMReader(wbi::wholeBodyInterface& robot, const wbi::Frame& world2BaseFrame);
            
            virtual ~COMReader();
            virtual Eigen::VectorXd& getSignal();
            virtual Eigen::VectorXd& getSignalDerivative();
            
        };
        
        class HandsPositionReader : public ReferenceGeneratorInputReader {
        private:
            wbi::wholeBodyInterface& m_robot;
            const wbi::Frame& m_world2BaseFrame;
            
            int m_leftHandLinkID;
            int m_rightHandLinkID;
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;
            
            Eigen::MatrixXd m_jacobian;
            
            void updateStatus();
        public:
            HandsPositionReader(wbi::wholeBodyInterface& robot, const wbi::Frame& world2BaseFrame);
            virtual ~HandsPositionReader();
            virtual Eigen::VectorXd& getSignal();
            virtual Eigen::VectorXd& getSignalDerivative();
        };
        
        class HandsForceReader : public ReferenceGeneratorInputReader {
        private:
            wbi::wholeBodyInterface& m_robot;
            const wbi::Frame& m_world2BaseFrame;
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;
            
            void updateStatus();
        public:
            HandsForceReader(wbi::wholeBodyInterface& robot, const wbi::Frame& world2BaseFrame);
            
            virtual ~HandsForceReader();
            virtual Eigen::VectorXd& getSignal();
            virtual Eigen::VectorXd& getSignalDerivative();
        };
    }
}


#endif /* end of include guard: REFERENCEGENERATORINPUTREADERIMPL_H */
