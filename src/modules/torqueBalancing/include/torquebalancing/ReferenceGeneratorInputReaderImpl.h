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
#include <wbi/wbiUtil.h>
#include <Eigen/Core>
#include <string>

namespace wbi {
    class wholeBodyInterface;
    class Frame;
}

namespace codyco {
    namespace torquebalancing {
        
        /** @class Implementation of ReferenceGeneratorInputReader to read the position of a generic endeffector
         * of the robot.
         *
         * It handles a 7-dimension vector representing the homogenous transformation of the endeffector position w.r.t. the world frame. The rotational component is expressed as angle-axis.
         *
         * @note this class is not thread safe: avoid cuncurrent calls to its methods.
         */
        class EndEffectorPositionReader : public ReferenceGeneratorInputReader {
        private:
            wbi::wholeBodyInterface& m_robot;
            wbi::Frame m_world2BaseFrame;
            wbi::Frame m_leftFootToBaseRotationFrame;
            
            const std::string& m_endEffectorLinkName;
            int m_endEffectorLinkID;
            int m_leftFootLinkID; /*!< this is temporary to allow robot localization */
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;
            
            Eigen::MatrixXd m_jacobian;
            
            void updateStatus();
        public:
            EndEffectorPositionReader(wbi::wholeBodyInterface& robot, std::string endEffectorLinkName);
            virtual ~EndEffectorPositionReader();
            virtual bool init();
            virtual const Eigen::VectorXd& getSignal();
            virtual const Eigen::VectorXd& getSignalDerivative();
            virtual int signalSize() const;
        };
        
        
        /** @class Implementation of ReferenceGeneratorInputReader to read the Center Of Mass position
         * of the robot.
         *
         * It handles a 3-dimension vector representing the x,y,z coordinates of the center of mass of the robot.
         *
         * @note this class is not thread safe: avoid cuncurrent calls to its methods.
         */
        class COMReader : public EndEffectorPositionReader {
        private:

            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;
//            void updateStatus();
        public:
            COMReader(wbi::wholeBodyInterface& robot);
            
            virtual ~COMReader();
            virtual const Eigen::VectorXd& getSignal();
            virtual const Eigen::VectorXd& getSignalDerivative();
            virtual int signalSize() const;
            
        };

        /** @class Implementation of ReferenceGeneratorInputReader to read the forces acting on the 
         * endeffector of the robot.
         *
         * It handles a 6-dimension vector representing the forces and torques acting on the hand.
         *
         * @note this class is not thread safe: avoid cuncurrent calls to its methods.
         */
        class EndEffectorForceReader : public ReferenceGeneratorInputReader {
        private:
            wbi::wholeBodyInterface& m_robot;
            wbi::Frame m_world2BaseFrame;
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;
            
            void updateStatus();
        public:
            EndEffectorForceReader(wbi::wholeBodyInterface& robot);
            
            virtual ~EndEffectorForceReader();
            virtual const Eigen::VectorXd& getSignal();
            virtual const Eigen::VectorXd& getSignalDerivative();
            virtual int signalSize() const;
            
        };
    }
}


#endif /* end of include guard: REFERENCEGENERATORINPUTREADERIMPL_H */
