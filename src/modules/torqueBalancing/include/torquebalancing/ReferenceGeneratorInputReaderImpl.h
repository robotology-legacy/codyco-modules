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
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#endif
#include <Eigen/Core>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif#include <string>

namespace wbi {
    class wholeBodyInterface;
    class Frame;
    class LocalId;
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
            
            int m_endEffectorLinkID;
            int m_leftFootLinkID; /*!< this is temporary to allow robot localization */
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_outputSignal;
            Eigen::VectorXd m_outputSignalDerivative;
            
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_jacobian;
            
            void updateStatus();
            void initializer();
        public:
            EndEffectorPositionReader(wbi::wholeBodyInterface& robot, std::string endEffectorLinkName);
            EndEffectorPositionReader(wbi::wholeBodyInterface& robot, int linkID);
            virtual ~EndEffectorPositionReader();
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
            wbi::Frame m_leftFootToBaseRotationFrame;
            
            int m_leftFootLinkID; /*!< this is temporary to allow robot localization */
            wbi::LocalId m_endEffectorLocalID;
            
            Eigen::VectorXd m_jointsPosition;
            Eigen::VectorXd m_jointsVelocity;
            Eigen::VectorXd m_outputSignal;
            
            void updateStatus();
        public:
            EndEffectorForceReader(wbi::wholeBodyInterface& robot, std::string endEffectorLinkName);
            
            virtual ~EndEffectorForceReader();
            virtual const Eigen::VectorXd& getSignal();
            virtual const Eigen::VectorXd& getSignalDerivative();
            virtual int signalSize() const;
            
        };
        
        /** @class Void implementation of ReferenceGeneratorInputReader.
         *
         * Specify in the constructor the size of the vectors returned by objects of this class
         * It outputs a zero vector of size signalSize()
         */
        class VoidReader : public ReferenceGeneratorInputReader {
        private:
            const int m_size;
            Eigen::VectorXd m_voidVector;
        public:
            explicit VoidReader(int size);
            virtual ~VoidReader();
            virtual const Eigen::VectorXd& getSignal();
            virtual const Eigen::VectorXd& getSignalDerivative();
            virtual int signalSize() const;
        };

    }
}


#endif /* end of include guard: REFERENCEGENERATORINPUTREADERIMPL_H */
