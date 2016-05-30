#include "InverseKinematics.h"
#include "InverseKinematicsData.h"
#include "Transform.h"
#include "InverseKinematicsNLP.h"

#include <iDynTree/Core/Transform.h>

//TEMP:Remove this line
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <iostream>

namespace kinematics {

    InverseKinematics::InverseKinematics()
    : m_pimpl(0)
    {
        m_pimpl = new InverseKinematicsData();
    }

    InverseKinematics::~InverseKinematics()
    {
        if (m_pimpl) {
            delete m_pimpl;
            m_pimpl = 0;
        }
    }

    bool InverseKinematics::loadModelFromURDFFileWithName(const std::string& urdfFile)
    {
        assert(m_pimpl);

        //First reset the problem
        m_pimpl->clearProblem();

        if (!m_pimpl->setupFromURDFModelWithFilePath(urdfFile))
            return false;

        return true;
    }

    void InverseKinematics::clearProblem()
    {
        assert(m_pimpl);
        m_pimpl->clearProblem();
    }

    bool InverseKinematics::setRobotConfiguration(const iDynTree::VectorDynSize& robotConfiguration)
    {
        assert(m_pimpl);
        return m_pimpl->setRobotConfiguration(robotConfiguration);
    }

    bool InverseKinematics::setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping)
    {
        assert(m_pimpl);
        return m_pimpl->setOptimizationVariablesToJointsMapping(variableToDoFMapping);
    }

    void InverseKinematics::setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization)
    {
        assert(m_pimpl);
        m_pimpl->setRotationParametrization(parametrization);
    }

    enum InverseKinematicsRotationParametrization InverseKinematics::rotationParametrization()
    {
        assert(m_pimpl);
        return m_pimpl->rotationParametrization();
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName)
    {
        assert(m_pimpl);
        iDynTree::Transform w_X_frame = m_pimpl->dynamics().getWorldTransform(frameName);
        return addFrameConstraint(frameName, w_X_frame);
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::transformConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::positionConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::positionConstraint(frameName, constraintValue.getPosition()));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::rotationConstraint(frameName, constraintValue));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addFrameConstraint(Transform::rotationConstraint(frameName, constraintValue.getRotation()));
    }

    bool InverseKinematics::addTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::transformConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::positionConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::positionConstraint(frameName,  constraintValue.getPosition()));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::rotationConstraint(frameName,  constraintValue));
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
        assert(m_pimpl);
        return m_pimpl->addTarget(Transform::rotationConstraint(frameName,  constraintValue.getRotation()));
    }

    bool InverseKinematics::setInitialCondition(const iDynTree::VectorDynSize& initialCondition)
    {
        assert(m_pimpl);
        return m_pimpl->setInitialCondition(initialCondition);
    }

    bool InverseKinematics::solve()
    {
        assert(m_pimpl);
        //TODO: add error output

        Ipopt::ApplicationReturnStatus solverStatus;

        if (Ipopt::IsNull(m_pimpl->solver)) {
            m_pimpl->solver = IpoptApplicationFactory();

            //TODO: set options
            m_pimpl->solver->Options()->SetStringValue("hessian_approximation", "limited-memory");
            m_pimpl->solver->Options()->SetIntegerValue("max_iter", 1);
#ifndef NDEBUG
            m_pimpl->solver->Options()->SetStringValue("derivative_test", "first-order");
#endif

            solverStatus = m_pimpl->solver->Initialize();
            if (solverStatus != Ipopt::Solve_Succeeded) {
                return false;
            }
        }

        //instantiate the IpOpt problem
        InverseKinematicsNLP *iKin = new InverseKinematicsNLP(*m_pimpl);
        //Do something (if necessary)
        Ipopt::SmartPtr<Ipopt::TNLP> problem(iKin);


        //Test derivatives
//        iDynTree::VectorDynSize point(m_pimpl->dynamics().getNrOfDegreesOfFreedom() + 6);
//        int index = m_pimpl->dynamics().getFrameIndex("base");
//
//        //First test: derivative in zero (and identity rotation)
//        for (int i = 0; i < 10; i++) {
//            point.zero();
//            iDynTree::Rotation R = iDynTree::Rotation::RotZ(M_PI/4);
//            R.getRPY(iDynTree::toEigen(point)(3), iDynTree::toEigen(point)(4), iDynTree::toEigen(point)(5));
//
//            //        iDynTree::toEigen(point).segment<4>(3).setRandom();
//            //        iDynTree::toEigen(point)(3) = std::abs(iDynTree::toEigen(point)(3));
//            //        iDynTree::toEigen(point).segment<4>(3).normalize();
//            //        = iDynTree::toEigen(R.asQuaternion());
//            iKin->testDerivatives(point, index, 1e-6, 1e-5, InverseKinematicsRotationParametrizationRollPitchYaw);
//        }

        //Second test: random
//        point.zero();
//        iDynTree::toEigen(point).setRandom();
//        iDynTree::toEigen(point).segment<4>(3).normalize();
//        iKin->testDerivatives(point, index, 0.5, 1e-5);


        // Ask Ipopt to solve the problem
        solverStatus = m_pimpl->solver->OptimizeTNLP(problem);

        if (solverStatus == Ipopt::Solve_Succeeded) {
            std::cout << "*** The problem solved!\n";
            return true;
        } else {
            return false;
        }
    }

}
