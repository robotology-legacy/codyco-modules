#include "InverseKinematicsData.h"
#include "Transform.h"

#include <cassert>

namespace kinematics {

//    InverseKinematicsData::InverseKinematicsData(const InverseKinematicsData&) {}
//    InverseKinematicsData& InverseKinematicsData::operator=(const InverseKinematicsData&) { return *this; }

    InverseKinematicsData::InverseKinematicsData()
    : m_dofs(0)
    , m_rotationParametrization(InverseKinematicsRotationParametrizationQuaternion)
    , solver(NULL)
    {
        m_worldGravity.zero();
        m_worldGravity.setLinearVec3(iDynTree::LinearMotionVector3(0, 0, -9.81));
    }

    bool InverseKinematicsData::setupFromURDFModelWithFilePath(std::string urdfFilePath)
    {
        bool result = m_dynamics.loadRobotModelFromFile(urdfFilePath);
        if (!result || !m_dynamics.isValid()) {
            std::cerr << "[ERROR] Error loading URDF model from " << urdfFilePath;
            return false;
        }
        m_dofs = m_dynamics.getNrOfDegreesOfFreedom();

        clearProblem();

        return true;
    }

    void InverseKinematicsData::clearProblem()
    {
        //resize vectors
        m_jointsConfiguration.resize(m_dofs);
        m_jointsConfiguration.zero();

        //TODO: Fill limits from URDF
        m_lowerJointLimit.resize(m_dofs);
        m_upperJointLimit.resize(m_dofs);
        for (unsigned i = 0; i < m_dofs; ++i) {
            m_lowerJointLimit(i) = -2e+19;
            m_upperJointLimit(i) =  2e+19;
        }

        std::vector<std::string> emptyVector;
        this->setOptimizationVariablesToJointsMapping(emptyVector);

        m_constraints.clear();
        m_targets.clear();
    }

    bool InverseKinematicsData::setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping)
    {
        //reset base
        m_optimizedBasePosition.zero();
        if (m_rotationParametrization == InverseKinematicsRotationParametrizationQuaternion) {
            m_optimizedBaseOrientation = iDynTree::Rotation::Identity().asQuaternion();
        } else if (m_rotationParametrization == InverseKinematicsRotationParametrizationRollPitchYaw) {
            m_optimizedBaseOrientation.zero();
            iDynTree::Rotation::Identity().getRPY(m_optimizedBaseOrientation(0), m_optimizedBaseOrientation(1), m_optimizedBaseOrientation(2));
        }

        unsigned optimizationVariablesSize = m_dofs;

        if (variableToDoFMapping.empty()) {
            //simply remove any mapping
            m_variablesToJointsMapping.clear();
            m_variablesToJointsMapping.reserve(m_dofs);
            for (unsigned i = 0; i < m_dofs; ++i) {
                m_variablesToJointsMapping.push_back(i);
            }
        } else {
            m_variablesToJointsMapping.reserve(variableToDoFMapping.size());
            for (std::vector<std::string>::const_iterator it = variableToDoFMapping.begin();
                 it != variableToDoFMapping.end(); ++it) {
                int jointIndex = m_dynamics.getJointIndex(*it);
                if (jointIndex < 0) {
                    std::cerr << "[ERROR] Could not find joint " << *it << std::endl;
                    return false;
                }
                m_variablesToJointsMapping.push_back(jointIndex);
            }
            optimizationVariablesSize = variableToDoFMapping.size();
        }

        //resize optimization variable
        m_optimizedRobotDofs.resize(optimizationVariablesSize);
        m_optimizedRobotDofs.zero();
        m_preferredJointsConfiguration.resize(optimizationVariablesSize);
        m_preferredJointsConfiguration.zero();

        return true;
    }

    bool InverseKinematicsData::addFrameConstraint(const kinematics::Transform& frameTransform)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransform.getFrameName());
        if (frameIndex < 0)
            return false;

        //add the constraint to the set
        std::pair<TransformMap::iterator, bool> result = m_constraints.insert(TransformMap::value_type(frameIndex, frameTransform));
        return result.second;
    }

    bool InverseKinematicsData::addTarget(const kinematics::Transform& frameTransform, double weight)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameTransform.getFrameName());
        if (frameIndex < 0)
            return false;

        std::pair<TransformMap::iterator, bool> result = m_targets.insert(TransformMap::value_type(frameIndex, frameTransform));
        return result.second;
    }

    iDynTree::HighLevel::DynamicsComputations& InverseKinematicsData::dynamics() { return m_dynamics; }

    bool InverseKinematicsData::setInitialCondition(const iDynTree::VectorDynSize& initialCondition)
    {
        //TODO: same function to initialize the base?
//        assert(initialCondition.size() == m_optimizationVariable.size());
//        m_optimizationVariable = initialCondition;
        return true;
    }

    bool InverseKinematicsData::setRobotConfiguration(const iDynTree::VectorDynSize& robotConfiguration)
    {
        assert(m_jointsConfiguration.size() == robotConfiguration.size());
        m_jointsConfiguration = robotConfiguration;
        return true;
    }

    bool InverseKinematicsData::setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration)
    {
        assert(m_optimizedRobotDofs.size() == desiredJointConfiguration.size());
        m_preferredJointsConfiguration = desiredJointConfiguration;
        return true;
    }

    void InverseKinematicsData::setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization)
    {
        m_rotationParametrization = parametrization;
    }

    enum InverseKinematicsRotationParametrization InverseKinematicsData::rotationParametrization() { return m_rotationParametrization; }

}
