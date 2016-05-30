#ifndef INVERSEKINEMATICSIMPLEMENTATION_H
#define INVERSEKINEMATICSIMPLEMENTATION_H

#include <iDynTree/HighLevel/DynamicsComputations.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <vector>
#include <map>
#include <IpIpoptApplication.hpp>

#include "InverseKinematics.h"

namespace kinematics {
    class InverseKinematicsData;
    class Transform;
    typedef std::map<int, kinematics::Transform> TransformMap; //ordered map. Order is important

    class InverseKinematicsNLP;
}

class kinematics::InverseKinematicsData {

//    InverseKinematicsData(const InverseKinematicsData&);
//    InverseKinematicsData& operator=(const InverseKinematicsData&);

    iDynTree::HighLevel::DynamicsComputations m_dynamics;

    //Joint configuration. Size dofs
    iDynTree::VectorDynSize m_jointsConfiguration;

    //Number of Dofs in the model
    unsigned m_dofs;

    enum InverseKinematicsRotationParametrization m_rotationParametrization;

    //Joint - variables mapping. By default they match the Dofs
    std::vector<int> m_variablesToJointsMapping;

    //Joint limit (size dofs)
    iDynTree::VectorDynSize m_lowerJointLimit;
    iDynTree::VectorDynSize m_upperJointLimit;

    //Constraints
    TransformMap m_constraints;
    TransformMap m_targets;

    //Preferred joints configuration for the optimization
    //Size #size of optimization variables
    iDynTree::VectorDynSize m_preferredJointsConfiguration;

    //Optimization buffers & variables
    //TODO: move buffer in NLP variables
    //This class should contain only "shared" data
    iDynTree::VectorDynSize m_optimizedRobotDofs;
    iDynTree::Position m_optimizedBasePosition;
    iDynTree::Vector4 m_optimizedBaseOrientation;
    iDynTree::SpatialAcc m_worldGravity;
    

public:
    InverseKinematicsData();
    
    bool setupFromURDFModelWithFilePath(std::string urdfFilePath);
    bool setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping);

    /**
     * Reset variables to defaults
     *
     * If the model has been loaded, defaults means #dofs size
     * Otherwise the size is zero.
     * All constraints are reset.
     */
    void clearProblem();

    bool addFrameConstraint(const kinematics::Transform& frameTransform);
    bool addTarget(const kinematics::Transform& frameTransform, double weight = 1);

    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);
    bool setRobotConfiguration(const iDynTree::VectorDynSize& robotConfiguration);
    bool setInitialCondition(const iDynTree::VectorDynSize& initialCondition);

    void setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization);
    enum InverseKinematicsRotationParametrization rotationParametrization();

    friend class InverseKinematicsNLP;


    iDynTree::HighLevel::DynamicsComputations& dynamics();
    Ipopt::SmartPtr<Ipopt::IpoptApplication> solver;

};

#endif /* end of include guard: INVERSEKINEMATICSIMPLEMENTATION_H */
