#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include <string>
#include <vector>

namespace iDynTree {
    class VectorDynSize;
    class Transform;
    class Position;
    class Rotation;
}


namespace kinematics {
    class InverseKinematics;
    class InverseKinematicsData;

    enum InverseKinematicsRotationParametrization {
        InverseKinematicsRotationParametrizationQuaternion,
        InverseKinematicsRotationParametrizationRollPitchYaw,
    };

    inline unsigned sizeOfRotationParametrization(enum InverseKinematicsRotationParametrization rotationParametrization)
    {
        switch (rotationParametrization) {
            case InverseKinematicsRotationParametrizationQuaternion:
                return 4;
            case InverseKinematicsRotationParametrizationRollPitchYaw:
                return 3;
        }
    }
}

//TODO: how to handle conflicting requirements
class kinematics::InverseKinematics
{

public:
    /**
     * Default constructor
     */
    InverseKinematics();

    /**
     * Destructor
     */
    ~InverseKinematics();

    /**
     * Loads the kinematic model from the URDF file
     *
     * @param urdfFile path to the urdf file describing the model
     *
     * @return true if successful. False otherwise
     */
    bool loadModelFromURDFFileWithName(const std::string& urdfFile);

    /**
     * Reset the variables.
     * @note the model is not removed
     */
    void clearProblem();

    /**
     * Sets the robot configuration
     *
     *
     *
     * @param robotConfiguration the robot configuration
     *
     * @return true if successful, false otherwise.
     */
    bool setRobotConfiguration(const iDynTree::VectorDynSize& robotConfiguration);

    /**
     * Sets which joints are considered as optimization variables
     *
     * The map is
     * [index] => joint name
     * If the joint is not in the map, it is kept fixed
     * @note if not called all the joints are considered in the optimization procedure
     *
     * @param variableToDoFMapping structure describing which joints are optimized
     *
     * @return true if successful, false otherwise.
     */
    bool setOptimizationVariablesToJointsMapping(const std::vector<std::string> &variableToDoFMapping);


    void setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization);

    enum InverseKinematicsRotationParametrization rotationParametrization();

    /**
     * Adds a (constancy) constraint for the specified frame
     *
     * The constraint is
     * \f$ {}^w_X_{frame}(q) = {}^w_X_{frame}(q^0) \f$
     * where the robot configuration \f$q\f$ is the one specified with setRobotConfiguration
     * @param frameName the frame name
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameConstraint(const std::string& frameName);

    /**
     * Adds a (constancy) constraint for the specified frame
     *
     * The homogeneous trasformation of the specified frame w.r.t. the inertial frame
     * will remain constant and equal to the specified second parameter
     *
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the transform to associate to the constraint.
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue);

    /**
     * Adds a (constancy) position constraint for the specified frame
     *
     * Only the position component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the position associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFramePositionConstraint(const std::string& frameName, const iDynTree::Position& constraintValue);

    /**
     * Adds a (constancy) position constraint for the specified frame
     *
     * Only the position component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the position associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFramePositionConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue);

    /**
     * Adds a (constancy) orientation constraint for the specified frame
     *
     * Only the orientation component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the orientation associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameRotationConstraint(const std::string& frameName, const iDynTree::Rotation& constraintValue);

    /**
     * Adds a (constancy) orientation constraint for the specified frame
     *
     * Only the orientation component of the frame is constrained
     * @param frameName       the name of the frame on which to attach the constraint
     * @param constraintValue the orientation associated to the constraint
     *
     * @return true if successful, false otherwise.
     */
    bool addFrameRotationConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue);

    /**
     * Adds a target for the specified frame
     *
     * @param frameName       the name of the frame which represents the target
     * @param constraintValue value that the frame should reach
     *
     * @return true if successful, false otherwise.
     */
    bool addTarget(const std::string& frameName, const iDynTree::Transform& constraintValue);
    bool addPositionTarget(const std::string& frameName, const iDynTree::Position& constraintValue);
    bool addPositionTarget(const std::string& frameName, const iDynTree::Transform& constraintValue);
    bool addRotationTarget(const std::string& frameName, const iDynTree::Rotation& constraintValue);
    bool addRotationTarget(const std::string& frameName, const iDynTree::Transform& constraintValue);

    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);

    bool setInitialCondition(const iDynTree::VectorDynSize& initialCondition);

    bool solve();


private:
    
    InverseKinematicsData *m_pimpl;

};

#endif /* end of include guard: INVERSEKINEMATICS_H */
