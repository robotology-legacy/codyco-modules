#ifndef SOLVERDATA_H
#define SOLVERDATA_H
#include <iDynTree/HighLevel/DynamicsComputations.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <Eigen/Core>
#include <vector>
#include <string>

typedef enum _FeetInContact {
    LEFT_FOOT_IN_CONTACT,
    RIGHT_FOOT_IN_CONTACT,
    BOTH_FEET_IN_CONTACT,
} FeetInContact;

typedef enum _SolverFinalStatus {
    SUCCESS,
    MAXITER,
    INFEASIBLE,
    ERROR
} SolverFinalStatus;


namespace yarp {
    namespace sig {
        class Vector;
    }
}

/**
 * Data divided in two parts:
 * model information (loaded once)
 * actual data (resetted every optimization run)
 */
class SolverData
{
public:
    SolverData();

    bool resetModelInformation(const std::string modelFile, const std::vector<std::string> &variableToDoFMapping);
    bool resetOptimizationData(const yarp::sig::Vector& desiredCoM, const yarp::sig::Vector& desiredJoints, std::string feetInContact);

private:
    //model information
    iDynTree::HighLevel::DynamicsComputations dynamics;
    iDynTree::VectorDynSize allJoints;

    std::vector<int> variableToDoFMapping;

    iDynTree::VectorDynSize dofsSizeZero;
    iDynTree::SpatialAcc world_gravity;
    int rightFootFrameID;
    int leftFootFrameID;
    unsigned int dofs;
    unsigned int optimVariableSize;
    unsigned int constraintsSize;

    //optimization-related variables
    Eigen::VectorXd qDes;
    Eigen::VectorXd comDes;
    FeetInContact feetInContact;
    iDynTree::Transform right_X_left;

    //Buffers
    Eigen::VectorXd qError;
    Eigen::MatrixXd hessian;
    iDynTree::MatrixDynSize comJacobian;


    //Solution
    SolverFinalStatus finalStatus;
    Eigen::VectorXd primalSolution;
    double optimum;
    Eigen::VectorXd constraintsMultipliers;
    Eigen::VectorXd constraintsValue;
    Eigen::VectorXd lowerBoundMultipliers;
    Eigen::VectorXd upperBoundMultipliers;


    friend class Solver;

};

#endif /* end of include guard: SOLVERDATA_H */
