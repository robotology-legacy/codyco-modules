#ifndef OPTIM_PROBLEM_H
#define OPTIM_PROBLEM_H

#include <IpTNLP.hpp>
#include <vector>

namespace yarp {
    namespace sig {
        class Vector;
    }
}

class SolverData;
/**
 * This class solves the following optimization problem:
 *
 *   minimize || q - qDes ||^2
 *      q
 *  
 * subject to
 *               com(q) = comDes
 * (if 2 feet in contact additional constraint)
 *               left_X_right = constant
 *
 *
 * Once you created an instance of this class, you should call (in order) the following two methods:
 * - initializeModel: it initialize the structure of the mechanical system. It needs the URDF description of the robot
 *                    and optionally a jointMapping structure specifying the correspondence between the variable index
 *                    i.e. the index in the variable qDes, and the name of the joint in the URDF
 * - solveOptimization: this actually perform the optimization procedure. It needs the desired com configuration,
 *                      together with the desired joints configuration and which foot is in contact
 */
class OptimProblem
{
    SolverData *pimpl;


public:

    OptimProblem();
    virtual ~OptimProblem();

    /**
     * Initializes the problem with information from the model
     *
     * @param modelFile full path to the URDF file
     * @param jointsMapping mapping information for the joints
     *
     * @return true if inizialization succeded. False otherwise.
     */
    bool initializeModel(const std::string modelFile, const std::vector<std::string>& jointsMapping = std::vector<std::string>());

    /**
     * Solve the optimization problem for the provided CoM, joints and feet configuration
     *
     * @param desiredCoM    desiredCoM to achieve (hard constraint)
     * @param desiredJoints desired Joints to achieve (soft constraint)
     * @param feetInContact configuration of the feet in contact
     *
     * @return true if the optimization succeded. False otherwise
     */
    bool solveOptimization(const yarp::sig::Vector& desiredCoM, const yarp::sig::Vector& desiredJoints, const std::string feetInContact);


};

#endif // OPTIM_PROBLEM_H
