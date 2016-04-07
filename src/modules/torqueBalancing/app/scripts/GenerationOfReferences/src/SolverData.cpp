#include "SolverData.h"

#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>

SolverData::SolverData() {}

bool SolverData::resetModelInformation(const std::string modelFile, const std::vector<std::string> &_jointsMapping)
{
    //load URDF model for dynamics computations
    bool result = dynamics.loadRobotModelFromFile(modelFile);
    if (!result) {
        yError() << "Error loading URDF model from " << modelFile;
        return false;
    }

    dofs = dynamics.getNrOfDegreesOfFreedom();
    yInfo() << "Model loaded with " << dofs << " DoFs";

    //allocate memory
    allJoints.resize(dofs);
    dofsSizeZero.resize(dofs);
    allJoints.zero();
    dofsSizeZero.zero();

    comJacobian.resize(3, 6 + dofs);

    if (_jointsMapping.empty()) {
        //one to one mapping:
        //each joint is an optimization variable (and same index)
        variableToDoFMapping.clear();
        for (unsigned i = 0; i < dofs; i++) {
            variableToDoFMapping.push_back(i);
        }

    } else {
        //We have less optimization variables of joints (or order is not the same)
        //Mapping each optimization variable to the correspinding joint index in the model
        variableToDoFMapping.reserve(_jointsMapping.size());
        qDes.setZero(_jointsMapping.size());
        optimVariableSize = qDes.size();

        //mapping contains the list of joints (name) which are used in the optimization
        //Idea:
        // allJoints = initial (zero) values
        // before set state I use the content of variableToDoFMapping: which is a map between the
        // optimization variable index (qDes) and the allJoints index.
        for (std::vector<std::string>::const_iterator it = _jointsMapping.begin();
             it != _jointsMapping.end(); ++it) {
            int index = dynamics.getJointIndex(*it);
            variableToDoFMapping.push_back(index);
        }
        yInfo() << "Mapping translated to (# => model index): " << variableToDoFMapping;
    }

    //looking for feet frames
    leftFootFrameID = dynamics.getFrameIndex("l_sole");
    rightFootFrameID = dynamics.getFrameIndex("r_sole");
    return result;
}

bool SolverData::resetOptimizationData(const yarp::sig::Vector& desiredCoM, const yarp::sig::Vector& desiredJoints, std::string feetInContact)
{
    if (variableToDoFMapping.size() > 0)
        assert(desiredJoints.size() == qDes.size());

    comDes.resize(3);
    for (int i = 0; i < 3; i++) {
        comDes[i] = desiredCoM[i];
    }
    
    for (int i = 0; i < desiredJoints.size(); i++) {
        qDes[i] = desiredJoints[i];
    }
    //specify dimensions
    optimVariableSize = qDes.size();
    constraintsSize = 3;

    if (feetInContact == "left") {
        this->feetInContact = LEFT_FOOT_IN_CONTACT;
        dynamics.setFloatingBase("l_sole");
    }
    else if (feetInContact == "right") {
        this->feetInContact = RIGHT_FOOT_IN_CONTACT;
        dynamics.setFloatingBase("r_sole");
    }
    else if (feetInContact == "both") {
        this->feetInContact = BOTH_FEET_IN_CONTACT;
        dynamics.setFloatingBase("l_sole");
        //TODO: read state - or take it from input
        //we must read the relative transform. I don't have the current state. I cannot compute it
        right_X_left = dynamics.getRelativeTransform("l_sole", "r_sole");
        constraintsSize += 3 //relative transform position constraint
                        + 4; //relative transform orientation (expressed in quaternion) constraint
    } else {
        yError() << "Unsupported feet configuration";
        return false;
    }

    //resizing buffers
    qError.resize(optimVariableSize);
    hessian.setIdentity(optimVariableSize, optimVariableSize);

    //resetting variables and solution
    comJacobian.zero();
    finalStatus = ERROR;
    primalSolution.resize(optimVariableSize); primalSolution.setZero();
    optimum = std::numeric_limits<double>::max();
    constraintsMultipliers.resize(constraintsSize); constraintsMultipliers.setZero();
    constraintsValue.resize(constraintsSize); constraintsValue.setZero();
    //TODO: check dimensions here
    lowerBoundMultipliers.resize(optimVariableSize + constraintsSize); lowerBoundMultipliers.setZero();
    upperBoundMultipliers.resize(optimVariableSize + constraintsSize); upperBoundMultipliers.setZero();

    return true;
}
