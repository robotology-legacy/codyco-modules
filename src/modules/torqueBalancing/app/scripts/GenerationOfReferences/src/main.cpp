#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include "OptimProblem.h"


/**
 *
 */
int main(int argc, char **argv) {
    using namespace yarp::os;

    ResourceFinder resourceFinder = ResourceFinder::getResourceFinderSingleton();
    resourceFinder.configure(argc, argv);

    if (resourceFinder.check("help")) {
        std::cout<< "Possible parameters" << std::endl << std::endl;
        std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /adaptiveControl/conf" << std::endl;
        std::cout<< "\t--robotURDFFile    :URDF file name" << std::endl;
        std::cout<< "\t--comDes           :Desired CoM of the robot." << std::endl;
        std::cout<< "\t--qDes             :Desired joint positions of the robot." << std::endl;
        std::cout<< "\t--feetInSupport    :left, right or both" << std::endl;
        std::cout<< "\t--jointMapping     :[optional]ordered list of joints name which should be used in the optimization. Size must match size of qDes. If missing all joints are assumed" << std::endl;
        return 0;
    }

    //read model file name
    std::string filename = resourceFinder.check("robotURDFFile", Value("model.urdf"), "Checking for model URDF file").asString();
    std::string filepath = resourceFinder.findFileByName(filename);

    yInfo() << "Robot model found in " << filepath;
    
    //read desired CoM
    if (!resourceFinder.check("comDes", "Checking desired CoM parameter")) {
        yError("Parameter comDes is required");
        return -1;
    }
    Value &comDes = resourceFinder.find("comDes");
    //Start checking: it should be a list of 3
    if (!comDes.isList() || comDes.asList()->size() != 3) {
        yError("Number of elements in comDes parameter is wrong. Expecting 3 values");
        return -2;
    }
    yarp::sig::Vector desiredCoM(3);
    Bottle* comList = comDes.asList();
    desiredCoM[0] = comList->get(0).asDouble();
    desiredCoM[1] = comList->get(1).asDouble();
    desiredCoM[2] = comList->get(2).asDouble();

    yInfo() << "Desired CoM is: " << desiredCoM.toString();

    //read desired Joints configuration
    if (!resourceFinder.check("qDes", "Checking desired joint configuration parameter")) {
        yError("Parameter qDes is required");
        return -1;
    }
    Value &qDes = resourceFinder.find("qDes");
    if (!qDes.isList()) {
        yError("qDes parameter should be a list");
        return -2;
    }
    Bottle *qDesBottle = qDes.asList();
    yarp::sig::Vector desiredJoints(qDesBottle->size());
    for (unsigned i = 0; i < qDesBottle->size(); i++) {
        desiredJoints[i] = qDesBottle->get(i).asDouble();
    }

    using namespace yarp::math;
    yInfo() << "Desired joint configuration is (rad): " << desiredJoints.toString();
    yInfo() << "Desired joint configuration is (deg): " << (desiredJoints * (180.0/M_PI)).toString();

    //read initial Joints configuration
    Value &qInit = resourceFinder.find("qInit");
    Bottle *qInitBottle = 0;
    yarp::sig::Vector initialJoints(0);

    if (!qInit.isNull()) {
        if (!qInit.isList()) {
            yError("qInit parameter should be a list");
            return -2;
        }
        qInitBottle = qInit.asList();
        initialJoints.resize(qInitBottle->size());
        for (unsigned i = 0; i < qInitBottle->size(); i++) {
            initialJoints[i] = qInitBottle->get(i).asDouble();
        }

        yInfo() << "Initial joint configuration is: " << initialJoints.toString();
    }

    //read which foot/feet is in support
    if (!resourceFinder.check("feetInSupport", "Checking feet in support parameter")) {
        yError("Parameter feetInSupport is required");
        return -1;
    }
    Value &feet = resourceFinder.find("feetInSupport");
    std::string feetInSupport = feet.asString();

    yInfo() << "Feet in support is: " << feetInSupport;

    std::vector<std::string> mapping;
    if (resourceFinder.check("jointMapping", "Checking joint mapping parameter")) {
        Bottle *mappingBottle = resourceFinder.find("jointMapping").asList();
        if (!mappingBottle || mappingBottle->size() != desiredJoints.size()) {
            yError("jointMapping parameter list has wrong size.");
            return -2;
        }
        mapping.reserve(mappingBottle->size());
        for (int i = 0; i < mappingBottle->size(); i++) {
            mapping.push_back(mappingBottle->get(i).asString());
        }
        yInfo() << "Joint mapping is (index => joint name): " << mapping;
    } else {
        yInfo("Joint mapping not specified");
    }

    OptimProblem problem;
    if (!problem.initializeModel(filepath, mapping)) {
        yError("Error initializing the robot model");
        return -2;
    }

    problem.solveOptimization(desiredCoM, desiredJoints, feetInSupport);


    return 0;
}
