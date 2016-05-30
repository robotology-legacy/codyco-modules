#include <InverseKinematics.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <string>
#include <iDynTree/Core/Transform.h>

using namespace yarp::os;

int main(int argc, char **argv) {

    ResourceFinder finder = ResourceFinder::getResourceFinderSingleton();
    finder.configure(argc, argv);
    finder.setVerbose();

    //Temp: to be removed
    Network::setEnvironment("YARP_DATA_DIRS", "/Users/makaveli/Projects/src/local/share/yarp:/Users/makaveli/Projects/src/local/share/iCub:/Users/makaveli/Projects/src/local/../codyco-superbuild/build/install/share/codyco");

    if (finder.check("robotName") && finder.find("robotName").isString()) {
        std::string robotName = finder.find("robotName").asString();
        Network::setEnvironment("YARP_ROBOT_NAME", robotName);
    }

    if (!finder.check("model") || !finder.find("model").isString()) {
        yError("Option 'model' is required");
        return -1;
    }
    std::string urdfModelFile = finder.findFile("model");
    
    //allocate inverse kinematics object
    kinematics::InverseKinematics robotIKin;
    if (!robotIKin.loadModelFromURDFFileWithName(urdfModelFile)) {
        yError("Cannot load model from %s", urdfModelFile.c_str());
    }
    
    //Limits are taken from URDF, but they can be overwritten
    //robotIKin.setJointLimit();
    
    //Set initial configuration for the robot
    //This must match the #Dofs of the URDF
    //robotIKin.setRobotConfiguration(qj);
    
    //Optionally: set joint map.
    //This is used to specify the optimization variables.
    //Joints not present in this list are not optimized
    //robotIKin.setOptimizationVariablesToJointsMap();

    //add constraints on links
    //two versions: one with name of the frame and homogeneous transformation
    //second one with only the name of the frame. In this case the frame is kept constant.
    //robotIKin.addFrameConstraint("root_link", w_X_r);
//    robotIKin.addFrameConstraint("lower_link");
    robotIKin.addFrameConstraint("r_sole");
//    robotIKin.addFrameConstraint("base");

    //Specify the target (or more than one?) frames
    iDynTree::Position w_p_lh(0.1, 0.2, 0.3);
    iDynTree::Rotation w_R_lh = iDynTree::Rotation::Identity();
    iDynTree::Transform w_X_lh(w_R_lh, w_p_lh);
    robotIKin.addTarget("r_upper_arm", w_X_lh);

    //robotIKin.setInitialCondition(x0);

    //test left end effector

//    robotIKin.setRotationParametrization(kinematics::InverseKinematicsRotationParametrizationRollPitchYaw);

    robotIKin.solve();

    return 0;
    
}
