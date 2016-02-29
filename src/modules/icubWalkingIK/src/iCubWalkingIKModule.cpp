#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include "iCubWalkingIKModule.h"


using namespace yarp::os;
using namespace yarpWbi;

iCubWalkingIKModule::iCubWalkingIKModule() {}

iCubWalkingIKModule::~iCubWalkingIKModule() { closure(); }

bool iCubWalkingIKModule::configure(ResourceFinder &rf) {
    // Load module parameters
    yarp::os::Property wbiProperties;
    if (!rf.check("wbi_config_file", "Checking WBI configuration file name")) {
        yError("No WBI configuration file was specified");
        return false;
    }
    if (!rf.check("wbi_joints_list", "Checking WBI joints list")) {
        yError("No joints list found");
        return false;
    }
    if (!wbiProperties.fromConfigFile(rf.findFile("wbi_config_file"))) {
        yError("Not possible to load WBI properties from file.");
        return false;
    }
    
    m_moduleName = rf.check("name", Value("iCubWalkingIK"), "Looking for module name").asString();
    m_robotName = rf.check("robot", Value("icubGazeboSim"), "Looking for robot name").asString();
    m_period = rf.check("period");
    
    wbiProperties.fromString(rf.toString(), false);
    yarp::os::ConstString jointList = rf.find("wbi_joints_list").asString();
    //retrieve all main joints
    wbi::IDList iCubMainJoints;
    if (!yarpWbi::loadIdListFromConfig(jointList, wbiProperties, iCubMainJoints)) {
        yError("Cannot find joint list");
        return false;
    }

    // Building robot model
    m_robotModel = new yarpWbi::yarpWholeBodyModel(m_moduleName.c_str(), wbiProperties);
    m_robotStates = new yarpWbi::yarpWholeBodyStates(m_moduleName.c_str(), wbiProperties);
    //add joints
    m_robotModel->addJoints(iCubMainJoints);
    if (!m_robotModel->init()) {
        yError("Could not initialize WBM.");
        return false;
    }
    
    yarp::sig::Vector initJointConf(m_robotModel->getDoFs());
    //TODO: This 15DOF list should actually be contained in the module's configuration file
    yInfo("[iCubWalkingIKModule::configure] A model of iCub with %i DOF has been created", m_robotModel->getDoFs());
    
    //Retrieve joints configuration
    m_robotStates->getEstimates(wbi::ESTIMATE_JOINT_POS, initJointConf.data());
    
    yInfo("[iCubWalkingIKModule::configure] Initial joint configuration \n, %s", initJointConf.toString().c_str() );
    
    // Copying params
    yarp::os::Bottle params = rf.findGroup("walking_params");
    m_params.z_c = params.check("z_c", 0.51).asDouble();
    m_params.n_strides = params.check("n_strides", 3).asInt();
    m_params.T_stride = params.check("T_stride", 12).asInt();
    m_params.T_switch = params.check("T_switch", 1).asInt();
    m_params.step_width = params.check("step_width", 0.16).asDouble();
    m_params.step_length = params.check("step_length", 0.05).asDouble();
    m_params.step_height = params.check("step_height", 0.04).asDouble();
    m_params.n_samples = params.check("n_samples", 4801).asInt();
    m_params.g = params.check("g",9.81).asDouble();
    
    // Load walking pattern file
    std::string m_walkingPatternFile = rf.find("patternFile").asString();
    yInfo("Pattern file is: %s", m_walkingPatternFile.c_str());
    
    thread = new iCubWalkingIKThread(m_period,
                                     m_robotModel,
                                     m_robotStates,
                                     m_params,
                                     m_walkingPatternFile);
    return true;
}

bool iCubWalkingIKModule::updateModule() {
    return true;
}

bool iCubWalkingIKModule::close() {
    
    closure();
    return true;
}

void iCubWalkingIKModule::closure() {
    if (thread) {
        thread->stop();
        delete thread;
        thread = 0;
    }
}
