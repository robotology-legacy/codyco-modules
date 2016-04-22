#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include "iCubWalkingIKModule.h"


using namespace yarp::os;
using namespace yarpWbi;
using namespace std;

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
    m_period = rf.check("period", Value(10), "Looking for period").asInt();
    m_outputDir = rf.check("outputDir", Value(""), "Output directory to store results").asString();
    
    wbiProperties.fromString(rf.toString(), false);
    yarp::os::ConstString jointList = rf.find("wbi_joints_list").asString();
    
    //Open RPC port
    // RPC Port opening
    m_rpc_port.open(std::string("/" + m_moduleName + "/rpc").c_str());
    attach(m_rpc_port);

    
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
    m_robotStates->addEstimates(wbi::ESTIMATE_JOINT_POS, iCubMainJoints);
    if ( !m_robotStates->init() ) {
        yError("Could not initialize WBS");
        return false;
    }
    
    yarp::sig::Vector initJointConf(m_robotModel->getDoFs());
    initJointConf.zero();
    //TODO: This 15DOF list should actually be contained in the module's configuration file
    yInfo("[iCubWalkingIKModule::configure] A model of iCub with %i DOF has been created", m_robotModel->getDoFs());
    
    yInfo("Waiting three seconds before getting first joint state");
    yarp::os::Time::delay(1);
    //Retrieve joints configuration
    if (!m_robotStates->getEstimates(wbi::ESTIMATE_JOINT_POS, initJointConf.data())) {
        yError("[iCubWalkingIKModule::configure] Could not retrieve joints state");
        return false;
    }
    
    yInfo("[iCubWalkingIKModule::configure] Initial joint configuration \n, %s", initJointConf.toString().c_str() );
    
    // Copying walking parameters file written by patternGenerator
    yarp::os::Property params;
    params.fromConfigFile(rf.findFile("walkingParams.txt"));
    m_params.z_c = params.check("z_c", 0.51).asDouble();
    m_params.n_strides = params.check("n_strides", 3).asInt();
    m_params.T_stride = params.check("T_stride", 12).asInt();
    m_params.T_switch = params.check("T_switch", 1).asInt();
    m_params.step_width = params.check("step_width", 0.16).asDouble();
    m_params.step_length = params.check("step_length", 0.05).asDouble();
    m_params.step_height = params.check("step_height", 0.04).asDouble();
    m_params.n_samples = params.check("n_samples", 4801).asInt();
    m_params.g = params.check("g",9.81).asDouble();
    
    // Copying odometry parameters
    yarp::os::Bottle odometryParamsBottle = rf.findGroup("odometry_params");
    m_odometryParams.initial_world_reference_frame = odometryParamsBottle.check("initial_world_reference_frame", yarp::os::Value("l_sole")).asString();
    m_odometryParams.initial_fixed_frame = odometryParamsBottle.check("initial_fixed_frame", yarp::os::Value("l_sole")).asString();
    m_odometryParams.floating_base = odometryParamsBottle.check("floating_base", yarp::os::Value("root_link")).asString();
    yarp::os::Bottle * tmpBot;
    tmpBot = new yarp::os::Bottle(*odometryParamsBottle.find("offset_from_world_reference").asList());
    for (unsigned int i = 0; i < 3; i++) {
        m_odometryParams.offset_from_world_reference_frame(i) = tmpBot->get(i).asDouble();
    }
    std::cerr << "[DEBUG] offset_from_world_reference_frame read from config file is: " << m_odometryParams.offset_from_world_reference_frame(0) << " "
                                                                                        << m_odometryParams.offset_from_world_reference_frame(1) << " "
                                                                                        << m_odometryParams.offset_from_world_reference_frame(2) << " "
                                                                                        << std::endl;
    m_odometryParams.world_between_feet = odometryParamsBottle.find("world_between_feet").asBool();
    
    // Copying inverse kinematics parameters
    yarp::os::Bottle inverseKinematicsBottle = rf.findGroup("inverse_kinematics_params");
    m_inverseKinematicsParams.lambda = inverseKinematicsBottle.find("lambda").asDouble();
    m_inverseKinematicsParams.step_tolerance = inverseKinematicsBottle.find("step_tolerance").asDouble();
    m_inverseKinematicsParams.max_iter = inverseKinematicsBottle.find("max_iter").asInt();
    m_inverseKinematicsParams.trials_initial_IK = inverseKinematicsBottle.find("trials_initial_IK").asInt();
    
    // Load walking pattern file
    std::string patternFile = rf.find("patternFile").asString();
    std::string m_walkingPatternFile = rf.findFile(std::string(patternFile+".csv"));
    yInfo("Pattern file is: %s", m_walkingPatternFile.c_str());
    
    thread = new iCubWalkingIKThread(m_period,
                                     m_robotModel,
                                     m_robotStates,
                                     m_params,
                                     m_odometryParams,
                                     m_inverseKinematicsParams,
                                     rf,
                                     m_walkingPatternFile,
                                     m_outputDir);
    bool ans = thread->start();
    return ans;
}

bool iCubWalkingIKModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply) {
    this->thread->thread_mutex.wait();
    bool ret = true;
    if (command.size()!=0) {
        string cmdstring = command.get(0).asString().c_str();
        if ( cmdstring == "help" ) {
            cerr << "Available commands: " << endl;
            cout << "run: Executes the module." << endl;
            cout << "quit: Stops the module." << endl;
            reply.addVocab(Vocab::encode("ack"));
        }
        else if ( cmdstring == "run" ) {
            if ( !this->thread->planner_flag ) {
                this->thread->planner_flag = true;
                reply.addVocab(Vocab::encode("ack"));
            }
        }
        else if ( cmdstring == "quit" ) {
//            this->close();
            reply.addVocab(Vocab::encode("ack"));
            ret = false;
        }
        else {
            reply.addVocab(Vocab::encode("nack"));
            ret = false;
        }
    } else {
        reply.addVocab(Vocab::encode("nack"));
        ret = false;
    }
    this->thread->thread_mutex.post();
    return ret;
}

bool iCubWalkingIKModule::updateModule() {
    if (this->thread == 0) {
        yInfo("The thread has been stopped ... ");
        return false;
    }
    return true;
}

bool iCubWalkingIKModule::interruptModule() {
    if (this->thread)
        this->thread->suspend();
    this->m_rpc_port.interrupt();
    return true;
}

bool iCubWalkingIKModule::close() {
    closure();
    return true;
}

void iCubWalkingIKModule::closure() {
    if (thread) {
//        this->thread->thread_mutex.wait();
        thread->stop();
//        this->thread->thread_mutex.post();
        delete thread;
        thread = 0;
    }
    if (m_robotModel) {
        delete m_robotModel;
        m_robotModel = 0;
    }
    if (m_robotStates) {
        delete m_robotStates;
        m_robotStates = 0;
    }
    m_rpc_port.interrupt();
    m_rpc_port.close();

    
}
