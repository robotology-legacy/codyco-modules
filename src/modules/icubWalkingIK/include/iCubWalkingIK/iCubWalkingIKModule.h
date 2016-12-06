#ifndef ICUB_WALK_IK_MODULE_H
#define ICUB_WALK_IK_MODULE_H

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "iCubWalkingIKThread.h"

class iCubWalkingIKModule: public yarp::os::RFModule {
private:
    int                             m_period;
    iCubWalkingIKThread*            thread;
    yarpWbi::yarpWholeBodyModelV1*           m_robotModel;
    yarpWbi::yarpWholeBodyStates*          m_robotStates;
    std::string                     m_moduleName;
    std::string                     m_robotName;
    walkingParams                   m_params;
    odometryParams                  m_odometryParams;
    inverseKinematicsParams         m_inverseKinematicsParams;
    std::string                     m_walkingPatternFile;
    std::string                     m_outputDir;
    yarp::os::Port                  m_rpc_port;
    
public:
    iCubWalkingIKModule();
    virtual ~iCubWalkingIKModule();
    bool configure(yarp::os::ResourceFinder &rf);
    double getPeriod(){ return m_period; }
    bool updateModule();
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
    bool close();
    void closure();
    bool interruptModule();
};

#endif
