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
    yarpWbi::yarpWholeBodyModel*           m_robotModel;
    yarpWbi::yarpWholeBodyStates*          m_robotStates;
    std::string                     m_moduleName;
    std::string                     m_robotName;
    walkingParams                   m_params;
    std::string                     m_walkingPatternFile;
    std::string                     m_outputDir;
    
public:
    iCubWalkingIKModule();
    virtual ~iCubWalkingIKModule();
    bool configure(yarp::os::ResourceFinder &rf);
    double getPeriod(){ return m_period; }
    bool updateModule();
    bool close();
    void closure();
};

#endif
