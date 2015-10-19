#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include "WholeBodyEstimatorModule.h"
#include "WholeBodyEstimatorThread.h"

using namespace yarp::os;

WholeBodyEstimatorModule::WholeBodyEstimatorModule()
{
    m_period = 10;
}

bool WholeBodyEstimatorModule::configure(ResourceFinder &rf) 
{
    bool ret = false;
    
    if (!rf.check("MODULE_PARAMETERS"))
    {
        yError("Group MODULE_PARAMS was not specified in the configuration file of this module. Please fix it and try again.");
        return false;
    } else
    {
        m_module_params.fromString(rf.findGroup("MODULE_PARAMETERS").tail().toString());
        yInfo(" [WholeBodyEstimatorModule::configure] MODULE_PARAMS group contents are: %s ", m_module_params.toString().c_str());
        
        m_period = m_module_params.find("period").asInt();
        ret = true;
    }

    m_estimatorThread = new WholeBodyEstimatorThread(rf, m_period);
    
    return ret;
}

bool WholeBodyEstimatorModule::updateModule()
{
    bool ret = false;
    return ret;
}

bool WholeBodyEstimatorModule::close()
{
    bool ret = false;
    return ret;
}