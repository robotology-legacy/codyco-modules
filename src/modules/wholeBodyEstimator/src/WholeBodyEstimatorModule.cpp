#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include "WholeBodyEstimatorModule.h"
#include "WholeBodyEstimatorThread.h"

using namespace yarp::os;
using namespace yarpWbi;

WholeBodyEstimatorModule::WholeBodyEstimatorModule()
{
    m_period = 10;
}

bool WholeBodyEstimatorModule::configure(ResourceFinder &rf) 
{
    
    if (!rf.check("MODULE_PARAMETERS"))
    {
        yError("Group MODULE_PARAMS was not specified in the configuration file of this module. Please fix it and try again.");
        return false;
    } else
    {
        m_module_params.fromString(rf.findGroup("MODULE_PARAMETERS").tail().toString());
        yInfo(" [WholeBodyEstimatorModule::configure] MODULE_PARAMS group contents are: %s ", m_module_params.toString().c_str());
        
        m_period = m_module_params.find("period").asInt();
        m_module_name = m_module_params.find("name").asString();
    }
    
    std::string wbiConfFile;
    yarp::os::Property yarpWbiOptions;
    if (!rf.check("wbi_conf_file"))
    {
        yError("WBI configuration file name not specified in config file of this module.");
        return false;
    } else
    {
        wbiConfFile = rf.findFile("wbi_conf_file");
        yarpWbiOptions.fromConfigFile(wbiConfFile);
    }
    
    yarpWholeBodySensors* wbs = new yarpWholeBodySensors(m_module_name.c_str(), yarpWbiOptions);
    
    // Configure yarpWholeBodySensors before passing it to WholeBodyEstimatorThread
    
    m_estimatorThread = new WholeBodyEstimatorThread(rf, wbs, m_period);
    if (!m_estimatorThread->start())
    {
        yError("[WholeBodyEstimatorModule::configure] Couldn't start thread!");
        return false;
    }

    return true;
}

bool WholeBodyEstimatorModule::updateModule()
{
    bool ret = true;
    return ret;
}

bool WholeBodyEstimatorModule::close()
{
    bool ret = true;
    return ret;
}