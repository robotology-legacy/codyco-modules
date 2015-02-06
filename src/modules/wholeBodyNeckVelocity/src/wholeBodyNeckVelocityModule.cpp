/**
 * Copyright (C) 2014 CoDyCo
 * @author: Jorhabib Eljaik
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "constants.h"
#include "wholeBodyNeckVelocityModule.h"
#include "wholeBodyNeckVelocityThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

WholeBodyNeckVelocityModule::WholeBodyNeckVelocityModule()
: m_robotInterface(0) {}

bool WholeBodyNeckVelocityModule::configure(yarp::os::ResourceFinder& rf) {
  
  yarp::os::Property wbiProperties;
  yarp::os::ConstString wbiConfFileFullPath;
  yarp::os::ConstString wbiConfFileName;
  
  //BEGINS PARAMETERS SECTION >>
  // Retrieve all parameters from configuration file
  if (!rf.check("wbi_config_file")) {
      std::cerr << "No WBI configuration file was specified. Using wbiConfigFileForwholeBodyVelocityNeck.txt as default" << std::endl;
      wbiConfFileName = "wbiConfigFileForwholeBodyVelocityNeck.ini";
  } else {
      wbiConfFileName = rf.find("wbi_config_file").asString();
  }
  wbiConfFileFullPath = rf.findFile(wbiConfFileName);
  
  if (!wbiProperties.fromConfigFile(wbiConfFileFullPath)) {
    std::cerr << "Configuration file for the whole body interface was not found!" << std::endl;
    return false;
  }
  
  // Update robot given by user
  std::string robot;
  if (!rf.check("robot"))
      robot = wbiProperties.find("robot").asString(); // "icubGazeboSim";
  else {
      robot = rf.find("robot").asString();
      wbiProperties.put("robot", robot);
  }
  
  // Update rate given by user
  int rate;
  if (!rf.check("rate"))
      rate = wbiProperties.find("rate").asInt(); // 10;
  else {
      rate = rf.find("rate").asInt();
      wbiProperties.put("rate", rate);
  }
  
  // Update swingingFoot given by user
  FOOT swingingFoot;
  if (!rf.check("swingingFoot"))
    swingingFoot = static_cast<FOOT>(wbiProperties.find("swingingFoot").asInt()); //LEFT FOOT
  else {
    swingingFoot = static_cast<FOOT>(rf.find("swingingFoot").asInt());
    wbiProperties.put("swingingFoot", swingingFoot);
  }
  
  // Update local given by user
  std::string local;
  if (!rf.check("local"))
    local =  wbiProperties.find("local").asString(); // "wholeBodyNeckVelocity";
  else {
    local = rf.find("local").asString();
    wbiProperties.put("local",local);
  }
  
  // Update WRF given by user
  FOOT WRF;
  if (!rf.check("wrf")) {
      WRF = static_cast<FOOT>(wbiProperties.find("WRF").asInt());
  } else {
      WRF = static_cast<FOOT>(rf.find("wrf").asInt());
      wbiProperties.put("wrf", WRF);
  }  
  //ENDS PARAMETERS SECTION <<
  
  
  //BEGINS PORTS SECTION >>
  //Port that will stream the neck velocity
//   m_neckVelocityPort = new BufferedPort<Vector>;
//   
//   cout << "DEBUG local is: " << local <<  std::endl;   //FIXME
//   if(!m_neckVelocityPort || !m_neckVelocityPort->open(("/" + local + "neckVelocity" + ":o").c_str())) {
//       std::cerr << "Could not open port to stream neck velocity" << std::endl;
//       return false;
//   }  
  //ENDS PORTS SECTION <<
  
  
  //BEGINS WBI CONFIGURATION SECTION >>
  // Create robot interface
  m_robotInterface = new yarpWbi::yarpWholeBodyInterface(local.c_str(), wbiProperties);
  if (!m_robotInterface) {
    std::cerr << "Could not create WBI object" << std::endl;
    return false;
  }
  
  // Add joints
  wbi::IDList robotMainJoints;
  if(!yarpWbi::loadIdListFromConfig("ROBOT_TORQUE_CONTROL_JOINTS", wbiProperties, robotMainJoints)) {
      std::cerr << "Could not find joints list" << std::endl;
      return false;
  }
  m_robotInterface->addJoints(robotMainJoints);
  //ENDS WBI CONFIGURATION SECTION <<
  
  
  //BEGINS THREAD CALL >>>
  m_neckVelocityThread = new WholeBodyNeckVelocityThread(*m_robotInterface, rate, local, swingingFoot, WRF, robot);
  if (!m_neckVelocityThread) {
    std::cerr << "Could not create thread...\n" << std::endl;
    return false;
  }
  
  if(!m_neckVelocityThread->start()) {
    std::cerr << "Thread could not start! Aborting..." << std::endl;
    return false;
  } else {
    std::cerr << "Thread started" << std::endl;
  }
  //ENDS THREAD CALL <<<
  
}

bool WholeBodyNeckVelocityModule::updateModule()
{
    return true;
}

bool WholeBodyNeckVelocityModule::close()
{
    closure();
}

double WholeBodyNeckVelocityModule::getPeriod()
{
return yarp::os::RFModule::getPeriod();
}

bool WholeBodyNeckVelocityModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
return yarp::os::RFModule::respond(command, reply);
}

bool WholeBodyNeckVelocityModule::closure() {
    if (m_neckVelocityThread) {
        m_neckVelocityThread->stop();
        delete m_neckVelocityThread;
    } else {
        cout << "ERR Could not close thread" << endl;
        return false;
    }
    
    if (m_robotInterface) {
        m_robotInterface->close();
        delete m_robotInterface;
        m_robotInterface = NULL;
    } else {
        cout << "ERR Could not stop/close interface" << endl;
        return false;
    }
    
    return true;
}
