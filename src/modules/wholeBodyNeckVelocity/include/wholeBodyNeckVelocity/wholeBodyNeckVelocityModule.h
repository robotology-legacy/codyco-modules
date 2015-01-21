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


#ifndef WHOLEBODYNECKVELOCITYMODULE_H
#define WHOLEBODYNECKVELOCITYMODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <wbi/wholeBodyInterface.h>
#include "wholeBodyNeckVelocityThread.h"
#include <Eigen/Core>

class WholeBodyNeckVelocityModule : public yarp::os::RFModule {
public:
  WholeBodyNeckVelocityModule();
//   virtual ~WholeBodyNeckVelocityModule();
  virtual bool   configure(yarp::os::ResourceFinder& rf);
  virtual bool   updateModule();
  virtual bool   close();
  virtual double getPeriod();
  virtual bool   respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
  bool closure();
private:
  std::string m_robotName;
  yarp::os::BufferedPort<yarp::sig::Vector>* m_neckVelocityPort;
  wbi::wholeBodyInterface* m_robotInterface;
  WholeBodyNeckVelocityThread* m_neckVelocityThread;
};


#endif

