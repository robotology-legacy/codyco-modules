/*
 * Copyright (C) 2013 Italian Institute of Technology CoDyCo Project
 * Authors: Daniele Pucci and Silvio Traversaro
 * email:   daniele.pucci@iit.it and silvio.traversaro@iit.it
 * website: www.codyco.eu
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

#include "iCub/linearRegressorsAdaptiveControl/linearRegressorsAdaptiveControlModule.h"

using namespace iCub::linearRegressorsAdaptiveControl;

int main(int argc, char * argv[])
{
   /* initialize yarp network */
   Network yarp;

   /* create your module */
   linearRegressorsAdaptiveControlModule module;

   /* prepare and configure the resource finder */
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("linearRegressorsAdaptiveControl.ini");		//overridden by --from parameter
   rf.setDefaultContext("linearRegressorsAdaptiveControl/conf");				//overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);

   /* run the module: runModule() calls configure first and, if successful, it then runs */
   module.runModule(rf);

   return 0;
}
