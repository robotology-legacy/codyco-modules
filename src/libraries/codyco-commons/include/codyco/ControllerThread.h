/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Francesco Romano
 *
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

#ifndef CONTROLLERTHREAD_H
#define CONTROLLERTHREAD_H

#include <yarp/os/RateThread.h>
#include <Eigen/Core>

namespace codyco {

    class ControllerThread;
    
    class ControllerThreadDelegate {
    public:
        virtual ~ControllerThreadDelegate();
        
        virtual void controllerWillActivate() = 0;
        virtual void controllerWillDeactivate() = 0;
        
        virtual void controllerWillRunLoop() = 0;
        virtual void controllerDidRunLoop() = 0;
    };
    
    class ControllerThread : public yarp::os::RateThread, private ControllerThreadDelegate
    {

    public:
        
        /** Constructor.
         *
         * @param period thread period in milliseconds
         */
        ControllerThread(int period);
        
        virtual ~ControllerThread();
        
       /** Sets the state of the controller.
        *
        * If the controller is already in the state requested as input the command is ignored.
        * @param isActive the new state of the controller
        */
       void setActiveState(bool isActive);
       
       /** Returns the current state of the controller
        * @return true if the controller is active. False otherwise.
        */
       bool isActiveState();
       
       /** Sets the delegate for this controller
        *
        * @note: the delegate is not retained. It is responsibility of the caller to keep the pointer to a valid state.
        * It is also responsibility of the caller to reset the current delegate when it is no longer valid
        * @param delegate the new delegate for the controller or 0 to remove it.
        */
       void setDelegate(ControllerThreadDelegate* delegate);
       
       /** Returns the current delegate of this controller
        * @return the current delegate
        */
       ControllerThreadDelegate* delegate();
       

       //management of print/debug variables. how to identify variables for later detaching??
       void attachDebugVariable(double *buffer, int size); //input type 
       void attachDebugVariable(Eigen::MatrixXd variable, std::string portName = "");
       void detachDebugVariable();

#pragma mark - RateThread methods
        virtual bool threadInit();
        virtual void threadRelease();
        virtual void run();
        
    private:
        virtual void controllerWillActivate();
        virtual void controllerWillDeactivate();
        
        virtual void controllerWillRunLoop();
        virtual void controllerDidRunLoop();
        
    };

}

#endif /* end of include guard: CONTROLLERTHREAD_H */
