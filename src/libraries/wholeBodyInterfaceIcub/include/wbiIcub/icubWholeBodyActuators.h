/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
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
 
#ifndef WBACTUATORS_ICUB_H
#define WBACTUATORS_ICUB_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <wbiIcub/wbiIcubUtil.h>
#include <map>

//*********TEMP************** -> for actuators //
#ifdef WBI_ICUB_COMPILE_PARAM_HELP
#include <yarp/sig/Vector.h>
namespace paramHelp {
    class ParamHelperClient;
}
#endif
//*********END TEMP**********//

namespace yarp {
    namespace os {
        class Property;
        class Value;
    }
}

 
 
namespace wbiIcub
{ 
    
    /*
     * Class for communicating with iCub's motor control boards.
     */
    class icubWholeBodyActuators : public wbi::iWholeBodyActuators
    {
    private:
        unsigned char *m_commandedParts; /*< This map is used during the setControlReference for whole-part set */
    protected:
        bool                        initDone;       // true after init has been called, false before
        int                         dof;            // number of actuators considered
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot
        std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    bodyPartNames;  // names of the body parts
        wbi::LocalIdList            jointIdList;    // list of the joint ids
        
        std::map<wbi::LocalId, wbi::ControlMode>        currentCtrlModes;    // current control mode of each actuator
        
        //std::map<std::string, std::string> configurationParameters; /*< Map containing parameters to be read at initialization time */
        yarp::os::Property configurationParameters; /*< Map containing parameters to be read at initialization time */

        // yarp drivers
        std::map<int, yarp::dev::IPositionControl*>     ipos;
        std::map<int, yarp::dev::IPositionDirect*>      positionDirectInterface;
        std::map<int, yarp::dev::ITorqueControl*>       itrq;
        std::map<int, yarp::dev::IImpedanceControl*>    iimp;
        std::map<int, yarp::dev::IControlMode*>         icmd;
        std::map<int, yarp::dev::IVelocityControl2*>    ivel;
        std::map<int, yarp::dev::IOpenLoopControl*>     iopl;
        std::map<int, yarp::dev::PolyDriver*>           dd;
        
        /** Open the yarp PolyDriver relative to the specified body part. */
        bool openDrivers(int bodyPart);
        /** Convert the control modes defined in yarp/dev/IControlMode.h into the one defined in wbi. */
        wbi::ControlMode yarpToWbiCtrlMode(int yarpCtrlMode);
        /** Set the reference speed for the position control of the specified joint(s). */
        virtual bool setReferenceSpeed(double *rspd, int joint=-1);
        
        //iCub specific option (for iCub should be true, for all other false [just a temporary workaround])
        bool reverse_torso_joints;
        
        //*********TEMP**************//
        #ifdef WBI_ICUB_COMPILE_PARAM_HELP
        paramHelp::ParamHelperClient *_torqueModuleConnection; /*< connection to the torque control module */
        yarp::sig::Vector _torqueRefs;
        #endif
        //*********END TEMP**********//

        /** Set the proportional, derivative and integrale gain for the current joint(s) controller.
         * If you want to leave some values unchanged simply pass NULL to the corresponding gain
         * @param pValue Value(s) of the proportional gain.
         * @param dValue Value(s) of the derivative gain.
         * @param iValue Value(s) of the integral gain.
         * @param joint Joint number, if negative, all joints are considered.
         * @return True if operation succeeded, false otherwise. */
        bool setPIDGains(const double *pValue, const double *dValue, const double *iValue, int joint = -1);
        
        /** Set the offset (feedforward term) for the current joint(s) controller.
         * @param value Value(s) of the parameter.
         * @param joint Joint number, if negative, all joints are considered.
         * @return True if operation succeeded, false otherwise. */
        bool setControlOffset(const double *value, int joint = -1);
        
    public:
        /** Constructor.
         * @param _name Name of this object, used as a stem for opening YARP ports.
         * @param _robot Name of the robot.
         * @param _bodyPartNames Vector containing the names of the body parts of iCub.
        */
        icubWholeBodyActuators(const char* _name,
                               const char* _robot,
                               const std::vector<std::string> &_bodyPartNames = std::vector<std::string>(iCub::skinDynLib::BodyPart_s,iCub::skinDynLib::BodyPart_s+sizeof(iCub::skinDynLib::BodyPart_s)/sizeof(std::string)));
        
        /** Constructor.
         * @param _name Name of this object, used as a stem for opening YARP ports.
         * @param _robot Name of the robot, prefix for its yarp ports
         * @param yarp_wbi_properties yarp::os::Property object used to configure the interface
        */
        icubWholeBodyActuators(const char* _name,
                               const char* _robot,
                               const yarp::os::Property & yarp_wbi_properties);

        
        virtual ~icubWholeBodyActuators();
        virtual bool init();
        virtual bool close();
        
        /* Configuration parameters section */
        static const std::string icubWholeBodyActuatorsUseExternalTorqueModule; /*< initialization parameter for iCub actuator class. The value associated is a boolean value. Default to false */
        static const std::string icubWholeBodyActuatorsExternalTorqueModuleName; /*< initialization parameter for iCub actuator class. Name of the torque external module */
        
        /** @brief Sets an initialization parameter.
         *
         * Sets a key-value pair parameter to be used during the initialization phase.
         * Note: this function must be called before init, otherwise it takes no effect
         * @param parameterName key for the parameter
         * @param parameterValue value for the parameter.
         * @return true if, false otherwise
        */
        virtual bool setConfigurationParameter(const std::string& parameterName, const yarp::os::Value& parameterValue);

        inline virtual int getActuatorNumber(){ return dof; }
        virtual bool removeActuator(const wbi::LocalId &j);
        virtual bool addActuator(const wbi::LocalId &j);
        virtual int addActuators(const wbi::LocalIdList &j);
        inline virtual const wbi::LocalIdList& getActuatorList(){ return jointIdList; }
        
        /** Set the control mode of the specified joint(s).
          * @param controlMode Id of the control mode.
          * @param ref Reference value(s) for the controller.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlMode(wbi::ControlMode controlMode, double *ref=0, int joint=-1);
        
        /** Set the reference value for the controller of the specified joint(s).
          * @param ref Reference value(s) for the controller.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlReference(double *ref, int joint=-1);

        /** Set a parameter (e.g. a gain) of one or more joint controllers.
          * @param paramId Id of the parameter.
          * @param value Value(s) of the parameter.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlParam(wbi::ControlParam paramId, const void *value, int joint=-1);
    };
}
    
#endif