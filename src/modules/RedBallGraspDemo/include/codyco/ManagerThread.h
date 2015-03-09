/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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

#ifndef MANAGERTHREAD_H
#define MANAGERTHREAD_H

#include "Utilities.h"

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <string>
#include <vector>

#define NOARM               0
#define LEFTARM             1
#define RIGHTARM            2
#define USEDARM             3

#define OPENHAND            0
#define CLOSEHAND           1

namespace yarp {
    namespace dev {
        class PolyDriver;
        class IEncoders;
        class IControlMode2;
        class IPositionControl2;
        class ICartesianControl;
        class IGazeControl;
    }
}

namespace codyco {

    class ManagerThread : public yarp::os::RateThread
    {
    protected:
        yarp::os::ResourceFinder &rf;

        std::string name;
        std::string robot;
        std::string eyeUsed;

        bool useLeftArm;
        bool useRightArm;
        int  armSel;

        yarp::dev::PolyDriver *drvTorso, *drvHead, *drvLeftArm, *drvRightArm;
        yarp::dev::PolyDriver *drvCartLeftArm, *drvCartRightArm;
        yarp::dev::PolyDriver *drvGazeCtrl;

        yarp::dev::IEncoders         *encTorso;
        yarp::dev::IEncoders         *encHead;
        yarp::dev::IControlMode2     *modeTorso;
        yarp::dev::IPositionControl  *posTorso;
        yarp::dev::IEncoders         *encArm;
        yarp::dev::IControlMode2     *modeArm;
        yarp::dev::IPositionControl2 *posArm;
        yarp::dev::ICartesianControl *cartArm;
        yarp::dev::IGazeControl      *gazeCtrl;

        yarp::os::BufferedPort<yarp::os::Bottle> inportTrackTarget;
        yarp::os::BufferedPort<yarp::os::Bottle> inportIMDTargetLeft;
        yarp::os::BufferedPort<yarp::os::Bottle> inportIMDTargetRight;
        yarp::os::Port outportGui;
        yarp::os::Port outportCmdFace;

        yarp::os::RpcClient breatherHrpc;
        yarp::os::RpcClient breatherLArpc;
        yarp::os::RpcClient breatherRArpc;
        yarp::os::RpcClient blinkerrpc;

        yarp::sig::Vector leftArmReachOffs;
        yarp::sig::Vector leftArmGraspOffs;
        yarp::sig::Vector leftArmGraspSigma;
        yarp::sig::Vector leftArmHandOrien;
        yarp::sig::Vector leftArmJointsStiffness;
        yarp::sig::Vector leftArmJointsDamping;

        yarp::sig::Vector rightArmReachOffs;
        yarp::sig::Vector rightArmGraspOffs;
        yarp::sig::Vector rightArmGraspSigma;
        yarp::sig::Vector rightArmHandOrien;
        yarp::sig::Vector rightArmJointsStiffness;
        yarp::sig::Vector rightArmJointsDamping;

        //Additions to change direct robot control with port writing (except hand and gaze)
        //List of joint indexes corresponding to the hand (for IPositionControl2)
        std::vector<int> handJointList;
        std::vector<int> handControlModes;
        //Filled (and ignored) by askForPose of ICartesianController
        yarp::sig::Vector proposedXPosition;
        yarp::sig::Vector proposedXOrientation;
        //(Joints) Results from ICartesianController for each arm, plus a pointer to the current arm
        yarp::sig::Vector leftSolverDesiredJointConfiguration;
        yarp::sig::Vector rightSolverDesiredJointConfiguration;
        yarp::sig::Vector* desiredJointConfiguration;
        //Joints splitted into torso, left and right arms
        yarp::sig::Vector torsoDesiredJointConfiguration;
        yarp::sig::Vector leftArmDesiredJointConfiguration;
        yarp::sig::Vector rightArmDesiredJointConfiguration;
        yarp::sig::Vector* currentDesiredArmJointConfiguration;
        //Output ports
        yarp::os::BufferedPort<yarp::sig::Vector> torsoDesiredJointConfigurationPort;
        yarp::os::BufferedPort<yarp::sig::Vector> leftArmDesiredJointConfigurationPort;
        yarp::os::BufferedPort<yarp::sig::Vector> rightArmDesiredJointConfigurationPort;
        yarp::os::BufferedPort<yarp::sig::Vector> *currentArmDesiredJointConfigurationPort;
        /* 
         RCP protocol:
         Writes the following command (two string). [ . ] is optional
         - IS_DONE
         - [part_name]

         Expect:
         - integer: 0 => false, != 0 => true
         */
        yarp::os::RpcClient askForMotionDoneRPCClient;
        //End additions

        yarp::sig::Vector *armReachOffs;
        yarp::sig::Vector *armGraspOffs;
        yarp::sig::Vector *armGraspSigma;
        yarp::sig::Vector *armHandOrien;

        yarp::sig::Vector homePoss, homeVels;

        codyco::Predictor pred;
        bool useNetwork;
        bool wentHome;
        bool leftArmImpVelMode;
        bool rightArmImpVelMode;

        double trajTime;
        double reachTol;
        double idleTimer, idleTmo;
        double hystThres;
        double sphereRadius, sphereTmo;
        double releaseTmo;

        double latchTimer;
        yarp::sig::Vector sphereCenter;

        yarp::sig::Vector openHandPoss, closeHandPoss;
        yarp::sig::Vector handVels;

        yarp::sig::Vector targetPos;
        yarp::sig::Vector torso;
        yarp::sig::Vector head;

        yarp::sig::Matrix R,Rx,Ry,Rz;

        int  state;
        bool state_breathers;
        int  startup_context_id_left;
        int  startup_context_id_right;
        int  startup_context_id_gaze;

        void checkMotionDone(bool &done, std::string part = "");

        void breathersHandler(const bool sw);

        void getTorsoOptions(yarp::os::Bottle &b, const char *type, const int i, yarp::sig::Vector &sw, yarp::sig::Matrix &lim);

        void getArmOptions(yarp::os::Bottle &b, yarp::sig::Vector &reachOffs, yarp::sig::Vector &graspOffs,
                           yarp::sig::Vector &graspSigma, yarp::sig::Vector &orien, bool &impVelMode,
                           yarp::sig::Vector &impStiff, yarp::sig::Vector &impDamp);

        void getHomeOptions(yarp::os::Bottle &b, yarp::sig::Vector &poss, yarp::sig::Vector &vels);

        void getGraspOptions(yarp::os::Bottle &b, yarp::sig::Vector &openPoss, yarp::sig::Vector &closePoss, yarp::sig::Vector &vels);

        void initCartesianCtrl(yarp::sig::Vector &sw, yarp::sig::Matrix &lim, const int sel=USEDARM);

        void getSensorData();

        void doIdle();

        bool checkForHomePos();

        void commandHead();

        void steerHeadToHome();

        void steerTorsoToHome();

        void checkTorsoHome(const double timeout=10.0);

        void steerArmToHome(const int sel=USEDARM);

        void checkArmHome(const int sel=USEDARM, const double timeout=10.0);

        void stopArmJoints(const int sel=USEDARM);

        void moveHand(const int action, const int sel=USEDARM);

        void openHand(const int sel=USEDARM);

        void closeHand(const int sel=USEDARM);

        void selectArm();

        void doReach();

        void doGrasp();
        
        void doRelease();
        
        void doWait();
        
        void commandFace();
        
        bool checkArmForGrasp();
        
        bool checkTargetForGrasp();
        
        void resetTargetBall();
        
        void stopControl();
        
        void setFace(const std::string &type);
        
        void limitRange(yarp::sig::Vector &x);
        
        yarp::sig::Matrix &rotx(const double theta);
        
        yarp::sig::Matrix &roty(const double theta);
        
        yarp::sig::Matrix &rotz(const double theta);
        
        void deleteGuiTarget();
        
        void close();
        
    public:
        ManagerThread(const std::string &_name, yarp::os::ResourceFinder &_rf);
        
        bool threadInit();
        
        void run();
        
        void threadRelease();
    };
    
}

#endif /* end of include guard: MANAGERTHREAD_H */
