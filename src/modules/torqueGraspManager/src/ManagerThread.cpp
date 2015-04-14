#include "ManagerThread.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_math.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

#define DEFAULT_THR_PER     20

#define FACE_HAPPY          ("hap")
#define FACE_SAD            ("sad")
#define FACE_ANGRY          ("ang")
#define FACE_SHY            ("shy")
#define FACE_EVIL           ("evi")
#define FACE_CUNNING        ("cun")
#define FACE_SURPRISED      ("sur")

#define STATE_DISABLED         -1
#define STATE_IDLE              0
#define STATE_REACH             1
#define STATE_CHECKMOTIONDONE   2
#define STATE_RELEASE           3
#define STATE_WAIT              4

namespace codyco {

    void ManagerThread::checkMotionDone(bool &done, std::string part)
    {
        Bottle response;
        Bottle command;
        command.addString("IS_DONE");
        if (part.length() > 0)
            command.addString(part);
        askForMotionDoneRPCClient.write(command, response);
        Value value = response.pop();
        done = false;
        if (value.isInt()) {
            done = value.asInt() != 0;
        }
    }

    void ManagerThread::breathersHandler(const bool sw)
    {
        Bottle msg,reply;
        msg.addString(sw?"start":"stop");

        if (breatherHrpc.getOutputCount()>0)
        {
            breatherHrpc.write(msg,reply);
        }

        if (breatherLArpc.getOutputCount()>0)
        {
            breatherLArpc.write(msg,reply);
        }

        if (breatherRArpc.getOutputCount()>0)
        {
            breatherRArpc.write(msg,reply);
        }

        if (blinkerrpc.getOutputCount()>0)
        {
            blinkerrpc.write(msg,reply);
        }

        state_breathers = !sw;
    }

    void ManagerThread::getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim)
    {
        if (b.check(type))
        {
            Bottle &grp=b.findGroup(type);
            sw[i]=grp.get(1).asString()=="on"?1.0:0.0;

            if (grp.check("min","Getting minimum value"))
            {
                lim(i,0)=1.0;
                lim(i,1)=grp.find("min").asDouble();
            }

            if (grp.check("max","Getting maximum value"))
            {
                lim(i,2)=1.0;
                lim(i,3)=grp.find("max").asDouble();
            }
        }
    }

    void ManagerThread::getArmOptions(Bottle &b, Vector &reachOffs, Vector &graspOffs,
                                      Vector &graspSigma, Vector &orien, bool &impVelMode,
                                      Vector &impStiff, Vector &impDamp, std::string partName, std::string partPrefix)
    {
        if (b.check("reach_offset","Getting reaching offset"))
        {
            Bottle &grp=b.findGroup("reach_offset");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                reachOffs[i]=grp.get(1+i).asDouble();
        }

        if (b.check("grasp_offset","Getting grasping offset"))
        {
            Bottle &grp=b.findGroup("grasp_offset");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                graspOffs[i]=grp.get(1+i).asDouble();
        }

        if (b.check("grasp_sigma","Getting grasping sigma"))
        {
            Bottle &grp=b.findGroup("grasp_sigma");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                graspSigma[i]=grp.get(1+i).asDouble();
        }

        if (b.check("hand_orientation","Getting hand orientation"))
        {
            Bottle &grp=b.findGroup("hand_orientation");
            int sz=grp.size()-1;
            int len=sz>4?4:sz;

            for (int i=0; i<len; i++)
                orien[i]=grp.get(1+i).asDouble();
        }

        impVelMode=b.check("impedance_velocity_mode",Value("off"),"Getting arm impedance-velocity-mode").asString()=="on"?true:false;

        if (b.check("impedance_stiffness","Getting joints stiffness"))
        {
            Bottle &grp=b.findGroup("impedance_stiffness");
            size_t sz=grp.size()-1;
            size_t len=sz>impStiff.length()?impStiff.length():sz;

            for (size_t i=0; i<len; i++)
                impStiff[i]=grp.get(1+i).asDouble();
        }

        if (b.check("impedance_damping","Getting joints damping"))
        {
            Bottle &grp=b.findGroup("impedance_damping");
            size_t sz=grp.size()-1;
            size_t len=sz>impDamp.length()?impDamp.length():sz;

            for (size_t i=0; i<len; i++)
                impDamp[i]=grp.get(1+i).asDouble();
        }

        if (partName != "" && partPrefix != "") {
            std::vector<JointLimits> partLimits;
            for (std::map<std::string, int>::const_iterator it = armJointsNamesToIndexes.begin();
                 it != armJointsNamesToIndexes.end(); it++) {
                std::string jointName = partPrefix + it->first;
                if (b.check(jointName, ("Looking for " + jointName + " configuration"))) {
                    Bottle &grp=b.findGroup(jointName);
                    JointLimits limits(it->second);
                    if (grp.check("min","Getting minimum value"))
                    {
                        limits.min.value = grp.find("min").asDouble();
                        limits.min.isDefined = true;
                        yInfo("Setting %lf as minimum for %s", limits.min.value, jointName.c_str());
                    }
                    if (grp.check("max","Getting maximum value"))
                    {
                        limits.max.value = grp.find("max").asDouble();
                        limits.max.isDefined = true;
                        yInfo("Setting %lf as maximum for %s", limits.max.value, jointName.c_str());
                    }
                    partLimits.push_back(limits);
                }
            }
            if (partLimits.size() > 0) {
                solverLimits.insert(std::pair<std::string, std::vector<JointLimits> >(partName, partLimits));
            }
        }

    }

    void ManagerThread::getHomeOptions(Bottle &b, Vector &poss, Vector &vels)
    {
        if (b.check("poss","Getting home poss"))
        {
            Bottle &grp=b.findGroup("poss");
            int sz=grp.size()-1;
            int len=sz>7?7:sz;

            for (int i=0; i<len; i++)
                poss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("vels","Getting home vels"))
        {
            Bottle &grp=b.findGroup("vels");
            int sz=grp.size()-1;
            int len=sz>7?7:sz;

            for (int i=0; i<len; i++)
                vels[i]=grp.get(1+i).asDouble();
        }
    }

    void ManagerThread::getGraspOptions(Bottle &b, Vector &openPoss, Vector &closePoss, Vector &vels)
    {
        if (b.check("open_hand","Getting openHand poss"))
        {
            Bottle &grp=b.findGroup("open_hand");
            int sz=grp.size()-1;
            int len=sz>9?9:sz;

            for (int i=0; i<len; i++)
                openPoss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("close_hand","Getting closeHand poss"))
        {
            Bottle &grp=b.findGroup("close_hand");
            int sz=grp.size()-1;
            int len=sz>9?9:sz;

            for (int i=0; i<len; i++)
                closePoss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("vels_hand","Getting hand vels"))
        {
            Bottle &grp=b.findGroup("vels_hand");
            int sz=grp.size()-1;
            int len=sz>9?9:sz;

            for (int i=0; i<len; i++)
                vels[i]=grp.get(1+i).asDouble();
        }
    }

    void ManagerThread::initCartesianCtrl(Vector &sw, Matrix &lim, const int sel)
    {
        ICartesianControl *icart=cartArm;
        Vector dof;
        std::string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvCartLeftArm->view(icart);
                icart->storeContext(&startup_context_id_left);
                icart->restoreContext(0);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvCartRightArm->view(icart);
                icart->storeContext(&startup_context_id_right);
                icart->restoreContext(0);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Initializing %s controller ...\n",type.c_str());

        icart->setTrackingMode(false);
        icart->setTrajTime(trajTime);
        icart->setInTargetTol(reachTol);
        icart->getDOF(dof);

        if (sel == LEFTARM || (sel != RIGHTARM && armSel == LEFTARM)) {
            leftSolverDesiredJointConfiguration.resize(dof.size(), 0.0);
        } else if (sel == RIGHTARM || (sel != LEFTARM && armSel == RIGHTARM)) {
            rightSolverDesiredJointConfiguration.resize(dof.size(), 0.0);
        } else return;

        for (size_t j=0; j<sw.length(); j++)
        {
            dof[j]=sw[j];

            if (sw[j] && (lim(j,0) || lim(j,2)))
            {
                double min, max;
                icart->getLimits(j,&min,&max);

                if (lim(j,0))
                    min=lim(j,1);

                if (lim(j,2))
                    max=lim(j,3);

                icart->setLimits(j,min,max);
                fprintf(stdout,"jnt #%d in [%g, %g] deg\n",(int)j,min,max);
            }
        }

        std::map<std::string, std::vector<JointLimits> >::const_iterator found = solverLimits.find(type);
        if (found != solverLimits.end() && found->second.size() > 0) {
            //set (more) limits
            for (std::vector<JointLimits>::const_iterator limitsIterator = found->second.begin(); limitsIterator != found->second.end(); limitsIterator++) {
                double min, max;
                icart->getLimits(3 + limitsIterator->jointIndex, &min, &max);
                if (limitsIterator->min.isDefined)
                    min = limitsIterator->min.value;
                if (limitsIterator->max.isDefined)
                    max = limitsIterator->max.value;
                icart->setLimits(3 + limitsIterator->jointIndex, min, max);
            }
        }

        icart->setDOF(dof,dof);

        std::string dofString = "DOF's=( ";
        for (size_t i=0; i<dof.length(); i++)
            dofString.append(dof[i]>0.0?"on ":"off ");
        dofString.append(")");
        yInfo("%s", dofString.c_str());
    }

    void ManagerThread::getSensorData()
    {
        bool newTarget=false;

        if (encTorso->getEncoders(torso.data()))
            R=rotx(torso[1])*roty(-torso[2])*rotz(-torso[0]);

        encHead->getEncoders(head.data());

        if (useNetwork)
        {
            Bottle *imdTargetLeft=inportIMDTargetLeft.read(false);
            Bottle *imdTargetRight=inportIMDTargetRight.read(false);

            if ((imdTargetLeft!=NULL) && (imdTargetRight!=NULL))
            {
                Vector x,o;
                if (eyeUsed=="left")
                    gazeCtrl->getLeftEyePose(x,o);
                else
                    gazeCtrl->getRightEyePose(x,o);

                Matrix T=axis2dcm(o);
                T(0,3)=x[0];
                T(1,3)=x[1];
                T(2,3)=x[2];

                Vector netout=pred.predict(head,imdTargetLeft,imdTargetRight);
                netout.push_back(1.0);
                targetPos=(T*netout).subVector(0,2);
                newTarget=true;
            }
        }
        else if (Bottle *targetPosNew=inportTrackTarget.read(false))
        {
            if (targetPosNew->size()>6)
            {
                if (targetPosNew->get(6).asDouble()==1.0)
                {
                    Vector fp(4);
                    fp[0]=targetPosNew->get(0).asDouble();
                    fp[1]=targetPosNew->get(1).asDouble();
                    fp[2]=targetPosNew->get(2).asDouble();
                    fp[3]=1.0;

                    if (!gsl_isnan(fp[0]) && !gsl_isnan(fp[1]) && !gsl_isnan(fp[2]))
                    {
                        Vector x,o;
                        if (eyeUsed=="left")
                            gazeCtrl->getLeftEyePose(x,o);
                        else
                            gazeCtrl->getRightEyePose(x,o);

                        Matrix T=axis2dcm(o);
                        T(0,3)=x[0];
                        T(1,3)=x[1];
                        T(2,3)=x[2];

                        targetPos=T*fp;
                        targetPos.pop_back();
                        newTarget=true;
                    }
                }
            }
        }

        if (newTarget)
        {
            //send new target to output port
            Vector &outTarget = targetPositionOutputPort.prepare();
            outTarget = targetPos;
            targetPositionOutputPort.write();

            idleTimer=Time::now();

            if (state==STATE_IDLE)
            {
                resetTargetBall();
                breathersHandler(false);
                fprintf(stdout,"--- Got target => REACHING\n");

                wentHome=false;
                state=STATE_REACH;
            }
        }
        else if (((state==STATE_IDLE) || (state==STATE_REACH)) &&
                 ( (Time::now()-idleTimer)>idleTmo || disablingRequested ) && !wentHome )
        {
            if( !disablingRequested )
            {
                fprintf(stdout,"--- Target timeout => IDLE\n");
            }
            else
            {
                fprintf(stdout,"--- Disabling requested => DISABLED\n");
            }

            steerHeadToHome();
            stopControl();
            steerTorsoToHome();
            steerArmToHome(LEFTARM);
            steerArmToHome(RIGHTARM);

            wentHome=true;
            deleteGuiTarget();
            if( !disablingRequested )
            {
                state=STATE_IDLE;
            }
            else
            {
                state=STATE_DISABLED;
                disablingRequested = false;
            }
        }
    }

    void ManagerThread::doIdle()
    {
        if (state==STATE_IDLE)
        {
            if (state_breathers)
                if (checkForHomePos())
                    breathersHandler(true);
        }
    }

    bool ManagerThread::checkForHomePos()
    {
        IEncoders  *iencsLA = NULL;
        IEncoders  *iencsRA = NULL;
        if (useLeftArm)   drvLeftArm->view(iencsLA);
        if (useRightArm)  drvRightArm->view(iencsRA);

        if (breatherHrpc.getOutputCount()>0)
        {
            bool done;
            gazeCtrl->checkMotionDone(&done);
            if (!done)
                return false;
        }

        int axes;
        Vector encs;

        if (useLeftArm && breatherLArpc.getOutputCount()>0)
        {
            iencsLA->getAxes(&axes);
            encs.resize(axes,0.0);
            iencsLA->getEncoders(encs.data());
            if (norm(encs.subVector(0,homePoss.length()-1)-homePoss)>4.0)
                return false;
        }

        if (useRightArm && breatherRArpc.getOutputCount()>0)
        {
            iencsRA->getAxes(&axes);
            encs.resize(axes,0.0);
            iencsRA->getEncoders(encs.data());
            if (norm(encs.subVector(0,homePoss.length()-1)-homePoss)>4.0)
                return false;
        }

        return true;
    }

    void ManagerThread::commandHead()
    {
        if (state!=STATE_IDLE)
        {
            gazeCtrl->lookAtFixationPoint(targetPos);

            if (outportGui.getOutputCount()>0)
            {
                Bottle obj;
                obj.addString("object");
                obj.addString("ball");

                // size
                obj.addDouble(50.0);
                obj.addDouble(50.0);
                obj.addDouble(50.0);

                // positions
                obj.addDouble(1000.0*targetPos[0]);
                obj.addDouble(1000.0*targetPos[1]);
                obj.addDouble(1000.0*targetPos[2]);

                // orientation
                obj.addDouble(0.0);
                obj.addDouble(0.0);
                obj.addDouble(0.0);

                // color
                obj.addInt(255);
                obj.addInt(0);
                obj.addInt(0);

                // transparency
                obj.addDouble(1.0);

                outportGui.write(obj);
            }
        }
    }

    void ManagerThread::steerHeadToHome()
    {
        Vector homeHead(3);

        homeHead[0]=-1.0;
        homeHead[1]=0.0;
        homeHead[2]=0.3;

        fprintf(stdout,"*** Homing head\n");

        gazeCtrl->lookAtFixationPoint(homeHead);
    }

    void ManagerThread::steerTorsoToHome()
    {
        torsoDesiredJointConfiguration.zero();
        Bottle bottle;
        bottle.addList().read(torsoDesiredJointConfiguration);
        Property &output = desiredJointConfigurationPort.prepare();
        output.clear(); //??
        output.put("torso", bottle.get(0));
        desiredJointConfigurationPort.write(true);

        fprintf(stdout,"*** Homing torso\n");
    }

    void ManagerThread::checkTorsoHome(const double timeout)
    {
        fprintf(stdout,"*** Checking torso home position... ");

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<timeout))
        {
            checkMotionDone(done, "torso");
            Time::delay(0.1);
        }

        fprintf(stdout,"*** done\n");
    }

    void ManagerThread::steerArmToHome(const int sel)
    {
        std::string type;

        if (sel==LEFTARM)
        {
            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Homing %s\n",type.c_str());

        Bottle bottle;
        bottle.addList().read(homePoss);
        Property &output = desiredJointConfigurationPort.prepare();
        output.clear();
        output.put(type, bottle.get(0));
        desiredJointConfigurationPort.write(true);

        openHand(sel);
    }

    void ManagerThread::checkArmHome(const int sel, const double timeout)
    {
        IPositionControl *ipos=posArm;
        std::string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
                drvLeftArm->view(ipos);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvRightArm->view(ipos);
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Checking %s home position... ",type.c_str());

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<timeout))
        {
            checkMotionDone(done, type);
            Time::delay(0.1);
        }

        fprintf(stdout,"*** done\n");
    }

    void ManagerThread::stopArmJoints(const int sel)
    {
        IEncoders        *ienc=encArm;
        Vector *armEncoder = NULL;
        Vector *arm = NULL;

        std::string type;
        int selectedArm = NOARM;
        if (sel == LEFTARM || sel == RIGHTARM) {
            selectedArm = sel;
        } else if (armSel != NOARM) {
            selectedArm = armSel;
        } else return;

        if (selectedArm==LEFTARM)
        {
            if (useLeftArm)
            {
                drvLeftArm->view(ienc);
                armEncoder = &leftArmCurrentPosition;
                arm = &leftArmDesiredJointConfiguration;
            }
            else
                return;

            type="left_arm";
        }
        else if (selectedArm==RIGHTARM)
        {
            if (useRightArm)
            {
                drvRightArm->view(ienc);
                armEncoder = &rightArmCurrentPosition;
                arm = &rightArmDesiredJointConfiguration;
            }
            else
                return;

            type="right_arm";
        }

        fprintf(stdout,"*** Stopping %s joints\n",type.c_str());

        ienc->getEncoders(armEncoder->data());
        for (int i = 0; i < arm->size(); i++) {
            (*arm)(i) = (*armEncoder)(i);
        }

        Bottle bottle;
        (*arm) *= CTRL_DEG2RAD;
        bottle.addList().read(*arm);

        Property& output = desiredJointConfigurationPort.prepare();
        output.clear();
        output.put(type, bottle.get(0));
        desiredJointConfigurationPort.write(true);
    }

    void ManagerThread::moveHand(const int action, const int sel)
    {
        IControlMode2    *imode=modeArm;
        IPositionControl2 *ipos=posArm;
        Vector *poss=NULL;
        std::string actionStr, type;

        switch (action)
        {
            case OPENHAND:
                poss=&openHandPoss;
                actionStr="Opening";
                break;

            case CLOSEHAND:
                poss=&closeHandPoss;
                actionStr="Closing";
                break;

            default:
                return;
        }

        if (sel==LEFTARM)
        {
            drvLeftArm->view(imode);
            drvLeftArm->view(ipos);
            type="left_hand";
        }
        else if (sel==RIGHTARM)
        {
            drvRightArm->view(imode);
            drvRightArm->view(ipos);
            type="right_hand";
        }
        else
            type=armSel==LEFTARM?"left_hand":"right_hand";

        fprintf(stdout,"*** %s %s\n",actionStr.c_str(),type.c_str());

        imode->setControlModes(handJointList.size(), handJointList.data(), handControlModes.data());
        ipos->setRefSpeeds(handJointList.size(), handJointList.data(), handVels.data());
        ipos->positionMove(handJointList.size(), handJointList.data(), poss->data());
    }

    void ManagerThread::openHand(const int sel)
    {
        moveHand(OPENHAND,sel);
    }

    void ManagerThread::closeHand(const int sel)
    {
        moveHand(CLOSEHAND,sel);
    }

    void ManagerThread::selectArm()
    {
        if (useLeftArm && useRightArm)
        {
            if (state==STATE_REACH)
            {
                // handle the hysteresis thresholds
                if ((armSel==LEFTARM) && (targetPos[1]>hystThres) ||
                    (armSel==RIGHTARM) && (targetPos[1]<-hystThres))
                {
                    fprintf(stdout,"*** Change arm event triggered\n");
                    state=STATE_CHECKMOTIONDONE;
                    latchTimer=Time::now();
                }
            }
            else if (state==STATE_CHECKMOTIONDONE)
            {
                bool done;
                checkMotionDone(done);
                if (!done)
                {
                    if (Time::now()-latchTimer>3.0*trajTime)
                    {
                        fprintf(stdout,"--- Timeout elapsed => FORCE STOP and CHANGE ARM\n");
                        done=true;
                    }
                }

                if (done)
                {
                    stopControl();
                    steerArmToHome();

                    // swap interfaces
                    if (armSel==RIGHTARM)
                    {
                        armSel=LEFTARM;

                        drvLeftArm->view(encArm);
                        drvLeftArm->view(modeArm);
                        drvLeftArm->view(posArm);
                        drvCartLeftArm->view(cartArm);
                        armReachOffs=&leftArmReachOffs;
                        armGraspOffs=&leftArmGraspOffs;
                        armGraspSigma=&leftArmGraspSigma;
                        armHandOrien=&leftArmHandOrien;

                        currentDesiredArmJointConfiguration = &leftArmDesiredJointConfiguration;
                        desiredJointConfiguration = &leftSolverDesiredJointConfiguration;
                        currentArmCurrentPosition = &leftArmCurrentPosition;
                    }
                    else
                    {
                        armSel=RIGHTARM;

                        drvRightArm->view(encArm);
                        drvRightArm->view(modeArm);
                        drvRightArm->view(posArm);
                        drvCartRightArm->view(cartArm);
                        armReachOffs=&rightArmReachOffs;
                        armGraspOffs=&rightArmGraspOffs;
                        armGraspSigma=&rightArmGraspSigma;
                        armHandOrien=&rightArmHandOrien;

                        currentDesiredArmJointConfiguration = &rightArmDesiredJointConfiguration;
                        desiredJointConfiguration = &rightSolverDesiredJointConfiguration;
                        currentArmCurrentPosition = &rightArmCurrentPosition;
                    }

                    fprintf(stdout,"*** Using %s\n",armSel==LEFTARM?"left_arm":"right_arm");
                    stopArmJoints();
                    state=STATE_REACH;
                }
            }
        }
    }

    void ManagerThread::doReach()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_REACH)
            {
                Vector x=R.transposed()*(targetPos+*armReachOffs);
                limitRange(x);
                x=R*x;

                cartArm->askForPose(x, *armHandOrien, proposedXPosition, proposedXOrientation, *desiredJointConfiguration);
                //split joints into torso and arm
                for (int i = 0; i < torsoDesiredJointConfiguration.size(); i++) {
                    torsoDesiredJointConfiguration(i) = (*desiredJointConfiguration)(i);
                }
                int size = std::min(desiredJointConfiguration->size() - torsoDesiredJointConfiguration.size(), currentDesiredArmJointConfiguration->size());
                for (int i = 0; i < size; i++) {
                    (*currentDesiredArmJointConfiguration)(i) = (*desiredJointConfiguration)(i + torsoDesiredJointConfiguration.size());
                }
                Bottle torsoBottle;
                Bottle armBottle;
                torsoDesiredJointConfiguration *= CTRL_DEG2RAD;
                (*currentDesiredArmJointConfiguration) *= CTRL_DEG2RAD;
                torsoBottle.addList().read(torsoDesiredJointConfiguration);
                armBottle.addList().read(*currentDesiredArmJointConfiguration);

                Property& output = desiredJointConfigurationPort.prepare();
                output.clear();
                output.put("torso", torsoBottle.get(0));
                output.put((armSel == LEFTARM ? "left_arm" : "right_arm"), armBottle.get(0));
                desiredJointConfigurationPort.write(true);

            }
        }
    }

    void ManagerThread::doGrasp()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_REACH)
            {
                if (checkTargetForGrasp() && checkArmForGrasp())
                {
                    Vector graspOffs(3);
                    for (size_t i=0; i<graspOffs.length(); i++)
                        graspOffs[i]=Random::normal((*armGraspOffs)[i],(*armGraspSigma)[i]);

                    Vector x=R.transposed()*(targetPos+graspOffs);
                    limitRange(x);
                    x=R*x;

                    fprintf(stdout,"--- Hand in position AND Target still => GRASPING\n");
                    fprintf(stdout,"--- Target in %s\n",targetPos.toString().c_str());
                    fprintf(stdout,"*** Grasping x=%s\n",x.toString().c_str());

                    cartArm->askForPose(x, *armHandOrien, proposedXPosition, proposedXOrientation, *desiredJointConfiguration);
                    //split joints into torso and arm
                    for (int i = 0; i < torsoDesiredJointConfiguration.size(); i++) {
                        torsoDesiredJointConfiguration(i) = (*desiredJointConfiguration)(i);
                    }
                    for (int i = 0; i < currentDesiredArmJointConfiguration->size(); i++) {
                        (*currentDesiredArmJointConfiguration)(i) = (*desiredJointConfiguration)(i + torsoDesiredJointConfiguration.size());
                    }
                    Bottle torsoBottle;
                    Bottle armBottle;
                    torsoDesiredJointConfiguration *= CTRL_DEG2RAD;
                    (*currentDesiredArmJointConfiguration) *= CTRL_DEG2RAD;
                    torsoBottle.addList().read(torsoDesiredJointConfiguration);
                    armBottle.addList().read(*currentDesiredArmJointConfiguration);


                    Property& output = desiredJointConfigurationPort.prepare();
                    output.clear();
                    output.put("torso", torsoBottle.get(0));
                    output.put((armSel == LEFTARM ? "left_arm" : "right_arm"), armBottle.get(0));
                    desiredJointConfigurationPort.write(true);

                    closeHand();

                    latchTimer=Time::now();
                    state=STATE_RELEASE;
                }
            }
        }
    }

    void ManagerThread::doRelease()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_RELEASE)
            {
                if ((Time::now()-latchTimer)>releaseTmo)
                {
                    fprintf(stdout,"--- Timeout elapsed => RELEASING\n");

                    openHand();

                    latchTimer=Time::now();
                    state=STATE_WAIT;
                }
            }
        }
    }

    void ManagerThread::doWait()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_WAIT)
            {
                if ((Time::now()-latchTimer)>idleTmo)
                {
                    fprintf(stdout,"--- Timeout elapsed => IDLING\n");
                    deleteGuiTarget();
                    state=STATE_IDLE;
                }
            }
        }
    }

    void ManagerThread::commandFace()
    {
        if (state==STATE_IDLE)
            setFace(state_breathers?FACE_SHY:FACE_HAPPY);
        else if (state==STATE_REACH)
        {
            if (useLeftArm || useRightArm)
            {
                if (checkArmForGrasp())
                    setFace(FACE_EVIL);
                else
                    setFace(FACE_ANGRY);
            }
            else
                setFace(FACE_EVIL);
        }
        else if (state==STATE_WAIT)
            setFace(FACE_HAPPY);
    }

    bool ManagerThread::checkArmForGrasp()
    {
        Vector x,o;
        cartArm->getPose(x,o);

        // true if arm has reached the position
        if (norm(targetPos+*armReachOffs-x)<sphereRadius)
            return true;
        else
            return false;
    }

    bool ManagerThread::checkTargetForGrasp()
    {
        const double t=Time::now();

        // false if target is considered to be still moving
        if (norm(targetPos-sphereCenter)>sphereRadius)
        {
            resetTargetBall();
            return false;
        }
        else if ((t-latchTimer<sphereTmo) || (t-idleTimer>1.0))
            return false;
        else
            return true;
    }

    void ManagerThread::resetTargetBall()
    {
        latchTimer=Time::now();
        sphereCenter=targetPos;
    }

    void ManagerThread::stopControl()
    {
        if (useLeftArm || useRightArm)
        {
            fprintf(stdout,"stopping control\n");
            IEncoders *torsoEnc = NULL;
            IEncoders *armEnc = NULL;

            drvTorso->view(torsoEnc);
            if (armSel==LEFTARM)
            {
                drvLeftArm->view(armEnc);

            }
            else if (armSel==RIGHTARM)
            {
                drvRightArm->view(armEnc);
            }

            torsoEnc->getEncoders(torsoDesiredJointConfiguration.data());
            armEnc->getEncoders(currentArmCurrentPosition->data());
            for (int i = 0; i < currentArmCurrentPosition->size(); i++) {
                (*currentDesiredArmJointConfiguration)(i) = (*currentArmCurrentPosition)(i);
            }

            Bottle torsoBottle;
            Bottle armBottle;
            torsoDesiredJointConfiguration *= CTRL_DEG2RAD;
            (*currentDesiredArmJointConfiguration) *= CTRL_DEG2RAD;
            torsoBottle.addList().read(torsoDesiredJointConfiguration);
            armBottle.addList().read(*currentDesiredArmJointConfiguration);

            Property output = desiredJointConfigurationPort.prepare();
            output.clear();
            output.put("torso", torsoBottle.get(0));
            output.put((armSel == LEFTARM ? "left_arm" : "right_arm"), armBottle.get(0));
            desiredJointConfigurationPort.write(true);

            Time::delay(0.1);
        }
    }

    void ManagerThread::setFace(const std::string &type)
    {
        Bottle in, out;

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("mou"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace.write(out,in);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("leb"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace.write(out,in);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("reb"));
        out.addVocab(Vocab::encode(type.c_str()));
        outportCmdFace.write(out,in);
    }

    void ManagerThread::limitRange(Vector &x)
    {
        x[0]=x[0]>-0.1 ? -0.1 : x[0];
    }

    Matrix &ManagerThread::rotx(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rx(1,1)=Rx(2,2)=c;
        Rx(1,2)=-s;
        Rx(2,1)=s;

        return Rx;
    }

    Matrix &ManagerThread::roty(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Ry(0,0)=Ry(2,2)=c;
        Ry(0,2)=s;
        Ry(2,0)=-s;

        return Ry;
    }

    Matrix &ManagerThread::rotz(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rz(0,0)=Rz(1,1)=c;
        Rz(0,1)=-s;
        Rz(1,0)=s;

        return Rz;
    }

    void ManagerThread::deleteGuiTarget()
    {
        if (outportGui.getOutputCount()>0)
        {
            Bottle obj;
            obj.addString("delete");
            obj.addString("ball");
            outportGui.write(obj);
        }
    }

    void ManagerThread::close()
    {
        delete drvTorso;
        delete drvHead;
        delete drvLeftArm;
        delete drvRightArm;
        delete drvCartLeftArm;
        delete drvCartRightArm;
        delete drvGazeCtrl;

        inportTrackTarget.interrupt();
        inportTrackTarget.close();

        inportIMDTargetLeft.interrupt();
        inportIMDTargetLeft.close();

        inportIMDTargetRight.interrupt();
        inportIMDTargetRight.close();

        desiredJointConfigurationPort.interrupt();
        desiredJointConfigurationPort.close();

        targetPositionOutputPort.interrupt();
        targetPositionOutputPort.close();

        setFace(FACE_HAPPY);
        outportCmdFace.interrupt();
        outportCmdFace.close();

        deleteGuiTarget();
        outportGui.interrupt();
        outportGui.close();

        breatherHrpc.close();
        breatherLArpc.close();
        breatherRArpc.close();
        blinkerrpc.close();
        askForMotionDoneRPCClient.close();
        eventsOutputPort.interrupt();
        eventsOutputPort.close();
    }

    ManagerThread::ManagerThread(const std::string &_name, ResourceFinder &_rf)
    : RateThread(DEFAULT_THR_PER)
    , rf(_rf)
    , name(_name)
    , drvTorso(0), drvHead(0), drvLeftArm(0), drvRightArm(0)
    , drvCartLeftArm(0), drvCartRightArm(0), drvGazeCtrl(0)
    , desiredJointConfiguration(0), currentDesiredArmJointConfiguration(0), currentArmCurrentPosition(0)
    , disablingRequested(false)
    {}

    bool ManagerThread::threadInit()
    {
        yarp::os::LockGuard guard(this->runMutex);

        //prepare configuration
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("shoulder_pitch", 0));
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("shoulder_roll", 1));
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("shoulder_yaw", 2));
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("elbow", 3));
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("wrist_prosup", 4));
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("wrist_pitch", 5));
        armJointsNamesToIndexes.insert(std::pair<std::string, int>("wrist_yaw", 6));

        // general part
        Bottle &bGeneral=rf.findGroup("general");
        bGeneral.setMonitor(rf.getMonitor());
        robot=bGeneral.check("robot",Value("icub"),"Getting robot name").asString().c_str();
        useLeftArm=bGeneral.check("left_arm",Value("on"),"Getting left arm use flag").asString()=="on"?true:false;
        useRightArm=bGeneral.check("right_arm",Value("on"),"Getting right arm use flag").asString()=="on"?true:false;
        useNetwork=bGeneral.check("use_network",Value("off"),"Getting network enable").asString()=="on"?true:false;
        trajTime=bGeneral.check("traj_time",Value(2.0),"Getting trajectory time").asDouble();
        reachTol=bGeneral.check("reach_tol",Value(0.01),"Getting reaching tolerance").asDouble();
        eyeUsed=bGeneral.check("eye",Value("left"),"Getting the used eye").asString().c_str();
        idleTmo=bGeneral.check("idle_tmo",Value(1e10),"Getting idle timeout").asDouble();
        setRate(bGeneral.check("thread_period",Value(DEFAULT_THR_PER),"Getting thread period [ms]").asInt());

        // torso part
        Bottle &bTorso=rf.findGroup("torso");
        bTorso.setMonitor(rf.getMonitor());

        Vector torsoSwitch(3);   torsoSwitch.zero();
        Matrix torsoLimits(3,4); torsoLimits.zero();

        getTorsoOptions(bTorso,"pitch",0,torsoSwitch,torsoLimits);
        getTorsoOptions(bTorso,"roll",1,torsoSwitch,torsoLimits);
        getTorsoOptions(bTorso,"yaw",2,torsoSwitch,torsoLimits);

        // arm parts
        Bottle &bLeftArm=rf.findGroup("left_arm");
        Bottle &bRightArm=rf.findGroup("right_arm");
        bLeftArm.setMonitor(rf.getMonitor());
        bRightArm.setMonitor(rf.getMonitor());

        proposedXPosition.resize(3, 0.0);
        proposedXOrientation.resize(4, 0.0);

        leftArmReachOffs.resize(3,0.0);
        leftArmGraspOffs.resize(3,0.0);
        leftArmGraspSigma.resize(3,0.0);
        leftArmHandOrien.resize(4,0.0);
        leftArmJointsStiffness.resize(5,0.0);
        leftArmJointsDamping.resize(5,0.0);
        rightArmReachOffs.resize(3,0.0);
        rightArmGraspOffs.resize(3,0.0);
        rightArmGraspSigma.resize(3,0.0);
        rightArmHandOrien.resize(4,0.0);
        rightArmJointsStiffness.resize(5,0.0);
        rightArmJointsDamping.resize(5,0.0);
        getArmOptions(bLeftArm,leftArmReachOffs,leftArmGraspOffs,
                      leftArmGraspSigma,leftArmHandOrien,leftArmImpVelMode,
                      leftArmJointsStiffness,leftArmJointsDamping, "left_arm", "l_");
        getArmOptions(bRightArm,rightArmReachOffs,rightArmGraspOffs,
                      rightArmGraspSigma,rightArmHandOrien,rightArmImpVelMode,
                      rightArmJointsStiffness,rightArmJointsDamping, "right_arm", "r_");

        // home part
        Bottle &bHome=rf.findGroup("home_arm");
        bHome.setMonitor(rf.getMonitor());
        homePoss.resize(7,0.0); homeVels.resize(7,0.0);
        getHomeOptions(bHome,homePoss,homeVels);
        homePoss *= CTRL_DEG2RAD;
        homeVels *= CTRL_DEG2RAD;

        // arm_selection part
        Bottle &bArmSel=rf.findGroup("arm_selection");
        bArmSel.setMonitor(rf.getMonitor());
        hystThres=bArmSel.check("hysteresis_thres",Value(0.0),"Getting hysteresis threshold").asDouble();

        // grasp part
        Bottle &bGrasp=rf.findGroup("grasp");
        bGrasp.setMonitor(rf.getMonitor());
        sphereRadius=bGrasp.check("sphere_radius",Value(0.0),"Getting sphere radius").asDouble();
        sphereTmo=bGrasp.check("sphere_tmo",Value(0.0),"Getting sphere timeout").asDouble();
        releaseTmo=bGrasp.check("release_tmo",Value(0.0),"Getting release timeout").asDouble();

        openHandPoss.resize(9,0.0); closeHandPoss.resize(9,0.0);
        handVels.resize(9,0.0);
        handJointList.clear(); handJointList.reserve(9);
        handControlModes.clear(); handControlModes.reserve(9);
        for (int i = 7; i < 7 + 9; i++) { //7: arm, 9: hand
            handJointList.push_back(i);
            handControlModes.push_back(VOCAB_CM_POSITION);
        }

        getGraspOptions(bGrasp,openHandPoss,closeHandPoss,handVels);

        // init network
        if (useNetwork)
        {
            Property options;
            options.fromConfigFile(rf.findFile(bGeneral.check("network",Value("network.ini"),
                                                              "Getting network data").asString().c_str()));

            if (!pred.configure(options))
                return false;
        }

        // open ports
        inportTrackTarget.open((name+"/trackTarget:i").c_str());
        inportIMDTargetLeft.open((name+"/imdTargetLeft:i").c_str());
        inportIMDTargetRight.open((name+"/imdTargetRight:i").c_str());
        outportCmdFace.open((name+"/cmdFace:rpc").c_str());
        outportGui.open((name+"/gui:o").c_str());
        breatherHrpc.open((name+"/breather/head:rpc").c_str());
        breatherLArpc.open((name+"/breather/left_arm:rpc").c_str());
        breatherRArpc.open((name+"/breather/right_arm:rpc").c_str());
        blinkerrpc.open((name+"/blinker:rpc").c_str());
        askForMotionDoneRPCClient.open(name + "/motionDone:rpc");
        eventsOutputPort.open(name + "/events:o");
        targetPositionOutputPort.open(name + "/target:o");

        std::string fwslash="/";

        // open remote_controlboard drivers
        Property optTorso("(device remote_controlboard)");
        Property optHead("(device remote_controlboard)");
        Property optLeftArm("(device remote_controlboard)");
        Property optRightArm("(device remote_controlboard)");

        optTorso.put("remote",(fwslash+robot+"/torso").c_str());
        optTorso.put("local",(name+"/torso").c_str());

        optHead.put("remote",(fwslash+robot+"/head").c_str());
        optHead.put("local",(name+"/head").c_str());

        optLeftArm.put("remote",(fwslash+robot+"/left_arm").c_str());
        optLeftArm.put("local",(name+"/left_arm").c_str());

        optRightArm.put("remote",(fwslash+robot+"/right_arm").c_str());
        optRightArm.put("local",(name+"/right_arm").c_str());

        drvTorso=new PolyDriver;
        if (!drvTorso->open(optTorso))
        {
            close();
            return false;
        }

        desiredJointConfigurationPort.open(name + "/qdes:o");

        //read dimension of torso
        int size = 0;
        drvTorso->view(posTorso);
        posTorso->getAxes(&size);
        posTorso = NULL;
        torsoDesiredJointConfiguration.resize(size, 0.0);

        drvHead=new PolyDriver;
        if (!drvHead->open(optHead))
        {
            close();
            return false;
        }

        if (useLeftArm)
        {
            drvLeftArm=new PolyDriver;
            if (!drvLeftArm->open(optLeftArm))
            {
                close();
                return false;
            }
            size = 0;
            drvLeftArm->view(posArm);
            posArm->getAxes(&size);
            posArm = NULL;
            leftArmDesiredJointConfiguration.resize((size > 7 ? 7 : size), 0.0);
            leftArmCurrentPosition.resize(size, 0.0);
        }

        if (useRightArm)
        {
            drvRightArm=new PolyDriver;
            if (!drvRightArm->open(optRightArm))
            {
                close();
                return false;
            }
            size = 0;
            drvRightArm->view(posArm);
            posArm->getAxes(&size);
            posArm = NULL;
            rightArmDesiredJointConfiguration.resize((size > 7 ? 7 : size), 0.0);
            rightArmCurrentPosition.resize(size, 0.0);
        }

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
        Property optGazeCtrl("(device gazecontrollerclient)");

        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(name+"/left_arm/cartesian").c_str());

        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(name+"/right_arm/cartesian").c_str());

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",(name+"/gaze").c_str());

        if (useLeftArm)
        {
            drvCartLeftArm=new PolyDriver;
            if (!drvCartLeftArm->open(optCartLeftArm))
            {
                close();
                return false;
            }
        }

        if (useRightArm)
        {
            drvCartRightArm=new PolyDriver;
            if (!drvCartRightArm->open(optCartRightArm))
            {
                close();
                return false;
            }
        }

        drvGazeCtrl=new PolyDriver;
        if (!drvGazeCtrl->open(optGazeCtrl))
        {
            close();
            return false;
        }

        // open views
        drvTorso->view(modeTorso);
        drvTorso->view(encTorso);
        drvTorso->view(posTorso);
        drvHead->view(encHead);
        drvGazeCtrl->view(gazeCtrl);

        gazeCtrl->storeContext(&startup_context_id_gaze);
        gazeCtrl->restoreContext(0);
        gazeCtrl->blockNeckRoll(0.0);
        gazeCtrl->setSaccadesActivationAngle(20.0);
        gazeCtrl->setSaccadesInhibitionPeriod(1.0);

        if (useLeftArm)
        {
            drvLeftArm->view(encArm);
            drvLeftArm->view(modeArm);
            drvLeftArm->view(posArm);
            drvCartLeftArm->view(cartArm);
            armReachOffs=&leftArmReachOffs;
            armGraspOffs=&leftArmGraspOffs;
            armGraspSigma=&leftArmGraspSigma;
            armHandOrien=&leftArmHandOrien;
            armSel=LEFTARM;
            desiredJointConfiguration = &leftSolverDesiredJointConfiguration;
            currentDesiredArmJointConfiguration = &leftArmDesiredJointConfiguration;
            currentArmCurrentPosition = &leftArmCurrentPosition;
        }
        else if (useRightArm)
        {
            drvRightArm->view(encArm);
            drvRightArm->view(modeArm);
            drvRightArm->view(posArm);
            drvCartRightArm->view(cartArm);
            armReachOffs=&rightArmReachOffs;
            armGraspOffs=&rightArmGraspOffs;
            armGraspSigma=&rightArmGraspSigma;
            armHandOrien=&rightArmHandOrien;
            armSel=RIGHTARM;
            desiredJointConfiguration = &rightSolverDesiredJointConfiguration;
            currentDesiredArmJointConfiguration = &rightArmDesiredJointConfiguration;
            currentArmCurrentPosition = &rightArmCurrentPosition;
        }
        else
        {
            encArm=NULL;
            modeArm=NULL;
            posArm=NULL;
            cartArm=NULL;
            armReachOffs=NULL;
            armGraspOffs=NULL;
            armGraspSigma=NULL;
            armHandOrien=NULL;
            armSel=NOARM;
            desiredJointConfiguration = NULL;
            currentDesiredArmJointConfiguration = NULL;
            currentArmCurrentPosition = NULL;
        }

        // init
        int torsoAxes;
        encTorso->getAxes(&torsoAxes);
        torso.resize(torsoAxes,0.0);

        int headAxes;
        encHead->getAxes(&headAxes);
        head.resize(headAxes,0.0);

        targetPos.resize(3,0.0);
        R=Rx=Ry=Rz=eye(3,3);

        initCartesianCtrl(torsoSwitch,torsoLimits,LEFTARM);
        initCartesianCtrl(torsoSwitch,torsoLimits,RIGHTARM);

        // steer the robot to the initial configuration
        steerHeadToHome();
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);

        idleTimer=Time::now();
        Random::seed((int)idleTimer);

        wentHome=false;
        state=STATE_IDLE;
        state_breathers=true;

        return true;
    }

    void ManagerThread::run()
    {
        yarp::os::LockGuard guard(this->runMutex);

        getSensorData();
        doIdle();
        commandHead();
        selectArm();
        doReach();
        doGrasp();
        doRelease();
        doWait();
        commandFace();
        sendEvents();
    }

    void ManagerThread::threadRelease()
    {
        steerHeadToHome();
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);
        
        checkTorsoHome(3.0);
        checkArmHome(LEFTARM,3.0);
        checkArmHome(RIGHTARM,3.0);
        
        if (useLeftArm)
        {
            ICartesianControl *icart;
            drvCartLeftArm->view(icart);
            icart->restoreContext(startup_context_id_left);
            
            if (leftArmImpVelMode)
            {
                IInteractionMode *imode;
                drvLeftArm->view(imode);
                
                int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
                leftArmJointsStiffness.length():leftArmJointsDamping.length();
                
                for (int j=0; j<len; j++)
                    imode->setInteractionMode(j,VOCAB_IM_STIFF);
            }
        }
        
        if (useRightArm)
        {
            ICartesianControl *icart;
            drvCartRightArm->view(icart);
            icart->restoreContext(startup_context_id_right);
            
            if (rightArmImpVelMode)
            {
                IInteractionMode *imode;
                drvRightArm->view(imode);
                
                int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
                rightArmJointsStiffness.length():rightArmJointsDamping.length();
                
                for (int j=0; j<len; j++)
                    imode->setInteractionMode(j,VOCAB_IM_STIFF);
            }
        }
        
        gazeCtrl->restoreContext(startup_context_id_gaze);
        
        close();
    }
    
    // Additions
    void ManagerThread::enableGrasping()
    {
        yarp::os::LockGuard guard(this->runMutex);
        if( state==STATE_DISABLED )
        {
            state=STATE_IDLE;
        }
    }
    
    void ManagerThread::disableGrasping()
    {
        yarp::os::LockGuard guard(this->runMutex);
        if( state!=STATE_DISABLED )
        {
            disablingRequested=true;
        }
    }
    // End Additions
    
    // Additions for event ports
    void ManagerThread::sendEvents()
    {
        yarp::os::Bottle & b = eventsOutputPort.prepare();
        b.clear();
        if( state==STATE_DISABLED )
        {
            b.addString("e_grasping_disabled");
        }
        else
        {
            b.addString("e_grasping_enabled");
        }
        eventsOutputPort.write();
    }
    
    // End Additions
    
}
